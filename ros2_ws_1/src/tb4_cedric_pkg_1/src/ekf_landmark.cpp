#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono_literals;

// Struktur für globale Landmarken-Karte
struct Landmark {
    int id;
    double x;
    double y;
    double radius;
};

class EKFWithLandmarksNode : public rclcpp::Node {
public:
    EKFWithLandmarksNode() : Node("ekf_landmark_node") {
        RCLCPP_INFO(this->get_logger(), "EKF mit spezifischer 35cm-Landmarken-Korrektur und Geister-Filter gestartet.");

        // Parameter deklarieren
        this->declare_parameter<double>("q_var_pos", 0.03);
        this->declare_parameter<double>("q_var_theta", 0.015);
        this->declare_parameter<double>("r_var_odom", 0.06);
        this->declare_parameter<double>("r_var_landmark_range", 0.04);  
        this->declare_parameter<double>("r_var_landmark_bearing", 0.03); 
        
        // Dynamische Parameter für die Laser-Geometrie und den Geister-Filter
        this->declare_parameter<double>("target_radius", 0.35);      
        this->declare_parameter<double>("tolerance", 0.015);          
        this->declare_parameter<int>("stride", 4);                    
        this->declare_parameter<int>("scan_skip", 1);                 
        this->declare_parameter<int>("min_votes", 4);                 
        this->declare_parameter<double>("max_jump_distance", 0.5);

        // Parameter einlesen
        q_pos_ = this->get_parameter("q_var_pos").as_double();
        q_theta_ = this->get_parameter("q_var_theta").as_double();
        double r_odom = this->get_parameter("r_var_odom").as_double();
        r_range_ = this->get_parameter("r_var_landmark_range").as_double();
        r_bearing_ = this->get_parameter("r_var_landmark_bearing").as_double();
        
        target_radius_ = this->get_parameter("target_radius").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        stride_ = this->get_parameter("stride").as_int();
        scan_skip_ = this->get_parameter("scan_skip").as_int();
        min_votes_ = this->get_parameter("min_votes").as_int();
        max_jump_distance_ = this->get_parameter("max_jump_distance").as_double();

        // Zustand & Kovarianz initialisieren
        x_hat_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Matrix3d::Identity() * 0.1;

        R_odom_ = Eigen::Matrix2d::Identity() * r_odom;

        // Globale Landmarken-Karte: Nur noch der verlässliche mittlere Pfosten aktiv
        map_.push_back({2, -1.2, 1.2, target_radius_}); // Mittlerer Zylinder bei (-1.2, 1.2), r=35cm

        // Geister-Filter Status zurücksetzen
        has_previous_prediction_ = false;
        last_valid_center_ = Eigen::Vector2d::Zero();

        // Subscriber & Publisher
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
                u_input_(0) = msg->linear.x;
                u_input_(1) = msg->angular.z;
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
                z_odom_(0) = msg->pose.pose.position.x;
                z_odom_(1) = msg->pose.pose.position.y;
                fresh_odom_ = true;
            });

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&EKFWithLandmarksNode::scan_callback, this, std::placeholders::_1));

        filter_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_pose_landmark", 10);
        last_time_ = this->get_clock()->now();
        filter_timer_ = this->create_wall_timer(20ms, std::bind(&EKFWithLandmarksNode::filter_loop, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_counter_++;
        if (scan_counter_ % scan_skip_ != 0) return;

        size_t num_points = msg->ranges.size();
        std::vector<Eigen::Vector2d> local_points(num_points);
        std::vector<bool> valid(num_points, false);

        for (size_t i = 0; i < num_points; ++i) {
            double r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max && !std::isnan(r) && !std::isinf(r)) {
                double angle = msg->angle_min + i * msg->angle_increment;
                local_points[i] = Eigen::Vector2d(r * std::cos(angle), r * std::sin(angle));
                valid[i] = true;
            }
        }

        struct LandmarkCluster {
            Eigen::Vector2d center_sum = Eigen::Vector2d::Zero();
            int votes = 0;
        };

        std::vector<LandmarkCluster> clusters;
        const double cluster_threshold = 0.15; 

        // 1. Geometrische Extraktion des 35cm Zylinders
        for (size_t i = 0; i + 2 * stride_ < num_points; i += 1) {
            size_t idx1 = i;
            size_t idx2 = i + stride_;
            size_t idx3 = i + 2 * stride_;

            if (!valid[idx1] || !valid[idx2] || !valid[idx3]) continue;
            auto p1 = local_points[idx1]; auto p2 = local_points[idx2]; auto p3 = local_points[idx3];

            double a = (p2 - p3).norm(); double b = (p1 - p3).norm(); double c = (p1 - p2).norm();
            double area = 0.5 * std::abs(p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
            if (area < 0.001) continue;

            double R = (a * b * c) / (4.0 * area);

            // Abgleich gegen den dynamischen Zielradius
            if (std::abs(R - target_radius_) < tolerance_) {
                double d = 2.0 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
                if (std::abs(d) < 0.001) continue;
                double cx = ((p1.squaredNorm()) * (p2.y() - p3.y()) + (p2.squaredNorm()) * (p3.y() - p1.y()) + (p3.squaredNorm()) * (p1.y() - p2.y())) / d;
                double cy = ((p1.squaredNorm()) * (p3.x() - p2.x()) + (p2.squaredNorm()) * (p1.x() - p3.x()) + (p3.squaredNorm()) * (p2.x() - p1.x())) / d;
                Eigen::Vector2d new_center(cx, cy);

                bool assigned = false;
                for (auto& clus : clusters) {
                    Eigen::Vector2d current_center = clus.center_sum / clus.votes;
                    if ((current_center - new_center).norm() < cluster_threshold) {
                        clus.center_sum += new_center;
                        clus.votes++;
                        assigned = true;
                        break;
                    }
                }
                if (!assigned) {
                    LandmarkCluster c_new;
                    c_new.center_sum = new_center;
                    c_new.votes = 1;
                    clusters.push_back(c_new);
                }
            }
        }

        // 2. Cluster filtern & zeitliche Vorzeichen- und Sprungkonsistenz (Geister-Filter)
        detected_landmarks_local_.clear();
        for (const auto& clus : clusters) {
            if (clus.votes >= min_votes_) {
                Eigen::Vector2d final_center = clus.center_sum / clus.votes;

                if (has_previous_prediction_) {
                    bool x_sign_changed = (std::signbit(final_center.x()) != std::signbit(last_valid_center_.x()));
                    bool y_sign_changed = (std::signbit(final_center.y()) != std::signbit(last_valid_center_.y()));
                    double jump_dist = (final_center - last_valid_center_).norm();

                    // Wenn Vorzeichenwechsel stattfindet UND ein massiver Koordinatensprung vorliegt -> Geist!
                    if ((x_sign_changed || y_sign_changed) && jump_dist > max_jump_distance_) {
                        RCLCPP_WARN(this->get_logger(), "👻 EKF-Korrektur verweigert: Geisterwert detektiert bei X=%.2f, Y=%.2f (Sprung: %.2fm)", 
                                    final_center.x(), final_center.y(), jump_dist);
                        continue; 
                    }
                }

                // Messung ist konsistent
                detected_landmarks_local_.push_back(final_center);
                last_valid_center_ = final_center;
                has_previous_prediction_ = true;
            }
        }
    }

    void filter_loop() {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0.0) return;

        double theta = x_hat_(2);
        double v = u_input_(0);
        double omega = u_input_(1);

        // =====================================================================
        // 1. PRÄDIKTION
        // =====================================================================
        x_hat_(0) += v * std::cos(theta) * dt;
        x_hat_(1) += v * std::sin(theta) * dt;
        x_hat_(2) += omega * dt;
        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -v * std::sin(theta) * dt;
        F(1, 2) =  v * std::cos(theta) * dt;

        Eigen::Matrix3d Q_local = Eigen::Matrix3d::Zero();
        Q_local(0,0) = q_pos_; Q_local(1,1) = q_pos_ * 0.1; Q_local(2,2) = q_theta_;
        Eigen::Matrix3d R_track = Eigen::Matrix3d::Identity();
        R_track(0,0) = std::cos(theta); R_track(0,1) = -std::sin(theta);
        R_track(1,0) = std::sin(theta); R_track(1,1) =  std::cos(theta);
        Eigen::Matrix3d Q_global = R_track * Q_local * R_track.transpose();

        P_ = F * P_ * F.transpose() + Q_global;

        // =====================================================================
        // 2. KORREKTUR ODOMETRIE
        // =====================================================================
        if (fresh_odom_) {
            Eigen::Matrix<double, 2, 3> H_odom = Eigen::Matrix<double, 2, 3>::Zero();
            H_odom(0, 0) = 1.0; H_odom(1, 1) = 1.0;
            Eigen::Vector2d y_res = z_odom_ - H_odom * x_hat_;
            Eigen::Matrix2d S = H_odom * P_ * H_odom.transpose() + R_odom_;
            Eigen::Matrix<double, 3, 2> K = P_ * H_odom.transpose() * S.inverse();
            x_hat_ = x_hat_ + K * y_res;
            P_ = (Eigen::Matrix3d::Identity() - K * H_odom) * P_;
            fresh_odom_ = false;
        }

        // =====================================================================
        // 3. KORREKTUR GEFILTERTE LANDMARKE (EKF Update)
        // =====================================================================
        for (const auto& local_lm : detected_landmarks_local_) {
            double z_range = local_lm.norm();
            double z_bearing = std::atan2(local_lm.y(), local_lm.x());

            double min_dist = 999.0;
            Landmark best_match;
            bool found_match = false;

            double global_x_est = x_hat_(0) + z_range * std::cos(theta + z_bearing);
            double global_y_est = x_hat_(1) + z_range * std::sin(theta + z_bearing);

            for (const auto& map_lm : map_) {
                double dist = std::hypot(map_lm.x - global_x_est, map_lm.y - global_y_est);
                if (dist < min_dist && dist < 0.6) { 
                    min_dist = dist;
                    best_match = map_lm;
                    found_match = true;
                }
            }

            if (found_match) {
                double dx = best_match.x - x_hat_(0);
                double dy = best_match.y - x_hat_(1);
                double r_est = std::hypot(dx, dy);
                double phi_est = std::atan2(dy, dx) - x_hat_(2);
                phi_est = std::atan2(std::sin(phi_est), std::cos(phi_est));

                Eigen::Vector2d z_meas(z_range, z_bearing);
                Eigen::Vector2d z_est(r_est, phi_est);
                Eigen::Vector2d y_residual = z_meas - z_est;
                y_residual(1) = std::atan2(std::sin(y_residual(1)), std::cos(y_residual(1)));

                Eigen::Matrix<double, 2, 3> H_lm = Eigen::Matrix<double, 2, 3>::Zero();
                H_lm(0, 0) = -dx / r_est;  H_lm(0, 1) = -dy / r_est;  H_lm(0, 2) = 0.0;
                H_lm(1, 0) =  dy / (r_est*r_est); H_lm(1, 1) = -dx / (r_est*r_est); H_lm(1, 2) = -1.0;

                Eigen::Matrix2d R_lm = Eigen::Matrix2d::Zero();
                R_lm(0,0) = r_range_; R_lm(1,1) = r_bearing_;

                Eigen::Matrix2d S = H_lm * P_ * H_lm.transpose() + R_lm;
                Eigen::Matrix<double, 3, 2> K = P_ * H_lm.transpose() * S.inverse();

                x_hat_ = x_hat_ + K * y_residual;
                P_ = (Eigen::Matrix3d::Identity() - K * H_lm) * P_;
                theta = x_hat_(2); 
            }
        }

        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // =====================================================================
        // 4. PUBLISH
        // =====================================================================
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = x_hat_(0);
        pose_msg.pose.pose.position.y = x_hat_(1);
        pose_msg.pose.pose.orientation.z = std::sin(x_hat_(2) / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(x_hat_(2) / 2.0);

        pose_msg.pose.covariance[0] = P_(0,0); pose_msg.pose.covariance[1] = P_(0,1);
        pose_msg.pose.covariance[6] = P_(1,0); pose_msg.pose.covariance[7] = P_(1,1);

        filter_pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub_;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    Eigen::Vector3d x_hat_;
    Eigen::Matrix3d P_;
    Eigen::Matrix2d R_odom_;
    double r_range_, r_bearing_, q_pos_, q_theta_;
    
    double target_radius_, tolerance_, max_jump_distance_;
    int stride_, scan_skip_, min_votes_;
    int scan_counter_ = 0;

    bool has_previous_prediction_;
    Eigen::Vector2d last_valid_center_;

    Eigen::Vector2d u_input_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d z_odom_ = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> detected_landmarks_local_;
    std::vector<Landmark> map_;

    bool fresh_odom_ = false;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFWithLandmarksNode>());
    rclcpp::shutdown();
    return 0;
}
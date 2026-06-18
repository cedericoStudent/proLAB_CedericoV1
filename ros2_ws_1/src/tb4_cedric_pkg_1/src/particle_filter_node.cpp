#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono_literals;

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode() : Node("particle_filter_node") {
        RCLCPP_INFO(this->get_logger(), "Partikelfilter mit Ausgangs-Glättung gestartet.");

        // Parameter
        this->declare_parameter<int>("num_particles", 1500);
        this->declare_parameter<double>("q_var_pos", 0.001);
        this->declare_parameter<double>("q_var_theta", 0.001);
        
        this->declare_parameter<double>("target_radius", 0.35);
        this->declare_parameter<double>("tolerance", 0.01);
        this->declare_parameter<int>("stride", 4);
        this->declare_parameter<int>("scan_skip", 1);
        this->declare_parameter<int>("min_votes", 5);
        this->declare_parameter<double>("max_jump_distance", 0.8);
        
        // NEU: Glättungsfaktor (0.0 = keine Aktualisierung, 1.0 = ungefiltert roher PF-Mittelwert)
        this->declare_parameter<double>("alpha_smooth", 0.15); 

        num_particles_ = this->get_parameter("num_particles").as_int();
        q_pos_ = this->get_parameter("q_var_pos").as_double();
        q_theta_ = this->get_parameter("q_var_theta").as_double();
        target_radius_ = this->get_parameter("target_radius").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        stride_ = this->get_parameter("stride").as_int();
        scan_skip_ = this->get_parameter("scan_skip").as_int();
        min_votes_ = this->get_parameter("min_votes").as_int();
        max_jump_distance_ = this->get_parameter("max_jump_distance").as_double();
        alpha_smooth_ = this->get_parameter("alpha_smooth").as_double();

        landmark_x_ = -1.2;
        landmark_y_ = 1.2;

        initialize_particles();

        // Internen geglätteten Zustand initialisieren
        x_smooth_ = Eigen::Vector3d::Zero();

        has_previous_prediction_ = false;
        last_valid_center_ = Eigen::Vector2d::Zero();

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
                u_input_(0) = msg->linear.x;
                u_input_(1) = msg->angular.z;
            });

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ParticleFilterNode::scan_callback, this, std::placeholders::_1));

        filter_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf_estimated_pose", 10);
        last_time_ = this->get_clock()->now();
        filter_timer_ = this->create_wall_timer(20ms, std::bind(&ParticleFilterNode::filter_loop, this));
    }

private:
    void initialize_particles() {
        particles_.resize(num_particles_);
        double initial_weight = 1.0 / num_particles_;
        std::normal_distribution<double> d_pos(0.0, 0.05);
        std::normal_distribution<double> d_theta(0.0, 0.02);

        for (int i = 0; i < num_particles_; ++i) {
            particles_[i].x = d_pos(gen_);
            particles_[i].y = d_pos(gen_);
            particles_[i].theta = d_theta(gen_);
            particles_[i].weight = initial_weight;
        }
    }

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

        for (size_t i = 0; i + 2 * stride_ < num_points; i += 1) {
            size_t idx1 = i; size_t idx2 = i + stride_; size_t idx3 = i + 2 * stride_;
            if (!valid[idx1] || !valid[idx2] || !valid[idx3]) continue;

            auto p1 = local_points[idx1]; auto p2 = local_points[idx2]; auto p3 = local_points[idx3];
            double a = (p2 - p3).norm(); double b = (p1 - p3).norm(); double c = (p1 - p2).norm();
            double area = 0.5 * std::abs(p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
            if (area < 0.001) continue;

            double R = (a * b * c) / (4.0 * area);

            if (std::abs(R - target_radius_) < tolerance_) {
                double d = 2.0 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
                if (std::abs(d) < 0.001) continue;
                double cx = ((p1.squaredNorm()) * (p2.y() - p3.y()) + (p2.squaredNorm()) * (p3.y() - p1.y()) + (p3.squaredNorm()) * (p1.y() - p2.y())) / d;
                double cy = ((p1.squaredNorm()) * (p3.x() - p2.x()) + (p2.squaredNorm()) * (p1.x() - p3.x()) + (p3.squaredNorm()) * (p2.x() - p1.x())) / d;
                Eigen::Vector2d new_center(cx, cy);

                bool assigned = false;
                for (auto& clus : clusters) {
                    if ((clus.center_sum / clus.votes - new_center).norm() < cluster_threshold) {
                        clus.center_sum += new_center; clus.votes++; assigned = true; break;
                    }
                }
                if (!assigned) {
                    LandmarkCluster c_new; c_new.center_sum = new_center; c_new.votes = 1;
                    clusters.push_back(c_new);
                }
            }
        }

        detected_landmarks_local_.clear();
        for (const auto& clus : clusters) {
            if (clus.votes >= min_votes_) {
                Eigen::Vector2d final_center = clus.center_sum / clus.votes;

                if (has_previous_prediction_) {
                    bool x_sign_changed = (std::signbit(final_center.x()) != std::signbit(last_valid_center_.x()));
                    bool y_sign_changed = (std::signbit(final_center.y()) != std::signbit(last_valid_center_.y()));
                    double jump_dist = (final_center - last_valid_center_).norm();

                    if ((x_sign_changed || y_sign_changed) && jump_dist > max_jump_distance_) {
                        continue;
                    }
                }
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

        // =====================================================================
        // 1. PRÄDIKTION
        // =====================================================================
        double v = u_input_(0);
        double omega = u_input_(1);

        std::normal_distribution<double> noise_pos(0.0, std::sqrt(q_pos_ * dt));
        std::normal_distribution<double> noise_theta(0.0, std::sqrt(q_theta_ * dt));

        for (auto& p : particles_) {
            p.x += (v * std::cos(p.theta)) * dt + noise_pos(gen_);
            p.y += (v * std::sin(p.theta)) * dt + noise_pos(gen_) * 0.1; 
            p.theta += omega * dt + noise_theta(gen_);
            p.theta = std::atan2(std::sin(p.theta), std::cos(p.theta));
        }

        // =====================================================================
        // 2. KORREKTUR (Nur wenn Landmarke sichtbar)
        // =====================================================================
        if (!detected_landmarks_local_.empty()) {
            Eigen::Vector2d z_local = detected_landmarks_local_[0];
            double z_range = z_local.norm();
            double z_bearing = std::atan2(z_local.y(), z_local.x());

            double weight_sum = 0.0;
            double r_var_euclidean = 0.08; // Erhöht für sanftere Übergänge

            for (auto& p : particles_) {
                double glob_x_meas = p.x + z_range * std::cos(p.theta + z_bearing);
                double glob_y_meas = p.y + z_range * std::sin(p.theta + z_bearing);

                double dx = landmark_x_ - glob_x_meas;
                double dy = landmark_y_ - glob_y_meas;
                double distance_error_sq = dx*dx + dy*dy;

                double likelihood = (1.0 / (std::sqrt(2.0 * M_PI * r_var_euclidean))) * std::exp(-distance_error_sq / (2.0 * r_var_euclidean));

                p.weight *= likelihood;
                if (p.weight < 1e-300) p.weight = 1e-300;
                weight_sum += p.weight;
            }

            if (weight_sum > 0.0) {
                for (auto& p : particles_) p.weight /= weight_sum;
            } else {
                for (auto& p : particles_) p.weight = 1.0 / num_particles_;
            }

            // =====================================================================
            // 3. RESAMPLING (Jetzt sauber verschachtelt im Messungs-Block!)
            // =====================================================================
            double n_eff = 0.0;
            for (const auto& p : particles_) n_eff += p.weight * p.weight;
            n_eff = 1.0 / n_eff;

            if (n_eff < num_particles_ / 2.0) {
                std::vector<Particle> resampled_particles(num_particles_);
                double r = std::uniform_real_distribution<double>(0.0, 1.0 / num_particles_)(gen_);
                double c = particles_[0].weight;
                int idx = 0;

                for (int i = 0; i < num_particles_; ++i) {
                    double u = r + (double)i / num_particles_;
                    while (u > c && idx < num_particles_ - 1) {
                        idx++;
                        c += particles_[idx].weight;
                    }
                    resampled_particles[i] = particles_[idx];
                    resampled_particles[i].weight = 1.0 / num_particles_;
                }
                particles_ = resampled_particles;
            }
        }

        // =====================================================================
        // 4. MITTELWERTS-BILDUNG & EXPONENTIELLE GLÄTTUNG (ALPHA-FILTER)
        // =====================================================================
        double raw_mean_x = 0.0, raw_mean_y = 0.0;
        double sum_sin = 0.0, sum_cos = 0.0;
        double var_x = 0.0, var_y = 0.0;

        for (const auto& p : particles_) {
            raw_mean_x += p.x * p.weight;
            raw_mean_y += p.y * p.weight;
            sum_cos += std::cos(p.theta) * p.weight;
            sum_sin += std::sin(p.theta) * p.weight;
        }
        double raw_mean_theta = std::atan2(sum_sin, sum_cos);

        // Der Glättungsschritt (Alpha-Filter) fängt Resampling-Sprünge ab
        if (x_smooth_(0) == 0.0 && x_smooth_(1) == 0.0) {
            // Beim allerersten Durchlauf direkt setzen
            x_smooth_(0) = raw_mean_x;
            x_smooth_(1) = raw_mean_y;
            x_smooth_(2) = raw_mean_theta;
        } else {
            x_smooth_(0) = (1.0 - alpha_smooth_) * x_smooth_(0) + alpha_smooth_ * raw_mean_x;
            x_smooth_(1) = (1.0 - alpha_smooth_) * x_smooth_(1) + alpha_smooth_ * raw_mean_y;
            
            // Winkelsprung-sichere Glättung der Orientierung
            double s_theta = (1.0 - alpha_smooth_) * std::sin(x_smooth_(2)) + alpha_smooth_ * std::sin(raw_mean_theta);
            double c_theta = (1.0 - alpha_smooth_) * std::cos(x_smooth_(2)) + alpha_smooth_ * std::cos(raw_mean_theta);
            x_smooth_(2) = std::atan2(s_theta, c_theta);
        }

        // Varianzschätzung auf Basis des glatten Zustands berechnen (für kleine Plotter-Ellipsen)
        for (const auto& p : particles_) {
            var_x += p.weight * (p.x - x_smooth_(0)) * (p.x - x_smooth_(0));
            var_y += p.weight * (p.y - x_smooth_(1)) * (p.y - x_smooth_(1));
        }

        // PUBLISH
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = x_smooth_(0);
        pose_msg.pose.pose.position.y = x_smooth_(1);
        pose_msg.pose.pose.orientation.z = std::sin(x_smooth_(2) / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(x_smooth_(2) / 2.0);

        pose_msg.pose.covariance[0] = var_x;   pose_msg.pose.covariance[1] = 0.0;
        pose_msg.pose.covariance[6] = 0.0;     pose_msg.pose.covariance[7] = var_y;

        filter_pose_pub->publish(pose_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    int num_particles_;
    double q_pos_, q_theta_;
    double target_radius_, tolerance_, max_jump_distance_, alpha_smooth_;
    int stride_, scan_skip_, min_votes_;
    int scan_counter_ = 0;

    double landmark_x_, landmark_y_;

    bool has_previous_prediction_;
    Eigen::Vector2d last_valid_center_;

    Eigen::Vector2d u_input_ = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> detected_landmarks_local_;
    std::vector<Particle> particles_;
    
    Eigen::Vector3d x_smooth_; // Interner geglätteter Zustand

    std::mt19937 gen_{std::random_device{}()};
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}
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

// Struktur für ein einzelnes Partikel
struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode() : Node("particle_filter_node") {
        RCLCPP_INFO(this->get_logger(), "Partikelfilter für 35cm-Landmarke gestartet.");

        // Parameter deklarieren
        this->declare_parameter<int>("num_particles", 500);
        this->declare_parameter<double>("q_var_pos", 0.03);
        this->declare_parameter<double>("q_var_theta", 0.015);
        this->declare_parameter<double>("r_var_landmark_range", 0.04);
        this->declare_parameter<double>("r_var_landmark_bearing", 0.03);

        // Geometrie- & Geister-Filter-Parameter
        this->declare_parameter<double>("target_radius", 0.35);
        this->declare_parameter<double>("tolerance", 0.015);
        this->declare_parameter<int>("stride", 4);
        this->declare_parameter<int>("scan_skip", 1);
        this->declare_parameter<int>("min_votes", 4);
        this->declare_parameter<double>("max_jump_distance", 0.2);

        // Parameter einlesen
        num_particles_ = this->get_parameter("num_particles").as_int();
        q_pos_ = this->get_parameter("q_var_pos").as_double();
        q_theta_ = this->get_parameter("q_var_theta").as_double();
        r_range_ = this->get_parameter("r_var_landmark_range").as_double();
        r_bearing_ = this->get_parameter("r_var_landmark_bearing").as_double();

        target_radius_ = this->get_parameter("target_radius").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        stride_ = this->get_parameter("stride").as_int();
        scan_skip_ = this->get_parameter("scan_skip").as_int();
        min_votes_ = this->get_parameter("min_votes").as_int();
        max_jump_distance_ = this->get_parameter("max_jump_distance").as_double();

        // Feste Landmarken-Position (analog zum EKF)
        landmark_x_ = -1.2;
        landmark_y_ = 1.2;

        // Partikel-Initialisierung um den Startpunkt (0,0,0)
        initialize_particles();

        // Geister-Filter Status
        has_previous_prediction_ = false;
        last_valid_center_ = Eigen::Vector2d::Zero();

        // Subscriber & Publisher
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
                u_input_(0) = msg->linear.x;
                u_input_(1) = msg->angular.z;
            });

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ParticleFilterNode::scan_callback, this, std::placeholders::_1));

        filter_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf_estimated_pose", 10);
        last_time_ = this->get_clock()->now();
        filter_timer_ = this->create_wall_timer(20ms, std::bind(&ParticleFilterNode::filter_loop, this));
    }

private:
    void initialize_particles() {
        particles_.resize(num_particles_);
        double initial_weight = 1.0 / num_particles_;
        
        // Initialer Zustand leicht verrauscht um (0, 0, 0) verteilen
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

        // Geister-Filter anwenden
        detected_landmarks_local_.clear();
        for (const auto& clus : clusters) {
            if (clus.votes >= min_votes_) {
                Eigen::Vector2d final_center = clus.center_sum / clus.votes;

                if (has_previous_prediction_) {
                    bool x_sign_changed = (std::signbit(final_center.x()) != std::signbit(last_valid_center_.x()));
                    bool y_sign_changed = (std::signbit(final_center.y()) != std::signbit(last_valid_center_.y()));
                    double jump_dist = (final_center - last_valid_center_).norm();

                    if ((x_sign_changed || y_sign_changed) && jump_dist > max_jump_distance_) {
                        RCLCPP_WARN(this->get_logger(), "👻 PF-Korrektur verweigert Geisterwert bei X=%.2f, Y=%.2f", final_center.x(), final_center.y());
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
        // 1. PRÄDIKTIONS-SCHRITT (Partikel mit Prozessrauschen bewegen)
        // =====================================================================
        double v = u_input_(0);
        double omega = u_input_(1);

        std::normal_distribution<double> noise_pos(0.0, std::sqrt(q_pos_ * dt));
        std::normal_distribution<double> noise_theta(0.0, std::sqrt(q_theta_ * dt));

        for (auto& p : particles_) {
            // Kinematik + Rauschen addieren
            p.x += (v * std::cos(p.theta)) * dt + noise_pos(gen_);
            p.y += (v * std::sin(p.theta)) * dt + noise_pos(gen_) * 0.1; // Querabweichung geringer analog zum EKF Setup
            p.theta += omega * dt + noise_theta(gen_);
            p.theta = std::atan2(std::sin(p.theta), std::cos(p.theta));
        }

        // =====================================================================
        // 2. KORREKTUR-SCHRITT (Weights updaten basierend auf Messung)
        // =====================================================================
        if (!detected_landmarks_local_.empty()) {
            // Wir nehmen die erste verifizierte Landmarken-Messung dieses Zyklus
            Eigen::Vector2d z_local = detected_landmarks_local_[0];
            double z_range = z_local.norm();
            double z_bearing = std::atan2(z_local.y(), z_local.x());

            double weight_sum = 0.0;

            for (auto& p : particles_) {
                // Erwartete Messung aus Sicht dieses spezifischen Partikels berechnen
                double dx = landmark_x_ - p.x;
                double dy = landmark_y_ - p.y;
                double h_range = std::hypot(dx, dy);
                double h_bearing = std::atan2(dy, dx) - p.theta;
                h_bearing = std::atan2(std::sin(h_bearing), std::cos(h_bearing));

                // Residuen bestimmen
                double error_r = z_range - h_range;
                double error_b = z_bearing - h_bearing;
                error_b = std::atan2(std::sin(error_b), std::cos(error_b));

                // Gaußsche Wahrscheinlichkeitsdichte (Likelihood) berechnen
                double prob_r = (1.0 / (std::sqrt(2.0 * M_PI * r_range_))) * std::exp(-(error_r * error_r) / (2.0 * r_range_));
                double prob_b = (1.0 / (std::sqrt(2.0 * M_PI * r_bearing_))) * std::exp(-(error_b * error_b) / (2.0 * r_bearing_));
                
                // Gewicht multiplizieren (Verknüpfung der Messungen)
                p.weight *= (prob_r * prob_b);
                if (p.weight < 1e-300) p.weight = 1e-300; // Unterlauf verhindern
                weight_sum += p.weight;
            }

            // Gewichte normalisieren
            if (weight_sum > 0.0) {
                for (auto& p : particles_) {
                    p.weight /= weight_sum;
                }
            } else {
                // Falls alle Partikel "sterben", gleichmäßig reinitialisieren
                for (auto& p : particles_) p.weight = 1.0 / num_particles_;
            }

            // =====================================================================
            // 3. RESAMPLING (Systematisch, falls N_eff kritisch niedrig)
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
        // 4. MITTELWERTS-BILDUNG & PUBLISH
        // =====================================================================
        double mean_x = 0.0, mean_y = 0.0;
        double sum_sin = 0.0, sum_cos = 0.0;
        double var_x = 0.0, var_y = 0.0;

        for (const auto& p : particles_) {
            mean_x += p.x * p.weight;
            mean_y += p.y * p.weight;
            sum_cos += std::cos(p.theta) * p.weight;
            sum_sin += std::sin(p.theta) * p.weight;
        }
        double mean_theta = std::atan2(sum_sin, sum_cos);

        // Einfache Varianzschätzung für Kovarianzmatrix-Visualisierung im Plotter
        for (const auto& p : particles_) {
            var_x += p.weight * (p.x - mean_x) * (p.x - mean_x);
            var_y += p.weight * (p.y - mean_y) * (p.y - mean_y);
        }

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = mean_x;
        pose_msg.pose.pose.position.y = mean_y;
        pose_msg.pose.pose.orientation.z = std::sin(mean_theta / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(mean_theta / 2.0);

        // Kovarianz-Zuweisung analog zum EKF, damit der Plotter fehlerfrei rendert
        pose_msg.pose.covariance[0] = var_x;   pose_msg.pose.covariance[1] = 0.0;
        pose_msg.pose.covariance[6] = 0.0;     pose_msg.pose.covariance[7] = var_y;

        filter_pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub_;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    int num_particles_;
    double q_pos_, q_theta_, r_range_, r_bearing_;
    double target_radius_, tolerance_, max_jump_distance_;
    int stride_, scan_skip_, min_votes_;
    int scan_counter_ = 0;

    double landmark_x_, landmark_y_;

    bool has_previous_prediction_;
    Eigen::Vector2d last_valid_center_;

    Eigen::Vector2d u_input_ = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> detected_landmarks_local_;
    std::vector<Particle> particles_;

    std::mt19937 gen_{std::random_device{}()};
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}
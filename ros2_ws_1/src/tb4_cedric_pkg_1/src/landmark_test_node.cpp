#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include <Eigen/Dense> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"

class LandmarkTestNode : public rclcpp::Node {
public:
    LandmarkTestNode() : Node("landmark_test_node") {
        RCLCPP_INFO(this->get_logger(), "Spezifischer Landmarken-Detektor mit Geister-Filter gestartet.");

        this->declare_parameter<double>("target_radius", 0.35);      
        this->declare_parameter<double>("tolerance", 0.015);          
        this->declare_parameter<int>("stride", 7);                    
        this->declare_parameter<int>("scan_skip", 1);                 
        this->declare_parameter<int>("min_votes", 4);                 
        
        // NEU: Parameter für die maximale Sprungdistanz zwischen zwei Scans (in Metern)
        this->declare_parameter<double>("max_jump_distance", 0.5);

        update_parameters();

        // Initialisierung des Tracking-Status
        has_previous_prediction_ = false;
        last_valid_center_ = Eigen::Vector2d::Zero();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LandmarkTestNode::scan_callback, this, std::placeholders::_1));
    }

private:
    void update_parameters() {
        target_radius_ = this->get_parameter("target_radius").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        stride_ = this->get_parameter("stride").as_int();
        scan_skip_ = this->get_parameter("scan_skip").as_int();
        min_votes_ = this->get_parameter("min_votes").as_int();
        max_jump_distance_ = this->get_parameter("max_jump_distance").as_double();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        update_parameters();

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

        // 1. Geometrische Suche
        for (size_t i = 0; i + 2 * stride_ < num_points; i += 1) { 
            size_t idx1 = i;
            size_t idx2 = i + stride_;
            size_t idx3 = i + 2 * stride_;

            if (!valid[idx1] || !valid[valid[idx2]] || !valid[idx3]) continue;
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

        // 2. Cluster filtern und zeitliche Konsistenz (Geister-Filter) prüfen
        detected_landmarks_local_.clear();
        
        for (const auto& clus : clusters) {
            if (clus.votes >= min_votes_) {
                Eigen::Vector2d final_center = clus.center_sum / clus.votes;
                
                // --- GEISTER-FILTER LOGIK ---
                if (has_previous_prediction_) {
                    // Prüfe, ob sich das Vorzeichen einer Achse geändert hat
                    bool x_sign_changed = (std::signbit(final_center.x()) != std::signbit(last_valid_center_.x()));
                    bool y_sign_changed = (std::signbit(final_center.y()) != std::signbit(last_valid_center_.y()));
                    
                    // Berechne die tatsächliche Sprungdistanz zum letzten bekannten Wert
                    double jump_dist = (final_center - last_valid_center_).norm();
                    
                    // Wenn Vorzeichenwechsel stattfindet UND der Sprung massiv ist (> max_jump_distance_)
                    if ((x_sign_changed || y_sign_changed) && jump_dist > max_jump_distance_) {
                        RCLCPP_WARN(this->get_logger(), "👻 GEIST GEFILTERT! Sprung detektiert nach X=%.3f, Y=%.3f (Distanz: %.3fm)", 
                                    final_center.x(), final_center.y(), jump_dist);
                        continue; // Überspringe diesen Geist, füge ihn nicht zu den stabilen Landmarken hinzu
                    }
                }
                
                // Wenn die Validierung bestanden wurde (oder es die allererste Messung ist)
                detected_landmarks_local_.push_back(final_center);
                last_valid_center_ = final_center; // Aktualisiere den "letzten guten Wert"
                has_previous_prediction_ = true;
                
                RCLCPP_INFO(this->get_logger(), "🎯 LANDMARKE VERIFIZIERT (r=%.2fm): Relativ X = %.3fm, Y = %.3fm | Distanz: %.3fm (Treffer: %d)", 
                            target_radius_, final_center.x(), final_center.y(), final_center.norm(), clus.votes);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double target_radius_, tolerance_, max_jump_distance_;
    int stride_, scan_skip_, min_votes_;
    int scan_counter_ = 0;

    // Speicher für den zeitlichen Filter
    bool has_previous_prediction_;
    Eigen::Vector2d last_valid_center_;

    std::vector<Eigen::Vector2d> detected_landmarks_local_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkTestNode>());
    rclcpp::shutdown();
    return 0;
}
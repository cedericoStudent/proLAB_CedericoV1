#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// CRUCIAL: Eigen Header einbinden, sonst kennt der Compiler Eigen::Vector2d nicht!
#include <Eigen/Dense> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"

struct DetectedPoint {
    double x;
    double y;
    std::string type;
};

class LandmarkTestNode : public rclcpp::Node {
public:
    LandmarkTestNode() : Node("landmark_test_node") {
        RCLCPP_INFO(this->get_logger(), "Landmarken-Detektor (Multi-Zylinder) gestartet.");

        // Parameter für die drei Zylinderradien (in Metern)
        this->declare_parameter<double>("radius_small", 0.1);   // Kleiner Zylinder
        this->declare_parameter<double>("radius_medium", 0.25);  // Mittlerer Zylinder
        this->declare_parameter<double>("radius_large", 0.4);   // Großer Zylinder
        this->declare_parameter<double>("tolerance", 0.02);      // Toleranzfenster: +/- 2cm
        this->declare_parameter<int>("stride", 4);               // Schrittweite im Dreierblock
        this->declare_parameter<int>("scan_skip", 2);            // Jeden X. Scan verarbeiten

        r_small_ = this->get_parameter("radius_small").as_double();
        r_medium_ = this->get_parameter("radius_medium").as_double();
        r_large_ = this->get_parameter("radius_large").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        stride_ = this->get_parameter("stride").as_int();
        scan_skip_ = this->get_parameter("scan_skip").as_int();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LandmarkTestNode::scan_callback, this, std::placeholders::_1));
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

        // Struktur für erweiterte Cluster-Validierung
        struct LandmarkCluster {
            Eigen::Vector2d center_sum = Eigen::Vector2d::Zero();
            int votes = 0;
            std::string type = "";
            double expected_r = 0.0;
        };

        std::vector<LandmarkCluster> clusters;
        const double cluster_threshold = 0.15; // 15cm Radius um Zentren zusammenzufassen
        const double tight_tolerance = 0.018;   // Verschärfte Radien-Toleranz (~1.2cm)

        // 1. Dreierblöcke prüfen und vorklassifizieren
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

            std::string detected_type = "";
            double target_r = 0.0;

            // Härteres Matching gegen deine exakten Radien
            if (std::abs(R - 0.10) < tight_tolerance) { detected_type = "KLEIN"; target_r = 0.10; }
            else if (std::abs(R - 0.25) < tight_tolerance) { detected_type = "MITTEL"; target_r = 0.25; }
            else if (std::abs(R - 0.40) < tight_tolerance) { detected_type = "GROSS"; target_r = 0.40; }

            if (!detected_type.empty()) {
                double d = 2.0 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
                if (std::abs(d) < 0.001) continue;
                double cx = ((p1.squaredNorm()) * (p2.y() - p3.y()) + (p2.squaredNorm()) * (p3.y() - p1.y()) + (p3.squaredNorm()) * (p1.y() - p2.y())) / d;
                double cy = ((p1.squaredNorm()) * (p3.x() - p2.x()) + (p2.squaredNorm()) * (p1.x() - p3.x()) + (p3.squaredNorm()) * (p2.x() - p1.x())) / d;
                Eigen::Vector2d new_center(cx, cy);

                // Zu existierendem Cluster hinzufügen oder neues eröffnen
                bool assigned = false;
                for (auto& clus : clusters) {
                    Eigen::Vector2d current_center = clus.center_sum / clus.votes;
                    if ((current_center - new_center).norm() < cluster_threshold) {
                        if (clus.type != detected_type) {
                            clus.type = "INVALID"; 
                        }
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
                    c_new.type = detected_type;
                    c_new.expected_r = target_r;
                    clusters.push_back(c_new);
                }
            }
        }

        // 2. Cluster filtern und finale Landmarken für den EKF bereitstellen
        detected_landmarks_local_.clear();
        
        for (const auto& clus : clusters) {
            if (clus.type != "INVALID" && clus.votes >= 4) {
                Eigen::Vector2d final_center = clus.center_sum / clus.votes;
                detected_landmarks_local_.push_back(final_center);
                
                // Stabiles Logging aktivieren, um den Erfolg im Terminal zu sehen:
                RCLCPP_INFO(this->get_logger(), "✅ Stabile Landmarke verifiziert: %s bei X=%.3f m, Y=%.3f m (Votes: %d)", 
                            clus.type.c_str(), final_center.x(), final_center.y(), clus.votes);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double r_small_, r_medium_, r_large_, tolerance_;
    int stride_, scan_skip_;
    int scan_counter_ = 0;

    // HIER GEFEHLT: Deklaration des lokalen Speicher-Vektors für detektierte Zentren
    std::vector<Eigen::Vector2d> detected_landmarks_local_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkTestNode>());
    rclcpp::shutdown();
    return 0;
}
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
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
        this->declare_parameter<double>("radius_medium", 0.2);  // Mittlerer Zylinder
        this->declare_parameter<double>("radius_large", 0.3);   // Großer Zylinder
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
        std::vector<geometry_msgs::msg::Point> points(num_points);
        std::vector<bool> valid_points(num_points, false);

        // 1. Polarkoordinaten in kartesische Roboterkoordinaten umrechnen
        for (size_t i = 0; i < num_points; ++i) {
            double r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max && !std::isnan(r) && !std::isinf(r)) {
                double angle = msg->angle_min + i * msg->angle_increment;
                points[i].x = r * std::cos(angle);
                points[i].y = r * std::sin(angle);
                valid_points[i] = true;
            }
        }

        std::vector<DetectedPoint> raw_detections;

        // 2. Geometrische Suche über Dreierblöcke mit Stride
        for (size_t i = 0; i + 2 * stride_ < num_points; i += 2) {
            size_t idx1 = i;
            size_t idx2 = i + stride_;
            size_t idx3 = i + 2 * stride_;

            if (!valid_points[idx1] || !valid_points[idx2] || !valid_points[idx3]) continue;

            auto p1 = points[idx1];
            auto p2 = points[idx2];
            auto p3 = points[idx3];

            double a = std::hypot(p2.x - p3.x, p2.y - p3.y);
            double b = std::hypot(p1.x - p3.x, p1.y - p3.y);
            double c = std::hypot(p1.x - p2.x, p1.y - p2.y);

            double area = 0.5 * std::abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
            if (area < 0.001) continue; // Wand-Ausschluss

            double R = (a * b * c) / (4.0 * area);

            // Klassifizierung basierend auf dem Radius
            std::string cylinder_type = "";
            if (std::abs(R - r_small_) < tolerance_) {
                cylinder_type = "KLEINER ZYLINDER (r=" + std::to_string(r_small_).substr(0,4) + "m)";
            } else if (std::abs(R - r_medium_) < tolerance_) {
                cylinder_type = "MITTLERER ZYLINDER (r=" + std::to_string(r_medium_).substr(0,4) + "m)";
            } else if (std::abs(R - r_large_) < tolerance_) {
                cylinder_type = "GROSSER ZYLINDER (r=" + std::to_string(r_large_).substr(0,4) + "m)";
            }

            // Wenn ein passender Radius gefunden wurde, berechne das Zentrum
            if (!cylinder_type.empty()) {
                double d = 2.0 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
                if (std::abs(d) < 0.001) continue;

                double cx = ((p1.x*p1.x + p1.y*p1.y) * (p2.y - p3.y) + (p2.x*p2.x + p2.y*p2.y) * (p3.y - p1.y) + (p3.x*p3.x + p3.y*p3.y) * (p1.y - p2.y)) / d;
                double cy = ((p1.x*p1.x + p1.y*p1.y) * (p3.x - p2.x) + (p2.x*p2.x + p2.y*p2.y) * (p1.x - p3.x) + (p3.x*p3.x + p3.y*p3.y) * (p2.x - p1.x)) / d;

                raw_detections.push_back({cx, cy, cylinder_type});
            }
        }

        // 3. Nachgelagertes Clustering, damit das Terminal lesbar bleibt
        // Benachbarte Erkennungen desselben Zylinders werden fusioniert
        std::vector<DetectedPoint> clustered_detections;
        const double cluster_threshold = 0.25; // 25cm maximale Distanz zwischen Punkten im selben Cluster

        for (const auto& raw : raw_detections) {
            bool found_cluster = false;
            for (auto& cluster : clustered_detections) {
                if (cluster.type == raw.type) {
                    double dist = std::hypot(cluster.x - raw.x, cluster.y - raw.y);
                    if (dist < cluster_threshold) {
                        // Fließender Mittelwert (Schwerpunkt) des Zylindermittelpunkts
                        cluster.x = (cluster.x + raw.x) / 2.0;
                        cluster.y = (cluster.y + raw.y) / 2.0;
                        found_cluster = true;
                        break;
                    }
                }
            }
            if (!found_cluster) {
                clustered_detections.push_back(raw);
            }
        }

        // 4. "Alarm" schlagen im Terminal
        if (!clustered_detections.empty()) {
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
            for (const auto& landmark : clustered_detections) {
                RCLCPP_INFO(this->get_logger(), "🚨 LANDMARKE DETEKTIERT: %s", landmark.type.c_str());
                RCLCPP_INFO(this->get_logger(), "   ↳ Position relativ zum Roboter: X = %.3f m, Y = %.3f m", landmark.x, landmark.y);
                RCLCPP_INFO(this->get_logger(), "   ↳ Distanz zum Roboter: %.3f m", std::hypot(landmark.x, landmark.y));
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double r_small_, r_medium_, r_large_, tolerance_;
    int stride_, scan_skip_;
    int scan_counter_ = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkTestNode>());
    rclcpp::shutdown();
    return 0;
}
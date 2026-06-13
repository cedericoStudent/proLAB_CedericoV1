#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Neu für die KF-Pose
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

class DataRecorder : public rclcpp::Node {
public:
    DataRecorder() : Node("data_recorder") {
        try {
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("tb4_cedric_pkg_1");
            
            // Pfade für BEIDE CSV-Dateien definieren
            fs::path path_gt = fs::path(pkg_share_dir) / "trajectories" / "robot_path1.csv";
            fs::path path_kf = fs::path(pkg_share_dir) / "trajectories" / "kf_path1.csv";
            
            // 1. Ground Truth Datei öffnen
            output_file_gt_.open(path_gt.string());
            if (output_file_gt_.is_open()) {
                output_file_gt_ << "x,y\n";
                RCLCPP_INFO(this->get_logger(), "GT CSV wird gespeichert in: %s", path_gt.string().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Konnte GT Datei nicht öffnen!");
            }

            // 2. KF Datei öffnen
            output_file_kf_.open(path_kf.string());
            if (output_file_kf_.is_open()) {
                output_file_kf_ << "x,y\n";
                RCLCPP_INFO(this->get_logger(), "KF CSV wird gespeichert in: %s", path_kf.string().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Konnte KF Datei nicht öffnen!");
            }

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Finden des Pakets: %s", e.what());
        }

        // Subscriber für Ground Truth
        subscription_gt_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DataRecorder::odom_callback, this, std::placeholders::_1));

        // Subscriber für Kalman-Filter Schätzung
        subscription_kf_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/kf_estimated_pose", 10, std::bind(&DataRecorder::kf_callback, this, std::placeholders::_1));
    }

    ~DataRecorder() {
        if (output_file_gt_.is_open()) output_file_gt_.close();
        if (output_file_kf_.is_open()) output_file_kf_.close();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (output_file_gt_.is_open()) {
            output_file_gt_ << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "\n";
            output_file_gt_.flush(); 
        }
    }

    void kf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (output_file_kf_.is_open()) {
            output_file_kf_ << msg->pose.position.x << "," << msg->pose.position.y << "\n";
            output_file_kf_.flush(); 
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_gt_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_kf_;
    std::ofstream output_file_gt_;
    std::ofstream output_file_kf_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorder>());
    rclcpp::shutdown();
    return 0;
}
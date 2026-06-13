#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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
            fs::path path_gt = fs::path(pkg_share_dir) / "trajectories" / "robot_path1.csv";
            fs::path path_kf = fs::path(pkg_share_dir) / "trajectories" / "kf_path1.csv";
            
            output_file_gt_.open(path_gt.string());
            if (output_file_gt_.is_open()) output_file_gt_ << "x,y\n";

            output_file_kf_.open(path_kf.string());
            if (output_file_kf_.is_open()) {
                // Zusätzliche Spalten für die Kovarianz-Matrix-Einträge
                output_file_kf_ << "x,y,p_xx,p_xy,p_yy\n";
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler: %s", e.what());
        }

        subscription_gt_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DataRecorder::odom_callback, this, std::placeholders::_1));

        subscription_kf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
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

    void kf_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (output_file_kf_.is_open()) {
            output_file_kf_ << msg->pose.pose.position.x << "," 
                            << msg->pose.pose.position.y << ","
                            << msg->pose.covariance[0] << ","  // p_xx
                            << msg->pose.covariance[1] << ","  // p_xy
                            << msg->pose.covariance[7] << "\n"; // p_yy
            output_file_kf_.flush(); 
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_gt_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_kf_;
    std::ofstream output_file_gt_;
    std::ofstream output_file_kf_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorder>());
    rclcpp::shutdown();
    return 0;
}
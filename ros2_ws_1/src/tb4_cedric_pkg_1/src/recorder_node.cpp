#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" // Erforderlich für Paket-Pfad
#include <fstream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

class DataRecorder : public rclcpp::Node {
public:
    DataRecorder() : Node("data_recorder") {
        try {
            // 1. Finde den Pfad zum "share"-Verzeichnis deines Pakets
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("tb4_cedric_pkg_1");
            
            // 2. Navigiere zum trajectories-Ordner
            // Da wir uns im install-Ordner befinden, gehen wir davon aus, dass trajectories dort mit-installiert wurde
            fs::path path = fs::path(pkg_share_dir) / "trajectories" / "robot_path1.csv";
            
            std::string file_path = path.string();
            
            output_file_.open(file_path);
            
            if (output_file_.is_open()) {
                output_file_ << "x,y\n";
                RCLCPP_INFO(this->get_logger(), "CSV wird gespeichert in: %s", file_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Konnte Datei nicht öffnen! Pfad: %s", file_path.c_str());
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Finden des Pakets: %s", e.what());
        }

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DataRecorder::odom_callback, this, std::placeholders::_1));
    }

    ~DataRecorder() {
        if (output_file_.is_open()) {
            output_file_.close();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (output_file_.is_open()) {
            output_file_ << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "\n";
            output_file_.flush(); 
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::ofstream output_file_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorder>());
    rclcpp::shutdown();
    return 0;
}
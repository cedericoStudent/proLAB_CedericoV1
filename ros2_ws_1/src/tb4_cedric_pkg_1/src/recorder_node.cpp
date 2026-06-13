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
        RCLCPP_INFO(this->get_logger(), "Recorder Node für EKF-Evaluierung initialisiert.");
        
        try {
            // Paket-Pfad ermitteln
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("tb4_cedric_pkg_1");
            fs::path dir_path = fs::path(pkg_share_dir) / "trajectories";
            
            // Ordner erstellen, falls er im Share-Verzeichnis noch nicht existiert
            fs::create_directories(dir_path);

            fs::path path_gt = dir_path / "robot_path_GT_EKF.csv";
            fs::path path_ekf = dir_path / "ekf_path1.csv";
            
            // Ground Truth CSV vorbereiten
            output_file_gt_.open(path_gt.string());
            if (output_file_gt_.is_open()) {
                output_file_gt_ << "x,y\n";
                RCLCPP_INFO(this->get_logger(), "Schreibe GT-Daten nach: %s", path_gt.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Konnte GT-Datei nicht öffnen!");
            }

            // EKF CSV vorbereiten (Wichtig: p_xy für die Ellipsendrehung!)
            output_file_ekf_.open(path_ekf.string());
            if (output_file_ekf_.is_open()) {
                output_file_ekf_ << "x,y,p_xx,p_xy,p_yy\n";
                RCLCPP_INFO(this->get_logger(), "Schreibe EKF-Daten nach: %s", path_ekf.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Konnte EKF-Datei nicht öffnen!");
            }

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler bei der Pfad-Initialisierung: %s", e.what());
        }

        // Subscriptions
        // 1. Ground Truth direkt aus der Gazebo-Simulation (/odom)
        subscription_gt_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DataRecorder::odom_callback, this, std::placeholders::_1));

        // 2. Deine EKF-Schätzung (/ekf_estimated_pose)
        subscription_ekf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf_estimated_pose", 10, std::bind(&DataRecorder::ekf_callback, this, std::placeholders::_1));
    }

    ~DataRecorder() {
        if (output_file_gt_.is_open()) output_file_gt_.close();
        if (output_file_ekf_.is_open()) output_file_ekf_.close();
        RCLCPP_INFO(this->get_logger(), "Dateien erfolgreich geschlossen und gespeichert.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (output_file_gt_.is_open()) {
            output_file_gt_ << msg->pose.pose.position.x << "," 
                            << msg->pose.pose.position.y << "\n";
            output_file_gt_.flush(); 
        }
    }

    void ekf_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (output_file_ekf_.is_open()) {
            // ROS covariance ist ein flaches 6x6 Array (36 Elemente).
            // Zeile 0 (X): [0]=P_xx, [1]=P_xy
            // Zeile 1 (Y): [6]=P_yx, [7]=P_yy
            double p_xx = msg->pose.covariance[0];
            double p_xy = msg->pose.covariance[1]; // Entspricht mathematisch P_yx
            double p_yy = msg->pose.covariance[7];

            output_file_ekf_ << msg->pose.pose.position.x << "," 
                             << msg->pose.pose.position.y << ","
                             << p_xx << ","
                             << p_xy << ","
                             << p_yy << "\n";
            output_file_ekf_.flush(); 
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_gt_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_ekf_;
    std::ofstream output_file_gt_;
    std::ofstream output_file_ekf_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorder>());
    rclcpp::shutdown();
    return 0;
}
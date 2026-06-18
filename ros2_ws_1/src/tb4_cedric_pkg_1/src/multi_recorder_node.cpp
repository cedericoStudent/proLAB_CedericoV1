#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

class MultiDataRecorder : public rclcpp::Node {
public:
    MultiDataRecorder() : Node("multi_recorder_node") {
        RCLCPP_INFO(this->get_logger(), "Multi-Recorder initialisiert. Warte auf gefilterte Daten...");
        
        try {
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("tb4_cedric_pkg_1");
            fs::path dir_path = fs::path(pkg_share_dir) / "trajectories";
            fs::create_directories(dir_path);

            file_gt_.open((dir_path / "multi_path_gt.csv").string());
            file_kf_.open((dir_path / "multi_path_kf.csv").string());
            file_ekf_std_.open((dir_path / "multi_path_ekf_std.csv").string());
            file_ekf_lm_.open((dir_path / "multi_path_ekf_lm.csv").string());
            file_pf_.open((dir_path / "multi_path_pf.csv").string());
            
            if (file_gt_.is_open())      file_gt_      << "x,y\n";
            if (file_kf_.is_open())      file_kf_      << "x,y,p_xx,p_xy,p_yy\n";
            if (file_ekf_std_.is_open()) file_ekf_std_ << "x,y,p_xx,p_xy,p_yy\n";
            if (file_ekf_lm_.is_open())  file_ekf_lm_  << "x,y,p_xx,p_xy,p_yy\n";
            if (file_pf_.is_open())      file_pf_      << "x,y,p_xx,p_xy,p_yy\n";
        } 
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Erstellen der CSVs: %s", e.what());
        }

        // Exakte Topic-Zuweisung passend zum Launchfile-Remapping!
        sub_gt_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MultiDataRecorder::gt_cb, this, std::placeholders::_1));
            
        sub_kf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/kf_estimated_pose", 10, std::bind(&MultiDataRecorder::kf_cb, this, std::placeholders::_1));
            
        sub_ekf_std_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf_pose_std", 10, std::bind(&MultiDataRecorder::ekf_std_cb, this, std::placeholders::_1));
            
        sub_ekf_lm_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf_pose_landmark", 10, std::bind(&MultiDataRecorder::ekf_lm_cb, this, std::placeholders::_1));
            
        sub_pf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pf_pose", 10, std::bind(&MultiDataRecorder::pf_cb, this, std::placeholders::_1));
    }

    ~MultiDataRecorder() {
        if (file_gt_.is_open())      file_gt_.close();
        if (file_kf_.is_open())      file_kf_.close();
        if (file_ekf_std_.is_open()) file_ekf_std_.close();
        if (file_ekf_lm_.is_open())  file_ekf_lm_.close();
        if (file_pf_.is_open())      file_pf_.close();
    }

private:
    void gt_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (file_gt_.is_open()) {
            file_gt_ << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "\n";
            file_gt_.flush();
        }
    }

    void write_filter_data(std::ofstream &file, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (file.is_open()) {
            file << msg->pose.pose.position.x << "," 
                 << msg->pose.pose.position.y << ","
                 << msg->pose.covariance[0] << ","
                 << msg->pose.covariance[1] << ","
                 << msg->pose.covariance[7] << "\n";
            file.flush();
        }
    }

    void kf_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)      { write_filter_data(file_kf_, msg); }
    void ekf_std_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { write_filter_data(file_ekf_std_, msg); }
    void ekf_lm_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)  { write_filter_data(file_ekf_lm_, msg); }
    void pf_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)      { write_filter_data(file_pf_, msg); }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_kf_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ekf_std_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ekf_lm_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pf_;

    std::ofstream file_gt_, file_kf_, file_ekf_std_, file_ekf_lm_, file_pf_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiDataRecorder>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <string>

class DataRecorder : public rclcpp::Node {
public:
    DataRecorder() : Node("data_recorder") {
        // Relativer Pfad zu deinem existierenden Ordner
        // WICHTIG: Starten aus ~/Desktop/PROLAB/proLAB_CedericoV1/
        std::string file_path = "trajectories/robot_path.csv";
        
        output_file_.open(file_path);
        
        if (output_file_.is_open()) {
            output_file_ << "x,y\n";
            RCLCPP_INFO(this->get_logger(), "Datei erfolgreich geöffnet: %s", file_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "FEHLER: Datei konnte nicht geöffnet werden!");
            RCLCPP_ERROR(this->get_logger(), "Prüfe, ob du im Root-Verzeichnis deines Repos bist.");
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
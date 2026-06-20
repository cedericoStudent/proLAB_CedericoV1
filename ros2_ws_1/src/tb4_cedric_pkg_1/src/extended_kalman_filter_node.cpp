#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono_literals;

class ExtendedKalmanFilterNode : public rclcpp::Node {
public:
    ExtendedKalmanFilterNode() : Node("extended_kalman_filter_node") {
        RCLCPP_INFO(this->get_logger(), "EKF Node aktiv. Prädiktion über Rad-Geschwindigkeiten, Korrektur über IMU.");

        // Parameter deklarieren
        this->declare_parameter<double>("q_var_pos", 0.02);    
        this->declare_parameter<double>("q_var_theta", 0.01);
        this->declare_parameter<double>("r_var_imu", 0.05);

        q_pos_ = this->get_parameter("q_var_pos").as_double();
        q_theta_ = this->get_parameter("q_var_theta").as_double();
        r_var_imu_ = this->get_parameter("r_var_imu").as_double();

        // Zustand: [x, y, theta]
        x_hat_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Matrix3d::Identity() * 0.1;

        // Subscriber für Prädiktions-Input (Lokale Rad-Geschwindigkeiten)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
                u_input_(0) = msg->twist.twist.linear.x;   // v (lokal)
                u_input_(1) = msg->twist.twist.angular.z;  // omega
            });

        // Subscriber für IMU-Korrektur (Unabhängige Drehrate)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
                z_omega_ = msg->angular_velocity.z;    
                fresh_imu_ = true;
            });

        filter_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_estimated_pose", 10);
        last_time_ = this->get_clock()->now();
        filter_timer_ = this->create_wall_timer(20ms, std::bind(&ExtendedKalmanFilterNode::filter_loop, this));
    }

private:
    void filter_loop() {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) return;

        double theta = x_hat_(2);
        double v = u_input_(0);
        double omega = u_input_(1);

        // =====================================================================
        // 1. PRÄDIKTION (Nichtlineares Koppelmodell)
        // =====================================================================
        x_hat_(0) += v * std::cos(theta) * dt; 
        x_hat_(1) += v * std::sin(theta) * dt; 
        x_hat_(2) += omega * dt;               
        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // Jacobimatrix F_x
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -v * std::sin(theta) * dt;
        F(1, 2) =  v * std::cos(theta) * dt;

        // Prozessrauschen Q im globalen Frame rotieren
        Eigen::Matrix3d Q_local = Eigen::Matrix3d::Zero();
        Q_local(0,0) = q_pos_;        
        Q_local(1,1) = q_pos_ * 0.1;  
        Q_local(2,2) = q_theta_;      

        Eigen::Matrix3d R_track = Eigen::Matrix3d::Identity();
        R_track(0,0) = std::cos(theta); R_track(0,1) = -std::sin(theta);
        R_track(1,0) = std::sin(theta); R_track(1,1) =  std::cos(theta);

        Eigen::Matrix3d Q_global = R_track * Q_local * R_track.transpose();
        P_ = F * P_ * F.transpose() + Q_global;

        // =====================================================================
        // 2. KORREKTUR (Echte Drehraten-Fusion über IMU)
        // =====================================================================
        if (fresh_imu_) {
            Eigen::Matrix<double, 1, 3> H_imu = Eigen::Matrix<double, 1, 3>::Zero();
            H_imu(0, 2) = 1.0; 

            // Berechneter Erwartungswert für die Orientierungsänderung
            double z_theta = x_hat_(2) + (z_omega_ - omega) * dt; 
            double y_residual = z_theta - x_hat_(2);
            y_residual = std::atan2(std::sin(y_residual), std::cos(y_residual));

            double S = P_(2,2) + r_var_imu_;
            Eigen::Vector3d K = P_.col(2) / S;

            x_hat_ = x_hat_ + K * y_residual;
            P_ = (Eigen::Matrix3d::Identity() - K * H_imu) * P_;
            
            fresh_imu_ = false;
        }

        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // =====================================================================
        // 3. PUBLISH
        // =====================================================================
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = x_hat_(0);
        pose_msg.pose.pose.position.y = x_hat_(1);
        pose_msg.pose.pose.orientation.z = std::sin(x_hat_(2) / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(x_hat_(2) / 2.0);

        pose_msg.pose.covariance[0] = P_(0,0); 
        pose_msg.pose.covariance[1] = P_(0,1); 
        pose_msg.pose.covariance[6] = P_(1,0); 
        pose_msg.pose.covariance[7] = P_(1,1); 

        filter_pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub_;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    Eigen::Vector3d x_hat_;
    Eigen::Matrix3d P_;
    
    double q_pos_;
    double q_theta_;
    double r_var_imu_;

    Eigen::Vector2d u_input_ = Eigen::Vector2d::Zero(); 
    double z_omega_ = 0.0;   

    bool fresh_imu_ = false;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
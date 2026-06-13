#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono_literals;

class ExtendedKalmanFilterNode : public rclcpp::Node {
public:
    ExtendedKalmanFilterNode() : Node("extended_kalman_filter_node") {
        RCLCPP_INFO(this->get_logger(), "EKF Node mit nichtlinearem Motionmodell gestartet.");

        this->declare_parameter<double>("q_var_pos", 0.05);    // Etwas erhöht für sichtbare Effekte
        this->declare_parameter<double>("q_var_theta", 0.02);
        this->declare_parameter<double>("r_var_odom", 0.08);   // Erhöht, damit der Filter der Prädiktion mehr vertraut
        this->declare_parameter<double>("r_var_imu", 0.05);

        q_pos_ = this->get_parameter("q_var_pos").as_double();
        q_theta_ = this->get_parameter("q_var_theta").as_double();
        double r_odom = this->get_parameter("r_var_odom").as_double();
        double r_imu = this->get_parameter("r_var_imu").as_double();

        // Zustand: [x, y, theta]
        x_hat_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Matrix3d::Identity() * 0.1;

        // Messrauschen R
        R_odom_ = Eigen::Matrix2d::Identity() * r_odom; 
        R_imu_ = Eigen::Matrix2d::Identity() * r_imu;   

        // Subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
                u_input_(0) = msg->linear.x;  
                u_input_(1) = msg->angular.z; 
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
                z_odom_(0) = msg->pose.pose.position.x;
                z_odom_(1) = msg->pose.pose.position.y;
                fresh_odom_ = true;
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
                z_imu_(0) = msg->angular_velocity.z;    
                z_imu_(1) = msg->linear_acceleration.x; 
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
        // 1. PRÄDIKTION (Nichtlineares Modell)
        // =====================================================================
        x_hat_(0) += v * std::cos(theta) * dt; 
        x_hat_(1) += v * std::sin(theta) * dt; 
        x_hat_(2) += omega * dt;               

        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // Jacobi-Matrix F (Linearisierung nach Zustand)
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -v * std::sin(theta) * dt;
        F(1, 2) =  v * std::cos(theta) * dt;

        // --- DYNAMISCHES PROZESSRAUSCHEN Q (Der Gamechanger!) ---
        // Wir verrauschen Translation und Rotation im lokalen Roboter-Frame
        // Lokales Rauschen: Viel Rauschen in Fahrtrichtung, weniger quer dazu.
        Eigen::Matrix3d Q_local = Eigen::Matrix3d::Zero();
        Q_local(0,0) = q_pos_;        // Rauschen entlang der Fahrtrichtung (V)
        Q_local(1,1) = q_pos_ * 0.1;  // Kaum Rauschen quer zur Fahrtrichtung
        Q_local(2,2) = q_theta_;      // Rotationsrauschen

        // Rotationsmatrix für den Zustand aufbauen
        Eigen::Matrix3d R_track = Eigen::Matrix3d::Identity();
        R_track(0,0) = std::cos(theta); R_track(0,1) = -std::sin(theta);
        R_track(1,0) = std::sin(theta); R_track(1,1) =  std::cos(theta);

        // Transformiere das lokale Rauschen in den globalen Welt-Frame!
        Eigen::Matrix3d Q_global = R_track * Q_local * R_track.transpose();

        // Kovarianz prädizieren mit dem mitdrehenden Q
        P_ = F * P_ * F.transpose() + Q_global;

        // =====================================================================
        // 2. KORREKTUR ODOMETRIE (x, y)
        // =====================================================================
        if (fresh_odom_) {
            Eigen::Matrix<double, 2, 3> H_odom = Eigen::Matrix<double, 2, 3>::Zero();
            H_odom(0, 0) = 1.0;
            H_odom(1, 1) = 1.0;

            Eigen::Vector2d y_residual = z_odom_ - H_odom * x_hat_;
            Eigen::Matrix2d S = H_odom * P_ * H_odom.transpose() + R_odom_;
            Eigen::Matrix<double, 3, 2> K = P_ * H_odom.transpose() * S.inverse();

            x_hat_ = x_hat_ + K * y_residual;
            P_ = (Eigen::Matrix3d::Identity() - K * H_odom) * P_;
            fresh_odom_ = false;
        }

        // =====================================================================
        // 3. KORREKTUR IMU (Kopplung auf Zustand Theta)
        // =====================================================================
        if (fresh_imu_) {
            double z_omega = z_imu_(0); 
            
            Eigen::Matrix<double, 1, 3> H_imu = Eigen::Matrix<double, 1, 3>::Zero();
            H_imu(0, 2) = 1.0; 

            double z_theta = x_hat_(2) + (z_omega - omega) * dt; 
            double y_residual = z_theta - x_hat_(2);
            y_residual = std::atan2(std::sin(y_residual), std::cos(y_residual));

            double S = P_(2,2) + R_imu_(0,0);
            Eigen::Vector3d K = P_.col(2) / S;

            x_hat_ = x_hat_ + K * y_residual;
            P_ = (Eigen::Matrix3d::Identity() - K * H_imu) * P_;
            
            fresh_imu_ = false;
        }

        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // =====================================================================
        // 4. PUBLISH
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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub_;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    Eigen::Vector3d x_hat_;
    Eigen::Matrix3d P_;
    Eigen::Matrix2d R_odom_;
    Eigen::Matrix2d R_imu_;
    
    double q_pos_;
    double q_theta_;

    Eigen::Vector2d u_input_ = Eigen::Vector2d::Zero(); 
    Eigen::Vector2d z_odom_ = Eigen::Vector2d::Zero();  
    Eigen::Vector2d z_imu_ = Eigen::Vector2d::Zero();   

    bool fresh_odom_ = false;
    bool fresh_imu_ = false;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
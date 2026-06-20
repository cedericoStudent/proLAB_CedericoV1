#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono_literals;

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kalman_filter_node") {
        RCLCPP_INFO(this->get_logger(), "Lineares KF gestartet. Korrektur erfolgt ausschließlich über globale Geschwindigkeiten.");

        // Parameter laden
        this->declare_parameter<double>("q_var_pos", 0.005);
        this->declare_parameter<double>("q_var_vel", 0.05);
        this->declare_parameter<double>("r_var_vel", 0.25);

        double q_var_pos = this->get_parameter("q_var_pos").as_double();
        double q_var_vel = this->get_parameter("q_var_vel").as_double();
        double r_var_vel = this->get_parameter("r_var_vel").as_double();

        // Zustand: [x, y, vx, vy]^T
        x_hat_ = Eigen::Vector4d::Zero();
        P_ = Eigen::Matrix4d::Identity() * 0.1;

        // Prozesstoleranz Q
        Q_ = Eigen::Matrix4d::Zero();
        Q_(0,0) = q_var_pos; Q_(1,1) = q_var_pos;
        Q_(2,2) = q_var_vel; Q_(3,3) = q_var_vel;

        // Messmatrix H: Wir messen nur vx (Index 2) und vy (Index 3)
        H_ = Eigen::Matrix<double, 2, 4>::Zero();
        H_(0, 2) = 1.0; 
        H_(1, 3) = 1.0;

        // Messrauschen R (2x2 für vx und vy)
        R_ = Eigen::Matrix2d::Identity() * r_var_vel;

        // Subscription: Liest nur Geschwindigkeiten im globalen Frame
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
                z_meas_(0) = msg->twist.twist.linear.x; // globale Geschwindigkeit v_x
                z_meas_(1) = msg->twist.twist.linear.y; // globale Geschwindigkeit v_y
                fresh_measurement_ = true;
            });

        filter_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_estimated_pose", 10);
        last_time_ = this->get_clock()->now();
        filter_timer_ = this->create_wall_timer(20ms, std::bind(&KalmanFilterNode::filter_loop, this));
    }

private:
    void filter_loop() {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) return;

        // Systemmatrix A dynamisch aufbauen
        Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
        A(0, 2) = dt;
        A(1, 3) = dt;

        // --- PRÄDIKTION ---
        x_hat_ = A * x_hat_;
        P_ = A * P_ * A.transpose() + Q_;

        // --- KORREKTUR ---
        if (fresh_measurement_) {
            Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
            Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
            
            x_hat_ = x_hat_ + K * (z_meas_ - H_ * x_hat_);
            P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
            
            fresh_measurement_ = false;
        }

        // --- PUBLISH ---
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = x_hat_(0);
        pose_msg.pose.pose.position.y = x_hat_(1);
        pose_msg.pose.pose.orientation.w = 1.0; 

        pose_msg.pose.covariance[0] = P_(0,0); 
        pose_msg.pose.covariance[1] = P_(0,1); 
        pose_msg.pose.covariance[7] = P_(1,1); 

        filter_pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub_;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    Eigen::Vector4d x_hat_;
    Eigen::Matrix4d P_;
    Eigen::Matrix4d Q_;
    Eigen::Matrix<double, 2, 4> H_;
    Eigen::Matrix2d R_;
    Eigen::Vector2d z_meas_ = Eigen::Vector2d::Zero();

    bool fresh_measurement_ = false;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
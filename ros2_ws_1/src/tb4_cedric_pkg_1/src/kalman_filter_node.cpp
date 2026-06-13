#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kalman_filter_node");

    RCLCPP_INFO(node->get_logger(), "Lineares KF mit externer u-Transformation gestartet.");

    // Parameter für Pflicht-Experimente
    node->declare_parameter<double>("q_var_pos", 0.002);
    node->declare_parameter<double>("q_var_vel", 0.02);
    node->declare_parameter<double>("r_var_pos", 0.08);
    node->declare_parameter<double>("r_var_vel", 0.15);

    double q_var_pos, q_var_vel, r_var_pos, r_var_vel;
    node->get_parameter("q_var_pos", q_var_pos);
    node->get_parameter("q_var_vel", q_var_vel);
    node->get_parameter("r_var_pos", r_var_pos);
    node->get_parameter("r_var_vel", r_var_vel);

    // KF-Matrizen Setup [x, y, vx, vy]
    Eigen::Vector4d x_hat = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity() * 1.0;
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    
    // Steuerungsmatrix B (wird im Timer mit dt befüllt)
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0,0) = q_var_pos; Q(1,1) = q_var_pos; Q(2,2) = q_var_vel; Q(3,3) = q_var_vel;

    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    R(0,0) = r_var_pos; R(1,1) = r_var_pos; R(2,2) = r_var_vel; R(3,3) = r_var_vel;

    // Variablen für die Transformation außerhalb des Filters
    double v_teleop = 0.0;
    double theta_ground_truth = 0.0;
    Eigen::Vector2d u_input = Eigen::Vector2d::Zero(); // [u_x, u_y]^T

    // Publisher für geschätzte Pose
    auto filter_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/kf_estimated_pose", 10);

    // 1. Subscriber: Verrauschte Odometrie für die KORREKTUR (Measurement)
    Eigen::Vector4d z_meas = Eigen::Vector4d::Zero();
    bool fresh_measurement = false;
    auto odom_noisy_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_noisy", 10,
        [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            z_meas(0) = msg->pose.pose.position.x;
            z_meas(1) = msg->pose.pose.position.y;
            z_meas(2) = msg->twist.twist.linear.x; 
            z_meas(3) = msg->twist.twist.linear.y; 
            fresh_measurement = true;
        }
    );

    // 2. Subscriber: Teleop-Befehle abgreifen
    auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            v_teleop = msg->linear.x; // Vorwärtsgeschwindigkeit im Roboterkoordinatensystem
        }
    );

    // 3. Subscriber: Echte Odometrie (Ground-Truth) NUR für den Orientierungswinkel theta
    auto ground_truth_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            // Quaternion in Yaw-Winkel (Theta) umrechnen
            double qz = msg->pose.pose.orientation.z;
            double qw = msg->pose.pose.orientation.w;
            theta_ground_truth = 2.0 * std::atan2(qz, qw);
        }
    );

    auto last_time = node->get_clock()->now();

    // Timer-Schleife mit 50 Hz
    auto filter_timer = node->create_wall_timer(20ms, [&]() {
        auto current_time = node->get_clock()->now();
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        if (dt <= 0.0) return;

        // --- EXTERNE TRANSFORMATION (Dein Lösungsansatz) ---
        // Wir rechnen den Roboter-Befehl mithilfe der Ground-Truth-Orientierung in Weltkoordinaten um
        u_input(0) = v_teleop * std::cos(theta_ground_truth); // u_x
        u_input(1) = v_teleop * std::sin(theta_ground_truth); // u_y

        // --- PREDICTION STEP (Mit u-Input) ---
        // Matrizen mit dt updaten
        A(0, 2) = dt; 
        A(1, 3) = dt;
        
        B(0, 0) = dt;  B(1, 1) = dt;
        B(2, 0) = 1.0; B(3, 1) = 1.0;

        // Zustand prädizieren: x_k = A * x_{k-1} + B * u_k
        x_hat = A * x_hat + B * u_input;
        
        // Kovarianz prädizieren
        P = A * P * A.transpose() + Q;

        // --- CORRECTION STEP ---
        if (fresh_measurement) {
            Eigen::Matrix4d S = H * P * H.transpose() + R;
            Eigen::Matrix4d K = P * H.transpose() * S.inverse();
            x_hat = x_hat + K * (z_meas - H * x_hat);
            P = (Eigen::Matrix4d::Identity() - K * H) * P;
            fresh_measurement = false;
        }

        // --- PUBLISH ---
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.position.x = x_hat(0);
        pose_msg.pose.position.y = x_hat(1);
        filter_pose_pub->publish(pose_msg);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
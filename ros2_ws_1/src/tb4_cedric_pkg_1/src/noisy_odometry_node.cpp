#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("noisy_odometry_node");

    RCLCPP_INFO(node->get_logger(), "Noisy Odometry Node (Kinematik) gestartet.");

    // TurtleBot3 Waffle Geometrie-Konstanten
    const double WHEEL_RADIUS = 0.033; // r in Metern
    const double WHEEL_SEPARATION = 0.287; // d in Metern

    // Interne Odometrie-Zustände (Integration)
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_theta = 0.0;

    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom_noisy", 10);

    auto last_time = node->get_clock()->now();

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_noisy", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            auto current_time = node->get_clock()->now();
            double dt = (current_time - last_time).seconds();
            last_time = current_time;

            if (dt <= 0.0) return;

            // Indizes für linkes und rechtes Rad finden
            double wheel_vel_left = 0.0;
            double wheel_vel_right = 0.0;

            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "wheel_left_joint") wheel_vel_left = msg->velocity[i];
                if (msg->name[i] == "wheel_right_joint") wheel_vel_right = msg->velocity[i];
            }

            // --- VORWÄRTSKINEMATIK (Differentialantrieb) ---
            // 1. Lineare und rotatorische Geschwindigkeit im Roboterkoordinatensystem
            double v_robot = (WHEEL_RADIUS / 2.0) * (wheel_vel_right + wheel_vel_left);
            double omega_robot = (WHEEL_RADIUS / WHEEL_SEPARATION) * (wheel_vel_right - wheel_vel_left);

            // 2. Odometrie-Integration (Euler-Int)
            odom_x += v_robot * std::cos(odom_theta) * dt;
            odom_y += v_robot * std::sin(odom_theta) * dt;
            odom_theta += omega_robot * dt;

            // Winkel auf [-PI, PI] normieren
            odom_theta = std::atan2(std::sin(odom_theta), std::cos(odom_theta));

            // 3. Transformation der Lineargeschwindigkeit in das GLOBALE Koordinatensystem
            // Das ist genau das, was das KF als Messung benötigt!
            double v_x_global = v_robot * std::cos(odom_theta);
            double v_y_global = v_robot * std::sin(odom_theta);

            // --- ODOMETRIE NACHRICHT BAUEN & PUBLIZIEREN ---
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_footprint";

            // Position
            odom_msg.pose.pose.position.x = odom_x;
            odom_msg.pose.pose.position.y = odom_y;

            // Orientierung (Z-Achsen-Quaternion aus Theta)
            odom_msg.pose.pose.orientation.z = std::sin(odom_theta / 2.0);
            odom_msg.pose.pose.orientation.w = std::cos(odom_theta / 2.0);

            // Globale Geschwindigkeiten packen wir in den Twist-Vektor für das KF
            odom_msg.twist.twist.linear.x = v_x_global;
            odom_msg.twist.twist.linear.y = v_y_global;
            odom_msg.twist.twist.angular.z = omega_robot;

            odom_pub->publish(odom_msg);
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
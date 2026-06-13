#include <memory>
#include <string>
#include <random>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Node erstellen
    auto node = rclcpp::Node::make_shared("joint_noise_node");

    // Parameter deklarieren und auslesen
    node->declare_parameter<double>("noise_std_dev_vel", 0.08);
    node->declare_parameter<double>("noise_std_dev_pos", 0.02);

    double std_dev_vel = 0.08;
    double std_dev_pos = 0.02;
    node->get_parameter("noise_std_dev_vel", std_dev_vel);
    node->get_parameter("noise_std_dev_pos", std_dev_pos);

    RCLCPP_INFO(node->get_logger(), "Joint Noise Node (Prozedural) gestartet.");
    RCLCPP_INFO(node->get_logger(), "Konfiguriertes Rauschen -> Vel: %.4f, Pos: %.4f", std_dev_vel, std_dev_pos);

    // Zufallszahlengenerator aufsetzen
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist_vel(0.0, std_dev_vel);
    std::normal_distribution<double> dist_pos(0.0, std_dev_pos);

    // Publisher erstellen
    auto noisy_joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states_noisy", 10);

    // Subscriber mit Lambda-Funktion (umgeht std::bind und placeholders Konflikte komplett)
    auto raw_joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 
        10, 
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Nachricht kopieren
            auto noisy_msg = *msg;

            // Rauschen auf Geschwindigkeiten addieren
            for (size_t i = 0; i < noisy_msg.velocity.size(); ++i) {
                noisy_msg.velocity[i] += dist_vel(gen);
            }

            // Rauschen auf Positionen addieren
            for (size_t i = 0; i < noisy_msg.position.size(); ++i) {
                noisy_msg.position[i] += dist_pos(gen);
            }

            // Zeitstempel aktualisieren
            noisy_msg.header.stamp = node->get_clock()->now();

            // Verrauschte Daten publizieren
            noisy_joint_pub->publish(noisy_msg);
        }
    );

    // In der Schleife laufen lassen, bis ROS beendet wird
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
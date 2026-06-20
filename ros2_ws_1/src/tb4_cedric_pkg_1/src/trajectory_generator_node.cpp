#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

enum class DriveState {
    START_FORWARD,          // 1. 0.5m geradeaus
    RIGHT_TURN_90,          // 2. 90° Rechtskurve (Bogen 1m)
    LEFT_TURN_90_SHORT,     // 3. 90° Linkskurve + 30cm Fahrt
    LEFT_TURN_90_LONG,      // 4. 90° Linkskurve + 1.5m Fahrt
    LEFT_TURN_90_RETURN,    // 5. 90° Linkskurve + 1m Fahrt
    LEFT_DIAGONAL_45_A,     // 6. 45° Linkskurve + 1m Fahrt
    LEFT_DIAGONAL_45_B,     // 7. Erneut 45° Linkskurve + 1m Fahrt
    NOCHMAL_DREHEN,        // 8. Zum Landmark nochmal finden
    STOP                    // 9. Ziel erreicht
};

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator_node"), current_state_(DriveState::START_FORWARD) {
        RCLCPP_INFO(this->get_logger(), "=== Präziser Trajectory Generator (Geometrie-Setup) gestartet ===");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryGeneratorNode::timer_callback, this));
        state_start_time_ = this->get_clock()->now();
    }

private:
    void timer_callback() {
        auto now = this->get_clock()->now();
        double elapsed_time = (now - state_start_time_).seconds();

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = 0.0; cmd_msg.angular.z = 0.0;

        switch (current_state_) {
            case DriveState::START_FORWARD:
                // 0.5m geradeaus bei 0.25 m/s -> 2 Sekunden
                cmd_msg.linear.x = 0.15;
                if (elapsed_time >= 1.0) switch_state(DriveState::RIGHT_TURN_90);
                break;

            case DriveState::RIGHT_TURN_90:
                // 90° Rechtskurve aus der Fahrt (Bogen 1m) -> 4 Sekunden
                cmd_msg.linear.x = 0.35;
                cmd_msg.angular.z = -0.593; // Berechnetes Omega für exakten 1m-Bogen
                if (elapsed_time >= 5.0) switch_state(DriveState::LEFT_TURN_90_SHORT);
                break;

            case DriveState::LEFT_TURN_90_SHORT:
                // 90° Linkskurve + 30cm Fahrt
                if (elapsed_time < 3.93) {
                    cmd_msg.angular.z = 0.65;
                } else {
                    cmd_msg.linear.x = 0.25;
                    cmd_msg.angular.z = 0.3;
                }
                if (elapsed_time >= 8.0) switch_state(DriveState::LEFT_TURN_90_LONG);
                break;

            case DriveState::LEFT_TURN_90_LONG:
                // 90° Linkskurve + 1.5m Fahrt
                if (elapsed_time < 4.0) {
                    cmd_msg.angular.z = 0.45;
                } else {
                    cmd_msg.linear.x = 0.27;
                    cmd_msg.angular.z = -0.05;
                }
                if (elapsed_time >= 14.0) switch_state(DriveState::LEFT_TURN_90_RETURN);
                break;

            case DriveState::LEFT_TURN_90_RETURN:
                // 90° Linkskurve + 1m Fahrt
                if (elapsed_time < 4.93) {
                    cmd_msg.angular.z = 0.3;
                } else {
                    cmd_msg.linear.x = 0.35;
                }
                if (elapsed_time >= 8.0) switch_state(DriveState::LEFT_DIAGONAL_45_A);
                break;

            case DriveState::LEFT_DIAGONAL_45_A:
                // 45° Linkskurve + 1m Fahrt
                if (elapsed_time < 2.96) {
                    cmd_msg.angular.z = 0.4;
                } else if (elapsed_time < 7.0) {
                    cmd_msg.linear.x = 0.25;
                    cmd_msg.angular.z = 0.2;
                }
                else {
                    cmd_msg.linear.x = 0.25;
                    cmd_msg.angular.z = -0.1;
                }
                if (elapsed_time >= 11.0) switch_state(DriveState::LEFT_DIAGONAL_45_B);
                break;

            case DriveState::LEFT_DIAGONAL_45_B:
                // Erneut 45° Linkskurve + 1m Fahrt
                if (elapsed_time < 1.96) {
                    cmd_msg.angular.z = 0.1;
                } else {
                    cmd_msg.linear.x = 0.15;
                    cmd_msg.angular.z = 0.22;
                }
                if (elapsed_time >= 7.0) switch_state(DriveState::NOCHMAL_DREHEN);
                break;

            case DriveState::NOCHMAL_DREHEN:
                // Zum Landmark nochmal drehen
                cmd_msg.angular.z = 0.5;
                if (elapsed_time >= 4.0) switch_state(DriveState::STOP);
                break;

            case DriveState::STOP:
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                RCLCPP_INFO_ONCE(this->get_logger(), "🏁 Exakter Geometrie-Pfad abgefahren! Roboter steht.");
                break;
        }

        cmd_vel_pub_->publish(cmd_msg);
    }

    void switch_state(DriveState new_state) {
        current_state_ = new_state;
        state_start_time_ = this->get_clock()->now();
        
        std::string state_name;
        switch (new_state) {
            case DriveState::START_FORWARD:       state_name = "START_FORWARD (0.5m)"; break;
            case DriveState::RIGHT_TURN_90:       state_name = "RIGHT_TURN_90 (Bogen 1m)"; break;
            case DriveState::LEFT_TURN_90_SHORT:  state_name = "LEFT_TURN_90_SHORT (0.3m)"; break;
            case DriveState::LEFT_TURN_90_LONG:   state_name = "LEFT_TURN_90_LONG (1.5m)"; break;
            case DriveState::LEFT_TURN_90_RETURN: state_name = "LEFT_TURN_90_RETURN (1m)"; break;
            case DriveState::LEFT_DIAGONAL_45_A:  state_name = "LEFT_DIAGONAL_45_A (45° + 1m)"; break;
            case DriveState::LEFT_DIAGONAL_45_B:  state_name = "LEFT_DIAGONAL_45_B (45° + 1m)"; break;
            case DriveState::NOCHMAL_DREHEN:      state_name = "NOCHMAL_DREHEN"; break;
            case DriveState::STOP:                state_name = "STOP"; break;
        }
        RCLCPP_INFO(this->get_logger(), "Wechsle in Phase: %s", state_name.c_str());
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    DriveState current_state_;
    rclcpp::Time state_start_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
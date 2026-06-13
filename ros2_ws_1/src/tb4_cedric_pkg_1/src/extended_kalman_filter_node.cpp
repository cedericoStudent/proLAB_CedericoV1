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
        RCLCPP_INFO(this->get_logger(), "EKF Node (7D-Zustand mit IMU) initialisiert.");

        // Parameter (Richtwerte, im Pflichtexperiment anzupassen)
        this->declare_parameter<double>("q_var", 0.01);
        this->declare_parameter<double>("r_var_odom", 0.05);
        this->declare_parameter<double>("r_var_imu", 0.1);

        double q_var = this->get_parameter("q_var").as_double();
        double r_odom = this->get_parameter("r_var_odom").as_double();
        double r_imu = this->get_parameter("r_var_imu").as_double();

        // 1. Systemzustand initialisieren: [x, y, theta, vx, vy, ax, ay]
        x_hat_ = Eigen::Matrix<double, 7, 1>::Zero();
        P_ = Eigen::Matrix<double, 7, 7>::Identity() * 0.5;

        // 2. Prozessrauschen Q
        Q_ = Eigen::Matrix<double, 7, 7>::Identity() * q_var;

        // 3. Messrauschen R_odom (4 Messwerte: x, y, vx, vy)
        R_odom_ = Eigen::Matrix4d::Identity() * r_odom;

        // 4. Messrauschen R_imu (3 Messwerte: d_theta, ax_robot, ay_robot)
        R_imu_ = Eigen::Matrix3d::Identity() * r_imu;

        // Subscriber
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
                u_input_(0) = msg->linear.x;  // v_cmd im Roboterkoordinatensystem
                u_input_(1) = msg->angular.z; // omega_cmd im Roboterkoordinatensystem
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, std::bind(&ExtendedKalmanFilterNode::odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ExtendedKalmanFilterNode::imu_callback, this, std::placeholders::_1));

        // Publisher für den Recorder und RViz
        filter_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_estimated_pose", 10);

        last_time_ = this->get_clock()->now();
        
        // Filter-Timer mit 50 Hz
        filter_timer_ = this->create_wall_timer(20ms, std::bind(&ExtendedKalmanFilterNode::filter_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        z_odom_(0) = msg->pose.pose.position.x;
        z_odom_(1) = msg->pose.pose.position.y;
        z_odom_(2) = msg->twist.twist.linear.x; // vx global aus Kinematik-Node
        z_odom_(3) = msg->twist.twist.linear.y; // vy global aus Kinematik-Node
        fresh_odom_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        z_imu_(0) = msg->angular_velocity.z;    // omega aus IMU
        z_imu_(1) = msg->linear_acceleration.x; // ax im Roboterkoordinatensystem
        z_imu_(2) = msg->linear_acceleration.y; // ay im Roboterkoordinatensystem
        fresh_imu_ = true;
    }

    void filter_loop() {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) return;

        // =====================================================================
        // 1. PRÄDIKTION (Nichtlineare Systemfunktion f(x, u))
        // =====================================================================
        double theta = x_hat_(2);
        double vx = x_hat_(3);
        double vy = x_hat_(4);
        double ax = x_hat_(5);
        double ay = x_hat_(6);

        // Zustand blind mathematisch weiterschreiben (Integration)
        x_hat_(0) += vx * dt + 0.5 * ax * dt * dt; // x
        x_hat_(1) += vy * dt + 0.5 * ay * dt * dt; // y
        x_hat_(2) += u_input_(1) * dt;             // theta integriert omega_cmd
        x_hat_(3) += ax * dt;                      // vx
        x_hat_(4) += ay * dt;                      // vy
        // Beschleunigungen ax, ay bleiben im Prädiktionsschritt konstant

        // Winkel normieren auf [-PI, PI]
        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // --- Jacobi-Matrix F (Linearisierung der Systemdynamik) ---
        Eigen::Matrix<double, 7, 7> F = Eigen::Matrix<double, 7, 7>::Identity();
        F(0, 3) = dt; F(0, 5) = 0.5 * dt * dt;
        F(1, 4) = dt; F(1, 5) = 0.5 * dt * dt;
        F(3, 5) = dt;
        F(4, 6) = dt;

        // Kovarianz prädizieren
        P_ = F * P_ * F.transpose() + Q_;

        // =====================================================================
        // 2. KORREKTUR ODOMETRIE
        // =====================================================================
        if (fresh_odom_) {
            // Da odom_noisy bereits globale Werte liefert, ist H_odom linear!
            Eigen::Matrix<double, 4, 7> H_odom = Eigen::Matrix<double, 4, 7>::Zero();
            H_odom(0, 0) = 1.0; // misst x
            H_odom(1, 1) = 1.0; // misst y
            H_odom(2, 3) = 1.0; // misst vx
            H_odom(3, 4) = 1.0; // misst vy

            Eigen::Vector4d y_residual = z_odom_ - H_odom * x_hat_;
            Eigen::Matrix4d S = H_odom * P_ * H_odom.transpose() + R_odom_;
            Eigen::Matrix<double, 7, 4> K = P_ * H_odom.transpose() * S.inverse();

            x_hat_ = x_hat_ + K * y_residual;
            P_ = (Eigen::Matrix<double, 7, 7>::Identity() - K * H_odom) * P_;
            fresh_odom_ = false;
        }

        // =====================================================================
        // 3. KORREKTUR IMU (Nichtlineare Messfunktion h(x))
        // =====================================================================
        if (fresh_imu_) {
            theta = x_hat_(2); // Aktuellen geschätzten Winkel holen
            ax = x_hat_(5);
            ay = x_hat_(6);

            // Erwartete IMU-Messung laut unserem globalen Zustand generieren
            // Die IMU misst die globalen Beschleunigungen zurück-rotiert in das Roboterkoordinatensystem!
            Eigen::Vector3d h_imu;
            h_imu(0) = u_input_(1); // Erwartete Drehrate entspricht cmd_omega
            h_imu(1) =  ax * std::cos(theta) + ay * std::sin(theta); // ax_robot
            h_imu(2) = -ax * std::sin(theta) + ay * std::cos(theta); // ay_robot

            // --- Jacobi-Matrix H_imu (Linearisierung der IMU-Messung nach dem Zustand) ---
            Eigen::Matrix<double, 3, 7> H_imu = Eigen::Matrix<double, 3, 7>::Zero();
            // Ableitungen nach theta (Zustandsindex 2)
            H_imu(1, 2) = -ax * std::sin(theta) + ay * std::cos(theta);
            H_imu(2, 2) = -ax * std::cos(theta) - ay * std::sin(theta);
            // Ableitungen nach globalen Beschleunigungen ax, ay (Zustandsindex 5 und 6)
            H_imu(1, 5) = std::cos(theta);  H_imu(1, 6) = std::sin(theta);
            H_imu(2, 5) = -std::sin(theta); H_imu(2, 6) = std::cos(theta);

            Eigen::Vector3d y_residual = z_imu_ - h_imu;
            Eigen::Matrix3d S = H_imu * P_ * H_imu.transpose() + R_imu_;
            Eigen::Matrix<double, 7, 3> K = P_ * H_imu.transpose() * S.inverse();

            x_hat_ = x_hat_ + K * y_residual;
            P_ = (Eigen::Matrix<double, 7, 7>::Identity() - K * H_imu) * P_;
            fresh_imu_ = false;
        }

        // Winkel nach Korrekturen erneut normieren
        x_hat_(2) = std::atan2(std::sin(x_hat_(2)), std::cos(x_hat_(2)));

        // =====================================================================
        // 4. PUBLISH GESCHÄTZTE POSE & KOVARIANZ
        // =====================================================================
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = x_hat_(0);
        pose_msg.pose.pose.position.y = x_hat_(1);
        
        // Euler-Winkel Theta in Z-Quaternion wandeln
        pose_msg.pose.pose.orientation.z = std::sin(x_hat_(2) / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(x_hat_(2) / 2.0);

        // Kovarianz übergeben (für den Ellipsenplotter)
        pose_msg.pose.covariance[0] = P_(0,0); // P_xx
        pose_msg.pose.covariance[1] = P_(0,1); // P_xy
        pose_msg.pose.covariance[6] = P_(1,0); // P_yx
        pose_msg.pose.covariance[7] = P_(1,1); // P_yy

        filter_pose_pub_->publish(pose_msg);
    }

    // ROS-Infrastruktur
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filter_pose_pub_;
    rclcpp::TimerBase::SharedPtr filter_timer_;

    // EKF Variablen
    Eigen::Matrix<double, 7, 1> x_hat_;
    Eigen::Matrix<double, 7, 7> P_;
    Eigen::Matrix<double, 7, 7> Q_;
    Eigen::Matrix4d R_odom_;
    Eigen::Matrix3d R_imu_;

    Eigen::Vector2d u_input_ = Eigen::Vector2d::Zero(); // [v, omega]
    Eigen::Vector4d z_odom_ = Eigen::Vector4d::Zero();  // [x, y, vx, vy]
    Eigen::Vector3d z_imu_ = Eigen::Vector3d::Zero();   // [omega, ax_rob, ay_rob]

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
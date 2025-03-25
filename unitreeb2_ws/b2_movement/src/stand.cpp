#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <signal.h>

using std::placeholders::_1;

class EffortController : public rclcpp::Node {
public:
    EffortController() : Node("effort_controller_node"), trajectory_active(false), initialized(false) {
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&EffortController::joint_state_callback, this, _1));
        effort_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10);

        initial_joint_angles.assign(12, 0.0);
        current_joint_angles.assign(12, 0.0);
        goal_joint_angles = {-2.2, 0.0, 1.3, 0.0, -2.2, 1.3, -2.2, 0.0, 1.3, 1.3, 0.0, -2.2};
        trajectory_duration = 10.0;

        kp = {225.0, 262.5, 300.0, 315.0, 352.5, 390.0};
        kd.assign(6, 10.4);
        ki.assign(6, 12.0);

        integral_max = 50.0;
        current_joint_positions.assign(12, 0.0);
        current_joint_velocities.assign(12, 0.0);
        error_integrals.assign(12, 0.0);
        prev_time = this->now();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub;
    std::vector<double> initial_joint_angles, current_joint_angles, goal_joint_angles;
    std::vector<double> kp, kd, ki;
    std::vector<double> current_joint_positions, current_joint_velocities, error_integrals;
    double trajectory_duration, integral_max;
    rclcpp::Time trajectory_start_time, prev_time;
    bool trajectory_active, initialized;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!initialized) {
            initial_joint_angles = msg->position;
            current_joint_angles = msg->position;
            trajectory_start_time = this->now();
            trajectory_active = true;
            initialized = true;
        }
        current_joint_positions = msg->position;
        current_joint_velocities = msg->velocity;

        double current_time = (this->now() - trajectory_start_time).seconds();
        if (trajectory_active) {
            if (current_time <= trajectory_duration) {
                for (size_t i = 0; i < 12; i++) {
                    current_joint_angles[i] = cycloidal_trajectory(initial_joint_angles[i], goal_joint_angles[i], current_time, trajectory_duration);
                }
            } else {
                trajectory_active = false;
                current_joint_angles = goal_joint_angles;
            }
        }

        auto efforts = compute_pid_efforts();
        publish_efforts(efforts);
    }

    double cycloidal_trajectory(double q0, double qf, double t, double T) {
        if (t >= T) return qf;
        double delta_q = qf - q0;
        double t_ratio = t / T;
        return q0 + delta_q * (t_ratio - (1 / (2 * M_PI)) * sin(2 * M_PI * t_ratio));
    }

    std::vector<double> compute_pid_efforts() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - prev_time).seconds();
        prev_time = current_time;
        if (dt <= 0 || dt > 0.1) dt = 0.01;

        std::vector<double> position_errors(12), velocity_errors(12), efforts(12);
        for (size_t i = 0; i < 12; i++) {
            position_errors[i] = current_joint_angles[i] - current_joint_positions[i];
            velocity_errors[i] = -current_joint_velocities[i];
            error_integrals[i] += position_errors[i] * dt;
            error_integrals[i] = std::clamp(error_integrals[i], -integral_max, integral_max);
            efforts[i] = kp[i % 6] * position_errors[i] + kd[i % 6] * velocity_errors[i] + ki[i % 6] * error_integrals[i];
        }
        return efforts;
    }

    void publish_efforts(const std::vector<double>& efforts) {
        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data = efforts;
        effort_pub->publish(effort_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EffortController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
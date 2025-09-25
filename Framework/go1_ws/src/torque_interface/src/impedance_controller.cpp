#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <vector>
#include <algorithm>

/**
ROS2 node implementing a joint space impedance controller

This node above the torque bridge in the control hierarchy.

This node subscribes to:
    - "filtered_joint_states" for current joint positions, velocities, and estimated torques coming from the low level controller via the buffer
    - "desired_joint_states" for target joint positions and velocities, provided by a higher controller

This controller computes the desired joint torques using the joint-space impedance control law (pulled from unitree_legged_sdk/example_torque.cpp)

T = stiffness * (desired_position - measured_position) + damping * (desired_velocity - measured_velocity)

The resulting torques are published to:
    - "desired_torques", which is then seen by the TorqueCommandNode to update the buffer for the low level loop
*/

ImpedanceController::ImpedanceController()
: Node("impedance_controller")
{
    // declare parameters for gain and torque limit
    this->declare_parameter<std::vector<double>>("stiffness", std::vector<double>(k_number_joints, 40.0));
    this->declare_parameter<std::vector<double>>("damping", std::vector<double>(k_number_joints, 2.0));
    this->declare_parameter<std::vector<double>>("torque_limits", std::vector<double>(k_number_joints, 10.0));
    this->declare_parameter<double>("control_rate_hz", 100.0);

    // load parameters
    K_ this->get_parameter("stiffness").as_double_array();
    D_ = this->get_parameter("damping").as_double_array();
    torque_limits_ = this->get_parameter("torque_limits").as_double_array();
    double rate = this->get_parameter("control_rate_hz").as_double();

    // verify correct size
    ensure_vector_size(stiffness_, 40.0);
    ensure_vector_size(damping_, 2.0);
    ensure_vector_size(torque_limits_, 10.0);

    // joint vectors
    measuredPosition_.assign(k_number_joints, 0.0);
    measuredVelocity_.assign(k_number_joints, 0.0);
    desiredPosition_.assign(k_number_joints, 0.0);
    desiredVelocity_.assign(k_number_joints, 0.0);

    // subscriptions
    sub_desired_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("desired_positions", 10, std::bind(&ImpedanceController:desiredPositionsCallback, this, std::placeholders::_1));
    sub_joint_state_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("filtered_positions", 10, std::bind(&ImpedanceController:jointStateCallback, this, std::placeholders::_1));

    // publish desired torque
    pub_torques_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("desired_torques", 10);


    // timer for control loop
    auto period = std::chrono::duration<double>(1.0/rate);
    timer_ = this->create_wall_timier(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&ImpedanceController:controlLoop, this));


    // logs initialization
    RCLCPP_INFO(this->get_logger(), "ImpedanceController initialized at %.1f Hz", rate);
}
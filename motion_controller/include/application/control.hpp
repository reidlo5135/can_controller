#ifndef APPLICATION_CONTROL_HPP
#define APPLICATION_CONTROL_HPP

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>
#include <memory>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <can_msgs/msg/ad_control_accelerate.hpp>
#include <can_msgs/msg/ad_control_brake.hpp>
#include <can_msgs/msg/ad_control_steering.hpp>
#include <can_msgs/msg/emergency.hpp>
#include <can_utils/logger.hpp>
#include <can_utils/math.hpp>

#include "domain/parameter.hpp"
#include "application/converter.hpp"

#define MOTION_CONTROL "motion_control"

using std::placeholders::_1;

namespace net::wavem::drive
{
	class Control final
	{
	private:
		rclcpp::Node::SharedPtr node_;
		Parameter::SharedPtr parameter_;
		net::wavem::can::Math::SharedPtr can_math_;

		bool emergency_check_;

		rclcpp::CallbackGroup::SharedPtr cmd_vel_subscription_cb_group_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
		void cmd_vel_subscription_cb(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

		rclcpp::CallbackGroup::SharedPtr accelerate_publisher_cb_group_;
		rclcpp::Publisher<can_msgs::msg::AdControlAccelerate>::SharedPtr accelerate_publisher_;

		rclcpp::CallbackGroup::SharedPtr brake_publisher_cb_group_;
		rclcpp::Publisher<can_msgs::msg::AdControlBrake>::SharedPtr brake_publisher_;

		rclcpp::CallbackGroup::SharedPtr steering_publisher_cb_group_;
		rclcpp::Publisher<can_msgs::msg::AdControlSteering>::SharedPtr steering_publisher_;

		rclcpp::CallbackGroup::SharedPtr emergency_subscription_cb_group_;
		rclcpp::Subscription<can_msgs::msg::Emergency>::SharedPtr emergency_subscription_;
		void emergency_subscription_cb(const can_msgs::msg::Emergency::SharedPtr stop);

	private:
		double pid_calculated_accelerate_;
		double pid_integral_;
		double pid_previous_error_;
		double convert_angular_to_ackermann(const double &origin_linear, const double &origin_angle);
		void calculate_pid_accelerate(const double &target_accelerate);

	public:
		explicit Control(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter);
		virtual ~Control();
		void control_vehicle_drive(double linear_x, double angular_z);
		void control_vehicle_brake(const uint32_t &pressure);
		void control_vehicle_reset(const int &work_mode, const int &gear);

	public:
		using SharedPtr = std::shared_ptr<Control>;
	};
}

#endif // APPLICATION_CONTROL_HPP

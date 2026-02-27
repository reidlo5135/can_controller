#include "application/control.hpp"

using namespace net::wavem::drive;

Control::Control(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter)
	: node_(node),
	  parameter_(parameter),
	  pid_calculated_accelerate_(0.0),
	  pid_integral_(0.0),
	  pid_previous_error_(0.0)
{
	this->can_math_ = std::make_shared<net::wavem::can::Math>();

	this->pid_integral_ = this->parameter_->vehicle_control_accelerate_pid_integral_;
	this->pid_previous_error_ = this->parameter_->vehicle_control_accelerate_pid_previous_error_;

	this->cmd_vel_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions cmd_vel_subscription_opts;
	cmd_vel_subscription_opts.callback_group = this->cmd_vel_subscription_cb_group_;
	this->cmd_vel_subscription_ = this->node_->create_subscription<geometry_msgs::msg::Twist>(
		this->parameter_->vehicle_control_twist_topic_,
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
		std::bind(&Control::cmd_vel_subscription_cb, this, _1),
		cmd_vel_subscription_opts);

	this->accelerate_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::PublisherOptions accelerate_publisher_opts;
	accelerate_publisher_opts.callback_group = this->accelerate_publisher_cb_group_;
	this->accelerate_publisher_ = this->node_->create_publisher<can_msgs::msg::AdControlAccelerate>(
		this->parameter_->vehicle_control_accelerate_topic_,
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
		accelerate_publisher_opts);

	this->brake_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::PublisherOptions brake_publisher_opts;
	brake_publisher_opts.callback_group = this->brake_publisher_cb_group_;
	this->brake_publisher_ = this->node_->create_publisher<can_msgs::msg::AdControlBrake>(
		this->parameter_->vehicle_control_brake_topic_,
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
		brake_publisher_opts);

	this->steering_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::PublisherOptions steering_publisher_opts;
	steering_publisher_opts.callback_group = this->steering_publisher_cb_group_;
	this->steering_publisher_ = this->node_->create_publisher<can_msgs::msg::AdControlSteering>(
		this->parameter_->vehicle_control_steering_topic_,
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
		steering_publisher_opts);

	this->emergency_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions emergency_subscription_opts;
	emergency_subscription_opts.callback_group = this->emergency_subscription_cb_group_;
	this->emergency_subscription_ = this->node_->create_subscription<can_msgs::msg::Emergency>(
		this->parameter_->vehicle_control_emergency_topic_,
		rclcpp::QoS(rclcpp::KeepLast(1)),
		std::bind(&Control::emergency_subscription_cb, this, _1),
		emergency_subscription_opts);
}

Control::~Control() = default;

/**
 * @brief cmd_vel receive function for robot control
 * @author changunAn(changun516@wavem.net)
 * @date 23.04.06
 * @author reidlo(naru5135@wavem.net)
 * @date 25.01.19
 */
void Control::cmd_vel_subscription_cb(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
	this->control_vehicle_drive(cmd_vel->linear.x, cmd_vel->angular.z);
}

void Control::emergency_subscription_cb(const can_msgs::msg::Emergency::SharedPtr emergency)
{
	this->emergency_check_ = emergency->stop;

	if (this->emergency_check_)
	{
		this->control_vehicle_brake(100);
	}
	else
	{
		this->control_vehicle_brake(0);
	}
}

void Control::control_vehicle_drive(double linear_x, double angular_z)
{
	linear_x = [](double a, double b)
	{
		return a > b ? b : a;
	}(linear_x, this->parameter_->vehicle_control_twist_limit_linear_);

	double target_accelerate = this->can_math_->convert_linear_to_kmh(linear_x, this->parameter_->vehicle_control_twist_correction_linear_);
	this->calculate_pid_accelerate(target_accelerate);

	can_msgs::msg::AdControlAccelerate::UniquePtr accelerate = std::make_unique<can_msgs::msg::AdControlAccelerate>();
	accelerate->set__ad_accelerate_valid(static_cast<uint8_t>(this->parameter_->vehicle_heartbeat_activate_));

	if (std::abs(this->pid_calculated_accelerate_) <= this->parameter_->vehicle_control_accelerate_limit_min_)
	{
		this->pid_calculated_accelerate_ = 0.0;
	}
	accelerate->set__ad_speed_control(this->pid_calculated_accelerate_);

	unsigned int gear;

	if (this->pid_calculated_accelerate_ < 0.0)
	{
		gear = this->parameter_->vehicle_mcu_gear_position_rear_;
	}
	else
	{
		gear = this->parameter_->vehicle_mcu_gear_position_drive_;
	}

	accelerate->set__ad_accelerate_gear(gear);

	can_msgs::msg::AdControlSteering::UniquePtr steering = std::make_unique<can_msgs::msg::AdControlSteering>();
	const double &max_steering = this->parameter_->vehicle_control_steering_limit_;
	double corrected_steering = angular_z * this->parameter_->vehicle_control_twist_correction_angular_;
	corrected_steering = std::clamp(corrected_steering, -max_steering, max_steering);

	const double &calculated_pid_linear = this->can_math_->convert_kmh_to_linear(this->pid_calculated_accelerate_, this->parameter_->vehicle_control_twist_correction_linear_);
	const double &ackermann_angle = -(this->convert_angular_to_ackermann(calculated_pid_linear, corrected_steering) * (180.0 / M_PI)) * this->parameter_->vehicle_control_twist_correction_angular_;

	steering->set__ad_steering_valid(static_cast<uint8_t>(this->parameter_->vehicle_heartbeat_activate_));
	steering->set__ad_steering_angle_cmd(ackermann_angle);

	this->accelerate_publisher_->publish(std::move(*accelerate));
	this->steering_publisher_->publish(std::move(*steering));

	RCUTILS_INFO(MOTION_CONTROL,
				 "\n\tTarget Accelerate : [%.2f]\n\tCalculated Accelerate : [%.2f]\n\tTarget Linear : [%.2f]\n\tCalculated Linear : [%.2f]\n\tCorrected Angular : [%.2f]\n\tAckermann Angle : [%.2f]",
				 target_accelerate,
				 this->pid_calculated_accelerate_,
				 linear_x,
				 calculated_pid_linear,
				 corrected_steering,
				 ackermann_angle);
}

void Control::calculate_pid_accelerate(const double &target_accelerate)
{
	double dt = this->parameter_->vehicle_control_accelerate_pid_dt_;
	double kp = this->parameter_->vehicle_control_accelerate_pid_kp_;
	double ki = this->parameter_->vehicle_control_accelerate_pid_ki_;
	double kd = this->parameter_->vehicle_control_accelerate_pid_kd_;

	// 1. E = Target Acceleration - Calculated Acceleration
	double error = target_accelerate - this->pid_calculated_accelerate_;

	// 2. IT = E * DT
	this->pid_integral_ += error * dt;

	// 3. DE = (E - Previous E) / DT
	double de = (error - this->pid_previous_error_) / dt;

	// 3. Output = KP * E + KI * IT + KD * DE
	const double &pid_output = kp * error + ki * this->pid_integral_ + kd * de;
	this->pid_previous_error_ = error;

	// 4. Calculated Acceleration = Calculated Acceleration + Output
	this->pid_calculated_accelerate_ += pid_output;

	// 5. Calculated Acceleration clamping
	double max_speed = this->parameter_->vehicle_control_accelerate_limit_max_;
	this->pid_calculated_accelerate_ = std::clamp(this->pid_calculated_accelerate_, -max_speed, max_speed);
}

void Control::control_vehicle_brake(const uint32_t &pressure)
{
	can_msgs::msg::AdControlBrake::UniquePtr brake = std::make_unique<can_msgs::msg::AdControlBrake>();
	brake->set__ad_dbs_valid(static_cast<uint8_t>(this->parameter_->vehicle_heartbeat_activate_));
	brake->set__ad_brakepressure_cmd(pressure);
	this->brake_publisher_->publish(std::move(*brake));
}

double Control::convert_angular_to_ackermann(const double &origin_linear, const double &origin_angle)
{
	AckermannDriver ackermann(this->parameter_->vehicle_wheel_base_);
	const double &steering_angle = ackermann.convert_trans_rot_vel_to_steering_angle(origin_linear, origin_angle);
	return steering_angle;
}

void Control::control_vehicle_reset(const int &work_mode, const int &gear)
{
	can_msgs::msg::AdControlAccelerate::UniquePtr accelerate = std::make_unique<can_msgs::msg::AdControlAccelerate>();
	accelerate->set__ad_accelerate_valid(static_cast<uint8_t>(this->parameter_->vehicle_heartbeat_deactivate_));
	accelerate->set__ad_accelerate_work_mode(work_mode);
	accelerate->set__ad_accelerate_gear(gear);
	accelerate->set__ad_speed_control(0.0);
	this->accelerate_publisher_->publish(std::move(*accelerate));
}
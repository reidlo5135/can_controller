#include "presentation/node.hpp"

using namespace net::wavem::drive;

/**
 * @brief Construct a new Wm Motion Controller:: Wm Motion Controller object
 * @author changunAn(changun516@wavem.net)
 * @date 23.04.06
 * @author reidlo(naru5135@wavem.net)
 * @date 25.01.19
 */
MotionController::MotionController()
	: Node(NODE_NAME)
{
	this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

	if (this->node_ == nullptr)
	{
		RCUTILS_ERROR(NODE_NAME, "%s, node pointer is nullptr...", NODE_NAME);
		return;
	}
	else
	{
		RCUTILS_INFO(NODE_NAME, "%s created", this->node_->get_name());

		this->parameter_ = std::make_shared<Parameter>();
		this->declare_parameters();
		this->control_ = std::make_shared<Control>(this->node_, this->parameter_);
		this->motion_ = std::make_shared<Motion>(this->node_, this->parameter_, this->control_);
	}
}

MotionController::~MotionController() = default;

void MotionController::declare_parameters()
{
	const std::string &vehicle_control_twist = std::string(PARAM_VEHICLE_CONTROL) + ".twist";
	const std::string &vehicle_control_accelerate = std::string(PARAM_VEHICLE_CONTROL) + ".accelerate";
	const std::string &vehicle_control_steering = std::string(PARAM_VEHICLE_CONTROL) + ".steering";
	const std::string &vehicle_control_brake = std::string(PARAM_VEHICLE_CONTROL) + ".brake";
	const std::string &vehicle_control_emergency = std::string(PARAM_VEHICLE_CONTROL) + ".emergency";
	const std::string &vehicle_control_deceleration = std::string(PARAM_VEHICLE_CONTROL) + ".deceleration";

	const std::string &vehicle_mcu_rpm = std::string(PARAM_VEHICLE_MCU) + ".rpm";
	const std::string &vehicle_mcu_gear = std::string(PARAM_VEHICLE_MCU) + ".gear";
	const std::string &vehicle_mcu_heartbeat = std::string(PARAM_VEHICLE_MCU) + ".heartbeat";

	const std::string &vehicle_wheel_size = std::string(PARAM_VEHICLE_WHEEL) + ".size";
	const std::string &vehicle_wheel_base = std::string(PARAM_VEHICLE_WHEEL) + ".base";

	const std::string &map = std::string(PARAM_MAP);
	const std::string &map_area = std::string(PARAM_MAP) + ".area";

	std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> param_vec =
		{
			{std::string(PARAM_VEHICLE_STATUS) + ".topic", ""},
			{vehicle_control_twist + ".topic", ""},
			{vehicle_control_twist + ".correction" + ".linear", 0.0},
			{vehicle_control_twist + ".correction" + ".angular", 0.0},
			{vehicle_control_twist + ".limit" + ".linear", 0.0},
			{vehicle_control_twist + ".limit" + ".angular", 0.0},
			{vehicle_control_accelerate + ".topic", ""},
			{vehicle_control_accelerate + ".limit" + ".max", 0.0},
			{vehicle_control_accelerate + ".limit" + ".min", 0.0},
			{vehicle_control_accelerate + ".pid" + ".rate", 0},
			{vehicle_control_accelerate + ".pid" + ".dt", 0.0},
			{vehicle_control_accelerate + ".pid" + ".kp", 0.0},
			{vehicle_control_accelerate + ".pid" + ".ki", 0.0},
			{vehicle_control_accelerate + ".pid" + ".kd", 0.0},
			{vehicle_control_accelerate + ".pid" + ".integral", 0.0},
			{vehicle_control_accelerate + ".pid" + ".previous_error", 0.0},
			{vehicle_control_steering + ".topic", ""},
			{vehicle_control_steering + ".limit", 0.0},
			{vehicle_control_brake + ".topic", ""},
			{vehicle_control_brake + ".limit", 0},
			{vehicle_control_emergency + ".topic", ""},
			{vehicle_control_emergency + "trigger", false},
			{vehicle_control_deceleration + ".status" + ".topic", ""},
			{vehicle_control_deceleration + ".activate" + ".topic", ""},
			{vehicle_control_deceleration + ".trigger", false},
			{vehicle_mcu_rpm + ".topic", ""},
			{vehicle_mcu_rpm + ".correction", 0.0},
			{vehicle_mcu_gear + ".topic", ""},
			{vehicle_mcu_gear + ".position" + ".parking", 0},
			{vehicle_mcu_gear + ".position" + ".drive", 0},
			{vehicle_mcu_gear + ".position" + ".neutral", 0},
			{vehicle_mcu_gear + ".position" + ".rear", 0},
			{vehicle_mcu_gear + ".ratio", 0.0},
			{vehicle_mcu_heartbeat + ".activate", 0},
			{vehicle_mcu_heartbeat + ".deactivate", 0},
			{vehicle_mcu_heartbeat + ".retry", 0},
			{vehicle_wheel_size, 0.0},
			{vehicle_wheel_base, 0.0},
			{map + ".topic", ""},
			{map_area + ".geo_fence", 0},
			{map_area + ".start", 0},
			{map_area + ".fair", 0},
			{map_area + ".road", 0}};

	for (const std::pair<std::string, std::variant<int, double, bool, std::string>> &param : param_vec)
	{
		const std::string &param_name = param.first;
		const auto &default_value = param.second;

		std::visit([this, &param_name](auto &&value)
				   {
                using T = std::decay_t<decltype(value)>;
                this->node_->declare_parameter<T>(param_name, value); }, default_value);
	}

	this->node_->get_parameter<std::string>(std::string(PARAM_VEHICLE_STATUS) + ".topic", this->parameter_->vehicle_status_topic_);
	this->node_->get_parameter<std::string>(vehicle_control_twist + ".topic", this->parameter_->vehicle_control_twist_topic_);
	this->node_->get_parameter<double>(vehicle_control_twist + ".correction" + ".linear", this->parameter_->vehicle_control_twist_correction_linear_);
	this->node_->get_parameter<double>(vehicle_control_twist + ".correction" + ".angular", this->parameter_->vehicle_control_twist_correction_angular_);
	this->node_->get_parameter<double>(vehicle_control_twist + ".limit" + ".linear", this->parameter_->vehicle_control_twist_limit_linear_);
	this->node_->get_parameter<double>(vehicle_control_twist + ".limit" + ".angular", this->parameter_->vehicle_control_twist_limit_angular_);

	this->node_->get_parameter<std::string>(vehicle_control_accelerate + ".topic", this->parameter_->vehicle_control_accelerate_topic_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".limit" + ".max", this->parameter_->vehicle_control_accelerate_limit_max_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".limit" + ".min", this->parameter_->vehicle_control_accelerate_limit_min_);
	this->node_->get_parameter<int>(vehicle_control_accelerate + ".pid" + ".rate", this->parameter_->vehicle_control_accelerate_pid_rate_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".pid" + ".dt", this->parameter_->vehicle_control_accelerate_pid_dt_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".pid" + ".kp", this->parameter_->vehicle_control_accelerate_pid_kp_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".pid" + ".ki", this->parameter_->vehicle_control_accelerate_pid_ki_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".pid" + ".kd", this->parameter_->vehicle_control_accelerate_pid_kd_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".pid" + ".integral", this->parameter_->vehicle_control_accelerate_pid_integral_);
	this->node_->get_parameter<double>(vehicle_control_accelerate + ".pid" + ".previous_error", this->parameter_->vehicle_control_accelerate_pid_previous_error_);

	this->node_->get_parameter<std::string>(vehicle_control_steering + ".topic", this->parameter_->vehicle_control_steering_topic_);
	this->node_->get_parameter<double>(vehicle_control_steering + ".limit", this->parameter_->vehicle_control_steering_limit_);

	this->node_->get_parameter<std::string>(vehicle_control_brake + ".topic", this->parameter_->vehicle_control_brake_topic_);
	this->node_->get_parameter<int>(vehicle_control_brake + ".limit", this->parameter_->vehicle_control_brake_limit_);

	this->node_->get_parameter<std::string>(vehicle_control_emergency + ".topic", this->parameter_->vehicle_control_emergency_topic_);
	this->node_->get_parameter<bool>(vehicle_control_emergency + ".trigger", this->parameter_->vehicle_control_emergency_trigger_);

	this->node_->get_parameter<std::string>(vehicle_control_deceleration + ".status" + ".topic", this->parameter_->vehicle_control_deceleration_status_topic_);
	this->node_->get_parameter<std::string>(vehicle_control_deceleration + ".activate" +".topic", this->parameter_->vehicle_control_deceleration_activate_topic_);
	this->node_->get_parameter<bool>(vehicle_control_deceleration + ".trigger", this->parameter_->vehicle_control_deceleration_trigger_);

	this->node_->get_parameter<std::string>(vehicle_mcu_rpm + ".topic", this->parameter_->vehicle_mcu_rpm_topic_);
	this->node_->get_parameter<double>(vehicle_mcu_rpm + ".correction", this->parameter_->vehicle_mcu_rpm_correction_);

	this->node_->get_parameter<std::string>(vehicle_mcu_gear + ".topic", this->parameter_->vehicle_mcu_gear_topic_);
	this->node_->get_parameter<int>(vehicle_mcu_gear + ".position" + ".parking", this->parameter_->vehicle_mcu_gear_position_parking_);
	this->node_->get_parameter<int>(vehicle_mcu_gear + ".position" + ".drive", this->parameter_->vehicle_mcu_gear_position_drive_);
	this->node_->get_parameter<int>(vehicle_mcu_gear + ".position" + ".neutral", this->parameter_->vehicle_mcu_gear_position_neutral_);
	this->node_->get_parameter<int>(vehicle_mcu_gear + ".position" + ".rear", this->parameter_->vehicle_mcu_gear_position_rear_);
	this->node_->get_parameter<double>(vehicle_mcu_gear + ".ratio", this->parameter_->vehicle_mcu_gear_ratio_);

	this->node_->get_parameter<int>(vehicle_mcu_heartbeat + ".activate", this->parameter_->vehicle_heartbeat_activate_);
	this->node_->get_parameter<int>(vehicle_mcu_heartbeat + ".deactivate", this->parameter_->vehicle_heartbeat_deactivate_);
	this->node_->get_parameter<int>(vehicle_mcu_heartbeat + ".retry", this->parameter_->vehicle_heartbeat_retry_);

	this->node_->get_parameter<double>(vehicle_wheel_size, this->parameter_->vehicle_wheel_size_);
	this->node_->get_parameter<double>(vehicle_wheel_base, this->parameter_->vehicle_wheel_base_);

	this->node_->get_parameter<std::string>(map + ".topic", this->parameter_->map_topic_);
	this->node_->get_parameter<int>(map_area + ".geo_fence", this->parameter_->map_area_geo_fence_);
	this->node_->get_parameter<int>(map_area + ".start", this->parameter_->map_area_start_);
	this->node_->get_parameter<int>(map_area + ".end", this->parameter_->map_area_end_);
	this->node_->get_parameter<int>(map_area + ".fair", this->parameter_->map_area_fair_);
	this->node_->get_parameter<int>(map_area + ".road", this->parameter_->map_area_road_);

	RCUTILS_INFO(NODE_NAME, "======= PARAMETERS DECLARED =======");
}
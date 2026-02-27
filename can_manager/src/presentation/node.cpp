#include "presentation/node.hpp"

using namespace net::wavem::can;

/**
 * @brief Construct a new CanManager object
 * @date 2025.04.23
 */
CanManager::CanManager()
    : Node(NODE_NAME), check_emergency_(false)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ == nullptr)
    {
        RCUTILS_ERROR(NODE_NAME, "node pointer is nullptr...");
        return;
    }
    else
    {
        RCUTILS_INFO(NODE_NAME, "%s created", this->node_->get_name());
        this->can_math_ = std::make_shared<Math>();
        this->parameter_ = std::make_shared<Parameter>();

        this->declare_parameters();

        if (this->can_initialize())
        {
            RCUTILS_INFO(NODE_NAME, "!!!!! CAN initialized !!!!!");
        }
        else
        {
            RCUTILS_ERROR(NODE_NAME, "@@@@@ CAN failed to initialize @@@@@");
            return;
        }
    }
}

/**
 * @brief Destroy the CanManager object
 * @date 2025.04.23
 */
CanManager::~CanManager() = default;

/**
 * @brief Declare all required ROS parameters for CAN configuration
 * @date 2025.04.23
 */
void CanManager::declare_parameters()
{
    const std::string param_can_send_accelerate = std::string(PARAM_CAN_SEND_ACCELERATE) + ".";
    const std::string param_can_send_steering = std::string(PARAM_CAN_SEND_STEERING) + ".";
    const std::string param_can_send_brake = std::string(PARAM_CAN_SEND_BRAKE) + ".";
    const std::string param_can_send_body = std::string(PARAM_CAN_SEND_BODY) + ".";
    const std::string param_can_dump_rpm = std::string(PARAM_CAN_DUMP_RPM) + ".";
    const std::string param_can_dump_bms = std::string(PARAM_CAN_DUMP_BMS) + ".";
    const std::string param_can_dump_odometry = std::string(PARAM_CAN_DUMP_ODOMETRY) + ".";
    const std::string param_can_dump_gear = std::string(PARAM_CAN_DUMP_GEAR) + ".";
    const std::string param_vehicle_wheel = std::string(PARAM_VEHICLE_WHEEL) + ".";
    const std::string param_vehicle_control = std::string(PARAM_VEHICLE_CONTROL) + ".";
    const std::string param_vehicle_status = std::string(PARAM_VEHICLE_STATUS) + ".";

    std::vector<std::pair<std::string, std::variant<std::string, int, bool, double>>> param_vec =
        {
            {PARAM_CAN_SEND_CHANNEL, ""},
            {PARAM_CAN_SEND_ACTIVATE_VALID, 0},
            {param_can_send_accelerate + PARAM_NAME, ""},
            {param_can_send_accelerate + PARAM_TOPIC, ""},
            {param_can_send_accelerate + PARAM_ID, 0},
            {param_can_send_accelerate + PARAM_FIELD, ""},
            {param_can_send_accelerate + PARAM_FACTOR, 0.0},
            {param_can_send_accelerate + PARAM_OFFSET, 0.0},
            {param_can_send_accelerate + PARAM_MAX_SPEED, 0.0},
            {param_can_send_accelerate + PARAM_MIN_SPEED, 0.0},
            {param_can_send_accelerate + PARAM_WORK_MODE + "." + PARAM_WORK_MODE_TORQUE, 0},
            {param_can_send_accelerate + PARAM_WORK_MODE + "." + PARAM_WORK_MODE_SPEED, 0},
            {param_can_send_accelerate + PARAM_WORK_MODE + "." + PARAM_WORK_MODE_ACCELERATE, 0},
            {param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_PARKING, 0},
            {param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_DRIVE, 0},
            {param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_NEUTRAL, 0},
            {param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_REAR, 0},
            {param_can_send_steering + PARAM_NAME, ""},
            {param_can_send_steering + PARAM_TOPIC, ""},
            {param_can_send_steering + PARAM_ID, 0},
            {param_can_send_steering + PARAM_FIELD, ""},
            {param_can_send_steering + PARAM_FACTOR, 0.0},
            {param_can_send_steering + PARAM_OFFSET, 0.0},
            {param_can_send_steering + PARAM_MAX_ANGLE, 0.0},
            {param_can_send_steering + PARAM_MIN_ANGLE, 0.0},
            {param_can_send_brake + PARAM_NAME, ""},
            {param_can_send_brake + PARAM_TOPIC, ""},
            {param_can_send_brake + PARAM_ID, 0},
            {param_can_send_brake + PARAM_FIELD, ""},
            {param_can_send_brake + PARAM_FACTOR, 0.0},
            {param_can_send_brake + PARAM_OFFSET, 0.0},
            {param_can_send_brake + PARAM_BRAKE_PRESS, 0},
            {param_can_send_brake + PARAM_BRAKE_RELEASE, 0},
            {param_can_send_body + PARAM_NAME, ""},
            {param_can_send_body + PARAM_TOPIC, ""},
            {param_can_send_body + PARAM_ID, 0},
            {param_can_send_body + PARAM_FIELD, ""},
            {param_can_send_body + PARAM_FACTOR, 0.0},
            {param_can_send_body + PARAM_OFFSET, 0.0},
            {param_can_send_body + PARAM_MAX_VALUE, 0},
            {param_can_send_body + PARAM_MIN_VALUE, 0},
            {PARAM_CAN_DUMP_CHANNEL, ""},
            {param_can_dump_rpm + PARAM_NAME, ""},
            {param_can_dump_rpm + PARAM_TOPIC, ""},
            {param_can_dump_rpm + PARAM_ID, 0},
            {param_can_dump_rpm + PARAM_FIELD, ""},
            {param_can_dump_rpm + PARAM_FACTOR, 0.0},
            {param_can_dump_rpm + PARAM_OFFSET, 0.0},
            {param_can_dump_rpm + "correction", 0.0},
            {param_can_dump_bms + PARAM_NAME, ""},
            {param_can_dump_bms + PARAM_TOPIC, ""},
            {param_can_dump_bms + PARAM_ID, 0},
            {param_can_dump_bms + PARAM_FIELD, ""},
            {param_can_dump_bms + PARAM_FACTOR, 0.0},
            {param_can_dump_bms + PARAM_OFFSET, 0.0},
            {param_can_dump_gear + PARAM_NAME, ""},
            {param_can_dump_gear + PARAM_TOPIC, ""},
            {param_can_dump_gear + PARAM_ID, 0},
            {param_can_dump_gear + PARAM_FIELD, ""},
            {param_can_dump_gear + PARAM_FACTOR, 0.0},
            {param_can_dump_gear + PARAM_OFFSET, 0.0},
            {param_can_dump_gear + PARAM_GEAR_PARKING, 0},
            {param_can_dump_gear + PARAM_GEAR_DRIVE, 0},
            {param_can_dump_gear + PARAM_GEAR_NEUTRAL, 0},
            {param_can_dump_gear + PARAM_GEAR_REAR, 0},
            {param_can_dump_odometry + PARAM_NAME, ""},
            {param_can_dump_odometry + PARAM_TOPIC, ""},
            {param_can_dump_odometry + PARAM_ID, 0},
            {param_can_dump_odometry + PARAM_FACTOR, 0.0},
            {param_can_dump_odometry + PARAM_OFFSET, 0.0},
            {param_vehicle_wheel + "size", 0.0},
            {param_vehicle_wheel + "base", 0.0},
            {param_vehicle_control + "loop" + ".trigger", false},
            {param_vehicle_control + "loop" + ".rate", 0},
            {param_vehicle_status + "topic", ""},
            {param_vehicle_status + "loop" + ".rate", 0},
        };

    for (const auto &param : param_vec)
    {
        const std::string &param_name = param.first;
        const auto &default_value = param.second;

        std::visit([this, &param_name](auto &&value)
                   {
                using T = std::decay_t<decltype(value)>;
                this->node_->declare_parameter<T>(param_name, value); }, default_value);
    }

    this->node_->get_parameter<std::string>(PARAM_CAN_SEND_CHANNEL, this->parameter_->can_send_channel_);
    this->node_->get_parameter<int>(PARAM_CAN_SEND_ACTIVATE_VALID, this->parameter_->can_send_activate_valid_);

    this->node_->get_parameter<std::string>(param_can_send_accelerate + PARAM_NAME, this->parameter_->can_send_accelerate_name_);
    this->node_->get_parameter<std::string>(param_can_send_accelerate + PARAM_TOPIC, this->parameter_->can_send_accelerate_topic_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_ID, this->parameter_->can_send_accelerate_id_);
    this->node_->get_parameter<std::string>(param_can_send_accelerate + PARAM_FIELD, this->parameter_->can_send_accelerate_field_);
    this->node_->get_parameter<double>(param_can_send_accelerate + PARAM_FACTOR, this->parameter_->can_send_accelerate_factor_);
    this->node_->get_parameter<double>(param_can_send_accelerate + PARAM_OFFSET, this->parameter_->can_send_accelerate_offset_);
    this->node_->get_parameter<double>(param_can_send_accelerate + PARAM_MAX_SPEED, this->parameter_->can_send_accelerate_max_speed_);
    this->node_->get_parameter<double>(param_can_send_accelerate + PARAM_MIN_SPEED, this->parameter_->can_send_accelerate_min_speed_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_WORK_MODE + "." + PARAM_WORK_MODE_TORQUE, this->parameter_->can_send_accelerate_work_mode_torque_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_WORK_MODE + "." + PARAM_WORK_MODE_SPEED, this->parameter_->can_send_accelerate_work_mode_speed_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_WORK_MODE + "." + PARAM_WORK_MODE_ACCELERATE, this->parameter_->can_send_accelerate_work_mode_accelerate_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_PARKING, this->parameter_->can_send_accelerate_gear_parking_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_DRIVE, this->parameter_->can_send_accelerate_gear_drive_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_NEUTRAL, this->parameter_->can_send_accelerate_gear_neutral_);
    this->node_->get_parameter<int>(param_can_send_accelerate + PARAM_GEAR + "." + PARAM_GEAR_REAR, this->parameter_->can_send_accelerate_gear_rear_);

    this->node_->get_parameter<std::string>(param_can_send_steering + PARAM_NAME, this->parameter_->can_send_steering_name_);
    this->node_->get_parameter<std::string>(param_can_send_steering + PARAM_TOPIC, this->parameter_->can_send_steering_topic_);
    this->node_->get_parameter<int>(param_can_send_steering + PARAM_ID, this->parameter_->can_send_steering_id_);
    this->node_->get_parameter<std::string>(param_can_send_steering + PARAM_FIELD, this->parameter_->can_send_steering_field_);
    this->node_->get_parameter<double>(param_can_send_steering + PARAM_FACTOR, this->parameter_->can_send_steering_factor_);
    this->node_->get_parameter<double>(param_can_send_steering + PARAM_OFFSET, this->parameter_->can_send_steering_offset_);
    this->node_->get_parameter<double>(param_can_send_steering + PARAM_MAX_ANGLE, this->parameter_->can_send_steering_max_angle_);
    this->node_->get_parameter<double>(param_can_send_steering + PARAM_MIN_ANGLE, this->parameter_->can_send_steering_min_angle_);

    this->node_->get_parameter<std::string>(param_can_send_brake + PARAM_NAME, this->parameter_->can_send_brake_name_);
    this->node_->get_parameter<std::string>(param_can_send_brake + PARAM_TOPIC, this->parameter_->can_send_brake_topic_);
    this->node_->get_parameter<int>(param_can_send_brake + PARAM_ID, this->parameter_->can_send_brake_id_);
    this->node_->get_parameter<std::string>(param_can_send_brake + PARAM_FIELD, this->parameter_->can_send_brake_field_);
    this->node_->get_parameter<double>(param_can_send_brake + PARAM_FACTOR, this->parameter_->can_send_brake_factor_);
    this->node_->get_parameter<double>(param_can_send_brake + PARAM_OFFSET, this->parameter_->can_send_brake_offset_);
    this->node_->get_parameter<int>(param_can_send_brake + PARAM_BRAKE_PRESS, this->parameter_->can_send_brake_press_);
    this->node_->get_parameter<int>(param_can_send_brake + PARAM_BRAKE_RELEASE, this->parameter_->can_send_brake_release_);

    this->node_->get_parameter<std::string>(param_can_send_body + PARAM_NAME, this->parameter_->can_send_body_name_);
    this->node_->get_parameter<std::string>(param_can_send_body + PARAM_TOPIC, this->parameter_->can_send_body_topic_);
    this->node_->get_parameter<int>(param_can_send_body + PARAM_ID, this->parameter_->can_send_body_id_);
    this->node_->get_parameter<std::string>(param_can_send_body + PARAM_FIELD, this->parameter_->can_send_body_field_);
    this->node_->get_parameter<double>(param_can_send_body + PARAM_FACTOR, this->parameter_->can_send_body_factor_);
    this->node_->get_parameter<double>(param_can_send_body + PARAM_OFFSET, this->parameter_->can_send_body_offset_);
    this->node_->get_parameter<int>(param_can_send_body + PARAM_MAX_VALUE, this->parameter_->can_send_body_max_value_);
    this->node_->get_parameter<int>(param_can_send_body + PARAM_MIN_VALUE, this->parameter_->can_send_body_min_value_);

    this->node_->get_parameter<std::string>(PARAM_CAN_DUMP_CHANNEL, this->parameter_->can_dump_channel_);

    this->node_->get_parameter<std::string>(param_can_dump_rpm + PARAM_NAME, this->parameter_->can_dump_rpm_name_);
    this->node_->get_parameter<std::string>(param_can_dump_rpm + PARAM_TOPIC, this->parameter_->can_dump_rpm_topic_);
    this->node_->get_parameter<int>(param_can_dump_rpm + PARAM_ID, this->parameter_->can_dump_rpm_id_);
    this->node_->get_parameter<std::string>(param_can_dump_rpm + PARAM_FIELD, this->parameter_->can_dump_rpm_field_);
    this->node_->get_parameter<double>(param_can_dump_rpm + PARAM_FACTOR, this->parameter_->can_dump_rpm_factor_);
    this->node_->get_parameter<double>(param_can_dump_rpm + PARAM_OFFSET, this->parameter_->can_dump_rpm_offset_);
    this->node_->get_parameter<double>(param_can_dump_rpm + "correction", this->parameter_->can_dump_rpm_correction_);

    this->node_->get_parameter<std::string>(param_can_dump_bms + PARAM_NAME, this->parameter_->can_dump_bms_name_);
    this->node_->get_parameter<std::string>(param_can_dump_bms + PARAM_TOPIC, this->parameter_->can_dump_bms_topic_);
    this->node_->get_parameter<int>(param_can_dump_bms + PARAM_ID, this->parameter_->can_dump_bms_id_);
    this->node_->get_parameter<std::string>(param_can_dump_bms + PARAM_FIELD, this->parameter_->can_dump_bms_field_);
    this->node_->get_parameter<double>(param_can_dump_bms + PARAM_FACTOR, this->parameter_->can_dump_bms_factor_);
    this->node_->get_parameter<double>(param_can_dump_bms + PARAM_OFFSET, this->parameter_->can_dump_bms_offset_);

    this->node_->get_parameter<std::string>(param_can_dump_gear + PARAM_NAME, this->parameter_->can_dump_gear_name_);
    this->node_->get_parameter<std::string>(param_can_dump_gear + PARAM_TOPIC, this->parameter_->can_dump_gear_topic_);
    this->node_->get_parameter<int>(param_can_dump_gear + PARAM_ID, this->parameter_->can_dump_gear_id_);
    this->node_->get_parameter<std::string>(param_can_dump_gear + PARAM_FIELD, this->parameter_->can_dump_gear_field_);
    this->node_->get_parameter<double>(param_can_dump_gear + PARAM_FACTOR, this->parameter_->can_dump_gear_factor_);
    this->node_->get_parameter<double>(param_can_dump_gear + PARAM_OFFSET, this->parameter_->can_dump_gear_offset_);
    this->node_->get_parameter<int>(param_can_dump_gear + PARAM_GEAR_PARKING, this->parameter_->can_dump_gear_parking_);
    this->node_->get_parameter<int>(param_can_dump_gear + PARAM_GEAR_DRIVE, this->parameter_->can_dump_gear_drive_);
    this->node_->get_parameter<int>(param_can_dump_gear + PARAM_GEAR_NEUTRAL, this->parameter_->can_dump_gear_neutral_);
    this->node_->get_parameter<int>(param_can_dump_gear + PARAM_GEAR_REAR, this->parameter_->can_dump_gear_rear_);

    this->node_->get_parameter<std::string>(param_can_dump_odometry + PARAM_NAME, this->parameter_->can_dump_odometer_name_);
    this->node_->get_parameter<std::string>(param_can_dump_odometry + PARAM_TOPIC, this->parameter_->can_dump_odometer_topic_);
    this->node_->get_parameter<int>(param_can_dump_odometry + PARAM_ID, this->parameter_->can_dump_odometer_id_);
    this->node_->get_parameter<double>(param_can_dump_odometry + PARAM_FACTOR, this->parameter_->can_dump_odometer_factor_);
    this->node_->get_parameter<double>(param_can_dump_odometry + PARAM_OFFSET, this->parameter_->can_dump_odometer_offset_);

    this->node_->get_parameter<double>(param_vehicle_wheel + "size", this->parameter_->vehicle_wheel_size_);
    this->node_->get_parameter<double>(param_vehicle_wheel + "base", this->parameter_->vehicle_wheel_base_);

    this->node_->get_parameter<bool>(param_vehicle_control + "loop" + ".trigger", this->parameter_->vehicle_control_loop_trigger_);
    this->node_->get_parameter<int>(param_vehicle_control + "loop" + ".rate", this->parameter_->vehicle_control_loop_rate_);

    this->node_->get_parameter<std::string>(param_vehicle_status + "topic", this->parameter_->vehicle_status_topic_);
    this->node_->get_parameter<int>(param_vehicle_status + "loop" + ".rate", this->parameter_->vehicle_status_loop_rate_);
}

bool CanManager::can_initialize()
{
    this->can_adaptor_ = std::make_shared<Adaptor>();
    this->can_adaptor_->initialize(this->system_endian_);
    this->c2r_ = std::make_shared<C2R>(this->node_, this->parameter_, this->can_adaptor_);
    this->r2c_ = std::make_shared<R2C>(this->node_, this->parameter_, this->can_adaptor_);

    return this->can_adaptor_ != nullptr && this->c2r_ != nullptr && this->r2c_ != nullptr;
}

/**
 * @brief Initialize CAN interface, check channel, and register callbacks
 * @date 2025.04.23
 */
void CanManager::can_run()
{
    RCUTILS_INFO(NODE_NAME, "===== Running Started !!! =====");
    this->c2r_->register_can_dump_callbacks();

    std::vector<std::string> device;
    device.emplace_back(this->parameter_->can_send_channel_);

    while (this->can_adaptor_->open(device) != 0)
    {
        RCUTILS_INFO(NODE_NAME, "Send Channel open fail");
        sleep(2);
    }

    RCUTILS_INFO(NODE_NAME, "Start checking for can channel fault");
}

#include "application/r2c.hpp"

using namespace net::wavem::can;

/**
 * @brief Construct a new R2C object
 * @param node ROS2 node handle
 * @param parameter Shared pointer to configuration
 * @param can_adaptor Shared pointer to CAN adaptor
 * @date 2025.04.23
 */
R2C::R2C(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Adaptor::SharedPtr &can_adaptor)
    : node_(node), parameter_(parameter), can_adaptor_(can_adaptor), is_vehicle_control_activated_(false), is_vehicle_brake_activated_(false)
{
    if (this->node_ == nullptr)
    {
        RCUTILS_ERROR(CAN_R2C, "node pointer is nullptr...");
        return;
    }

    this->can_send_channel_ = this->parameter_->can_send_channel_;
    this->vehicle_control_ = std::make_shared<Control>();
    this->system_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    this->vehicle_control_watchdog_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->vehicle_control_watchdog_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&R2C::vehicle_control_watchdog_timer_cb, this),
        this->vehicle_control_watchdog_timer_cb_group_);

    // Vehicle control
    this->vehicle_control_loop_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->vehicle_control_loop_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(this->parameter_->vehicle_control_loop_rate_),
        std::bind(&R2C::vehicle_control_loop_timer_cb, this),
        this->vehicle_control_loop_timer_cb_group_);

    // Acceleration
    this->ad_accelerate_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ad_accelerate_subscription_opts;
    ad_accelerate_subscription_opts.callback_group = this->ad_accelerate_subscription_cb_group_;
    this->ad_accelerate_subscription_ = this->node_->create_subscription<can_msgs::msg::AdControlAccelerate>(
        this->parameter_->can_send_accelerate_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&R2C::ad_accelerate_subscription_cb, this, _1),
        ad_accelerate_subscription_opts);

    // Steering
    this->ad_steering_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ad_steering_subscription_opts;
    ad_steering_subscription_opts.callback_group = this->ad_steering_subscription_cb_group_;
    this->ad_steering_subscription_ = this->node_->create_subscription<can_msgs::msg::AdControlSteering>(
        this->parameter_->can_send_steering_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&R2C::ad_steering_subscription_cb, this, _1),
        ad_steering_subscription_opts);

    // Brake
    this->ad_control_brake_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ad_control_brake_subscription_opts;
    ad_control_brake_subscription_opts.callback_group = this->ad_control_brake_subscription_cb_group_;
    this->ad_control_brake_subscription_ = this->node_->create_subscription<can_msgs::msg::AdControlBrake>(
        this->parameter_->can_send_brake_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&R2C::ad_control_brake_subscription_cb, this, _1),
        ad_control_brake_subscription_opts);

    // Body Control
    this->ad_control_body_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ad_control_body_subscription_opts;
    ad_control_body_subscription_opts.callback_group = this->ad_control_body_subscription_cb_group_;
    this->ad_control_body_subscription_ = this->node_->create_subscription<can_msgs::msg::AdControlBody>(
        this->parameter_->can_send_body_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&R2C::ad_control_body_subscription_cb, this, _1),
        ad_control_body_subscription_opts);
}

/**
 * @brief Destroy the R2C object
 * @date 2025.04.23
 */
R2C::~R2C() = default;

void R2C::vehicle_control_watchdog_timer_cb()
{
    rclcpp::Time now = this->system_clock_->now();

    if ((now - this->last_accelerate_msg_time_).seconds() > 1.0)
    {
        this->is_vehicle_control_activated_ = false;
    }

    if ((now - this->last_brake_msg_time_).seconds() > 1.0)
    {
        this->is_vehicle_brake_activated_ = false;
    }
}

void R2C::vehicle_control_loop_timer_cb()
{
    if (!this->parameter_->vehicle_control_loop_trigger_)
    {
        return;
    }

    // 초기 상태
    if (this->vehicle_control_->get__accelerate_valid() == 2)
    {
        return;
    }

    // 브레이크 수신 시
    if (this->is_vehicle_brake_activated_)
    {
        this->control_velocity(this->vehicle_control_->get__accelerate_valid(), 0, 0.0);
        this->control_brake(this->vehicle_control_->get__brake_pressure());
    }

    // 가감속 / 조향 수신 시
    if (!this->is_vehicle_control_activated_)
    {
        return;
    }

    this->control_velocity(this->vehicle_control_->get__accelerate_valid(), this->vehicle_control_->get__accelerate_gear(), this->vehicle_control_->get__accelerate_speed());

    if (this->vehicle_control_->get__accelerate_valid() == this->parameter_->can_send_activate_valid_)
    {
        this->control_steering(this->vehicle_control_->get__steering_angle());
    }
    else
    {
        return;
    }
}

/**
 * @brief Callback for AdControlAccelerate subscription
 * @param control_accelerate Acceleration control message
 * @date 2025.04.23
 */
void R2C::ad_accelerate_subscription_cb(can_msgs::msg::AdControlAccelerate::SharedPtr control_accelerate)
{
    this->last_accelerate_msg_time_ = this->system_clock_->now();
    this->is_vehicle_control_activated_ = true;

    if (this->parameter_->vehicle_control_loop_trigger_)
    {
        this->vehicle_control_->set__accelerate_valid(control_accelerate->ad_accelerate_valid);
        this->vehicle_control_->set__accelerate_gear(control_accelerate->ad_accelerate_gear);
        this->vehicle_control_->set__accelerate_work_mode(control_accelerate->ad_accelerate_work_mode);
        this->vehicle_control_->set__accelerate_speed(control_accelerate->ad_speed_control);
    }
    else
    {
        this->control_velocity(control_accelerate->ad_accelerate_valid, control_accelerate->ad_accelerate_gear, control_accelerate->ad_speed_control);
    }
}

/**
 * @brief Callback for AdControlSteering subscription
 * @param control_steering Steering control message
 * @date 2025.04.23
 */
void R2C::ad_steering_subscription_cb(can_msgs::msg::AdControlSteering::SharedPtr control_steering)
{
    if (this->parameter_->vehicle_control_loop_trigger_)
    {
        this->vehicle_control_->set__accelerate_valid(this->parameter_->can_send_activate_valid_);
        this->vehicle_control_->set__steering_valid(this->parameter_->can_send_activate_valid_);
        this->vehicle_control_->set__steering_angle(control_steering->ad_steering_angle_cmd);
    }
    else
    {
        this->control_steering(control_steering->ad_steering_angle_cmd);
    }
}

/**
 * @brief Callback for AdControlBrake subscription
 * @param control_brake Brake control message
 * @date 2025.04.23
 */
void R2C::ad_control_brake_subscription_cb(can_msgs::msg::AdControlBrake::SharedPtr control_brake)
{
    this->last_brake_msg_time_ = this->system_clock_->now();
    this->is_vehicle_brake_activated_ = true;

    if (this->parameter_->vehicle_control_loop_trigger_)
    {
        this->vehicle_control_->set__accelerate_valid(this->parameter_->can_send_activate_valid_);
        this->vehicle_control_->set__brake__valid(control_brake->ad_dbs_valid);
        this->vehicle_control_->set__brake_pressure(control_brake->ad_brakepressure_cmd);
    }
    else
    {
        this->control_brake(control_brake->ad_brakepressure_cmd);
    }
}

/**
 * @brief Callback for AdControlBody subscription
 * @param control_body Body control message
 * @date 2025.04.23
 */
void R2C::ad_control_body_subscription_cb(can_msgs::msg::AdControlBody::SharedPtr control_body)
{
    this->control_body(
        control_body->ad_horn_1_control, control_body->ad_horn_2_control,
        control_body->ad_high_beam, control_body->ad_brake_light,
        control_body->ad_left_turn_light, control_body->ad_right_turn_light);
}

/**
 * @brief Send control command for acceleration (velocity control)
 * @param valid Flag indicating if control is valid
 * @param gear Gear to apply (Drive, Rear, Parking)
 * @param velocity Physical acceleration value to send
 * @date 2025.04.23
 */
void R2C::control_velocity(const int &valid, const int &gear, const double &velocity)
{
    net::wavem::can::can1::AD_Control_Accelerate ad_control_accelerate{};
    memset(&ad_control_accelerate, 0x00, CAN_MAX_DLEN);
    ad_control_accelerate.AD_Accelerate_Valid = static_cast<unsigned char>(valid);

    if (velocity != 0.0)
    {
        if (velocity > 0.0)
        {
            ad_control_accelerate.AD_Accelerate_Gear = static_cast<unsigned char>(this->parameter_->can_send_accelerate_gear_drive_);
        }
        else
        {
            ad_control_accelerate.AD_Accelerate_Gear = static_cast<unsigned char>(this->parameter_->can_send_accelerate_gear_rear_);
        }
    }
    else
    {
        ad_control_accelerate.AD_Accelerate_Gear = static_cast<unsigned char>(this->parameter_->can_send_accelerate_gear_parking_);
    }

    ad_control_accelerate.AD_Accelerate_Gear = gear;
    ad_control_accelerate.AD_Accelerate_Work_Mode = static_cast<unsigned char>(this->parameter_->can_send_accelerate_work_mode_speed_);
    ad_control_accelerate.AD_Speed_Control = static_cast<unsigned char>([this](const double &v)
                                                                        { return this->can_math_->apply_send_factor_offset(v, this->parameter_->can_send_accelerate_factor_, this->parameter_->can_send_accelerate_offset_); }(std::fabs(velocity)));

    RCUTILS_INFO(CAN_R2C,
                 "==============\n\tvalid : [%d]\n\tgear : [%d]\n\twork_mode : [%d]\n\tspeed physical : [%f]\n\tspeed raw : [%d]\n==============",
                 ad_control_accelerate.AD_Accelerate_Valid,
                 ad_control_accelerate.AD_Accelerate_Gear,
                 ad_control_accelerate.AD_Accelerate_Work_Mode,
                 velocity,
                 ad_control_accelerate.AD_Speed_Control);

    this->can_adaptor_->post_can_message<net::wavem::can::can1::AD_Control_Accelerate>(ad_control_accelerate, this->parameter_->can_send_accelerate_id_, this->can_send_channel_);
}

/**
 * @brief Send control command for steering angle
 * @param angle Desired steering angle (in degrees or raw)
 * @date 2025.04.23
 */
void R2C::control_steering(const double &angle)
{
    net::wavem::can::can1::AD_Control_Steering ad_control_steering{};
    memset(&ad_control_steering, 0x00, CAN_MAX_DLEN);
    ad_control_steering.AD_Steering_Valid = static_cast<unsigned char>(this->parameter_->can_send_activate_valid_);

    double steering_angle_cmd = angle;

    if (angle > this->parameter_->can_send_steering_max_angle_)
    {
        steering_angle_cmd = this->parameter_->can_send_steering_max_angle_;
    }
    else if (angle < this->parameter_->can_send_steering_min_angle_)
    {
        steering_angle_cmd = this->parameter_->can_send_steering_min_angle_;
    }
    else
    {
        steering_angle_cmd = steering_angle_cmd;
    }

    ad_control_steering.AD_Steering_Angle_Cmd = static_cast<unsigned short>([this](const double &v)
                                                                            { return this->can_math_->apply_send_factor_offset(v, this->parameter_->can_send_steering_factor_, this->parameter_->can_send_steering_offset_); }(steering_angle_cmd));

    RCUTILS_INFO(CAN_R2C,
                 "\n============================\n\tvalid : [%d]\n\tangle physical : [%f]\n\tangle raw : [%d]\n============================",
                 ad_control_steering.AD_Steering_Valid,
                 steering_angle_cmd,
                 ad_control_steering.AD_Steering_Angle_Cmd);

    this->can_adaptor_->post_can_message<can1::AD_Control_Steering>(ad_control_steering, this->parameter_->can_send_steering_id_, this->can_send_channel_);
}

/**
 * @brief Send control command for braking
 * @param brake Brake value (typically pressure level)
 * @date 2025.04.23
 */
void R2C::control_brake(const int &brake)
{
    net::wavem::can::can1::AD_Control_Brake ad_control_brake{};
    memset(&ad_control_brake, 0x00, CAN_MAX_DLEN);
    ad_control_brake.AD_DBS_Valid = static_cast<unsigned char>(this->parameter_->can_send_activate_valid_);
    ad_control_brake.AD_BrakePressure_Cmd = static_cast<unsigned char>([this](const double &v)
                                                                       { return this->can_math_->apply_send_factor_offset(v, this->parameter_->can_send_brake_factor_, this->parameter_->can_send_brake_offset_); }(brake));

    RCUTILS_INFO(CAN_R2C,
                 "\n============================\n\tvalid : [%d]\n\tpressure physical : [%d]\n\tpressure raw : [%d]\n============================",
                 ad_control_brake.AD_DBS_Valid,
                 brake,
                 ad_control_brake.AD_BrakePressure_Cmd);

    this->can_adaptor_->post_can_message<net::wavem::can::can1::AD_Control_Brake>(ad_control_brake, this->parameter_->can_send_brake_id_, this->can_send_channel_);
}

/**
 * @brief Send control command for vehicle body features
 * @param horn1 Horn 1 control (true/false)
 * @param horn2 Horn 2 control (true/false)
 * @param high_beam High beam light toggle
 * @param brake_light Brake light toggle
 * @param left_turn_light Left turn signal toggle
 * @param right_turn_light Right turn signal toggle
 * @date 2025.04.23
 */
void R2C::control_body(
    const bool &horn1, const bool &horn2,
    const bool &high_beam, const bool &brake_light,
    const bool &left_turn_light, const bool &right_turn_light)
{
    net::wavem::can::can1::AD_Control_Body ad_control_body{};
    memset(&ad_control_body, 0x00, CAN_MAX_DLEN);
    ad_control_body.AD_Body_Valid = static_cast<unsigned char>(this->parameter_->can_send_activate_valid_);
    ad_control_body.reserved = 0;
    ad_control_body.AD_Brake_Light = brake_light ? 1 : 0;
    ad_control_body.AD_Reversing_Lights = 0;
    ad_control_body.AD_Low_Beam = 0;
    ad_control_body.AD_Left_Turn_Light = left_turn_light ? 1 : 0;
    ad_control_body.AD_Right_Turn_Light = right_turn_light ? 1 : 0;
    ad_control_body.AD_Horn_1_Control = horn1 ? 1 : 0;
    ad_control_body.AD_High_Beam = high_beam ? 1 : 0;
    ad_control_body.AD_Fog_Light = 0;
    ad_control_body.AD_ADS_Light = 0;
    ad_control_body.reserved2 = 0;
    ad_control_body.AD_Horn_2_Control = horn2 ? 1 : 0;
    ad_control_body.reserved3 = 0;
    unsigned char temp_array[4] = {0, 0, 0, 0};
    memcpy(ad_control_body.reserved4, temp_array, sizeof(ad_control_body.reserved4));
    // this->can_adaptor_->post_can_message<net::wavem::can::can1::AD_Control_Body>(ad_control_body, this->parameter_->can_send_contro, this->can_send_channel_);
}

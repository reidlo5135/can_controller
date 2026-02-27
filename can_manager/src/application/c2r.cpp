#include "application/c2r.hpp"

using namespace net::wavem::can;

/**
 * @brief Construct a new C2R object
 * @param node ROS2 node shared pointer
 * @param parameter Shared pointer to configuration parameters
 * @param can_adaptor Shared pointer to CAN adaptor
 * @date 2025.04.23
 */
C2R::C2R(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Adaptor::SharedPtr &can_adaptor)
    : node_(node),
      parameter_(parameter),
      can_adaptor_(can_adaptor)
{
    if (this->node_ == nullptr)
    {
        RCUTILS_ERROR(CAN_C2R, "node pointer is nullptr...");
        return;
    }

    this->can_dump_channel_ = this->parameter_->can_dump_channel_;
    this->can_send_channel_ = this->parameter_->can_send_channel_;
    this->vehicle_status_ = std::make_shared<can_msgs::msg::VehicleStatus>();

    this->vehicle_status_publish_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->vehicle_status_publish_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(this->parameter_->vehicle_status_loop_rate_),
        std::bind(&C2R::vehicle_status_publish_timer_cb, this),
        this->vehicle_status_publish_timer_cb_group_);

    this->vehicle_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions vehicle_status_publisher_opts;
    vehicle_status_publisher_opts.callback_group = this->vehicle_status_publisher_cb_group_;
    this->vehicle_status_publisher_ = this->node_->create_publisher<can_msgs::msg::VehicleStatus>(
        this->parameter_->vehicle_status_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        vehicle_status_publisher_opts);

    this->set_up_dump_handlers();
}

/**
 * @brief Destroy the C2R object
 * @date 2025.04.23
 */
C2R::~C2R() = default;

/**
 * @brief Set up CAN dump handlers
 * @date 2025.04.23
 */
void C2R::set_up_dump_handlers()
{
    this->can_adaptor_->set_handler<C2R>(this, &C2R::mcu_status_2_cb, this->parameter_->can_dump_rpm_id_, this->can_dump_channel_);
    this->can_adaptor_->set_handler<C2R>(this, &C2R::bms_pack_status_cb, this->parameter_->can_dump_bms_id_, this->can_dump_channel_);
    this->can_adaptor_->set_handler<C2R>(this, &C2R::mcu_general_status_cb, this->parameter_->can_dump_gear_id_, this->can_dump_channel_);
    this->can_adaptor_->set_handler<C2R>(this, &C2R::vehicle_odometer_status_cb, this->parameter_->can_dump_odometer_id_, this->can_send_channel_);
}

/**
 * @brief Register internal callback bindings for CAN data processing
 * @date 2025.04.23
 */
void C2R::register_can_dump_callbacks()
{
    this->register_mcu_status_2_cb<C2R>(this, &C2R::can_mcu_status_2_cb);
    RCUTILS_INFO(CAN_C2R, "MCUStatus2Callback registered");

    this->register_bms_pack_status_cb<C2R>(this, &C2R::can_bms_pack_status_cb);
    RCUTILS_INFO(CAN_C2R, "BMSPackStatusCallback registered");

    this->register_mcu_general_status_cb<C2R>(this, &C2R::can_mcu_general_status_cb);
    RCUTILS_INFO(CAN_C2R, "MCUGeneralStatusCallback registered");

    this->register_vehicle_odometer_status_cb<C2R>(this, &C2R::can_vehicle_odometer_status_cb);
    RCUTILS_INFO(CAN_C2R, "VehicleOdometerStatusCallback registered");
}

void C2R::vehicle_status_publish_timer_cb()
{
    this->vehicle_status_publisher_->publish(*this->vehicle_status_);
}

/**
 * @brief Callback when receiving MCUStatus2 data from CAN
 * @param mcu_status_2 MCU status 2 structure
 * @date 2025.04.23
 */
void C2R::mcu_status_2_cb(can2::MCU_Status_2 mcu_status_2)
{
    this->mcu_status_2_func_(
        static_cast<short>(this->can_math_->to_big_endian(mcu_status_2.MCU_Udc)),
        static_cast<short>(this->can_math_->to_big_endian(mcu_status_2.MCU_Idc)),
        this->can_math_->to_big_endian(mcu_status_2.MCU_Temp),
        this->can_math_->to_big_endian(mcu_status_2.Motor_Temp),
        static_cast<short>(this->can_math_->to_big_endian(mcu_status_2.Motor_speed)));
}

/**
 * @brief Callback when receiving BMSPackStatus data from CAN
 * @param bms_pack_status BMS pack status structure
 * @date 2025.04.23
 */
void C2R::bms_pack_status_cb(can2::BMS_Pack_Status bms_pack_status)
{
    this->bms_pack_status_func_(
        static_cast<short>(this->can_math_->to_big_endian(bms_pack_status.Total_voltage__battery_pack)),
        static_cast<short>(this->can_math_->to_big_endian(bms_pack_status.Total_current__battery_pack)),
        bms_pack_status.BMS_SOC,
        bms_pack_status.BMS_SOH,
        this->can_math_->to_big_endian(bms_pack_status.Remaining_range),
        this->can_math_->to_big_endian(bms_pack_status.BMS_Pack_Status_56));
}

/**
 * @brief Callback when receiving MCUGeneralStatus data from CAN
 * @param mcu_general_status MCU general status structure
 * @date 2025.04.23
 */
void C2R::mcu_general_status_cb(can2::MCU_General_Status mcu_general_status)
{
    this->mcu_general_status_func_(
        mcu_general_status.MCU_Workmode,
        mcu_general_status.Motor_high_voltage_contactor_status,
        mcu_general_status.MCU_FaultGrade,
        mcu_general_status.Speed_State,
        mcu_general_status.MCU_General_Status_8,
        mcu_general_status.MCU_Gear,
        mcu_general_status.Motor_system_driving_mode,
        mcu_general_status.MCU_Control_mode,
        mcu_general_status.MCU_Ready,
        mcu_general_status.MCU_Acc_pedal,
        mcu_general_status.MCU_Acc_Fault_State,
        mcu_general_status.MCU_Break_pedal,
        mcu_general_status.MCU_Break_Fault_State,
        mcu_general_status.MCU_FaultCode,
        mcu_general_status.Charging_anti_driving,
        mcu_general_status.Controller_cooling_fan_status,
        mcu_general_status.MCU_General_Status_51,
        mcu_general_status.Heat_Count);
}

void C2R::vehicle_odometer_status_cb(can1::Vehicle_Odometer_Status vehicle_odometer_status)
{
    this->vehicle_odometer_status_func_(
        vehicle_odometer_status.Vehicle_ODO,
        vehicle_odometer_status.Vehicle_TRIP,
        vehicle_odometer_status.reserved,
        vehicle_odometer_status.Vehicle_AD_Mileage,
        vehicle_odometer_status.Vehicle_Odometer_MsgCntr);
}

/**
 * @brief Publishes MCU status data to ROS after decoding CAN values
 * @param mcu_udc MCU voltage
 * @param mcu_idc MCU current
 * @param mcu_temp MCU temperature
 * @param motor_temp Motor temperature
 * @param motor_speed Motor RPM
 * @date 2025.04.23
 */
void C2R::can_mcu_status_2_cb(
    short mcu_udc, short mcu_idc,
    char mcu_temp, char motor_temp, short motor_speed)
{
    const double &rpm = static_cast<double>(this->can_math_->apply_dump_factor_offset<uint16_t>(motor_speed, this->parameter_->can_dump_rpm_factor_, this->parameter_->can_dump_rpm_offset_));
    this->vehicle_status_->set__rpm(rpm);

    const double &circum = M_PI * 2 * this->parameter_->vehicle_wheel_size_;
    const double &speed_p_m = rpm * circum;
    const double &speed_p_h = (speed_p_m * 60) / 1000;
    const double &speed_corrected_p_h = speed_p_h / this->parameter_->can_dump_rpm_correction_;
    this->vehicle_status_->set__speed(speed_corrected_p_h);
}

/**
 * @brief Publishes BMS pack status to ROS after decoding CAN values
 * @param total_voltage_battery_pack Battery voltage
 * @param total_current_battery_pack Battery current
 * @param bms_soc Battery state of charge
 * @param bms_soh Battery state of health
 * @param remaining_range Estimated range
 * @param bms_pack_status_56 Additional status flags
 * @date 2025.04.23
 */
void C2R::can_bms_pack_status_cb(
    short total_voltage_battery_pack, short total_current_battery_pack,
    char bms_soc, char bms_soh,
    char remaining_range, char bms_pack_status_56)
{
    const double &battery = this->can_math_->apply_dump_factor_offset<double>(bms_soc, this->parameter_->can_dump_bms_factor_, this->parameter_->can_dump_bms_offset_);
    this->vehicle_status_->set__battery(battery);
}

/**
 * @brief Publishes MCU general status to ROS after decoding CAN values
 * @param mcu_work_mode Operation mode
 * @param motor_high_voltage_contactor_status High voltage contactor state
 * @param mcu_fault_grade Fault severity
 * @param speed_state Speed flag
 * @param mcu_general_status_8 General status byte 8
 * @param mcu_gear Current gear status
 * @param motor_system_driving_mode Driving mode
 * @param mcu_control_mode Control mode
 * @param mcu_ready Ready flag
 * @param mcu_acc_pedal Accelerator pedal value
 * @param mcu_acc_fault_state Accelerator fault
 * @param mcu_break_pedal Brake pedal value
 * @param mcu_break_fault_state Brake fault
 * @param mcu_fault_code Fault code
 * @param charging_anti_driving Charge lock indicator
 * @param controller_cooling_fan_status Fan status
 * @param mcu_general_status_51 General status byte 51
 * @param heat_count Heat-related metric
 * @date 2025.04.23
 */
void C2R::can_mcu_general_status_cb(
    char mcu_work_mode, char motor_high_voltage_contactor_status, char mcu_fault_grade,
    char speed_state, char mcu_general_status_8, char mcu_gear,
    char motor_system_driving_mode, char mcu_control_mode, char mcu_ready,
    char mcu_acc_pedal, char mcu_acc_fault_state, char mcu_break_pedal,
    char mcu_break_fault_state, char mcu_fault_code, char charging_anti_driving,
    char controller_cooling_fan_status, char mcu_general_status_51, char heat_count)
{
    const uint8_t &gear = this->can_math_->apply_dump_factor_offset<uint8_t>(mcu_gear, this->parameter_->can_dump_gear_factor_, this->parameter_->can_dump_gear_offset_);
    this->vehicle_status_->set__gear(gear);
}

void C2R::can_vehicle_odometer_status_cb(long long vehicle_odo, long long vehicle_trip, long long reserved,
                                         long long vehicle_ad_mileage, long long vehicle_odometer_msgcntr)
{
    can_msgs::msg::VehicleOdometerStatus::UniquePtr vehicle_odometer_status = std::make_unique<can_msgs::msg::VehicleOdometerStatus>();
    vehicle_odometer_status->set__vehicle_odo(
        this->can_math_->apply_dump_factor_offset<unsigned long long>(vehicle_odo, this->parameter_->can_dump_odometer_factor_, this->parameter_->can_dump_odometer_offset_));
    vehicle_odometer_status->set__vehicle_trip(
        this->can_math_->apply_dump_factor_offset<unsigned long long>(vehicle_trip, this->parameter_->can_dump_odometer_factor_, this->parameter_->can_dump_odometer_offset_));
    vehicle_odometer_status->set__vehicle_ad_mileage(
        this->can_math_->apply_dump_factor_offset<unsigned long long>(vehicle_ad_mileage, this->parameter_->can_dump_odometer_factor_, this->parameter_->can_dump_odometer_offset_));
    vehicle_odometer_status->set__vehicle_odometer_msg_cntr(
        this->can_math_->apply_dump_factor_offset<unsigned long long>(vehicle_odometer_msgcntr, this->parameter_->can_dump_odometer_factor_, this->parameter_->can_dump_odometer_offset_));

    this->vehicle_status_->set__can_odometry(std::move(*vehicle_odometer_status));
}
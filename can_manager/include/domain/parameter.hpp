#ifndef DOMAIN_PARAMETER_HPP
#define DOMAIN_PARAMETER_HPP

#include <string>
#include <memory>

#define PARAM_CAN_SEND_CHANNEL "can.send.channel"
#define PARAM_CAN_SEND_ACTIVATE_VALID "can.send.activate_valid"
#define PARAM_CAN_SEND_ACCELERATE "can.send.accelerate"
#define PARAM_CAN_SEND_STEERING "can.send.steering"
#define PARAM_CAN_SEND_BRAKE "can.send.brake"
#define PARAM_CAN_SEND_BODY "can.send.body"

#define PARAM_CAN_DUMP_CHANNEL "can.dump.channel"
#define PARAM_CAN_DUMP_RPM "can.dump.rpm"
#define PARAM_CAN_DUMP_BMS "can.dump.bms"
#define PARAM_CAN_DUMP_GEAR "can.dump.gear"
#define PARAM_CAN_DUMP_ODOMETRY "can.dump.odometry"

#define PARAM_VEHICLE_WHEEL "vehicle.wheel"
#define PARAM_VEHICLE_CONTROL "vehicle.control"
#define PARAM_VEHICLE_STATUS "vehicle.status"

#define PARAM_NAME "name"
#define PARAM_TOPIC "topic"
#define PARAM_ID "id"
#define PARAM_FIELD "field"
#define PARAM_FACTOR "factor"
#define PARAM_OFFSET "offset"
#define PARAM_MAX_SPEED "max_speed"
#define PARAM_MIN_SPEED "min_speed"
#define PARAM_MAX_ANGLE "max_angle"
#define PARAM_MIN_ANGLE "min_angle"
#define PARAM_WORK_MODE "work_mode"
#define PARAM_WORK_MODE_TORQUE "torque"
#define PARAM_WORK_MODE_SPEED "speed"
#define PARAM_WORK_MODE_ACCELERATE "accelerate"
#define PARAM_GEAR "gear"
#define PARAM_BRAKE_PRESS "press"
#define PARAM_BRAKE_RELEASE "release"
#define PARAM_GEAR_PARKING "parking"
#define PARAM_GEAR_DRIVE "drive"
#define PARAM_GEAR_NEUTRAL "neutral"
#define PARAM_GEAR_REAR "rear"
#define PARAM_MAX_VALUE "max_value"
#define PARAM_MIN_VALUE "min_value"

#define PARAM_VEHICLE_WHEEL_SIZE "vehicle.wheel"

namespace net::wavem::can
{
    class Parameter final
    {
    public:
        explicit Parameter() = default;
        virtual ~Parameter() = default;

        // Send Start
        std::string can_send_channel_;
        int can_send_activate_valid_ = 0;

        std::string can_send_accelerate_name_;
        std::string can_send_accelerate_topic_;
        int can_send_accelerate_id_;
        std::string can_send_accelerate_field_;
        double can_send_accelerate_factor_ = 0.0;
        double can_send_accelerate_offset_ = 0.0;
        double can_send_accelerate_max_speed_ = 0.0;
        double can_send_accelerate_min_speed_ = 0.0;
        int can_send_accelerate_work_mode_torque_ = 0;
        int can_send_accelerate_work_mode_speed_ = 0;
        int can_send_accelerate_work_mode_accelerate_ = 0;
        int can_send_accelerate_gear_parking_ = 0;
        int can_send_accelerate_gear_drive_ = 0;
        int can_send_accelerate_gear_neutral_ = 0;
        int can_send_accelerate_gear_rear_ = 0;

        std::string can_send_steering_name_;
        std::string can_send_steering_topic_;
        int can_send_steering_id_;
        std::string can_send_steering_field_;
        double can_send_steering_factor_ = 0.0;
        double can_send_steering_offset_ = 0.0;
        double can_send_steering_max_angle_ = 0.0;
        double can_send_steering_min_angle_ = 0.0;

        std::string can_send_brake_name_;
        std::string can_send_brake_topic_;
        int can_send_brake_id_;
        std::string can_send_brake_field_;
        double can_send_brake_factor_ = 0.0;
        double can_send_brake_offset_ = 0.0;
        int can_send_brake_press_ = 0;
        int can_send_brake_release_ = 0;

        std::string can_send_body_name_;
        std::string can_send_body_topic_;
        int can_send_body_id_;
        std::string can_send_body_field_;
        double can_send_body_factor_ = 0.0;
        double can_send_body_offset_ = 0.0;
        int can_send_body_max_value_ = 0;
        int can_send_body_min_value_ = 0;
        // Send End

        // Dump Start
        std::string can_dump_channel_;
        std::string can_dump_rpm_name_;
        std::string can_dump_rpm_topic_;
        int can_dump_rpm_id_;
        std::string can_dump_rpm_field_;
        double can_dump_rpm_factor_ = 0.0;
        double can_dump_rpm_offset_ = 0.0;
        double can_dump_rpm_correction_ = 0.0;

        std::string can_dump_bms_name_;
        std::string can_dump_bms_topic_;
        int can_dump_bms_id_;
        std::string can_dump_bms_field_;
        double can_dump_bms_factor_ = 0.0;
        double can_dump_bms_offset_ = 0.0;

        std::string can_dump_gear_name_;
        std::string can_dump_gear_topic_;
        int can_dump_gear_id_;
        std::string can_dump_gear_field_;
        double can_dump_gear_factor_;
        double can_dump_gear_offset_;
        int can_dump_gear_parking_;
        int can_dump_gear_drive_;
        int can_dump_gear_neutral_;
        int can_dump_gear_rear_;

        std::string can_dump_odometer_name_;
        std::string can_dump_odometer_topic_;
        int can_dump_odometer_id_;
        double can_dump_odometer_factor_;
        double can_dump_odometer_offset_;
        // Dump End

        double vehicle_wheel_size_ = 0.0;
        double vehicle_wheel_base_ = 0.0;

        bool vehicle_control_loop_trigger_ = false;
        int vehicle_control_loop_rate_ = 0;

        std::string vehicle_status_topic_;
        int vehicle_status_loop_rate_ = 0;

    public:
        using SharedPtr = std::shared_ptr<Parameter>;
    };
}

#endif // DOMAIN_PARAMETER_HPP
#ifndef DOMAIN_PARAMETER_HPP
#define DOMAIN_PARAMETER_HPP

#include <memory>
#include <rcutils/logging_macros.h>

#define PARAM_VEHICLE_CONTROL "vehicle.control"
#define PARAM_VEHICLE_STATUS "vehicle.status"
#define PARAM_VEHICLE_DECELERATION "vehicle.deceleration"
#define PARAM_VEHICLE_MCU "vehicle.mcu"
#define PARAM_VEHICLE_WHEEL "vehicle.wheel"
#define PARAM_MAP "map"

namespace net::wavem::drive
{
    class Parameter final
    {
    public:
        explicit Parameter() = default;
        virtual ~Parameter() = default;

        std::string vehicle_status_topic_;
        std::string vehicle_control_twist_topic_;
        double vehicle_control_twist_correction_linear_ = 0.0;
        double vehicle_control_twist_correction_angular_ = 0.0;
        double vehicle_control_twist_limit_linear_ = 0.0;
        double vehicle_control_twist_limit_angular_ = 0.0;

        std::string vehicle_control_accelerate_topic_;
        double vehicle_control_accelerate_limit_max_ = 0.0;
        double vehicle_control_accelerate_limit_min_ = 0.0;
        int vehicle_control_accelerate_pid_rate_ = 0;
        double vehicle_control_accelerate_pid_dt_ = 0.0;
        double vehicle_control_accelerate_pid_kp_ = 0.0;
        double vehicle_control_accelerate_pid_ki_ = 0.0;
        double vehicle_control_accelerate_pid_kd_ = 0.0;
        double vehicle_control_accelerate_pid_integral_ = 0.0;
        double vehicle_control_accelerate_pid_previous_error_ = 0.0;

        std::string vehicle_control_steering_topic_;
        double vehicle_control_steering_limit_ = 0.0;

        std::string vehicle_control_brake_topic_;
        int vehicle_control_brake_limit_ = 0;

        std::string vehicle_control_emergency_topic_;
        bool vehicle_control_emergency_trigger_;

        std::string vehicle_control_deceleration_status_topic_;
        std::string vehicle_control_deceleration_activate_topic_;
        bool vehicle_control_deceleration_trigger_;

        std::string vehicle_mcu_rpm_topic_;
        double vehicle_mcu_rpm_correction_ = 0.0;

        std::string vehicle_mcu_gear_topic_;
        int vehicle_mcu_gear_position_parking_ = 0;
        int vehicle_mcu_gear_position_drive_ = 0;
        int vehicle_mcu_gear_position_neutral_ = 0;
        int vehicle_mcu_gear_position_rear_ = 0;
        double vehicle_mcu_gear_ratio_ = 0.0;

        int vehicle_heartbeat_activate_ = 0;
        int vehicle_heartbeat_deactivate_ = 0;
        int vehicle_heartbeat_retry_ = 0;

        double vehicle_wheel_size_ = 0.0;
        double vehicle_wheel_base_ = 0.0;

        std::string map_topic_;
        int map_area_geo_fence_ = 0;
        int map_area_start_ = 0;
        int map_area_end_ = 0;
        int map_area_fair_ = 0;
        int map_area_road_ = 0;

    public:
        using SharedPtr = std::shared_ptr<Parameter>;
    };
}

#endif // DOMAIN_PARAMETER_HPP

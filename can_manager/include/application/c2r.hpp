/**
 * @file c2r.hpp
 * @brief CAN to ROS Publisher Module
 * @author reidlo
 * @date 2025.04.23
 */

#ifndef APPLICATION_C2R_HPP
#define APPLICATION_C2R_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_utils/adaptor.hpp>
#include <can_utils/logger.hpp>
#include <can_utils/math.hpp>
#include <can_dbc/dbc1.hpp>
#include <can_dbc/dbc2.hpp>
#include <can_msgs/msg/bms_charge_status.hpp>
#include <can_msgs/msg/bms_error.hpp>
#include <can_msgs/msg/bms_pack_status.hpp>
#include <can_msgs/msg/emergency.hpp>
#include <can_msgs/msg/mcu_general_status.hpp>
#include <can_msgs/msg/mcu_request.hpp>
#include <can_msgs/msg/mcu_status2.hpp>
#include <can_msgs/msg/vehicle_odometer_status.hpp>
#include <can_msgs/msg/vehicle_status.hpp>

#include "domain/parameter.hpp"

#define CAN_C2R "C2R"

using std::placeholders::_1;
using std::placeholders::_10;
using std::placeholders::_11;
using std::placeholders::_12;
using std::placeholders::_13;
using std::placeholders::_14;
using std::placeholders::_15;
using std::placeholders::_16;
using std::placeholders::_17;
using std::placeholders::_18;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::placeholders::_8;
using std::placeholders::_9;

namespace net::wavem::can
{
    /**
     * @class C2R
     * @brief A class that converts CAN data into ROS messages and publishes them.
     * @author reidlo
     * @date 2025.04.23
     */
    class C2R final
    {
    private:
        rclcpp::Node::SharedPtr node_;
        Parameter::SharedPtr parameter_;
        can_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;

        rclcpp::CallbackGroup::SharedPtr vehicle_status_publish_timer_cb_group_;
        rclcpp::TimerBase::SharedPtr vehicle_status_publish_timer_;
        void vehicle_status_publish_timer_cb();

        rclcpp::CallbackGroup::SharedPtr vehicle_status_publisher_cb_group_;
        rclcpp::Publisher<can_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_publisher_;

    private:
        std::string can_dump_channel_;
        std::string can_send_channel_;
        Adaptor::SharedPtr can_adaptor_;
        Math::SharedPtr can_math_;
        void set_up_dump_handlers();

        typedef std::function<void(short, short, char, char, short)> mcu_status_2_func;
        typedef std::function<void(short, short, char, char, char, char)> bms_pack_status_func;
        typedef std::function<void(
            char, char, char, char, char, char,
            char, char, char, char, char, char,
            char, char, char, char, char, char)>
            mcu_general_status_func;
        typedef std::function<void(long long, long long, long long, long long, long long)> vehicle_odometer_status_func;

        /**
         * @brief Callback function for receiving MCUStatus2 data from CAN.
         * @date 2025.04.23
         */
        mcu_status_2_func mcu_status_2_func_;

        /**
         * @brief Callback function for receiving BMSPackStatus data from CAN.
         * @date 2025.04.23
         */
        bms_pack_status_func bms_pack_status_func_;

        /**
         * @brief Callback function for receiving MCUGeneralStatus data from CAN.
         * @date 2025.04.23
         */
        mcu_general_status_func mcu_general_status_func_;

        vehicle_odometer_status_func vehicle_odometer_status_func_;

        void mcu_status_2_cb(can2::MCU_Status_2 mcu_status_2);
        void bms_pack_status_cb(can2::BMS_Pack_Status bms_pack_status);
        void mcu_general_status_cb(can2::MCU_General_Status mcu_general_status);
        void vehicle_odometer_status_cb(can1::Vehicle_Odometer_Status vehicle_odometer_status);

        void can_mcu_status_2_cb(
            short mcu_udc, short mcu_idc,
            char mcu_temp, char motor_temp, short motor_speed);
        void can_bms_pack_status_cb(
            short voltage_battery_pack, short current_battery_pack,
            char bms_soc, char bms_soh,
            char remaining_range, char bms_pack_status_56);
        void can_mcu_general_status_cb(
            char mcu_work_mode, char motor_high_voltage_contactor_status, char mcu_fault_grade,
            char speed_state, char mcu_general_general_status_8, char mcu_gear,
            char motor_system_driving_mode, char mcu_control_mode, char mcu_ready,
            char mcu_acc_pedal, char mcu_acc_fault_state, char mcu_break_pedal,
            char mcu_break_fault_state, char mcu_fault_code, char charging_anti_driving,
            char controller_cooling_fan_status, char mcu_general_status, char heat_count);
        void can_vehicle_odometer_status_cb(
            long long vehicle_odo, long long vehicle_trip, long long reserved,
            long long vehicle_ad_mileage, long long vehicle_odometer_msgcntr);

        /**
         * @brief Register user callback for MCUStatus2 data.
         * @tparam T Class type
         * @param p_class_type Class instance pointer
         * @param p_func Callback function pointer
         * @date 2025.04.23
         */
        template <typename T>
        void register_mcu_status_2_cb(T *p_class_type, void (T::*p_func)(short, short, char, char, short))
        {
            this->mcu_status_2_func_ = std::move(std::bind(p_func, p_class_type, _1, _2, _3, _4, _5));
        }

        /**
         * @brief Register user callback for BMSPackStatus data.
         * @tparam T Class type
         * @param p_class_type Class instance pointer
         * @param p_func Callback function pointer
         * @date 2025.04.23
         */
        template <typename T>
        void register_bms_pack_status_cb(T *p_class_type, void (T::*p_func)(short, short, char, char, char, char))
        {
            this->bms_pack_status_func_ = std::move(std::bind(p_func, p_class_type, _1, _2, _3, _4, _5, _6));
        }

        /**
         * @brief Register user callback for MCUGeneralStatus data.
         * @tparam T Class type
         * @param p_class_type Class instance pointer
         * @param p_func Callback function pointer
         * @date 2025.04.23
         */
        template <typename T>
        void register_mcu_general_status_cb(T *p_class_type, void (T::*p_func)(
                                                                 char, char, char, char, char, char,
                                                                 char, char, char, char, char, char,
                                                                 char, char, char, char, char, char))
        {
            this->mcu_general_status_func_ = std::move(std::bind(p_func, p_class_type,
                                                                 _1, _2, _3, _4, _5, _6,
                                                                 _7, _8, _9, _10, _11, _12,
                                                                 _13, _14, _15, _16, _17, _18));
        }

        template <typename T>
        void register_vehicle_odometer_status_cb(T *p_class_type, void (T::*p_func)(
                                                                      long long, long long, long long, long long, long long))
        {
            this->vehicle_odometer_status_func_ = std::move(std::bind(p_func, p_class_type,
                                                                      _1, _2, _3, _4, _5));
        }

    public:
        /**
         * @brief Constructor of C2R.
         * @param node ROS2 node pointer
         * @param parameter Shared pointer to parameters
         * @param can_adaptor Shared pointer to CAN adaptor
         * @date 2025.04.23
         */
        explicit C2R(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Adaptor::SharedPtr &can_adaptor);

        /**
         * @brief Destructor of C2R.
         * @date 2025.04.23
         */
        virtual ~C2R();

        /**
         * @brief Register all CAN data dump callback bindings.
         * @date 2025.04.23
         */
        void register_can_dump_callbacks();

    public:
        using SharedPtr = std::shared_ptr<C2R>;
    };
}

#endif

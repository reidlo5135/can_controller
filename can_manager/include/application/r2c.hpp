/**
 * @file r2c.hpp
 * @brief ROS to CAN Sender Module
 * @author reidlo
 * @date 2025.04.23
 */

#ifndef APPLICATION_R2C_HPP
#define APPLICATION_R2C_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_utils/adaptor.hpp>
#include <can_utils/logger.hpp>
#include <can_utils/math.hpp>
#include <can_dbc/dbc1.hpp>
#include <can_dbc/dbc2.hpp>
#include <can_msgs/msg/ad_control_body.hpp>
#include <can_msgs/msg/ad_control_accelerate.hpp>
#include <can_msgs/msg/ad_control_brake.hpp>
#include <can_msgs/msg/ad_control_steering.hpp>
#include <can_msgs/msg/bms_charge_status.hpp>
#include <can_msgs/msg/bms_error.hpp>
#include <can_msgs/msg/bms_pack_status.hpp>
#include <can_msgs/msg/emergency.hpp>
#include <can_msgs/msg/mcu_general_status.hpp>
#include <can_msgs/msg/mcu_request.hpp>
#include <can_msgs/msg/mcu_status2.hpp>
#include <can_msgs/msg/vehicle_odometer_status.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "domain/parameter.hpp"
#include "domain/vehicle.hpp"

#define CAN_R2C "R2C"

namespace net::wavem::can
{
    /**
     * @class R2C
     * @brief A class that listens to ROS messages and sends appropriate CAN signals.
     * @author reidlo
     * @date 2025.04.23
     */
    class R2C final
    {
    private:
        rclcpp::Node::SharedPtr node_;       ///< ROS2 node handle
        Parameter::SharedPtr parameter_;     ///< Parameter configuration
        Control::SharedPtr vehicle_control_; ///< Vehicle Control domain

        std::string can_send_channel_;   ///< CAN channel name to send messages
        Adaptor::SharedPtr can_adaptor_; ///< CAN adaptor instance
        Math::SharedPtr can_math_;       ///< Math utility instance for factor/offset

        rclcpp::Clock::SharedPtr system_clock_;
        bool is_vehicle_control_activated_;
        bool is_vehicle_brake_activated_;

        rclcpp::Time last_accelerate_msg_time_;
        rclcpp::CallbackGroup::SharedPtr vehicle_control_watchdog_timer_cb_group_;
        rclcpp::TimerBase::SharedPtr vehicle_control_watchdog_timer_;
        void vehicle_control_watchdog_timer_cb();

        rclcpp::Time last_brake_msg_time_;
        rclcpp::CallbackGroup::SharedPtr vehicle_brake_watchdog_timer_cb_group_;
        rclcpp::TimerBase::SharedPtr vehicle_brake_watchdog_timer_;
        void vehicle_brake_wathcog_timer_cb();

        rclcpp::CallbackGroup::SharedPtr vehicle_control_loop_timer_cb_group_;
        rclcpp::TimerBase::SharedPtr vehicle_control_loop_timer_;
        void vehicle_control_loop_timer_cb();

        // Subscriptions and Callbacks
        rclcpp::CallbackGroup::SharedPtr ad_accelerate_subscription_cb_group_;
        rclcpp::Subscription<can_msgs::msg::AdControlAccelerate>::SharedPtr ad_accelerate_subscription_;

        /**
         * @brief Callback for receiving acceleration command
         * @param control_accelerate Incoming acceleration message
         * @date 2025.04.23
         */
        void ad_accelerate_subscription_cb(can_msgs::msg::AdControlAccelerate::SharedPtr control_accelerate);

        rclcpp::CallbackGroup::SharedPtr ad_steering_subscription_cb_group_;
        rclcpp::Subscription<can_msgs::msg::AdControlSteering>::SharedPtr ad_steering_subscription_;

        /**
         * @brief Callback for receiving steering command
         * @param control_steering Incoming steering message
         * @date 2025.04.23
         */
        void ad_steering_subscription_cb(can_msgs::msg::AdControlSteering::SharedPtr control_steering);

        rclcpp::CallbackGroup::SharedPtr ad_control_brake_subscription_cb_group_;
        rclcpp::Subscription<can_msgs::msg::AdControlBrake>::SharedPtr ad_control_brake_subscription_;

        /**
         * @brief Callback for receiving brake command
         * @param control_brake Incoming brake message
         * @date 2025.04.23
         */
        void ad_control_brake_subscription_cb(can_msgs::msg::AdControlBrake::SharedPtr control_brake);

        rclcpp::CallbackGroup::SharedPtr ad_control_body_subscription_cb_group_;
        rclcpp::Subscription<can_msgs::msg::AdControlBody>::SharedPtr ad_control_body_subscription_;

        /**
         * @brief Callback for receiving body control command (lights, horn)
         * @param control_body Incoming body control message
         * @date 2025.04.23
         */
        void ad_control_body_subscription_cb(can_msgs::msg::AdControlBody::SharedPtr control_body);

        // Internal control methods

        /**
         * @brief Sends velocity control signal via CAN
         * @param valid Valid flag
         * @param gear Current gear
         * @param velocity Desired velocity
         * @date 2025.04.23
         */
        void control_velocity(const int &valid, const int &gear, const double &velocity);

        /**
         * @brief Sends steering control signal via CAN
         * @param angle Desired steering angle
         * @date 2025.04.23
         */
        void control_steering(const double &angle);

        /**
         * @brief Sends brake control signal via CAN
         * @param brake Desired brake value
         * @date 2025.04.23
         */
        void control_brake(const int &brake);

        /**
         * @brief Sends miscellaneous body signals via CAN
         * @param horn1 Horn 1 activation
         * @param horn2 Horn 2 activation
         * @param high_beam High beam light
         * @param brake_light Brake light activation
         * @param left_turn_light Left turn signal
         * @param right_turn_light Right turn signal
         * @date 2025.04.23
         */
        void control_body(
            const bool &horn1, const bool &horn2,
            const bool &high_beam, const bool &brake_light,
            const bool &left_turn_light, const bool &right_turn_light);

    public:
        /**
         * @brief Construct a new R2C object
         * @param node ROS2 node handle
         * @param parameter Shared pointer to configuration
         * @param can_adaptor Shared pointer to CAN adaptor
         * @date 2025.04.23
         */
        explicit R2C(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Adaptor::SharedPtr &can_adaptor);

        /**
         * @brief Destroy the R2C object
         * @date 2025.04.23
         */
        virtual ~R2C();

        using SharedPtr = std::shared_ptr<R2C>;
    };
}

#endif
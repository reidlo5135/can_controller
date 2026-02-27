#ifndef APPLICATION_MOTION_HPP
#define APPLICATION_MOTION_HPP

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/vehicle_status.hpp>
#include <map_handle_msgs/msg/position.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_utils/logger.hpp>

#include "domain/parameter.hpp"
#include "domain/vehicle.hpp"
#include "application/converter.hpp"
#include "application/control.hpp"

#define MISSION_TYPE_SUBSCRIPTION_TOPIC "/devices/mission/type"

#define MCU_GEAR_DRIVE 3
#define MCU_GEAR_NEUTRAL 2
#define MCU_GEAR_REAR 1

#define DECELERATION_STATUS_NEAR_TO_GEO_FENCE 0
#define DECELERATION_STATUS_START 1
#define DECELERATION_STATUS_END 2
#define DECELERATION_STATUS_DRIVE_GEAR -1
#define DECELERATION_STATUS_RELEASE 3

using std::placeholders::_1;

namespace net::wavem::drive
{
    class Motion final
    {
    private:
        rclcpp::Node::SharedPtr node_;
        Parameter::SharedPtr parameter_;
        Control::SharedPtr control_;
        Vehicle::SharedPtr current_vehicle_;

        rclcpp::Time current_time_;
        rclcpp::Time last_time_;
        int8_t current_mission_type_;
        bool is_need_to_decelerate_;
        bool is_in_road_;
        bool is_in_geo_fence_;
        bool is_deceleration_started_;
        bool is_deceleration_ended_;
        bool is_started_from_geo_fence_;
        double last_speed_;
        double distance_geo_fence_;
        int mcu_heartbeat_retry_count_;

        rclcpp::CallbackGroup::SharedPtr vehicle_status_subscription_cb_group_;
        rclcpp::Subscription<can_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
        void vehicle_status_subscription_cb(const can_msgs::msg::VehicleStatus::SharedPtr vehicle_status);

        rclcpp::CallbackGroup::SharedPtr map_position_subscription_cb_group_;
        rclcpp::Subscription<map_handle_msgs::msg::Position>::SharedPtr map_position_subscription_;
        void map_position_subscription_cb(const map_handle_msgs::msg::Position::SharedPtr map_position);

        rclcpp::CallbackGroup::SharedPtr mission_type_subscription_cb_group_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_type_subscription_;
        void mission_type_subscription_cb(const std_msgs::msg::Int8::SharedPtr mission_type);

        rclcpp::CallbackGroup::SharedPtr deceleration_status_publisher_cb_group_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr deceleration_status_publisher_;
        void publish_deceleration_status(const int &status);

        rclcpp::CallbackGroup::SharedPtr deceleration_activate_subscription_cb_group_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deceleration_activate_subscription_;
        void deceleration_activate_subscription_cb(const std_msgs::msg::Bool::SharedPtr is_activate);

    public:
        explicit Motion(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Control::SharedPtr &control);
        virtual ~Motion();

    public:
        using SharedPtr = std::shared_ptr<Motion>;

    };
}

#endif
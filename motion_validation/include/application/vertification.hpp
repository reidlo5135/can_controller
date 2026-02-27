#ifndef APPLICATION_VERTIFICATION_HPP
#define APPLICATION_VERTIFICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <map_handle_msgs/msg/position.hpp>

#include "domain/parameter.hpp"
#include "application/kalman.hpp"

using std::placeholders::_1;

namespace net::wavem::drive
{
    class Vertification final
    {
    private:
        rclcpp::Node::SharedPtr node_;
        Parameter::SharedPtr parameter_;
        can_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;
        KalmanFilter kalman_;

        bool is_vehicle_driving_;
        bool is_start_gps_recorded_;

        rclcpp::Time vehicle_status_time_;
        rclcpp::Time gps_fix_time_;

        rclcpp::Time start_time_;
        double start_lat_, start_lon_;
        rclcpp::Time stop_time_;
        double stop_lat_, stop_lon_;

        double prev_lat_;
        double prev_lon_;
        double gps_distance_;

        double start_can_odometer_;
        double end_can_odometer_;
        double can_odometer_distance_;

        rclcpp::Time last_rpm_update_time_;
        double start_rpm_odometer_;
        double end_rpm_odometer_;
        double rpm_odometer_distance_;

        rclcpp::CallbackGroup::SharedPtr veritification_timer_cb_group_;
        rclcpp::TimerBase::SharedPtr vertification_timer_;
        void vertification_timer_cb();

        rclcpp::CallbackGroup::SharedPtr vehicle_status_subscription_cb_group_;
        rclcpp::Subscription<can_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
        void vehicle_status_subscription_cb(const can_msgs::msg::VehicleStatus::SharedPtr vehicle_status);

        rclcpp::CallbackGroup::SharedPtr gps_fix_subscription_cb_group_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_subscription_;
        void gps_fix_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix);

    private:
        double haversine(double lat1, double lon1, double lat2, double lon2);
        void report_vertification_result();
        void reset_verification_state();

    public:
        explicit Vertification(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter);
        virtual ~Vertification();

    public:
        using SharedPtr = std::shared_ptr<Vertification>;
    };
}

#endif
#include "application/vertification.hpp"

using namespace net::wavem::drive;

Vertification::Vertification(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter)
    : node_(node),
      parameter_(parameter),
      is_vehicle_driving_(false),
      is_start_gps_recorded_(false)
{
    this->vehicle_status_ = std::make_shared<can_msgs::msg::VehicleStatus>();

    this->vehicle_status_time_ = this->node_->now();
    this->gps_fix_time_ = this->node_->now();

    this->veritification_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->vertification_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(this->parameter_->vehicle_vertification_loop_rate_),
        std::bind(&Vertification::vertification_timer_cb, this),
        this->veritification_timer_cb_group_);

    this->vehicle_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions vehicle_status_subscription_opts;
    vehicle_status_subscription_opts.callback_group = this->vehicle_status_subscription_cb_group_;
    this->vehicle_status_subscription_ = this->node_->create_subscription<can_msgs::msg::VehicleStatus>(
        this->parameter_->vehicle_status_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Vertification::vehicle_status_subscription_cb, this, _1),
        vehicle_status_subscription_opts);

    this->gps_fix_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_fix_subscription_opts;
    gps_fix_subscription_opts.callback_group = this->gps_fix_subscription_cb_group_;
    this->gps_fix_subscription_ = this->node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        this->parameter_->gps_topic_,
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&Vertification::gps_fix_subscription_cb, this, _1),
        gps_fix_subscription_opts);
}

Vertification::~Vertification()
{
}

void Vertification::vertification_timer_cb()
{
}

void Vertification::vehicle_status_subscription_cb(const can_msgs::msg::VehicleStatus::SharedPtr vehicle_status)
{
    this->vehicle_status_ = vehicle_status;

    rclcpp::Time now = this->node_->now();

    if (this->last_rpm_update_time_.nanoseconds() == 0)
    {
        this->last_rpm_update_time_ = now;
        return;
    }

    if (this->is_vehicle_driving_ && this->last_rpm_update_time_.nanoseconds() > 0)
    {
        double delta_sec = (now - this->last_rpm_update_time_).seconds();
        this->last_rpm_update_time_ = now;

        double rpm = static_cast<double>(vehicle_status->rpm);
        double rev_per_sec = rpm / 60.0;
        double delta_distance = rev_per_sec * this->parameter_->vehicle_wheel_size_ * M_PI * delta_sec;
        double rpm_vel = rev_per_sec * (this->parameter_->vehicle_wheel_size_ * M_PI);
        this->kalman_.predict(delta_sec, rpm_vel);
        this->end_rpm_odometer_ += delta_distance;
    }

    if (!this->is_vehicle_driving_)
    {
        if (vehicle_status->speed > 0.1)
        {
            this->start_time_ = this->node_->now();
            this->is_vehicle_driving_ = true;

            this->start_can_odometer_ = static_cast<double>(vehicle_status->can_odometry.vehicle_ad_mileage);
            this->start_rpm_odometer_ = this->end_rpm_odometer_;

            double rpm_vel = static_cast<double>(vehicle_status->rpm) / 60.0 * (this->parameter_->vehicle_wheel_size_ * M_PI);
            this->kalman_.init(this->start_rpm_odometer_, rpm_vel);

            RCLCPP_INFO(this->node_->get_logger(), "[Vertification] Vehicle started moving");
        }
    }
    else
    {
        if (vehicle_status->speed <= 0.1)
        {
            this->stop_time_ = this->node_->now();
            this->is_vehicle_driving_ = false;

            this->end_can_odometer_ = static_cast<double>(vehicle_status->can_odometry.vehicle_ad_mileage);
            this->can_odometer_distance_ = std::abs(this->end_can_odometer_ - this->start_can_odometer_);
            this->rpm_odometer_distance_ = std::abs(this->end_rpm_odometer_ - this->start_rpm_odometer_);

            RCLCPP_INFO(this->node_->get_logger(), "[Vertification] Vehicle stopped moving");

            this->report_vertification_result();
            this->reset_verification_state();
        }
    }
}

void Vertification::gps_fix_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix)
{
    if (this->is_vehicle_driving_)
    {
        if (!this->is_start_gps_recorded_)
        {
            this->start_lat_ = gps_fix->latitude;
            this->start_lon_ = gps_fix->longitude;
            this->prev_lat_ = gps_fix->latitude;
            this->prev_lon_ = gps_fix->longitude;
            this->is_start_gps_recorded_ = true;
        }
        else
        {
            double delta = this->haversine(this->prev_lat_, this->prev_lon_, gps_fix->latitude, gps_fix->longitude);
            this->gps_distance_ += delta;

            this->prev_lat_ = gps_fix->latitude;
            this->prev_lon_ = gps_fix->longitude;

            this->kalman_.update(this->gps_distance_);
            RCLCPP_INFO(this->node_->get_logger(), "[Vertification] GPS Δd: %.2f m, Accumulated: %.2f m", delta, this->gps_distance_);
        }
    }
}

double Vertification::haversine(double lat1, double lon1, double lat2, double lon2)
{
    static constexpr double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) + std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) * std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(sqrt(a), std::sqrt(1 - a));
    return R * c;
}

void Vertification::report_vertification_result()
{
    double elapsed_sec = (this->stop_time_ - this->start_time_).seconds();
    double expected_distance = (this->vehicle_status_->speed / 3.6) * elapsed_sec;

    double gps_distance = this->haversine(this->start_lat_, this->start_lon_, this->stop_lat_, this->stop_lon_);
    double gps_error = std::fabs(expected_distance - this->gps_distance_) / expected_distance * 100.0;

    double rpm_error_bt_can_n_rpm = std::fabs(this->can_odometer_distance_ - this->rpm_odometer_distance_);

    RCLCPP_INFO(this->node_->get_logger(), "======== Verification Result ========");
    RCLCPP_INFO(this->node_->get_logger(), "Speed : %.2f km/h", this->vehicle_status_->speed);
    RCLCPP_INFO(this->node_->get_logger(), "Elasped Seconds : %.2f s", elapsed_sec);
    RCLCPP_INFO(this->node_->get_logger(), "Expected Distance : %.2f m", expected_distance);
    RCLCPP_INFO(this->node_->get_logger(), "GPS Distance      : %.2f m (error %.2f %%)", this->gps_distance_, gps_error);
    RCLCPP_INFO(this->node_->get_logger(), "CAN Odometer Dist : %.2f m", this->can_odometer_distance_);
    RCLCPP_INFO(this->node_->get_logger(), "RPM Odometer Dist : %.2f m", this->rpm_odometer_distance_);
    RCLCPP_INFO(this->node_->get_logger(), "RPM Error Between CAN & RPM : %.2f m", rpm_error_bt_can_n_rpm);
    RCLCPP_INFO(this->node_->get_logger(), "Kalman Filtered Dist : %.2f m", this->kalman_.get__position());
    RCLCPP_INFO(this->node_->get_logger(), "============================");
}

void Vertification::reset_verification_state()
{
    this->is_vehicle_driving_ = false;
    this->is_start_gps_recorded_ = false;

    this->vehicle_status_time_ = this->node_->now();
    this->gps_fix_time_ = this->node_->now();

    this->start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    this->stop_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    this->start_lat_ = 0.0;
    this->start_lon_ = 0.0;
    this->stop_lat_ = 0.0;
    this->stop_lon_ = 0.0;

    this->prev_lat_ = 0.0;
    this->prev_lon_ = 0.0;
    this->gps_distance_ = 0.0;

    this->start_can_odometer_ = 0.0;
    this->end_can_odometer_ = 0.0;
    this->can_odometer_distance_ = 0.0;

    this->last_rpm_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    this->start_rpm_odometer_ = 0.0;
    this->end_rpm_odometer_ = 0.0;
    this->rpm_odometer_distance_ = 0.0;
}
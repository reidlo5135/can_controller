#include "application/motion.hpp"

using namespace net::wavem::drive;

Motion::Motion(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Control::SharedPtr &control)
    : node_(node),
      parameter_(parameter),
      control_(control)
{
    this->current_vehicle_ = std::make_shared<Vehicle>(this->parameter_->vehicle_wheel_size_, this->parameter_->vehicle_mcu_gear_ratio_);

    this->vehicle_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions vehicle_status_subscription_opts;
    vehicle_status_subscription_opts.callback_group = this->vehicle_status_subscription_cb_group_;
    this->vehicle_status_subscription_ = this->node_->create_subscription<can_msgs::msg::VehicleStatus>(
        this->parameter_->vehicle_status_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Motion::vehicle_status_subscription_cb, this, _1),
        vehicle_status_subscription_opts);

    this->map_position_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions map_position_subscription_opts;
    map_position_subscription_opts.callback_group = this->map_position_subscription_cb_group_;
    this->map_position_subscription_ = this->node_->create_subscription<map_handle_msgs::msg::Position>(
        this->parameter_->map_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Motion::map_position_subscription_cb, this, _1),
        map_position_subscription_opts);

    this->mission_type_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_type_subscription_opts;
    mission_type_subscription_opts.callback_group = this->mission_type_subscription_cb_group_;
    this->mission_type_subscription_ = this->node_->create_subscription<std_msgs::msg::Int8>(
        MISSION_TYPE_SUBSCRIPTION_TOPIC,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Motion::mission_type_subscription_cb, this, _1),
        mission_type_subscription_opts);

    this->deceleration_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions deceleration_status_publisher_opts;
    deceleration_status_publisher_opts.callback_group = this->deceleration_status_publisher_cb_group_;
    this->deceleration_status_publisher_ = this->node_->create_publisher<std_msgs::msg::Int8>(
        this->parameter_->vehicle_control_deceleration_status_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        deceleration_status_publisher_opts);

    this->deceleration_activate_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions deceleration_subscription_opts;
    deceleration_subscription_opts.callback_group = this->deceleration_activate_subscription_cb_group_;
    this->deceleration_activate_subscription_ = this->node_->create_subscription<std_msgs::msg::Bool>(
        this->parameter_->vehicle_control_deceleration_activate_topic_,
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        std::bind(&Motion::deceleration_activate_subscription_cb, this, _1),
        deceleration_subscription_opts);
}

Motion::~Motion()
{
}

void Motion::vehicle_status_subscription_cb(const can_msgs::msg::VehicleStatus::SharedPtr vehicle_status)
{
    this->current_vehicle_->set__current_speed(vehicle_status->speed);
    this->current_vehicle_->set__current_rpm(vehicle_status->rpm);
    this->current_vehicle_->set__current_gear(vehicle_status->gear);
}

void Motion::mission_type_subscription_cb(const std_msgs::msg::Int8::SharedPtr mission_type)
{
    this->current_mission_type_ = mission_type->data;
}

void Motion::publish_deceleration_status(const int &status)
{
    std_msgs::msg::Int8::UniquePtr deceleration_status = std::make_unique<std_msgs::msg::Int8>();
    deceleration_status->set__data(static_cast<int8_t>(status));
    this->deceleration_status_publisher_->publish(std::move(*deceleration_status));
}

void Motion::deceleration_activate_subscription_cb(const std_msgs::msg::Bool::SharedPtr is_activate)
{
    this->parameter_->vehicle_control_deceleration_trigger_ = is_activate->data;
}

void Motion::map_position_subscription_cb(const map_handle_msgs::msg::Position::SharedPtr map_position)
{
    const int8_t &area_type = map_position->type;
    const double &distance_geo_fence = map_position->distance_geofence;

    if (!this->parameter_->vehicle_control_deceleration_trigger_)
    {
        return;
    }

    if (area_type == this->parameter_->map_area_road_)
    {
        return;
    }

    if (distance_geo_fence == -1)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "DISTANCE GEO FENCE IS NULL");
        return;
    }

    const bool &is_in_geo_fence = (map_position->type == this->parameter_->map_area_geo_fence_);

    if (is_in_geo_fence)
    {
        this->is_need_to_decelerate_ = true;

        if (distance_geo_fence == 0.0)
        {
            RCLCPP_WARN(this->node_->get_logger(), "!!!!! STARTED FROM GEO FENCE !!!!!");
            this->is_started_from_geo_fence_ = true;
        }
    }

    if (!is_in_geo_fence && distance_geo_fence == 0.0)
    {
        this->is_need_to_decelerate_ = false;
    }

    if (!is_in_geo_fence && distance_geo_fence > 0 && this->current_vehicle_->get__current_gear() == MCU_GEAR_REAR)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "NEAR TO GEO FENCE BUT REAR GEAR");
        this->is_need_to_decelerate_ = false;
        return;
    }

    if (this->current_mission_type_ != 0)
    {
        RCLCPP_WARN(this->node_->get_logger(), "@@@@@ GEO FENCE BUT MISSION PROCEEDING @@@@@");
        return;
    }

    if (!is_in_geo_fence && this->is_need_to_decelerate_)
    {
        /**
         * @brief 1. 매뉴얼 주행 중 지오펜스 근접 이후 최초의 상태(매뉴얼로부터 CAN 제어권 탈취)
         * deceleration status type : 0
         * CAN 주도권 : 매뉴얼 -> 자율
         * a. is_deceleration_started_ = false -> true
         * b. is_deceleration_ended_ = false
         * c. current_vehicle_->get__current_speed() != 0.0
         */
        if (!this->is_deceleration_started_ && !this->is_deceleration_ended_ && this->current_vehicle_->get__current_speed() != 0.0)
        {
            if (this->mcu_heartbeat_retry_count_ < this->parameter_->vehicle_heartbeat_retry_)
            {
                RCLCPP_ERROR(this->node_->get_logger(), "!!!!! DECELERATION START / VALID ZERO [%d]!!!!!", this->mcu_heartbeat_retry_count_);
                this->control_->control_vehicle_reset(0, 0);
                this->mcu_heartbeat_retry_count_++;
            }
            else
            {
                RCLCPP_ERROR(this->node_->get_logger(), "!!!!! DECELERATION START / VALID ZERO END !!!!!");
                this->is_deceleration_started_ = true;
                this->is_deceleration_ended_ = false;
                return;
            }
        }

        /**
         * @brief 2. 감속 시작(D기어 한정)
         * deceleration status type : 1
         * CAN 주도권 : 자율
         * a. is_deceleration_started_ = true -> false
         * b. is_deceleration_ended_ = false -> true
         */
        if (this->is_deceleration_started_ && !this->is_deceleration_ended_)
        {
            if (this->current_vehicle_->get__current_gear() == MCU_GEAR_DRIVE)
            {
                if (this->current_vehicle_->get__current_speed() != 0.0)
                {
                    this->publish_deceleration_status(DECELERATION_STATUS_START);
                    RCLCPP_WARN(this->node_->get_logger(), "=============================================");
                    RCLCPP_WARN(this->node_->get_logger(), "!!!!! DECELERATION START !!!!!");
                    RCLCPP_WARN(this->node_->get_logger(), "last_rpm : [%f]r/m", this->current_vehicle_->get__current_rpm());

                    if (this->current_vehicle_->get__current_rpm() <= 200)
                    {
                        RCLCPP_WARN(this->node_->get_logger(), "!!!!! DECELERATION COMPLETED !!!!!");
                        this->is_deceleration_started_ = false;
                        this->is_deceleration_ended_ = true;
                        this->publish_deceleration_status(DECELERATION_STATUS_END);
                        return;
                    }
                    else
                    {
                        this->control_->control_vehicle_drive(0.0, 0.0);
                        this->control_->control_vehicle_brake(100);
                    }
                    RCLCPP_WARN(this->node_->get_logger(), "=============================================");
                }
            }
        }
    }
    else if (is_in_geo_fence && this->is_need_to_decelerate_)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "==================== GEO FENCE =========================");
        /**
         * @brief 3. 감속 완료
         * deceleration status type : 2
         * CAN 주도권 : 자율 -> 매뉴얼
         * a. is_deceleration_started_ = false
         * b. is_deceleration_ended_ = true
         */
        if ((this->is_deceleration_ended_ && !this->is_deceleration_started_) || this->is_started_from_geo_fence_)
        {
            /**
             * @brief 4. 사용자의 매뉴얼 D기어 해제 시까지 CAN 주도권 탈취 후 차량 브레이크
             * CAN 주도권 : 자율
             * deceleration status type : -1
             */
            if (this->current_vehicle_->get__current_gear() == MCU_GEAR_DRIVE)
            {
                if (this->current_vehicle_->get__current_rpm() > 0)
                {
                    if (this->mcu_heartbeat_retry_count_ < this->parameter_->vehicle_heartbeat_retry_)
                    {
                        this->control_->control_vehicle_reset(0, 0);
                        this->mcu_heartbeat_retry_count_++;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->node_->get_logger(), "@@@@@@ GEO FENCE DECELERATION DRIVE GEAR BRAKE @@@@@@");
                        this->control_->control_vehicle_drive(0.0, 0.0);
                        this->control_->control_vehicle_brake(100);
                        this->publish_deceleration_status(DECELERATION_STATUS_DRIVE_GEAR);
                        this->mcu_heartbeat_retry_count_++;
                    }
                }
                else
                {
                    if (this->mcu_heartbeat_retry_count_ < this->parameter_->vehicle_heartbeat_retry_)
                    {
                        RCLCPP_ERROR(this->node_->get_logger(), "@@@@@ GEO FENCE DECELERATION DRIVE GEAR / VALID ZERO @@@@@");
                        this->control_->control_vehicle_brake(0);
                        this->control_->control_vehicle_reset(0, 0);
                        this->mcu_heartbeat_retry_count_++;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->node_->get_logger(), "@@@@@ GEO FENCE DECELERATION DRIVE GEAR / BRAKE RELEASE @@@@@");
                    }
                    this->mcu_heartbeat_retry_count_ = 0;
                }
            }
            else
            {
                if (this->mcu_heartbeat_retry_count_ < this->parameter_->vehicle_heartbeat_retry_)
                {
                    RCLCPP_ERROR(this->node_->get_logger(), "@@@@@ GEO FENCE DECELERATION END / VALID ZERO @@@@@");
                    this->control_->control_vehicle_brake(0);
                    this->control_->control_vehicle_reset(0, 0);
                    this->mcu_heartbeat_retry_count_++;
                }
                else
                {
                    // RCLCPP_ERROR(this->node_->get_logger(), "@@@@@ GEO FENCE DECELERATION END / BRAKE RELEASE @@@@@");
                }
            }
            RCLCPP_ERROR(this->node_->get_logger(), "=============================================");
        }
    }
    else
    {
        /**
         * @brief 5. 지오펜스 해제
         * deceleration status type : 3
         * a. is_deceleration_started_ = false
         * b. is_deceleration_ended_ = true
         */
        this->publish_deceleration_status(DECELERATION_STATUS_RELEASE);
        this->is_need_to_decelerate_ = false;
        this->last_speed_ = 0.0;
        this->is_deceleration_started_ = false;
        this->is_deceleration_ended_ = false;
        this->mcu_heartbeat_retry_count_ = 0;
        return;
    }
}
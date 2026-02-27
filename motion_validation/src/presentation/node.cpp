#include "presentation/node.hpp"

using namespace net::wavem::drive;

MotionValidation::MotionValidation()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ == nullptr)
    {
        RCUTILS_ERROR(NODE_NAME, "%s, node pointer is nullptr...", NODE_NAME);
        return;
    }
    else
    {
        RCUTILS_INFO(NODE_NAME, "%s created", this->node_->get_name());

        this->parameter_ = std::make_shared<Parameter>();
        this->declare_parameters();

        this->vertification_ = std::make_shared<Vertification>(this->node_, this->parameter_);
        this->correction_ = std::make_shared<Correction>(this->node_, this->parameter_, this->vertification_);
    }
}

MotionValidation::~MotionValidation()
{
}

void MotionValidation::declare_parameters()
{
    const std::string param_vehicle = std::string(PARAM_VEHICLE) + ".";
    const std::string param_position = std::string(PARAM_POSITION) + ".";

    std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> param_vec =
        {
            {param_vehicle + "vertification" + ".loop.rate", 0},
            {param_vehicle + "status" + ".topic", ""},
            {param_vehicle + "wheel" + ".size", 0.0},
            {param_vehicle + "wheel" + ".base", 0.0},
            {param_position + "gps" + ".topic", ""},
            {param_position + "map" + ".topic", ""},
        };

    for (const std::pair<std::string, std::variant<int, double, bool, std::string>> &param : param_vec)
    {
        const std::string &param_name = param.first;
        const auto &default_value = param.second;

        std::visit([this, &param_name](auto &&value)
                   {
                using T = std::decay_t<decltype(value)>;
                this->node_->declare_parameter<T>(param_name, value); }, default_value);
    }

    this->node_->get_parameter<int>(param_vehicle + "vertification" + ".loop.rate", this->parameter_->vehicle_vertification_loop_rate_);
    this->node_->get_parameter<std::string>(param_vehicle + "status" + ".topic", this->parameter_->vehicle_status_topic_);
    this->node_->get_parameter<double>(param_vehicle + "wheel" + ".size", this->parameter_->vehicle_wheel_size_);
    this->node_->get_parameter<double>(param_vehicle + "wheel" + ".base", this->parameter_->vehicle_wheel_base_);
    this->node_->get_parameter<std::string>(param_position + "gps" + ".topic", this->parameter_->gps_topic_);
    this->node_->get_parameter<std::string>(param_position + "map" + ".topic", this->parameter_->map_position_topic_);
}
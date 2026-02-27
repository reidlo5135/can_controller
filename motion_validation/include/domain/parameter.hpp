#ifndef DOMAIN_PARAMETER_HPP
#define DOMAIN_PARAMETER_HPP

#include <memory>

#define PARAM_VEHICLE "vehicle"
#define PARAM_POSITION "position"

namespace net::wavem::drive
{
    class Parameter final
    {
    public:
        int vehicle_vertification_loop_rate_ = 0;
        std::string vehicle_status_topic_;
        double vehicle_wheel_size_ = 0.0;
        double vehicle_wheel_base_ = 0.0;
        std::string gps_topic_;
        std::string map_position_topic_;

    public:
        explicit Parameter() = default;
        virtual ~Parameter() = default;

    public:
        using SharedPtr = std::shared_ptr<Parameter>;
    };
}

#endif
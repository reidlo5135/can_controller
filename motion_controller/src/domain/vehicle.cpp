#include "domain/vehicle.hpp"

#include <unistd.h>
#include <oneapi/tbb/detail/_template_helpers.h>

namespace net::wavem::drive
{
    Vehicle::Vehicle(double wheel, double gear_ratio)
        : wheel_(wheel), gear_ratio_(gear_ratio), current_gear_(0), current_rpm_(0.0), current_distance_(0.0), current_speed_(0.0), current_acc_pedal_(0.0)
    {
    }

    Vehicle::~Vehicle() = default;

    std::chrono::system_clock::time_point
    Vehicle::get__current_time() const
    {
        return this->current_time_;
    }

    /**
     * @brief set the current time
     * @config current_time
     */
    void
    Vehicle::set__current_time(const std::chrono::system_clock::time_point &current_time)
    {
        this->current_time_ = current_time;
    }

    /**
     * @brief
     *
     * @return const int
     */
    int
    Vehicle::get__gear_ratio() const
    {
        return this->gear_ratio_;
    }

    /**
     * @brief
     * @return const double
     */
    double
    Vehicle::get__wheel() const
    {
        return this->wheel_;
    }

    int
    Vehicle::get__current_gear() const
    {
        return this->current_gear_;
    }

    void
    Vehicle::set__current_gear(const int &current_gear)
    {
        this->current_gear_ = current_gear;
    }

    double
    Vehicle::get__current_rpm() const
    {
        return this->current_rpm_;
    }

    /**
     * @brief set the current rpm
     * @config current_rpm
     */
    void
    Vehicle::set__current_rpm(const double &current_rpm)
    {
        this->current_rpm_ = current_rpm;
    }

    double
    Vehicle::get__current_distance() const
    {
        return this->current_distance_;
    }

    void
    Vehicle::set__current_distance(const double &current_distance)
    {
        this->current_distance_ = current_distance;
    }

    double
    Vehicle::get__current_speed() const
    {
        return this->current_speed_;
    }

    void
    Vehicle::set__current_speed(const double &current_speed)
    {
        this->current_speed_ = current_speed;
    }

    double
    Vehicle::get__current_acc_pedal() const
    {
        return this->current_acc_pedal_;
    }

    void
    Vehicle::set__current_acc_pedal(const double &current_acc_pedal)
    {
        this->current_acc_pedal_ = current_acc_pedal;
    }
}

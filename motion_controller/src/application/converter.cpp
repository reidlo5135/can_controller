#include "application/converter.hpp"

namespace net::wavem::drive
{
    Converter::Converter(double wheel, double gear_ratio)
        : rpm_standard_(RPM_STANDARD)
    {
        this->target_vehicle_ = std::make_unique<Vehicle>(wheel, gear_ratio);
    }

    Converter::~Converter() = default;

    float
    Converter::rpm_to_distance(Vehicle previous_vehicle, Vehicle current_vehicle)
    {
        std::chrono::duration<double> time_span = current_vehicle.get__current_time() - previous_vehicle.get__current_time();
        float time_dif = static_cast<float>(time_span.count());

        float temp_correction = 1.16;
        float result = 0;

        result = ((current_vehicle.get__wheel() * M_PI) * ((previous_vehicle.get__current_rpm() - RPM_STANDARD) + (current_vehicle.get__current_rpm() - RPM_STANDARD)) / 2 * (time_dif / 60) / current_vehicle.get__gear_ratio());
        result /= 2;
        result *= 10;
        result *= temp_correction;

        return static_cast<float>(result);
    }

    long long
    Converter::calculate_time_difference(const std::chrono::system_clock::time_point &start_time, const std::chrono::system_clock::time_point &end_time)
    {
        const std::chrono::milliseconds &duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        return duration.count();
    }

    AckermannDriver::AckermannDriver(float wheel_base)
        : wheel_base_(wheel_base)
    {
    }

    AckermannDriver::~AckermannDriver() = default;

    float
    AckermannDriver::convert_trans_rot_vel_to_steering_angle(float velocity, float angular)
    {
        if (velocity == 0 || angular == 0)
        {
            return 0;
        }

        const float &radius = [](float v, float a)
        {
            return v / a;
        }(velocity, angular);

        return std::atan(this->wheel_base_ / radius);
    }
}

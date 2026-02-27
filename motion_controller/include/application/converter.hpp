#ifndef APPLICATION_CONVERTER_HPP
#define APPLICATION_CONVERTER_HPP

#include <memory>
#include <chrono>
#include <cmath>

#include "domain/vehicle.hpp"

#define RPM_STANDARD 100000
#define OFFSET_STEERING 30
#define OFFSET_STRANGLE 3000
#define CORRECTION_STEERING 0.44323941137954047907456588439588

namespace net::wavem::drive
{
    /**
     * @author reidlo(naru5135@wavem.net)
     * @date 25.01.19
     */
    class Converter final
    {
    private:
        const int rpm_standard_;
        std::unique_ptr<Vehicle> target_vehicle_;
        long long calculate_time_difference(const std::chrono::system_clock::time_point &start_time, const std::chrono::system_clock::time_point &end_time);

    public:
        explicit Converter(double wheel, double gear_ratio);
        virtual ~Converter();
        float rpm_to_distance(Vehicle previous_vehicle, Vehicle current_vehicle);

    };

    class AckermannDriver final
    {
    private:
        float wheel_base_;

    public:
        explicit AckermannDriver(float wheel_base);
        virtual ~AckermannDriver();
        float convert_trans_rot_vel_to_steering_angle(float velocity, float angular);

    };
}

#endif
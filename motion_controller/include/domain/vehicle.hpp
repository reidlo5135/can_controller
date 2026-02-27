#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <iostream>
#include <chrono>
#include <memory>
#include <rcutils/logging_macros.h>

#define GEAR_RATIO 20

namespace net::wavem::drive
{
    class Vehicle final
    {
    private:
        double wheel_;
        double max_speed_;
        int gear_ratio_;
        std::chrono::system_clock::time_point current_time_;
        int current_gear_;
        double current_rpm_;
        double current_distance_;
        double current_speed_;
        double current_acc_pedal_;

    public:
        explicit Vehicle(double wheel, double gear_ratio);
        virtual ~Vehicle();

        [[nodiscard]] std::chrono::system_clock::time_point get__current_time() const;
        void set__current_time(const std::chrono::system_clock::time_point &current_time);

        [[nodiscard]] int get__gear_ratio() const;
        [[nodiscard]] double get__wheel() const;

        [[nodiscard]] int get__current_gear() const;
        void set__current_gear(const int &current_gear);

        [[nodiscard]] double get__current_rpm() const;
        void set__current_rpm(const double &current_rpm);

        [[nodiscard]] double get__current_distance() const;
        void set__current_distance(const double &current_distance);

        [[nodiscard]] double get__current_speed() const;
        void set__current_speed(const double &current_speed);

        [[nodiscard]] double get__current_acc_pedal() const;
        void set__current_acc_pedal(const double &current_acc_pedal);

    public:
        using SharedPtr = std::shared_ptr<Vehicle>;
        using UniquePtr = std::unique_ptr<Vehicle>;
    };

    enum class BREAK
    {
        LED = 1,
        STOP = 100,
        GO = 0
    };
}

#endif
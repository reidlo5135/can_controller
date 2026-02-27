/**
 * @file vehicle.hpp
 * @brief Class definitions for vehicle control and status abstraction
 * @author reidlo
 * @date 2025.04.23
 */

#ifndef DOMAIN_VEHICLE_HPP
#define DOMAIN_VEHICLE_HPP

#include <memory>

namespace net::wavem::can
{
    /**
     * @brief Class representing vehicle control inputs
     */
    class Control final
    {
    private:
        unsigned char accelerate_valid_ = 2; /**< Activate valid for acceleration control */
        int accelerate_gear_ = 0;            /**< Gear position (e.g., D, R, P) */
        double accelerate_work_mode_ = 0;    /**< Work mode value (speed/torque) */
        double accelerate_speed_ = 0;        /**< Speed value for acceleration */

        unsigned char steering_valid_; /**< Activate valid for steering control */
        double steering_angle_;        /**< Steering wheel angle in degrees */

        unsigned char brake_valid_; /**< Activate valid for brake control */
        double brake_pressure_;     /**< Pressure value for braking */

    public:
        explicit Control();
        virtual ~Control();

        /** @brief Get accelerate valid */
        unsigned char get__accelerate_valid() const;
        /** @brief Set accelerate valid */
        void set__accelerate_valid(const unsigned char &accelerate_valid);

        /** @brief Get accelerate gear */
        int get__accelerate_gear() const;
        /** @brief Set accelerate gear */
        void set__accelerate_gear(const int &accelerate_gear);

        /** @brief Get work mode */
        double get__accelerate_work_mode() const;
        /** @brief Set work mode */
        void set__accelerate_work_mode(const double &mode);

        /** @brief Get accelerate pressure */
        double get__accelerate_speed() const;
        /** @brief Set accelerate pressure */
        void set__accelerate_speed(const double &accelerate_speed);

        /** @brief Get steering valid */
        unsigned char get__steering_valid() const;
        /** @brief Set steering valid */
        void set__steering_valid(const unsigned char &steering_valid);

        /** @brief Get steering angle */
        double get__steering_angle() const;
        /** @brief Set steering angle */
        void set__steering_angle(const double &angle);

        /** @brief Get brake valid */
        unsigned char get__brake__valid() const;
        /** @brief Set brake  valid */
        void set__brake__valid(const unsigned char &brake__valid);

        /** @brief Get brake pressure */
        double get__brake_pressure() const;
        /** @brief Set brake pressure */
        void set__brake_pressure(const double &pressure);

    public:
        using SharedPtr = std::shared_ptr<Control>;
    };

    /**
     * @brief Class representing current vehicle status
     */
    class Status final
    {
    private:
        double rpm_;     /**< Engine or motor RPM */
        double speed_;   /**< Vehicle speed */
        int gear_;       /**< Current gear */
        double battery_; /**< Battery percentage or level */

    public:
        explicit Status();
        virtual ~Status();

        /** @brief Get RPM */
        double get__rpm() const;
        /** @brief Set RPM */
        void set__rpm(const double &rpm);

        /** @brief Get speed */
        double get__speed() const;
        /** @brief Set speed */
        void set__speed(const double &speed);

        /** @brief Get gear */
        int get__gear() const;
        /** @brief Set gear */
        void set__gear(const int &gear);

        /** @brief Get battery */
        double get__battery() const;
        /** @brief Set battery */
        void set__battery(const double &battery);

    public:
        using SharedPtr = std::shared_ptr<Status>;
    };
}

#endif
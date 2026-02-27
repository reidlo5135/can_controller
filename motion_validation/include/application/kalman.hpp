#pragma once

#ifndef APPLICATION_KALMAN_HPP
#define APPLICATION_KALMAN_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace net::wavem::drive
{
    class KalmanFilter final
    {
    private:
        Eigen::Vector2d x_;    // [position, velocity]
        Eigen::Matrix2d P_;    // Covariance
        Eigen::Matrix2d A_;    // Transition
        Eigen::Matrix2d Q_;    // Process noise
        Eigen::RowVector2d H_; // Measurement model
        double R_;             // Measurement noise

    public:
        explicit KalmanFilter();
        virtual ~KalmanFilter();

    public:
        void init(double init_pos, double init_vel);
        void predict(double dt, double measured_vel);
        void update(double measured_pos);
        double get__position() const;
        double get__velocity() const;
    };
}

#endif
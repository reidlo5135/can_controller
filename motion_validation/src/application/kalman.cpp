#include "application/kalman.hpp"

using namespace net::wavem::drive;

KalmanFilter::KalmanFilter()
{
    x_ << 0.0, 0.0;
    P_ = Eigen::Matrix2d::Identity();
    Q_ = Eigen::Matrix2d::Identity() * 0.1;
    R_ = 5.0; // Measurement noise (tune per sensor)
}

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::init(double init_pos, double init_vel)
{
    x_ << init_pos, init_vel;
    P_ << 1, 0, 0, 1;
    A_ << 1, 0, 0, 1;
    Q_ << 0.1, 0, 0, 0.1;
    H_ << 1, 0;
    R_ = 5.0;
}

void KalmanFilter::predict(double dt, double measured_vel)
{
    A_ << 1, dt, 0, 1;
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(double measured_pos)
{
    double y = measured_pos - H_ * x_;
    auto S = H_ * P_ * H_.transpose() + R_;
    auto K = P_ * H_.transpose() / S;
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
}

double KalmanFilter::get__position() const
{
    return x_(0);
}

double KalmanFilter::get__velocity() const
{
    return x_(1);
}
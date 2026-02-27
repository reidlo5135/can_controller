/**
 * @file vehicle.cpp
 * @brief Implementation of Control and Status classes for vehicle abstraction
 * @author reidlo
 * @date 2025.04.23
 */

#include "domain/vehicle.hpp"

using namespace net::wavem::can;

Control::Control() = default;
Control::~Control() = default;

/** @brief Get accelerate valid */
unsigned char Control::get__accelerate_valid() const { return this->accelerate_valid_; }
/** @brief Set accelerate valid */
void Control::set__accelerate_valid(const unsigned char &accelerate_valid) { this->accelerate_valid_ = accelerate_valid; }

/** @brief Get accelerate gear */
int Control::get__accelerate_gear() const { return this->accelerate_gear_; }
/** @brief Set accelerate gear */
void Control::set__accelerate_gear(const int &accelerate_gear) { this->accelerate_gear_ = accelerate_gear; }

/** @brief Get accelerate work mode */
double Control::get__accelerate_work_mode() const { return this->accelerate_work_mode_; }
/** @brief Set accelerate work mode */
void Control::set__accelerate_work_mode(const double &mode) { this->accelerate_work_mode_ = mode; }

/** @brief Get accelerate speed */
double Control::get__accelerate_speed() const { return this->accelerate_speed_; }
/** @brief Set accelerate speed */
void Control::set__accelerate_speed(const double &accelerate_speed) { this->accelerate_speed_ = accelerate_speed; }

/** @brief Get steering valid */
unsigned char Control::get__steering_valid() const { return this->steering_valid_; }
/** @brief Set steering valid */
void Control::set__steering_valid(const unsigned char &steering_valid) { this->steering_valid_ = steering_valid; }

/** @brief Get steering angle */
double Control::get__steering_angle() const { return this->steering_angle_; }
/** @brief Set steering angle */
void Control::set__steering_angle(const double &angle) { this->steering_angle_ = angle; }

/** @brief Get brake valid */
unsigned char Control::get__brake__valid() const { return this->brake_valid_; }
/** @brief Set brake valid */
void Control::set__brake__valid(const unsigned char &brake__valid) { this->brake_valid_ = brake_valid_; }

/** @brief Get brake pressure */
double Control::get__brake_pressure() const { return this->brake_pressure_; }
/** @brief Set brake pressure */
void Control::set__brake_pressure(const double &pressure) { this->brake_pressure_ = pressure; }

Status::Status() = default;
Status::~Status() = default;

/** @brief Get RPM */
double Status::get__rpm() const { return this->rpm_; }
/** @brief Set RPM */
void Status::set__rpm(const double &rpm) { this->rpm_ = rpm; }

/** @brief Get speed */
double Status::get__speed() const { return this->speed_; }
/** @brief Set speed */
void Status::set__speed(const double &speed) { this->speed_ = speed; }

/** @brief Get gear */
int Status::get__gear() const { return this->gear_; }
/** @brief Set gear */
void Status::set__gear(const int &gear) { this->gear_ = gear; }

/** @brief Get battery level */
double Status::get__battery() const { return this->battery_; }
/** @brief Set battery level */
void Status::set__battery(const double &battery) { this->battery_ = battery; }
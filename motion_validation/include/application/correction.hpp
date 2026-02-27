#ifndef APPLICATION_CORRECTION_HPP
#define APPLICATION_CORRECTION_HPP

#include <rclcpp/rclcpp.hpp>

#include "domain/parameter.hpp"
#include "application/vertification.hpp"

namespace net::wavem::drive
{
    class Correction final
    {
    private:
        rclcpp::Node::SharedPtr node_;
        Parameter::SharedPtr parameter_;
        Vertification::SharedPtr vertification_;

    public:
        explicit Correction(const rclcpp::Node::SharedPtr &node, const Parameter::SharedPtr &parameter, const Vertification::SharedPtr &vertification);
        virtual ~Correction();

    public:
        using SharedPtr = std::shared_ptr<Correction>;
    };
}

#endif
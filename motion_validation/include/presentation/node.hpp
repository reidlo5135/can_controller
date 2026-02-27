#ifndef PRESENTATION_NODE_HPP
#define PRESENTATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_utils/logger.hpp>

#include "domain/parameter.hpp"
#include "application/vertification.hpp"
#include "application/correction.hpp"

#define NODE_NAME "motion_validation"

namespace net::wavem::drive
{
    class MotionValidation final : public rclcpp::Node
    {
    private:
        rclcpp::Node::SharedPtr node_;
        Vertification::SharedPtr vertification_;
        Correction::SharedPtr correction_;
        Parameter::SharedPtr parameter_;
        void declare_parameters();

    public:
        explicit MotionValidation();
        virtual ~MotionValidation();
    };
}

#endif
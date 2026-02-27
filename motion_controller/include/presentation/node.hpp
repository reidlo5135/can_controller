#ifndef PRESENTATION_NODE_HPP
#define PRESENTATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_utils/logger.hpp>

#include "domain/parameter.hpp"
#include "domain/vehicle.hpp"
#include "application/converter.hpp"
#include "application/control.hpp"
#include "application/motion.hpp"

#define NODE_NAME "motion_controller"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace net::wavem::drive
{
    /**
     * @brief Class controlled by robot motion can communication
     * @author changunAn(changun516@wavem.net)
     * @date 23.04.05
     * @author reidlo(naru5135@wavem.net)
     * @date 25.01.19
     */
    class MotionController final : public rclcpp::Node
    {
    private:
        rclcpp::Node::SharedPtr node_;
        Control::SharedPtr control_;
        Motion::SharedPtr motion_;
        Parameter::SharedPtr parameter_;
        void declare_parameters();

    public:
        explicit MotionController();
        virtual ~MotionController();
    };
}

#endif

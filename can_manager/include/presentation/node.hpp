/**
 * @file presentation_node.hpp
 * @brief CAN Manager ROS2 Node Interface Header
 * @author reidlo
 * @date 2025.04.23
 */

#ifndef PRESENTATION_NODE_HPP
#define PRESENTATION_NODE_HPP

#include <unistd.h>
#include <iomanip>
#include <chrono>
#include <memory>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <can_utils/math.hpp>
#include <can_utils/adaptor.hpp>

#include "domain/parameter.hpp"
#include "application/c2r.hpp"
#include "application/r2c.hpp"

#define NODE_NAME "can_manager"

/**
 * @brief Namespace for drive packages
 * @namespace net::wavem::can
 * @date 2025.04.23
 */
namespace net::wavem::can
{
    /**
     * @class CanManager
     * @brief ROS2 node to manage CAN communication (send/receive)
     * @author changunAn, reidlo
     * @date 2025.04.23
     * @note Controls initialization, coordination between C2R and R2C components
     * @warning Be cautious with multithreaded access to CAN adaptor (mutex recommended)
     */
    class CanManager final : public rclcpp::Node
    {
    private:
        rclcpp::Node::SharedPtr node_;   ///< ROS2 Node shared pointer
        Parameter::SharedPtr parameter_; ///< CAN parameter configuration
        Math::SharedPtr can_math_;       ///< Utility for factor/offset calculations
        C2R::SharedPtr c2r_;             ///< CAN to ROS relay handler
        R2C::SharedPtr r2c_;             ///< ROS to CAN relay handler
        Adaptor::SharedPtr can_adaptor_; ///< CAN communication adaptor

        bool check_emergency_; ///< Emergency condition flag
        bool system_endian_;   ///< Endianness of the system

        /**
         * @brief Declare ROS2 parameters (used in initialization)
         * @date 2025.04.23
         */
        void declare_parameters();

        bool can_initialize();

    public:
        /**
         * @brief Construct a new CanManager object
         * @date 2025.04.23
         */
        explicit CanManager();

        /**
         * @brief Destroy the CanManager object
         * @date 2025.04.23
         */
        virtual ~CanManager();

        /**
         * @brief Main run loop for initializing and spinning CAN logic
         * @date 2025.04.23
         */
        void can_run();

        using SharedPtr = std::shared_ptr<CanManager>;
    };
}

#endif // PRESENTATION_NODE_HPP
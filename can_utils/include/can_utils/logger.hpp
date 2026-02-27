#ifndef CAN_UTILS_LOGGER_HPP
#define CAN_UTILS_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#define RCL_INFO(logger, format, ...) RCLCPP_INFO(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_WARN(logger, format, ...) RCLCPP_WARN(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_DEBUG(logger, format, ...) RCLCPP_DEBUG(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_ERROR(logger, format, ...) RCLCPP_ERROR(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCL_FATAL(logger, format, ...) RCLCPP_FATAL(logger, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)

#define RCUTILS_INFO(node_name, format, ...) RCUTILS_LOG_INFO_NAMED(node_name, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCUTILS_WARN(node_name, format, ...) RCUTILS_LOG_WARN_NAMED(node_name, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCUTILS_DEBUG(node_name, format, ...) RCUTILS_LOG_DEBUG_NAMED(node_name, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCUTILS_ERROR(node_name, format, ...) RCUTILS_LOG_ERROR_NAMED(node_name, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)
#define RCUTILS_FATAL(node_name, format, ...) RCUTILS_LOG_FATAL_NAMED(node_name, "%s():%d:" format, __func__, __LINE__, ##__VA_ARGS__)

#endif
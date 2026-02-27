#include "presentation/node.hpp"

/**
 * @brief The main function of the WmMotionController node.
 * @config argc int paramter , Number of factors delivered at the start of the program
 * @config argv char**, The factors you gave me at the start of the program
 * @return int Program normal operation
 * @author changunAn(changun516@wavem.net)
 * @date 23.04.06
 * @author reidlo(naru5135@wavem.net)
 * @date 25.01.19
 */
int main(int argc, const char *const *argv)
{
    rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr node = std::make_shared<net::wavem::drive::MotionController>();
    rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();

	return 0;
}
#include "presentation/node.hpp"

int main(int argc, const char* const *argv)
{
    rclcpp::init(argc,argv);
    net::wavem::can::CanManager::SharedPtr can_manager = std::make_shared<net::wavem::can::CanManager>();
    sleep(1);
    std::thread thread_run(&net::wavem::can::CanManager::can_run, can_manager);
    rclcpp::spin(can_manager);
    thread_run.join();
    return 0;
}
#include "driver_data_publisher.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROMUR::STMDataPublisher>());
    rclcpp::shutdown();
    return 0;
}
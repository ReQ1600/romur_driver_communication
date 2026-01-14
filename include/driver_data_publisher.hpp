#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "romur_interfaces/msg/romur_control.hpp"

#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

#include "serial_port.hpp"

namespace ROMUR
{
#define MAX_BUFFER_SIZE 512

#define MIN_BUFFER_SIZE     8
#define DEFAULT_BUFFER_SIZE 11

#define MAX_BAUDRATE 250000
#define MIN_BAUDRATE 300

#define MSG_SIZE 8  // must be the same as message size defined inside the stm!

class STMDataPublisher : public rclcpp::Node
{
  public:
    STMDataPublisher();

  private:
    unsigned int                                                         baudrate_;
    std::string                                                          port_name_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr         p_publisher_;
    rclcpp::Subscription<romur_interfaces::msg::ROMURControl>::SharedPtr p_subscriber_;
    rclcpp::TimerBase::SharedPtr                                         p_timer_;
    std::unique_ptr<ROMUR::SerialPort>                                   p_serial_port_;
    unsigned int                                                         buffer_size_;
    std::vector<uint8_t>                                                 buffer_;

    void readDataFromStm();
    void writeDataToStm(const romur_interfaces::msg::ROMURControl& msg);
    void exchangeData(const romur_interfaces::msg::ROMURControl msg);
};
}  // namespace ROMUR
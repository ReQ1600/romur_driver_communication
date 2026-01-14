#pragma once
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace ROMUR
{
class SerialPort
{
  public:
    SerialPort(const std::string&              port_name,
               const unsigned int&             baudrate,
               const unsigned int&             read_size,
               std::shared_ptr<rclcpp::Logger> lgr);

    ~SerialPort();

    // returns false on failure
    bool       Setup();
    inline int getSerialPort() const
    {
        return serial_port_;
    };

  private:
    const int                       serial_port_;
    const unsigned int              baudrate_;
    const unsigned int              read_size_;
    std::shared_ptr<rclcpp::Logger> p_logger_;
};
}  // namespace ROMUR
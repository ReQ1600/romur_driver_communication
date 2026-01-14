#include "driver_data_publisher.hpp"

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "romur_interfaces/msg/romur_control.hpp"

#include <poll.h>

ROMUR::STMDataPublisher::STMDataPublisher() : Node("stm_data_publisher")
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    rcl_interfaces::msg::IntegerRange        range;

    desc.description = "Serial port device file name";
    this->declare_parameter<std::string>("port", "/dev/ttyACM0", desc);

    desc.description = "Serial baudrate. Needs to allign with device baudrate";
    range.from_value = MIN_BAUDRATE;
    range.to_value   = MAX_BAUDRATE;
    desc.integer_range.push_back(range);
    this->declare_parameter<int>("baudrate", 115200, desc);
    desc.integer_range.pop_back();

    range.from_value = MIN_BUFFER_SIZE;
    range.to_value   = MAX_BUFFER_SIZE;
    desc.integer_range.push_back(range);
    desc.description = "Message frame size";
    this->declare_parameter<int>("buffer_size", DEFAULT_BUFFER_SIZE);

    p_publisher_ =
        this->create_publisher<std_msgs::msg::UInt8MultiArray>("driver_feedback_data", 10);

    p_subscriber_ = this->create_subscription<romur_interfaces::msg::ROMURControl>(
        "romur_control",
        10,
        std::bind(&STMDataPublisher::exchangeData, this, std::placeholders::_1));

    port_name_ = this->get_parameter("port").as_string();

    baudrate_ = this->get_parameter("baudrate").as_int();
    if (baudrate_ > MAX_BAUDRATE)
        baudrate_ = MAX_BAUDRATE;

    buffer_size_ = this->get_parameter("buffer_size").as_int();
    if (buffer_size_ > MAX_BUFFER_SIZE)
        buffer_size_ = MAX_BUFFER_SIZE;
    if (buffer_size_ < MIN_BUFFER_SIZE)
        buffer_size_ = MIN_BUFFER_SIZE;

    buffer_.resize(buffer_size_);

    // serial port as uniqueptr so it always closes socket on exit
    p_serial_port_ = std::make_unique<ROMUR::SerialPort>(
        port_name_, baudrate_, buffer_size_, std::make_shared<rclcpp::Logger>(this->get_logger()));
    if (!p_serial_port_->Setup())
        exit(0);
}

void ROMUR::STMDataPublisher::readDataFromStm()
{
    pollfd pfd;
    pfd.fd     = p_serial_port_->getSerialPort();
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 200);

    if (ret == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Serial read timeout");
        return;
    }
    else if (ret < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Poll error: %s", strerror(errno));
        return;
    }

    int msgSize = read(pfd.fd, buffer_.data(), buffer_size_);
    RCLCPP_INFO(this->get_logger(), "reading data");

    if (msgSize < 0)
        RCLCPP_ERROR(this->get_logger(),
                     "Encountered error %s while reading from %s",
                     strerror(errno),
                     port_name_.c_str());

    else if (msgSize == buffer_size_)
    {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.assign(buffer_.data(), buffer_.data() + msgSize);

        p_publisher_->publish(msg);
    }
}

void ROMUR::STMDataPublisher::writeDataToStm(const romur_interfaces::msg::ROMURControl& msg)
{
    uint8_t bf[MSG_SIZE] = {static_cast<uint8_t>(msg.motors.motor0_pwm),
                            static_cast<uint8_t>(msg.motors.motor1_pwm),
                            static_cast<uint8_t>(msg.motors.motor2_pwm),
                            static_cast<uint8_t>(msg.motors.motor3_pwm),
                            static_cast<uint8_t>(msg.light.status),
                            0x00,
                            0x00,
                            0x00};

    int msgSize = write(p_serial_port_->getSerialPort(), bf, 8);
    RCLCPP_INFO(this->get_logger(), "writing data");

    if (msgSize < 0)
        RCLCPP_ERROR(this->get_logger(),
                     "Encountered error %s while writing to %s",
                     strerror(errno),
                     port_name_.c_str());
}

void ROMUR::STMDataPublisher::exchangeData(const romur_interfaces::msg::ROMURControl msg)
{
    writeDataToStm(msg);
    RCLCPP_INFO(this->get_logger(), "writing done");
    readDataFromStm();
    RCLCPP_INFO(this->get_logger(), "reading done");
}
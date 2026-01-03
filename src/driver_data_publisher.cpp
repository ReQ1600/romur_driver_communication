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

namespace ROMUR
{
#define MAX_BUFFER_SIZE 512

#define MIN_BUFFER_SIZE     8
#define DEFAULT_BUFFER_SIZE 11

#define MAX_BAUDRATE 250000
#define MIN_BAUDRATE 300

#define MSG_SIZE 8  // must be the same as message size defined inside the stm!

std::unique_ptr<rclcpp::Logger> g_Logger;

class SerialPort
{
  public:
    SerialPort(const std::string&  port_name,
               const unsigned int& baudrate,
               const unsigned int& read_size)
        : serial_port_(open(port_name.c_str(), O_RDWR)),
          baudrate_(baudrate),
          read_size_(read_size) {};

    ~SerialPort()
    {
        close(serial_port_);
    };

    // returns false on failure
    bool Setup()
    {
        termios2 tty;

        // read existing config
        if (ioctl(serial_port_, TCGETS2, &tty))
        {
            RCLCPP_ERROR(*g_Logger, "tcgetattr returned with an error code: %d\n", errno);
            return false;
        }

        // setup tty
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        // wait time
        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN]  = read_size_;

        // for custom baudrate
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        tty.c_ispeed = baudrate_;
        tty.c_ospeed = baudrate_;

        // configure new settings
        if (ioctl(serial_port_, TCSETS2, &tty))
        {
            RCLCPP_ERROR(*g_Logger, "tcsetattr returned with error code: %d\n", errno);
            return false;
        }
        return true;
    }

    inline int getSerialPort() const
    {
        return serial_port_;
    };

  private:
    const int          serial_port_;
    const unsigned int baudrate_;
    const unsigned int read_size_;
};

class STMDataPublisher : public rclcpp::Node
{
  public:
    STMDataPublisher() : Node("stm_data_publisher")
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
            "motor_control",
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

        // p_timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
        //                                    std::bind(&STMDataPublisher::exchangeData, this));

        g_Logger = std::make_unique<rclcpp::Logger>(this->get_logger());

        // serial port as uniqueptr so it always closes socket on exit
        p_serial_port_ = std::make_unique<SerialPort>(port_name_, baudrate_, buffer_size_);
        if (!p_serial_port_->Setup())
            exit(0);
    }

  private:
    unsigned int                                                         baudrate_;
    std::string                                                          port_name_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr         p_publisher_;
    rclcpp::Subscription<romur_interfaces::msg::ROMURControl>::SharedPtr p_subscriber_;
    rclcpp::TimerBase::SharedPtr                                         p_timer_;
    std::unique_ptr<SerialPort>                                          p_serial_port_;
    unsigned int                                                         buffer_size_;
    std::vector<uint8_t>                                                 buffer_;

    void readDataFromStm()
    {
        pollfd pfd;
        pfd.fd     = p_serial_port_->getSerialPort();
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 200);

        if (ret == 0)
        {
            RCLCPP_WARN(*g_Logger, "Serial read timeout");
            return;
        }
        else if (ret < 0)
        {
            RCLCPP_ERROR(*g_Logger, "Poll error: %s", strerror(errno));
            return;
        }

        int msgSize = read(pfd.fd, buffer_.data(), buffer_size_);
        RCLCPP_INFO(*g_Logger, "reading data");

        if (msgSize < 0)
            RCLCPP_ERROR(*g_Logger,
                         "Encountered error %s while reading from %s",
                         strerror(errno),
                         port_name_.c_str());

        else if (msgSize == buffer_size_)
        {
            auto msg = std_msgs::msg::UInt8MultiArray();
            msg.data.assign(buffer_.data(), buffer_.data() + msgSize);

            p_publisher_->publish(msg);
        }
    };

    void writeDataToStm(const romur_interfaces::msg::ROMURControl& msg)
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
        RCLCPP_INFO(*g_Logger, "writing data");

        if (msgSize < 0)
            RCLCPP_ERROR(*g_Logger,
                         "Encountered error %s while writing to %s",
                         strerror(errno),
                         port_name_.c_str());
    }

    void exchangeData(const romur_interfaces::msg::ROMURControl msg)
    {
        writeDataToStm(msg);
        RCLCPP_INFO(*g_Logger, "writing done");
        readDataFromStm();
        RCLCPP_INFO(*g_Logger, "reading done");
    }
};
}  // namespace ROMUR

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROMUR::STMDataPublisher>());
    rclcpp::shutdown();
    return 0;
}
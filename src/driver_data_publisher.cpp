#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

namespace ROMUR
{
    #define MAX_BUFFER_SIZE 512

    #define MIN_BUFFER_SIZE 32
    #define DEFAULT_BUFFER_SIZE 128

    #define MAX_BAUDRATE 250000
    #define MIN_BAUDRATE 300
    
    std::unique_ptr<rclcpp::Logger> gLogger;

    class SerialPort
    {
    public:
        SerialPort(const std::string& port_name, const unsigned int& baudrate) : serial_port_(open(port_name.c_str(), O_RDWR)), baudrate_(baudrate) {};
        ~SerialPort()
        {
            close(serial_port_);
        };

        //returns false on failure
        bool Setup()
        {
            termios2 tty;

            //read existing config
            if (ioctl(serial_port_, TCGETS2, &tty))
            {
                RCLCPP_ERROR(*gLogger, "tcgetattr returned with an error code: %d\n", errno);
                return false;
            }

            //setup tty
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag |= CREAD | CLOCAL;

            tty.c_lflag |= ICANON;
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
            tty.c_cc[VMIN] = 0;

            //for custom baudrate
            tty.c_cflag &= ~CBAUD;
            tty.c_cflag |= CBAUDEX;
            tty.c_ispeed = baudrate_;
            tty.c_ospeed = baudrate_;

            //configure new settings
             if (ioctl(serial_port_, TCSETS2, &tty))
            {
                RCLCPP_ERROR(*gLogger, "tcsetattr returned with error code: %d\n", errno);
                return false;
            }
            return true;
        }
        
        inline int getSerialPort() const 
        {
            return serial_port_;
        };
        
    private:
        const int serial_port_;
        const unsigned int baudrate_;
    };

    class STMDataPublisher : public rclcpp::Node
    {
        public:
        STMDataPublisher() : Node("stm_data_publisher")
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            rcl_interfaces::msg::IntegerRange range;

            desc.description = "Serial port device file name";
            this->declare_parameter<std::string>("port", "/dev/ttyACM0", desc);

            desc.description = "Serial baudrate. Needs to allign with device baudrate";
            range.from_value = MIN_BAUDRATE;
            range.to_value = MAX_BAUDRATE;
            this->declare_parameter<int>("baudrate", 115200, desc);
            
            desc.integer_range.push_back(range);
            range.from_value = MIN_BUFFER_SIZE;
            range.to_value = MAX_BUFFER_SIZE;
            desc.description = "Message frame size";
            this->declare_parameter<int>("buffer_size", DEFAULT_BUFFER_SIZE);

            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("stm_data_publisher", 10);
            
            port_name_ = this->get_parameter("port").as_string();

            baudrate_ = this->get_parameter("baudrate").as_int();
            if (baudrate_ > MAX_BAUDRATE) baudrate_ = MAX_BAUDRATE;

            buffer_size_ = this->get_parameter("buffer_size").as_int();
            if (buffer_size_ > MAX_BUFFER_SIZE) buffer_size_ = MAX_BUFFER_SIZE;
            if (buffer_size_ < MIN_BUFFER_SIZE) buffer_size_ = MIN_BUFFER_SIZE;

            timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&STMDataPublisher::readDataFromStm, this));

            gLogger = std::make_unique<rclcpp::Logger>(this->get_logger());

            // serial port as uniqueptr so it always closes socket on exit 
            pSerial_port_ = std::make_unique<SerialPort>(port_name_, baudrate_);
            if (!pSerial_port_->Setup()) exit(0);
        }

        private:
            unsigned int baudrate_;
            std::string port_name_;
            rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::unique_ptr<SerialPort> pSerial_port_;
            unsigned int buffer_size_;
            uint8_t buffer_[MAX_BUFFER_SIZE] = {0};

            void readDataFromStm()
            {
                int msgSize = read(pSerial_port_->getSerialPort(), buffer_, sizeof(buffer_));
                RCLCPP_INFO(*gLogger, "reading data");

                if (msgSize < 0)
                    RCLCPP_ERROR(*gLogger, "readDataFromStm has encountered error: %s", strerror(errno));
                    
                auto msg = std_msgs::msg::UInt8MultiArray();
                msg.data.assign(buffer_, buffer_ + buffer_size_);

                publisher_->publish(msg);
            };
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROMUR::STMDataPublisher>());
    rclcpp::shutdown();
    return 0;
}
#include "serial_port.hpp"

#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

ROMUR::SerialPort::SerialPort(const std::string&              port_name,
                              const unsigned int&             baudrate,
                              const unsigned int&             read_size,
                              std::shared_ptr<rclcpp::Logger> lgr)
    : serial_port_(open(port_name.c_str(), O_RDWR)),
      baudrate_(baudrate),
      read_size_(read_size),
      p_logger_(std::move(lgr))
{
}

ROMUR::SerialPort::SerialPort::~SerialPort()
{
    close(serial_port_);
}

// returns false on failure
bool ROMUR::SerialPort::Setup()
{
    termios2 tty;

    // read existing config
    if (ioctl(serial_port_, TCGETS2, &tty))
    {
        RCLCPP_ERROR(*p_logger_, "tcgetattr returned with an error code: %d\n", errno);
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
        RCLCPP_ERROR(*p_logger_, "tcsetattr returned with error code: %d\n", errno);
        return false;
    }
    return true;
}
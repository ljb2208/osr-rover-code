#include "../include/serial.h"


serial::serial(string port, int32_t baud)
{
    this->port = port;
    this->baud = baud;
}

serial::~serial()
{

}

int32_t serial::open()
{
    serial_port = open(port.c_str(), O_RDWR);

    if (serial_port < 0)
        return serial_port;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));    
    
    if(tcgetattr(serial_port, &tty) != 0) {
       printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
 
    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    return 0;
}

void serial::close()
{
    close(serial_port);
}

bool serial::readBytes(char* read_buffer)
{
    int32_t bytesRead = read(serial_port, read_buffer, sizeof(read_buffer));

    if (bytesRead == sizeof(read_buffer))
        return true;
    else
        return false;
}

bool serial::writeBytes(char* write_buffer)
{
    write(serial_port, write_buffer, sizeof(write_buffer));

    return true;
}
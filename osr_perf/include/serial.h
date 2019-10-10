#ifndef SERIAL_H
#define SERIAL_H

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string>

using namespace std;


class serial
{
public:
    serial(string port, int32_t baud);
    ~serial();

    int32_t open();
    void close();

    bool readBytes(char* read_buffer);
    bool writeBytes(char* write_buffer);

private:
    string port;    
    int32_t baud;
    int32_t serial_port;

};
#endif



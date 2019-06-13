#ifndef INCLUDED_SERIAL_PORT_HPP
#define INCLUDED_SERIAL_PORT_HPP

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include <windows.h>

class serial_port
{
private:
    HANDLE handler;
    bool connected;
    COMSTAT status;
    DWORD errors;
public:
    serial_port(char *portName);
    ~serial_port();

    int readSerialPort(char *buffer, unsigned int buf_size);
    bool writeSerialPort(char *buffer, unsigned int buf_size);
    bool isConnected();
};

#endif  // INCLUDED_SERIALPORT_HPP
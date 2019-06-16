#include "serial/serial_port.hpp"
#include <algorithm>
#include <cstdio>

serial_port::serial_port(const char *portName)
{
    this->connected = false;

    this->handler = CreateFileA(static_cast<LPCSTR>(portName),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                NULL,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL,
                                NULL);
    if (this->handler == INVALID_HANDLE_VALUE){
        if (GetLastError() == ERROR_FILE_NOT_FOUND){
            printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
        }
    else
        {
            printf("ERROR!!!");
        }
    }
    else {
        DCB dcbSerialParameters = {0};

        if (!GetCommState(this->handler, &dcbSerialParameters)) {
            printf("failed to get current serial parameters");
        }
        else {
            dcbSerialParameters.BaudRate = CBR_9600;
            dcbSerialParameters.ByteSize = 8;
            dcbSerialParameters.StopBits = ONESTOPBIT;
            dcbSerialParameters.Parity = NOPARITY;
            dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

            if (!SetCommState(handler, &dcbSerialParameters))
            {
                printf("ALERT: could not set Serial port parameters\n");
            }
            else {
                this->connected = true;
                PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
                Sleep(ARDUINO_WAIT_TIME);
            }
        }
    }
}

serial_port::~serial_port()
{
    if (this->connected){
        this->connected = false;
        CloseHandle(this->handler);
    }
}

int serial_port::readSerialPort(char *buffer, unsigned int buf_size)
{
  DWORD bytesRead = 0;
  unsigned int toRead;

  ClearCommError(this->handler, &this->errors, &this->status);

  if (this->status.cbInQue > 0)
  {
    toRead = std::min((unsigned int)this->status.cbInQue, buf_size);
    ReadFile(this->handler, buffer, toRead, &bytesRead, NULL);
  }

  return bytesRead;
}

bool serial_port::writeSerialPort(char *buffer, unsigned int buf_size)
{
    DWORD bytesSend;

    if (!WriteFile(this->handler, (void*) buffer, buf_size, &bytesSend, 0)){
        ClearCommError(this->handler, &this->errors, &this->status);
        return false;
    }
    else return true;
}

bool serial_port::isConnected()
{
    return this->connected;
}

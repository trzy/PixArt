#include "serial/serial_port.hpp"
#include "util/format.hpp"
#include <stdexcept>
#include <algorithm>

serial_port::serial_port(const std::string &port_name, unsigned baud_rate)
  : m_connected(false)
{
  m_handler = CreateFileA(
    port_name.c_str(),
    GENERIC_READ | GENERIC_WRITE,
    0,
    NULL,
    OPEN_EXISTING,
    FILE_ATTRIBUTE_NORMAL,
    NULL);

  if (m_handler == INVALID_HANDLE_VALUE)
  {
    DWORD error = GetLastError();
    if (error == ERROR_FILE_NOT_FOUND)
    {
      throw std::runtime_error(util::format() << "Unable to open '" << port_name << "'. Is it busy or disconnected?");
    }
    else
    {
      throw std::runtime_error(util::format() << "Unable to open '" << port_name << "' (error code = " << util::hex((uint32_t) error) << ')');
    }
  }
  else
  {
    DCB dcb_serial_params = {0};

    if (!GetCommState(m_handler, &dcb_serial_params))
    {
      throw std::runtime_error("Failed to obtain current serial port parameters");
    }
    else
    {
      dcb_serial_params.BaudRate = baud_rate;
      dcb_serial_params.ByteSize = 8;
      dcb_serial_params.StopBits = ONESTOPBIT;
      dcb_serial_params.Parity = NOPARITY;
      dcb_serial_params.fDtrControl = DTR_CONTROL_ENABLE;

      if (!SetCommState(m_handler, &dcb_serial_params))
      {
        throw std::runtime_error("Failed to set serial port parameters");
      }
      else
      {
        m_connected = true;
        PurgeComm(m_handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
        Sleep(ARDUINO_WAIT_TIME);
      }
    }
  }
}

serial_port::~serial_port()
{
  if (m_connected)
  {
    m_connected = false;
    CloseHandle(m_handler);
  }
}

uint32_t serial_port::read(uint8_t *buffer, uint32_t buf_size)
{
  DWORD bytes_read = 0;
  uint32_t to_read;

  ClearCommError(m_handler, &m_errors, &m_status);

  if (m_status.cbInQue > 0)
  {
    to_read = std::min((unsigned int)m_status.cbInQue, buf_size);
    ReadFile(m_handler, buffer, to_read, &bytes_read, NULL);
  }

  return bytes_read;
}

bool serial_port::write(const uint8_t *buffer, uint32_t buf_size)
{
  DWORD bytes_sent;

  if (!WriteFile(m_handler, reinterpret_cast<const void *>(buffer), buf_size, &bytes_sent, 0))
  {
    ClearCommError(m_handler, &m_errors, &m_status);
    return false;
  }
  else
  {
    return true;
  }
}

bool serial_port::is_connected() const
{
  return m_connected;
}

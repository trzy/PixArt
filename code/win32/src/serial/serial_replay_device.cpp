#include "serial/serial_replay_device.hpp"
#include "util/logging.hpp"
#include "util/format.hpp"
#include <utility>
#include <stdexcept>

bool serial_replay_device::is_recording() const
{
  return m_serial_device != nullptr;
}

// Block is 32-bit size followed by bytes
void serial_replay_device::sync_to_block_boundary()
{
  if (m_read_idx == m_next_block_idx)
  {
    uint32_t size = *(uint32_t *) &m_buffer[m_read_idx];
    m_read_idx += sizeof(uint32_t);
    m_next_block_idx = m_read_idx + size;

    if (m_next_block_idx > m_buffer_size)
    {
      throw std::runtime_error(util::format() << "Encountered block exceeding file size in '" <<m_filename << "'. File is corrupt or has incorrect format.");
    }
  }
}

uint32_t serial_replay_device::read_from_block(uint8_t *buffer, uint32_t buf_size)
{
  if (m_read_idx == m_buffer_size)
  {
    return 0;
  }

  sync_to_block_boundary();

  size_t bytes_read = buf_size;
  size_t end = m_read_idx + buf_size;
  if (end >= m_next_block_idx)
  {
    // Consume only up to the end of the current block
    bytes_read = m_next_block_idx - m_read_idx;
    end = m_next_block_idx;
  }

  if (end >= m_buffer_size)
  {
    if (end == m_buffer_size)
    {
      LOG_INFO("Finished replay");
    }
    else
    {
      throw std::runtime_error(util::format() << "Attempted to read past end of file '" << m_filename << "'. File is corrupt or has incorrect format.");
    }
  }

  memcpy(buffer, &m_buffer[m_read_idx], bytes_read);
  m_read_idx += bytes_read;
  return bytes_read;
}

serial_replay_device::serial_replay_device(const std::string &capture_file, std::unique_ptr<i_serial_device> serial_device)
  : m_serial_device(std::move(serial_device)),
    m_filename(capture_file)
{
  m_of.open(capture_file.c_str(), std::ios::out | std::ios::binary);
  if (!m_of.is_open())
  {
    throw std::runtime_error(util::format() << "Failed to open '" << capture_file << "' for writing");
  }
}

serial_replay_device::serial_replay_device(const std::string &replay_file)
  : m_filename(replay_file)
{
  std::ifstream file(replay_file.c_str(), std::ios::in | std::ios::ate | std::ios::binary);
  if (!file.is_open())
  {
    throw std::runtime_error(util::format() << "Failed to open '" << replay_file << "' for reading");
  }
  m_buffer_size = file.tellg();
  m_buffer = std::make_unique<uint8_t[]>(m_buffer_size);
  file.seekg(0);
  file.read((char *) m_buffer.get(), m_buffer_size);
  if (!file)
  {
    throw std::runtime_error(util::format() << "Failed to read from '" << replay_file << "'");
  }
  file.close();
  if (m_buffer_size < 4)
  {
    throw std::runtime_error(util::format() << "File '" << replay_file << "' appears to have incorrect format");
  }
  sync_to_block_boundary();
}

serial_replay_device::~serial_replay_device()
{
  if (m_of.is_open())
  {
    m_of.close();
  }
}

uint32_t serial_replay_device::read(uint8_t *buffer, uint32_t buf_size)
{
  if (is_recording())
  {
    uint32_t num_bytes = m_serial_device->read(buffer, buf_size);
    if (num_bytes > 0)
    {
      m_of.write((const char *) &num_bytes, sizeof(uint32_t));
      m_of.write((const char *) buffer, num_bytes);
    }
    return num_bytes;
  }
  else
  {
    return read_from_block(buffer, buf_size);
  }
}

bool serial_replay_device::write(const uint8_t *buffer, uint32_t buf_size)
{
  if (is_recording())
  {
    return m_serial_device->write(buffer, buf_size);
  }
  else
  {
    // When replaying, writes always "succeed"
    return true;
  }
}

bool serial_replay_device::is_connected() const
{
  if (is_recording())
  {
    return m_serial_device->is_connected();
  }
  else
  {
    // "Disconnect" when replay is finished
    return m_read_idx >= m_buffer_size ? false : true;
  }
}

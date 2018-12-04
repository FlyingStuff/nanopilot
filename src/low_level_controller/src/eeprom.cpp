#include "eeprom.hpp"
#include <cstring>

bool EEPROM::write(size_t addr, size_t size, const void *buffer) const
{
    i2cAcquireBus(m_i2c);
    size_t bytes_written = 0;
    while (bytes_written < size) {
        size_t transmit_size = size - bytes_written;
        if (transmit_size > 64) {
            transmit_size = 64;
        }
        uint8_t transmit_buffer[64+2];
        size_t write_addr = addr + bytes_written;
        transmit_buffer[0] = static_cast<uint8_t>(write_addr >> 8);
        transmit_buffer[1] = static_cast<uint8_t>(write_addr);
        std::memcpy(&transmit_buffer[2], &static_cast<const uint8_t*>(buffer)[bytes_written], transmit_size);
        if (i2cMasterTransmit(m_i2c, m_addr, transmit_buffer, transmit_size+2, NULL, 0) != MSG_OK) {
            return false;
        }
        chThdSleepMilliseconds(6);
        bytes_written += transmit_size;
    }
    i2cReleaseBus(m_i2c);
    return true;
}

bool EEPROM::read(size_t addr, size_t size, void *buffer) const
{
    i2cAcquireBus(m_i2c);
    size_t bytes_read = 0;
    while (bytes_read < size) {
        size_t read_size = size-bytes_read;
        if (read_size > 64) {
            read_size = 64;
        }
        size_t read_addr = addr + bytes_read;
        uint8_t addr_buf[2] = {static_cast<uint8_t>(read_addr >> 8), static_cast<uint8_t>(read_addr)};
        if (i2cMasterTransmit(m_i2c, m_addr, addr_buf, 2, &static_cast<uint8_t*>(buffer)[bytes_read], read_size) != MSG_OK) {
            return false;
        }
        bytes_read += read_size;
    }
    i2cReleaseBus(m_i2c);
    return true;
}


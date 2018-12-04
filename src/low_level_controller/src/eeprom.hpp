#ifndef EEPROM_HPP
#define EEPROM_HPP

#include <hal.h>

class EEPROM {
public:
    EEPROM(I2CDriver *i2c, uint8_t addr) : m_i2c(i2c), m_addr(addr) {};
    bool write(size_t addr, size_t size, const void *buffer) const;
    bool read(size_t addr, size_t size, void *buffer) const;
private:
    I2CDriver *const m_i2c;
    const uint8_t m_addr;
};

#endif /* EEPROM_HPP */
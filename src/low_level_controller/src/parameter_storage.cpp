#include "log.h"
#include "eeprom.hpp"
#include <parameter/parameter_msgpack.h>
#include <crc/crc32.h>
#include "parameter_storage.h"

parameter_namespace_t parameters;
I2CDriver *_eeprom_i2c;
uint8_t _eeprom_addr;

static uint8_t parameter_storage_buffer[5*1024];
#define CRC32_INIT_VAL 0xABABABAB


struct storage_buffer_header_s {
    uint32_t crc;
    uint32_t buf_len;
};

void parameter_init(I2CDriver *eeprom_i2c, uint8_t eeprom_addr)
{
    parameter_namespace_declare(&parameters, NULL, NULL);
    _eeprom_i2c = eeprom_i2c;
    _eeprom_addr = eeprom_addr;
}

static void parameter_load_err_cb(void *arg, const char *id, const char *err)
{
    (void)arg;
    log_error("parameter load error %s: %s", id, err);
}

static void parameter_store_err_cb(void *arg, const char *id, const char *err)
{
    (void)arg;
    log_error("parameter store error %s: %s", id, err);
}

bool parameter_load_from_persistent_store()
{
    EEPROM eeprom(_eeprom_i2c, _eeprom_addr);
    struct storage_buffer_header_s header = {0, 0};
    bool read_error = false;
    read_error = read_error || !eeprom.read(0, sizeof(header), &header);
    if (header.buf_len == 0xffffffff) {
        log_error("parameter load failed, memory not initialized");
        return false;
    }
    if (header.buf_len > sizeof(parameter_storage_buffer)) {
        log_error("parameter load failed, buffer too small %d", (int)header.buf_len);
        return false;
    }
    read_error = read_error || !eeprom.read(64, header.buf_len, parameter_storage_buffer);
    if (read_error) {
        log_error("parameter load: eeprom read failed");
        return false;
    }
    if (header.crc != crc32(CRC32_INIT_VAL, parameter_storage_buffer, static_cast<size_t>(header.buf_len))) {
        log_error("parameter load failed, crc mismatch");
        return false;
    }
    return parameter_msgpack_read(&parameters,
                           parameter_storage_buffer,
                           header.buf_len,
                           parameter_load_err_cb,
                           NULL) == 0;
}

bool parameter_save_to_persistent_store()
{
    EEPROM eeprom(_eeprom_i2c, _eeprom_addr);
    size_t buf_len = sizeof(parameter_storage_buffer);
    int ret = parameter_msgpack_write(&parameters,
                            parameter_storage_buffer,
                            &buf_len,
                            parameter_store_err_cb,
                            NULL);
    if (ret != 0) {
        log_error("parameter save: serialization failed");
        return false;
    }
    struct storage_buffer_header_s header;
    header.buf_len = buf_len;
    header.crc = crc32(CRC32_INIT_VAL, parameter_storage_buffer, buf_len);
    bool write_error = false;
    write_error = write_error || !eeprom.write(0, sizeof(header), &header);
    write_error = write_error || !eeprom.write(64, buf_len, parameter_storage_buffer);
    if (write_error) {
        log_error("parameter save: eeprom write failed");
        return false;
    }
    return true;
}

bool parameter_erase_persistent_store()
{
    EEPROM eeprom(_eeprom_i2c, _eeprom_addr);
    struct storage_buffer_header_s header = {0xffffffff, 0xffffffff};
    if (!eeprom.write(0, sizeof(header), &header)) {
        log_error("parameter erase: eeprom write failed");
        return false;
    }
    return true;
}

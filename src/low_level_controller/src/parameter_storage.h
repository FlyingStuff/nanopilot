#ifndef PARAMETER_STORAGE_H
#define PARAMETER_STORAGE_H

#include <hal.h>
#include <parameter/parameter.h>

extern parameter_namespace_t parameters;


#ifdef __cplusplus
extern "C" {
#endif

void parameter_init(I2CDriver *eeprom_i2c, uint8_t eeprom_addr);
bool parameter_load_from_persistent_store(void);
bool parameter_save_to_persistent_store(void);
bool parameter_erase_persistent_store(void);

#ifdef __cplusplus
}
#endif


#endif /* PARAMETER_STORAGE_H */

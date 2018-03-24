#ifndef ANALOG_H
#define ANALOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define ANALOG_CH_VREFINT   0
#define ANALOG_CH_TEMP      1
#define ANALOG_CH_VDC_MON   2
#define ANALOG_CH_CONN2_TX  3
#define ANALOG_CH_CONN2_RX  4
#define ANALOG_CH_CONN3_TX  5
#define ANALOG_CH_CONN3_RX  6
#define ANALOG_NB_CHANNELS  7


void analog_start(void);
float analog_get_raw(unsigned int channel);
float analog_get_voltage(unsigned int channel);
float analog_get_vcc(void);
float analog_get_vdc(void);
float analog_get_cpu_temp(void);

void analog_configure_conn_3(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_H */

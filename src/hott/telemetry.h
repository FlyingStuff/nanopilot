/** HOTT Telemetry Messages */

/*
 * UART is 19200 Baud, 8 Bits, 1 Stopbit, no Parity
 */

#ifndef HOTT_TELEMETRY_H
#define HOTT_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** General Air Module (GAM) */
#define HOTT_TM_GAM_ALARM_OFF 0
#define HOTT_TM_GAM_ALARM_MIN_SPEED 1
#define HOTT_TM_GAM_ALARM_MAX_DISTANCE 4
#define HOTT_TM_GAM_ALARM_READ_TELEM 5
#define HOTT_TM_GAM_ALARM_MAX_SPEED 12
#define HOTT_TM_GAM_ALARM_MIN_ALT 15
#define HOTT_TM_GAM_ALARM_MIN_INP_VOLT 16
#define HOTT_TM_GAM_ALARM_MIN_CELL_VOLT 17
#define HOTT_TM_GAM_ALARM_MIN_FUEL 21
#define HOTT_TM_GAM_ALARM_CAPACITY 22
#define HOTT_TM_GAM_ALARM_MAX_CURRENT 23
#define HOTT_TM_GAM_ALARM_MAX_RPM 25
#define HOTT_TM_GAM_ALARM_MAX_ALT 26

struct hott_tm_gam_s {
    uint8_t alarm;
    float cell_volt[6];
    float batt1_volt;
    float batt2_volt;
    int16_t temp1;
    int16_t temp2;
    uint8_t fuel_percent;
    uint16_t fuel_ml;
    uint16_t rpm;
    float height;
    float climb_rate;
    float climb_rate_3s_avrg;
    float current;
    float voltage;
    uint16_t capacity_mAh;
    float speed;
    float lowest_cell_voltage;
    uint8_t lowest_cell_nbr;
    uint16_t rpm2;
    float pressure;
};

void hott_tm_gam_zero(struct hott_tm_gam_s *msg);

struct hott_tm_gps_s {
    uint8_t alarm;
    // todo
};

void hott_tm_gps_zero(struct hott_tm_gps_s *msg);




typedef struct {
    void (*putchar)(char c);
    void (*delay_ms)(int ms);
    struct hott_tm_gam_s *gam;
    struct hott_tm_gps_s *gps;
    bool (*text_cb)(char key, uint8_t *alarm, char *buf);
    enum {HOTT_TM_HANDLE_STATE_RESET,
          HOTT_TM_HANDLE_STATE_START_BINARY_RECEIVED,
          HOTT_TM_HANDLE_STATE_START_TEXT_RECEIVED} state;
} hott_tm_handler_t;

void hott_tm_handler_init(hott_tm_handler_t *tm, void (*putchar)(char c), void (*delay_ms)(int ms));
void hott_tm_handler_enable_gam(hott_tm_handler_t *tm, struct hott_tm_gam_s *gam);
void hott_tm_handler_enable_gps(hott_tm_handler_t *tm, struct hott_tm_gps_s *gps);
void hott_tm_handler_enable_text(hott_tm_handler_t *tm, bool (*text_cb)(char key, uint8_t *alarm, char *buf));
void hott_tm_handler_receive(hott_tm_handler_t *tm, char c);



// internal API
#define HOTT_TM_GAM_MSG_LEN 45
void hott_tm_gam_serialize(struct hott_tm_gam_s *msg, uint8_t *buf);
#define HOTT_TM_GPS_MSG_LEN 45
void hott_tm_gps_serialize(struct hott_tm_gps_s *msg, uint8_t *buf);
#define HOTT_TM_TEXT_MSG_LEN 173

#ifdef __cplusplus
}
#endif

#endif /* HOTT_TELEMETRY_H */
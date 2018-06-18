#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "telemetry.h"

#define BYTE_DELAY_MS 2
#define IDLE_LINE_MS 5


void hott_tm_gam_zero(struct hott_tm_gam_s *msg)
{
    memset(msg, 0, sizeof(struct hott_tm_gam_s));
}

void hott_tm_gps_zero(struct hott_tm_gps_s *msg)
{
    memset(msg, 0, sizeof(struct hott_tm_gam_s));
}

static uint8_t checksum(uint8_t *buf, size_t len)
{
    unsigned i;
    uint8_t sum = 0;
    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

static uint8_t saturate_u8(float value)
{
    if (value > UINT8_MAX) {
        return UINT8_MAX;
    }
    if (value < 0) {
        return 0;
    }
    return roundf(value);
}

static void saturate_u16(uint8_t *buf, float value)
{
    uint16_t value_i;
    if (value > UINT16_MAX) {
        value_i = UINT16_MAX;
    } else if (value < 0) {
        value_i = 0;
    } else {
        value_i = roundf(value);
    }

    buf[0] = (value_i & 0xff);
    buf[1] = ((value_i>>8) & 0xff);
}

void hott_tm_gam_serialize(struct hott_tm_gam_s *msg, uint8_t *buf)
{
    buf[0] = 0x7C;
    buf[1] = 0x8D;
    buf[2] = msg->alarm;
    buf[3] = 0xD0;
    buf[4] = 0; // alarm 1 inverted
    buf[5] = 0; // alarm 2 inverted
    buf[6] = saturate_u8(msg->cell_volt[0]*50);
    buf[7] = saturate_u8(msg->cell_volt[1]*50);
    buf[8] = saturate_u8(msg->cell_volt[2]*50);
    buf[9] = saturate_u8(msg->cell_volt[3]*50);
    buf[10] = saturate_u8(msg->cell_volt[4]*50);
    buf[11] = saturate_u8(msg->cell_volt[5]*50);
    buf[12] = saturate_u8(msg->cell_volt[5]*50);
    saturate_u16(&buf[12], msg->batt1_volt*10);
    saturate_u16(&buf[14], msg->batt2_volt*10);
    buf[16] = saturate_u8(msg->temp1+20);
    buf[17] = saturate_u8(msg->temp2+20);
    buf[18] = msg->fuel_percent;
    saturate_u16(&buf[19], msg->fuel_ml);
    saturate_u16(&buf[21], msg->rpm/10);
    saturate_u16(&buf[23], msg->height+500);
    saturate_u16(&buf[25], msg->climb_rate*100 + 30000);
    buf[27] = saturate_u8(msg->climb_rate_3s_avrg + 120);
    saturate_u16(&buf[28], msg->current*10);
    saturate_u16(&buf[30], msg->voltage*10);
    saturate_u16(&buf[32], msg->capacity_mAh);
    saturate_u16(&buf[34], msg->speed);
    buf[36] = saturate_u8(msg->lowest_cell_voltage*50);
    buf[37] = msg->lowest_cell_nbr;
    saturate_u16(&buf[38], msg->rpm2/10);
    buf[40] = 0; // general error nbr
    buf[41] = saturate_u8(msg->pressure*1e-4);
    buf[42] = 0; // version number
    buf[43] = 0x7D;
    buf[44] = checksum(buf, 44);
}

void hott_tm_gps_serialize(struct hott_tm_gps_s *msg, uint8_t *buf)
{
    buf[0] = 0x7C;
    buf[1] = 0x8A;
    buf[2] = msg->alarm;
    buf[3] = 0xA0;
    buf[4] = 0; // status 1 inverted
    buf[5] = 0; // status 2 inverted
    buf[6] = saturate_u8(msg->heading_deg/2);
    saturate_u16(&buf[7], msg->speed * 3.6f);
    buf[9] = (msg->latitude_deg < 0);
    uint16_t lat_min_integer = msg->latitude_arcminutes;
    uint16_t lat_DDMM = abs(msg->latitude_deg)*100 + lat_min_integer;
    uint16_t lat_fractional_MMMM = (msg->latitude_arcminutes - lat_min_integer)*10000;
    saturate_u16(&buf[10], lat_DDMM);
    saturate_u16(&buf[12], lat_fractional_MMMM); // (four fractional arminute decimals)
    buf[14] = (msg->longitude_deg < 0);
    uint16_t lon_min_integer = msg->longitude_arcminutes;
    uint16_t lon_DDMM = abs(msg->longitude_deg)*100 + lon_min_integer;
    uint16_t lon_fractional_MMMM = (msg->longitude_arcminutes - lon_min_integer)*10000;
    saturate_u16(&buf[15], lon_DDMM);
    saturate_u16(&buf[17], lon_fractional_MMMM); // (four fractional arminute decimals)
    saturate_u16(&buf[19], msg->distance);
    saturate_u16(&buf[21], msg->altitude+500);
    saturate_u16(&buf[23], msg->climb_rate*100 + 30000);
    buf[25] = saturate_u8(msg->climb_rate_3s_avrg + 120);
    buf[26] = msg->nb_gps_satellites;
    buf[27] = (uint8_t)msg->fix_char;
    buf[28] = (uint8_t)(msg->home_direction_deg/2);
    buf[29] = (uint8_t)(msg->roll_deg/2);
    buf[30] = (uint8_t)(msg->nick_deg/2);
    buf[31] = (uint8_t)(msg->compass_deg/2);
    buf[32] = msg->time_hours;
    buf[33] = msg->time_minutes;
    buf[34] = msg->time_seconds;
    buf[35] = msg->time_ms/10;
    saturate_u16(&buf[36], msg->altitude_above_mean_sea_level);
    buf[38] = msg->vibration_percent;
    buf[39] = (uint8_t)msg->free_char_1;
    buf[40] = (uint8_t)msg->free_char_2;
    buf[41] = (uint8_t)msg->free_char_3;
    buf[42] = 255; // version
    buf[43] = 0x7d;
    buf[44] = checksum(buf, 44);
}



void hott_tm_handler_init(hott_tm_handler_t *tm, void (*putchar)(char c), void (*delay_ms)(int ms))
{
    tm->putchar = putchar;
    tm->delay_ms = delay_ms;
    tm->gam = NULL;
    tm->gps = NULL;
    tm->text_cb = NULL;
    tm->state = HOTT_TM_HANDLE_STATE_RESET;
}

void hott_tm_handler_enable_gam(hott_tm_handler_t *tm, struct hott_tm_gam_s *gam)
{
    tm->gam = gam;
}

void hott_tm_handler_enable_gps(hott_tm_handler_t *tm, struct hott_tm_gps_s *gps)
{
    tm->gps = gps;
}

void hott_tm_handler_enable_text(hott_tm_handler_t *tm, bool (*text_cb)(char key, uint8_t *alarm, char *buf))
{
    tm->text_cb = text_cb;
}


static void handle_gam(hott_tm_handler_t *tm, uint8_t *buf)
{
    hott_tm_gam_serialize(tm->gam, buf);
    int i;
    tm->delay_ms(IDLE_LINE_MS);
    for (i = 0; i < HOTT_TM_GAM_MSG_LEN; i++) {
        tm->putchar(buf[i]);
        tm->delay_ms(BYTE_DELAY_MS);
    }
}

static void handle_gps(hott_tm_handler_t *tm, uint8_t *buf)
{
    hott_tm_gps_serialize(tm->gps, buf);
    int i;
    tm->delay_ms(IDLE_LINE_MS);
    for (i = 0; i < HOTT_TM_GPS_MSG_LEN; i++) {
        tm->putchar(buf[i]);
        tm->delay_ms(BYTE_DELAY_MS);
    }
}

static void handle_text(hott_tm_handler_t *tm, uint8_t *buf, char c)
{
    tm->delay_ms(IDLE_LINE_MS);
    uint8_t alarm = 0;
    char key = c & 0x0f;
    bool esc = tm->text_cb(key, &alarm, (char*)&buf[3]);
    buf[0] = 0x7b;
    if (esc) {
        buf[1] = 0x01;
    } else {
        buf[1] = 0xD0;
    }
    buf[2] = alarm;
    buf[171] = 0x7D;
    buf[172] = checksum(buf, HOTT_TM_TEXT_MSG_LEN);
    int i;
    for (i = 0; i < HOTT_TM_TEXT_MSG_LEN; i++) {
        tm->putchar(buf[i]);
        tm->delay_ms(BYTE_DELAY_MS);
    }
}

void hott_tm_handler_receive(hott_tm_handler_t *tm, char c)
{
    uint8_t buf[HOTT_TM_TEXT_MSG_LEN];
    switch (tm->state) {

    case HOTT_TM_HANDLE_STATE_RESET:
        if ((uint8_t)c == 0x80) {
            tm->state = HOTT_TM_HANDLE_STATE_START_BINARY_RECEIVED;
        }
        if ((uint8_t)c == 0x7F) {
            tm->state = HOTT_TM_HANDLE_STATE_START_TEXT_RECEIVED;
        }
        break;

    case HOTT_TM_HANDLE_STATE_START_BINARY_RECEIVED:
        if ((uint8_t)c == 0x8D && tm->gam) {
            handle_gam(tm, buf);
        } else if ((uint8_t)c == 0x8A && tm->gps) {
            handle_gps(tm, buf);
        }
        tm->state = HOTT_TM_HANDLE_STATE_RESET;
        break;

    case HOTT_TM_HANDLE_STATE_START_TEXT_RECEIVED:
        if (((uint8_t)c & 0xf0) == 0xD0 && tm->text_cb && tm->gam) {
            handle_text(tm, buf, c);
        }
        tm->state = HOTT_TM_HANDLE_STATE_RESET;
        break;

    default:
        tm->state = HOTT_TM_HANDLE_STATE_RESET;
        break;
    }
}


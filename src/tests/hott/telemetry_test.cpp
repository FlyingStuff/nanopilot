#include "../../hott/telemetry.h"
#include <CppUTest/TestHarness.h>



TEST_GROUP(HOTT_TM)
{

};

uint8_t checksum(uint8_t *buf, size_t len)
{
    unsigned i;
    uint8_t sum = 0;
    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

TEST(HOTT_TM, gam_serialize)
{
    struct hott_tm_gam_s msg;
    hott_tm_gam_zero(&msg);
    msg.batt1_volt = 5;
    msg.climb_rate = -20;

    uint8_t buf[HOTT_TM_GAM_MSG_LEN];
    hott_tm_gam_serialize(&msg, buf);
    uint8_t exp[HOTT_TM_GAM_MSG_LEN] = {
        0x7C, 0x8D, 0, 0xD0, 0, 0,
        0, 0, 0, 0, 0, 0, // cells
        50, 0, 0, 0, // batt
        20, 20, // temp
        0, 0, 0, // fuel
        0, 0, 244, 1, // rpm & height
        96, 109, 120, // climb rate
        0, 0, 0, 0, 0, 0, // current, voltage & capacity
        0, 0, // speed
        0, 0, 0, 0, 0, 0, 0, 0x7d,
    };
    exp[HOTT_TM_GAM_MSG_LEN - 1] = checksum(exp, HOTT_TM_GAM_MSG_LEN-1);
    MEMCMP_EQUAL(exp, buf, HOTT_TM_GAM_MSG_LEN);
}

TEST(HOTT_TM, GAM_send)
{
    
}

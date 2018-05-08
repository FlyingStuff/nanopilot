#include "sumd.h"

#define STATE_RESET             0
#define STATE_VENDOR_RECEIVED   1
#define STATE_STATUS_RECEIVED   2
#define STATE_NB_CH_RECEIVED    3


void sumd_receiver_init(struct sumd_receiver_s *receiver)
{
    receiver->_state = STATE_RESET;
    receiver->error_cnt = 0;
}


static uint16_t crc16(uint16_t crc, uint8_t value)
{
    const uint16_t polynome = 0x1021;
    uint8_t i;
    crc = crc ^ (int16_t)value << 8;
    for(i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ polynome;
        } else {
            crc = (crc << 1);
        }
    }
    return crc;
}


static inline void increment_error_cnt(uint16_t *error_cnt)
{
    if (*error_cnt < 0xffff) {
        (*error_cnt)++;
    }
}


int sumd_receive(struct sumd_receiver_s *receiver, uint8_t input_byte)
{
    if (receiver->_state >= STATE_NB_CH_RECEIVED) { // chanels & crc
        int rcv_channel = (receiver->_state - STATE_NB_CH_RECEIVED) / 2;
        bool first_byte = !((receiver->_state - STATE_NB_CH_RECEIVED) & 0x01);
        if (rcv_channel < receiver->nb_channels) { // channel
            receiver->_crc = crc16(receiver->_crc, input_byte);
            if (first_byte) { // high byte
                receiver->channel[rcv_channel] = (uint16_t)input_byte << 8;
            } else { // low byte
                receiver->channel[rcv_channel] |= (uint16_t)input_byte;
            }
            receiver->_state++;
            return SUMD_RECEIVE_ONGOING;
        }
        if (rcv_channel == receiver->nb_channels) { // crc
            if (first_byte) { // high byte
                if (input_byte != ((receiver->_crc >> 8) & 0xff)) {
                    increment_error_cnt(&receiver->error_cnt);
                    goto error;
                }
                receiver->_state++;
                return SUMD_RECEIVE_ONGOING;
            } else { // low byte
                if (input_byte != (receiver->_crc & 0xff)) {
                    increment_error_cnt(&receiver->error_cnt);
                    goto error;
                }
                receiver->_state = STATE_RESET;
                return SUMD_RECEIVE_COMPLETE;
            }
        }
    }
    if (receiver->_state == STATE_RESET) {
        if (input_byte == 0xA8) { // Vendor ID: Graupner
            receiver->_crc = crc16(0x0000, 0xA8);
            receiver->_state = STATE_VENDOR_RECEIVED;
            return SUMD_RECEIVE_ONGOING;
        }
        goto error;
    }
    if (receiver->_state == STATE_VENDOR_RECEIVED) {
        receiver->_crc = crc16(receiver->_crc, input_byte);
        if (input_byte == 0x01) { // live frame
            receiver->no_signal_failsafe = false;
            receiver->_state = STATE_STATUS_RECEIVED;
            return SUMD_RECEIVE_ONGOING;
        } else if (input_byte == 0x81) { // failsafe frame
            receiver->no_signal_failsafe = true;
            receiver->_state = STATE_STATUS_RECEIVED;
            return SUMD_RECEIVE_ONGOING;
        }
        goto error;
    }
    if (receiver->_state == STATE_STATUS_RECEIVED) {
        receiver->_crc = crc16(receiver->_crc, input_byte);
        if (input_byte >= 0x02 && input_byte <= 0x20) { // valid number of channels
            receiver->_state = STATE_NB_CH_RECEIVED;
            receiver->nb_channels = input_byte;
            return SUMD_RECEIVE_ONGOING;
        }
        goto error;
    }
error:
    receiver->_state = STATE_RESET;
    return SUMD_RECEIVE_ERROR;
}

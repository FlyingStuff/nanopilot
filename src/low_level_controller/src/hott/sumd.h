#ifndef HOTTSUMD_H
#define HOTTSUMD_H

/*
 * HoTT SUMD sum-signal protocol
 *
 * data is sent 115200 Bit/s, 8 Databits, no Paritybit, 1 Stopbit on highest
 * servo channel.
 *
 * set CH OUT TYPE setting (on SMART-BOX) to SUMDXXYY where:
 * XX: no signal behaviour {OF: stop sending, HD: hold, FS: failsafe position},
 *  recommended: set XX to FS.
 *  note: You should flash the *_FS_gr_rx*.bin firmware version on your receiver
 *        so that the no_signal_failsafe flag is correctly set.
 * YY: number of channels to be sent
 */

#include <stdint.h>
#include <stdbool.h>

#define SUMD_MAX_NB_CHANNELS 0x20

struct sumd_receiver_s {
    uint16_t _crc;              // [internal] (crc buffer)
    uint8_t _state;             // [internal] (state machine)
    uint8_t nb_channels;        // number of channels received in frame
    bool no_signal_failsafe;    // whether the receiver is in failsafe mode
    uint16_t error_cnt;         // the number of crc errors since initialization
    uint16_t channel[SUMD_MAX_NB_CHANNELS]; // servo positions in 1/8 us
};

// channel values [1/8 us]
#define SUMD_POS_EXTENDED_LOW    0x1c20  // (-150%)  900μs pulse length
#define SUMD_POS_LOW             0x2260  // (-100%) 1100μs pulse length
#define SUMD_POS_NEUTRAL         0x2ee0  // (0%)    1500μs pulse length
#define SUMD_POS_HIGH            0x3b60  // (+100%) 1900μs pulse length
#define SUMD_POS_EXTENDED_HIGH   0x41a0  // (+150%) 2100μs pulse length

#ifdef __cplusplus
extern "C" {
#endif

void sumd_receiver_init(struct sumd_receiver_s *receiver);

#define SUMD_RECEIVE_ONGOING    0
#define SUMD_RECEIVE_COMPLETE   1
#define SUMD_RECEIVE_ERROR      2
int sumd_receive(struct sumd_receiver_s *receiver, uint8_t input_byte);

#ifdef __cplusplus
}
#endif

#endif // HOTTSUMD_H

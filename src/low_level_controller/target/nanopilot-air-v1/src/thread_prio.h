#ifndef THREAD_PRIO_H
#define THREAD_PRIO_H

// use priorities in the range of 2 to 127, higher number means higher priority

#define THD_PRIO_LED                                    2
#define THD_PRIO_SHELL                                  80
#define THD_PRIO_COMM_RX                                30
#define THD_PRIO_COMM_TX                                30
#define THD_PRIO_RC_SUMD_IN                             101
#define THD_PRIO_CONTROL_LOOP                           100
#define THD_PRIO_HOTT_TM                                20

#endif /* THREAD_PRIO_H */

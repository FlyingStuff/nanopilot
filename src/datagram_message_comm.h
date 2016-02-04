#ifndef DATAGRAM_MESSAGE_COMM_H
#define DATAGRAM_MESSAGE_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

void datagram_message_init(void);
void datagram_message_start(BaseSequentialStream *in);
void datagram_message_send(const char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* DATAGRAM_MESSAGE_COMM_H */

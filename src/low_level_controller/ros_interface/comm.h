#ifndef ROS_INTERFACE_COMM_H
#define ROS_INTERFACE_COMM_H

#include <stdint.h>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#include <pthread.h>
#else // ChibiOS
#include <ch.h>
#endif

#include <serial-datagram/serial_datagram.h>



typedef uint16_t comm_msg_id_t;

#define ROS_INTERFACE_MTU 1024

typedef void (*comm_rcv_cb_t)(comm_msg_id_t msg_id, const uint8_t *msg, size_t len);

typedef struct {
#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    int fd;
    pthread_mutex_t send_lock;
#else // ChibiOS
    BaseSequentialStream *fd;
    mutex_t send_lock;
#endif
    uint8_t send_msg_buffer[ROS_INTERFACE_MTU+sizeof(comm_msg_id_t)];
    uint8_t send_datagram_buffer[ROS_INTERFACE_MTU*2];
    uint8_t rcv_datagram_buffer[ROS_INTERFACE_MTU*2];
    serial_datagram_rcv_handler_t datagram_rcv_handler;
    comm_rcv_cb_t rcv_cb;
} comm_interface_t;



#ifdef __cplusplus
extern "C" {
#endif


#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
bool comm_init(comm_interface_t *i,
               const char *serial_device,
               unsigned baudrate,
               comm_rcv_cb_t rcv_cb);
#else // ChibiOS
void comm_init(comm_interface_t *i,
               BaseSequentialStream* serial_device,
               comm_rcv_cb_t rcv_cb);
#endif

void comm_send(comm_interface_t *i,
               comm_msg_id_t msg_id,
               const void *msg,
               size_t size);

void comm_receive(comm_interface_t *i);

#ifdef __cplusplus
}
#endif

#endif /* ROS_INTERFACE_COMM_H */

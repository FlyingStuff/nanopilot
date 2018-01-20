#ifndef NET_H
#define NET_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <mcucom_port_sync.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NET_BROADCAST_DEST 0

// header [destination 8bit, source 8bit, priority 3bit, protocol 5bit]
#define NET_HEADER_LEN 3

typedef struct net_node_s net_node_t;


struct net_route_tab_entry_s {
    uint8_t dest_addr; // destination to be routed
    uint8_t dest_mask; // bits set to 1 will be compared
    int8_t link_layer_via_interface_idx; // index of the interface to forward, -1 for sentinel at end of list
    uint8_t link_layer_via_addr; // 0 means the node is directly reachable, ll address is dest addr
};


typedef void (*net_protocol_rcv_cb_t)(net_node_t *node, const char *pkt, size_t len, uint8_t src_addr, uint8_t prio, uint8_t interface_idx);

struct net_protocol_table_entry_s {
    int8_t protocol_nbr; // last entry set to -1 for sentinel
    net_protocol_rcv_cb_t protocol_rcv_cb;
};


struct net_if_s {
    void (*send_fn)(void *arg, const char *frame, size_t len, uint8_t prio, uint8_t dest); // driver send function, NULL for sentinel at end of list
    void *arg; // argument to be passed to the driver send function
    mcucom_port_mutex_t send_lock; // lock is initialized by net_node_init
};


struct net_node_s {
    uint8_t own_addr;
    uint8_t routing_table_size;
    struct net_route_tab_entry_s *routing_table;
    struct net_if_s *interface_list;
    const struct net_protocol_table_entry_s *protocol_table;
    mcucom_port_mutex_t node_lock;
};


/** Initialize Network Node
 * @parameter [out] node node to be initialized
 * @parameter [in] addr network address of the node
 * @parameter [in] routing_table routing table
 * @parameter [in] routing_table_size size of the routing table in nb of entries
 * @parameter [in] interface_list list of link layer interfaces
 * @parameter [in] protocol_table list of protocol callbacks
 */
void net_node_init(net_node_t *node,
                   uint8_t addr,
                   struct net_route_tab_entry_s *routing_table,
                   uint8_t routing_table_size,
                   struct net_if_s *interface_list,
                   const struct net_protocol_table_entry_s *protocol_table);

/** Handle incoming frame
 * @parameter [in,out] node node object
 * @parameter [in] frame received frame
 * @parameter [in] len length of the received frame
 * @parameter [in] interface_idx index of the interface (in the node interface list)
 * @parameter [in] frame_src_addr link layer source address of the frame
 */
void net_handle_incoming_frame(net_node_t *node,
                               const char *frame,
                               size_t len,
                               uint8_t interface_idx,
                               uint8_t frame_src_addr);

/** Write network header to frame and send it using the nodes routing table
 * @parameter [in, out] node node object
 * @parameter [in] protocol protocol number used
 * @parameter [in] frame frame-buffer with the packet to be sent, the first three bytes should be reserved for the header and are overwritten
 * @parameter [in] len length of the frame (including the header)
 * @parameter [in] dest_addr destination address to which the packet should be sent
 * @parameter [in] priority priority with which the packet should be sent
 */
bool net_write_header_and_send_frame(net_node_t *node,
                                     uint8_t protocol,
                                     char *frame,
                                     size_t len,
                                     uint8_t dest_addr,
                                     uint8_t priority);

// todo
bool net_write_header_and_broadcast_frame(net_node_t *node,
                                          uint8_t protocol,
                                          char *frame,
                                          size_t len,
                                          uint8_t interface_idx,
                                          uint8_t priority);


/** Adds a new route to the end of the routing table of a node
 * @parameter [in,out] node node for which the routing table should be modified
 * @parameter [in] dest_addr destination to be routed
 * @parameter [in] dest_mask bits for which the destination should match the address
 * @parameter [in] ll_via_interface_idx interface index to which the packet should be routed
 * @parameter [in] ll_via_addr interface address to which the packet should be routed
 * @returns true if added, false if there was no more space in the routing table
 */
bool net_route_add(net_node_t *node,
                   uint8_t dest_addr,
                   uint8_t dest_mask,
                   uint8_t ll_via_interface_idx,
                   uint8_t ll_via_addr);

/** searches the routing table for the forwarding interface & address
 * @parameter [in] node node with a routing table
 * @parameter [in] addr the address to be routed
 * @parameter [out] via_addr the address to which the packet should be forwarded
 * @returns the interface index or -1 if no route exists
 * @note if multiple routes match the destination, the first in the table is returned
 */
int8_t net_route_lookup(net_node_t *node,
                        uint8_t addr,
                        uint8_t *via_addr);



/** @defgroup internal  Internal API
 *  @{ */


/** Adds a new route to the end of the routing table
 * @parameter [in,out] tab routing table to be modified
 * @parameter [in] tab_len length of the routing table (max nb. elements)
 * @parameter [in] dest_addr destination to be routed
 * @parameter [in] dest_mask bits for which the destination should match the address
 * @parameter [in] ll_via_interface_idx interface index to which the packet should be routed
 * @parameter [in] ll_via_addr interface address to which the packet should be routed
 * @returns true if added, false if there was no more space in the routing table
 *
 * @note must be called with the node locked
 */
bool _net_route_add_with_lock(struct net_route_tab_entry_s *tab,
                              uint8_t tab_len,
                              uint8_t dest_addr,
                              uint8_t dest_mask,
                              uint8_t ll_via_interface_idx,
                              uint8_t ll_via_addr);


/** searches the routing table for the forwarding interface & address
 * @parameter [in] tab routing table pointer
 * @parameter [in] addr the address to be routed
 * @parameter [out] via_addr the address to which the packet should be forwarded
 * @returns the interface index or -1 if no route exists
 *
 * @note if multiple routes match the destination, the first in the table is returned
 * @note must be called with the node locked
 */
int8_t _net_route_lookup_with_lock(const struct net_route_tab_entry_s *tab,
                                   uint8_t addr,
                                   uint8_t *via_addr);


/** Write header bytes to the buffer
 * @parameter [in] dest destination, 8 bits
 * @parameter [in] src source, 8 bits
 * @parameter [in] prio packet priority, 3 bits
 * @parameter [in] protocol protocol number, 5 bits
 * @note this function writes the first 3 bytes of the buffer
 */
void _net_write_header(char *buf,
                       uint8_t dest,
                       uint8_t src,
                       uint8_t prio,
                       uint8_t protocol);

/** Read header from the buffer
 * @parameter [out] dest destination, 8 bits
 * @parameter [out] src source, 8 bits
 * @parameter [out] prio packet priority, 3 bits
 * @parameter [out] protocol protocol number, 5 bits
 * @note this function reads the first 3 bytes of the buffer
 */
void _net_read_header(const char *buf,
                      uint8_t *dest,
                      uint8_t *src,
                      uint8_t *prio,
                      uint8_t *protocol);


/** Lookup protocol table to find the packet handler for the protocol number
 * @parameter [in] protocol_list array of protocol entries, terminated by a -1 protocol number as sentinel
 * @parameter [in] protocol the protocol number requested
 * @returns the packet handler corresponding to the protocol number
 */
net_protocol_rcv_cb_t _net_get_protocol_cb(const struct net_protocol_table_entry_s *protocol_list, uint8_t protocol);


/** Send frame over network interface
 * @parameter [in, out] net_if pointer to the network interface
 * @parameter [in] frame pointer to the frame buffer
 * @parameter [in] len length of the frame buffer
 * @parameter [in] priority priority of the frame
 * @parameter [in] ll_addr link layer address to which the frame is sent
 */
void _net_if_send_frame(struct net_if_s *net_if, const char *frame, size_t len, uint8_t priority, uint8_t ll_addr);


/** @} */ // end of internal API



#ifdef __cplusplus
}
#endif

#endif /* NET_H */

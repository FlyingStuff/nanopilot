#include "net.h"


int8_t net_route_lookup(const struct net_route_tab_entry_s *tab, uint8_t addr, uint8_t *via_addr)
{
    while (tab->link_layer_via_interface_idx != -1) {
        if ((tab->dest_addr & tab->dest_mask) == (addr & tab->dest_mask)) {
            *via_addr = tab->link_layer_via_addr;
            return tab->link_layer_via_interface_idx;
        }
        tab++;
    }
    return -1;
}


bool net_route_add(struct net_route_tab_entry_s *tab, uint8_t tab_len, uint8_t dest_addr, uint8_t dest_mask, uint8_t ll_via_interface_idx, uint8_t ll_via_addr)
{
    int i = 0;
    while (i < tab_len -1) {
        if (tab[i].link_layer_via_interface_idx == -1) {
            tab[i+1].link_layer_via_interface_idx = -1; // move sentinel
            tab[i].dest_addr = dest_addr;
            tab[i].dest_mask = dest_mask;
            tab[i].link_layer_via_interface_idx = ll_via_interface_idx;
            tab[i].link_layer_via_addr = ll_via_addr;
            return true;
        }
        i++;
    }
    return false;
}



void net_write_header(char *buf, uint8_t dest, uint8_t src, uint8_t prio, uint8_t protocol)
{
    uint8_t *header = (uint8_t*)buf;
    header[0] = dest;
    header[1] = src;
    header[2] = ((prio & 0x07) << 5) + (protocol & 0x1F);
}


void net_read_header(const char *buf, uint8_t *dest, uint8_t *src, uint8_t *prio, uint8_t *protocol)
{
    uint8_t *header = (uint8_t*)buf;
    *dest = header[0];
    *src = header[1];
    *prio = header[2] >> 5;
    *protocol = header[2] & 0x1F;
}



net_protocol_rcv_cb_t net_get_protocol_cb(const struct net_protocol_table_entry_s *protocol_list, uint8_t protocol)
{
    const struct net_protocol_table_entry_s *entry = protocol_list;
    while (entry->protocol_nbr != -1) {
        if (entry->protocol_nbr == protocol) {
            return entry->protocol_rcv_cb;
        }
        entry++;
    }
    return NULL;
}


void net_node_init(net_node_t *node,
                   uint8_t addr,
                   struct net_route_tab_entry_s *routing_table,
                   uint8_t routing_table_size,
                   const net_if_t *interface_list,
                   const struct net_protocol_table_entry_s *protocol_table)
{
    node->own_addr = addr;
    node->routing_table_size = routing_table_size;
    node->routing_table = routing_table;
    node->interface_list = interface_list;
    node->protocol_table = protocol_table;
}

void net_handle_incoming_frame(net_node_t *node, const char *frame, size_t len, uint8_t interface_idx, uint8_t frame_src_addr)
{
    uint8_t dest, src_addr, prio, protocol;
    net_read_header(frame, &dest, &src_addr, &prio, &protocol);

    // update routing table
    uint8_t _via_addr;
    if (net_route_lookup(node->routing_table, src_addr, &_via_addr) == -1) {
        net_route_add(node->routing_table, node->routing_table_size, src_addr, 0xff, interface_idx, frame_src_addr);
    }

    if (dest == node->own_addr || dest == NET_BROADCAST_DEST) {
        // call protocol handler
        net_protocol_rcv_cb_t cb = net_get_protocol_cb(node->protocol_table, protocol);
        if (cb != NULL) {
            cb(&frame[NET_HEADER_LEN], len - NET_HEADER_LEN, src_addr, prio, interface_idx);
        }
    } else {
        // route packet
        uint8_t via_addr;
        int8_t if_idx = net_route_lookup(node->routing_table, dest, &via_addr);
        if (if_idx != -1) {
            net_if_t if_obj = node->interface_list[if_idx];
            if_obj.send_fn(if_obj.arg, frame, len, prio, via_addr);
        }
    }

}

bool net_write_header_and_send_frame(net_node_t *node, uint8_t protocol, char *frame, size_t len, uint8_t dest_addr, uint8_t priority)
{
    net_write_header(frame, dest_addr, node->own_addr, priority, protocol);
    uint8_t via_addr;
    int8_t if_idx = net_route_lookup(node->routing_table, dest_addr, &via_addr);
    if (if_idx != -1) {
        net_if_t if_obj = node->interface_list[if_idx];
        if_obj.send_fn(if_obj.arg, frame, len, priority, via_addr);
        return true;
    }
    return false; // no route to host
}


#include "net.h"


int8_t net_route_lookup(net_node_t *node, uint8_t addr, uint8_t *via_addr)
{
    mcucom_port_mutex_acquire(&node->node_lock);

    int8_t via_if = _net_route_lookup_with_lock(node->routing_table, addr, via_addr);

    mcucom_port_mutex_release(&node->node_lock);
    return via_if;
}

int8_t _net_route_lookup_with_lock(const struct net_route_tab_entry_s *tab, uint8_t addr, uint8_t *via_addr)
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


bool net_route_add(net_node_t *node, uint8_t dest_addr, uint8_t dest_mask, uint8_t ll_via_interface_idx, uint8_t ll_via_addr)
{
    mcucom_port_mutex_acquire(&node->node_lock);

    bool success = _net_route_add_with_lock(node->routing_table, node->routing_table_size, dest_addr, dest_mask, ll_via_interface_idx, ll_via_addr);

    mcucom_port_mutex_release(&node->node_lock);
    return success;
}

bool _net_route_add_with_lock(struct net_route_tab_entry_s *tab, uint8_t tab_len, uint8_t dest_addr, uint8_t dest_mask, uint8_t ll_via_interface_idx, uint8_t ll_via_addr)
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



void _net_write_header(char *buf, uint8_t dest, uint8_t src, uint8_t prio, uint8_t protocol)
{
    uint8_t *header = (uint8_t*)buf;
    header[0] = dest;
    header[1] = src;
    header[2] = ((prio & 0x07) << 5) + (protocol & 0x1F);
}


void _net_read_header(const char *buf, uint8_t *dest, uint8_t *src, uint8_t *prio, uint8_t *protocol)
{
    uint8_t *header = (uint8_t*)buf;
    *dest = header[0];
    *src = header[1];
    *prio = header[2] >> 5;
    *protocol = header[2] & 0x1F;
}



net_protocol_rcv_cb_t _net_get_protocol_cb(const struct net_protocol_table_entry_s *protocol_list, uint8_t protocol)
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
                   struct net_if_s *interface_list,
                   const struct net_protocol_table_entry_s *protocol_table)
{
    node->own_addr = addr;
    node->routing_table_size = routing_table_size;
    node->routing_table = routing_table;
    node->interface_list = interface_list;
    node->protocol_table = protocol_table;
    mcucom_port_mutex_init(&node->node_lock);
    struct net_if_s *net_if = interface_list;
    while (net_if->send_fn != NULL) {
        mcucom_port_mutex_init(&net_if->send_lock);
        net_if++;
    }
}


void net_handle_incoming_frame(net_node_t *node, const char *frame, size_t len, uint8_t interface_idx, uint8_t frame_src_addr)
{

    uint8_t dest, src_addr, prio, protocol;
    _net_read_header(frame, &dest, &src_addr, &prio, &protocol);

    mcucom_port_mutex_acquire(&node->node_lock);

    // update routing table
    uint8_t _via_addr;
    if (_net_route_lookup_with_lock(node->routing_table, src_addr, &_via_addr) == -1) {
        _net_route_add_with_lock(node->routing_table, node->routing_table_size, src_addr, 0xff, interface_idx, frame_src_addr);
    }

    if (dest == node->own_addr || dest == NET_BROADCAST_DEST) {
        // call protocol handler
        net_protocol_rcv_cb_t cb = _net_get_protocol_cb(node->protocol_table, protocol);

        mcucom_port_mutex_release(&node->node_lock);

        if (cb != NULL) {
            cb(node, &frame[NET_HEADER_LEN], len - NET_HEADER_LEN, src_addr, prio, interface_idx);
        }
    } else {
        // route packet
        uint8_t via_addr;
        int8_t if_idx = _net_route_lookup_with_lock(node->routing_table, dest, &via_addr);

        mcucom_port_mutex_release(&node->node_lock);

        if (if_idx != -1) {
            struct net_if_s *net_if = &node->interface_list[if_idx];
            _net_if_send_frame(net_if, frame, len, prio, via_addr);
        }
    }

}


void _net_if_send_frame(struct net_if_s *net_if, const char *frame, size_t len, uint8_t priority, uint8_t ll_addr)
{
    mcucom_port_mutex_acquire(&net_if->send_lock);

    net_if->send_fn(net_if->arg, frame, len, priority, ll_addr);

    mcucom_port_mutex_release(&net_if->send_lock);
}



bool net_write_header_and_send_frame(net_node_t *node, uint8_t protocol, char *frame, size_t len, uint8_t dest_addr, uint8_t priority)
{
    _net_write_header(frame, dest_addr, node->own_addr, priority, protocol);
    uint8_t via_addr;
    int8_t if_idx = net_route_lookup(node, dest_addr, &via_addr);
    if (if_idx != -1) {
        struct net_if_s *net_if = &node->interface_list[if_idx];
        _net_if_send_frame(net_if, frame, len, priority, via_addr);
        return true;
    }
    return false; // no route to host
}


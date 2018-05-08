#include "../net.h"
#include <stdint.h>
#include <CppUTest/TestHarness.h>
#include "CppUTestExt/MockSupport.h"



TEST_GROUP(NetRoutingTest)
{

};

TEST(NetRoutingTest, RoutingTableLookupNotFound)
{
    struct net_route_tab_entry_s tab[] = {
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.link_layer_via_interface_idx = -1}
    };
    uint8_t via_addr;
    int8_t res = _net_route_lookup_with_lock(tab, 0x11, &via_addr);
    CHECK_EQUAL(-1, res);
}

TEST(NetRoutingTest, RoutingTableLookupMatch)
{
    struct net_route_tab_entry_s tab[] = {
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.link_layer_via_interface_idx = -1}
    };
    uint8_t via_addr;
    int8_t res = _net_route_lookup_with_lock(tab, 0x10, &via_addr);
    CHECK_EQUAL(0, res);
    CHECK_EQUAL(0x2a, via_addr);
}

TEST(NetRoutingTest, RoutingTableLookupSecondMatch)
{
    struct net_route_tab_entry_s tab[] = {
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.dest_addr = 0x11, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 1, .link_layer_via_addr = 0x2b},
        {.link_layer_via_interface_idx = -1}
    };
    uint8_t via_addr;
    int8_t res = _net_route_lookup_with_lock(tab, 0x11, &via_addr);
    CHECK_EQUAL(1, res);
    CHECK_EQUAL(0x2b, via_addr);
}

TEST(NetRoutingTest, RoutingTableLookupMatchWithMask)
{
    struct net_route_tab_entry_s tab[] = {
        {.dest_addr = 0x1a, .dest_mask = 0x0f,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.link_layer_via_interface_idx = -1}
    };
    uint8_t via_addr;
    int8_t res = _net_route_lookup_with_lock(tab, 0x0a, &via_addr);
    CHECK_EQUAL(0, res);
    CHECK_EQUAL(0x2a, via_addr);
}

TEST(NetRoutingTest, RoutingTableLookupMulitpleMatchesReportFirst)
{
    struct net_route_tab_entry_s tab[] = {
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 1, .link_layer_via_addr = 0x2b},
        {.link_layer_via_interface_idx = -1}
    };
    uint8_t via_addr;
    int8_t res = _net_route_lookup_with_lock(tab, 0x10, &via_addr);
    CHECK_EQUAL(0, res);
    CHECK_EQUAL(0x2a, via_addr);
}

TEST(NetRoutingTest, RouteAdd)
{
    struct net_route_tab_entry_s tab[4] = {
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.dest_addr = 0x11, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 1, .link_layer_via_addr = 0x2b},
        {.link_layer_via_interface_idx = -1}
    };

    uint8_t dest = 0x12;
    uint8_t mask = 0xff;
    uint8_t if_idx = 2;
    uint8_t via_addr = 0x2c;
    bool res = _net_route_add_with_lock(tab, 4, dest, mask, if_idx, via_addr);

    CHECK_EQUAL(true, res);
    CHECK_EQUAL(dest, tab[2].dest_addr);
    CHECK_EQUAL(mask, tab[2].dest_mask);
    CHECK_EQUAL(if_idx, tab[2].link_layer_via_interface_idx);
    CHECK_EQUAL(via_addr, tab[2].link_layer_via_addr);
    CHECK_EQUAL(-1, tab[3].link_layer_via_interface_idx); // new sentinel -1
}

TEST(NetRoutingTest, RouteAddFailsWhenTableIsFull)
{
    struct net_route_tab_entry_s tab[3] = {
        {.dest_addr = 0x10, .dest_mask = 0xff,
         .link_layer_via_interface_idx = 0, .link_layer_via_addr = 0x2a},
        {.link_layer_via_interface_idx = -1},
        {.link_layer_via_interface_idx = -1}, // this is to check it doesnt write outside buffer
    };
    bool res = _net_route_add_with_lock(tab, 2, 0, 0, 0, 0);
    CHECK_EQUAL(false, res);
    CHECK_EQUAL(-1, tab[1].link_layer_via_interface_idx); // didnt overwrite sentinel
    CHECK_EQUAL(-1, tab[2].link_layer_via_interface_idx); // didnt overwrite outside buffer
}


TEST_GROUP(NetHeaderTest)
{

};

TEST(NetHeaderTest, TestWrite)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    char *bufp = (char*)buf;
    uint8_t dest = 21;
    uint8_t src = 42;
    uint8_t prio = 3;
    uint8_t protocol = 20;

    _net_write_header(bufp, dest, src, prio, protocol);

    CHECK_EQUAL(dest, buf[0]);
    CHECK_EQUAL(src, buf[1]);
    CHECK_EQUAL(protocol, buf[2] & 0x1f);
    CHECK_EQUAL(prio, (buf[2] & ~0x1f)>>5);
    CHECK_EQUAL(0, buf[3]); // didn't overwrite
}

TEST(NetHeaderTest, TestRead)
{
    uint8_t dest = 21;
    uint8_t src = 42;
    uint8_t prio = 3;
    uint8_t protocol = 20;
    uint8_t buf[] = {dest, src, (uint8_t)((prio << 5) + protocol)};
    char *bufp = (char*)buf;
    uint8_t read_dest, read_src, read_prio, read_protocol;

    _net_read_header(bufp, &read_dest, &read_src, &read_prio, &read_protocol);

    CHECK_EQUAL(dest, read_dest);
    CHECK_EQUAL(src, read_src);
    CHECK_EQUAL(prio, read_prio);
    CHECK_EQUAL(protocol, read_protocol);
}


TEST_GROUP(NetProtocolLookup)
{

};

TEST(NetProtocolLookup, TestLookup)
{
    struct net_protocol_table_entry_s list[] = {
        {.protocol_nbr = 0, .protocol_rcv_cb = (net_protocol_rcv_cb_t)0xaa},
        {.protocol_nbr = 1, .protocol_rcv_cb = (net_protocol_rcv_cb_t)0xbb},
        {.protocol_nbr = 2, .protocol_rcv_cb = (net_protocol_rcv_cb_t)0xcc},
        {.protocol_nbr = -1},
    };
    POINTERS_EQUAL((net_protocol_rcv_cb_t)0xbb, _net_get_protocol_cb(list, 1));
    POINTERS_EQUAL((net_protocol_rcv_cb_t)0xcc, _net_get_protocol_cb(list, 2));
}

TEST(NetProtocolLookup, TestLookupFails)
{
    struct net_protocol_table_entry_s list[] = {
        {.protocol_nbr = 0, .protocol_rcv_cb = (net_protocol_rcv_cb_t)0xaa},
        {.protocol_nbr = 1, .protocol_rcv_cb = (net_protocol_rcv_cb_t)0xbb},
        {.protocol_nbr = 2, .protocol_rcv_cb = (net_protocol_rcv_cb_t)0xcc},
        {.protocol_nbr = -1},
    };
    POINTERS_EQUAL((net_protocol_rcv_cb_t)NULL, _net_get_protocol_cb(list, 5));
}




TEST_GROUP(NetNodePublicAPI)
{

    const int8_t MY_PROTOCOL = 2;
    const int8_t NON_EXISTENT_PROTOCOL = 3;
    static void protocol_cb(net_node_t *node, const char *pkt, size_t len, uint8_t src_addr, uint8_t prio, uint8_t interface_idx)
    {
        mock().actualCall("protocol_cb")
            .withParameter("node", node)
            .withParameter("pkt", pkt)
            .withParameter("len", len)
            .withParameter("src_addr", src_addr)
            .withParameter("prio", prio)
            .withParameter("interface_idx", interface_idx);
    }

    int if0_arg;
    static void if0_send(void *arg, const char *frame, size_t len, uint8_t prio, uint8_t dest)
    {
        mock().actualCall("if0")
            .withParameter("arg", arg)
            .withParameter("frame", frame)
            .withParameter("len", len)
            .withParameter("prio", prio)
            .withParameter("dest", dest);
    }

    int if1_arg;
    static void if1_send(void *arg, const char *frame, size_t len, uint8_t prio, uint8_t dest)
    {
        mock().actualCall("if1")
            .withParameter("arg", arg)
            .withParameter("frame", frame)
            .withParameter("len", len)
            .withParameter("prio", prio)
            .withParameter("dest", dest);
    }

    static const int routing_tab_size = 3;
    struct net_route_tab_entry_s routing_tab[routing_tab_size];
    struct net_if_s interface_list[3] = {
        {.send_fn = if0_send, .arg = &if0_arg},
        {.send_fn = if1_send, .arg = &if1_arg},
        {.send_fn = NULL}
    };
    const struct net_protocol_table_entry_s protocol_table[2] = {
        {.protocol_nbr = MY_PROTOCOL, .protocol_rcv_cb = protocol_cb},
        {.protocol_nbr = -1}
    };

    net_node_t node;
    const int node_addr = 42;

    void setup()
    {
        routing_tab[0].dest_addr = 23;
        routing_tab[0].dest_mask = 0xff; // exact match only
        routing_tab[0].link_layer_via_interface_idx = 1;
        routing_tab[0].link_layer_via_addr = 32;
        routing_tab[1].link_layer_via_interface_idx = -1; // sentinel
        net_node_init(&node, node_addr, routing_tab, routing_tab_size, interface_list, protocol_table);
    }
};

TEST(NetNodePublicAPI, NodeInit)
{
    net_node_t node;
    net_node_init(&node, node_addr, routing_tab, routing_tab_size, interface_list, protocol_table);
    CHECK_EQUAL(node_addr, node.own_addr);
    POINTERS_EQUAL(routing_tab, node.routing_table);
    CHECK_EQUAL(3, node.routing_table_size);
    POINTERS_EQUAL(interface_list, node.interface_list);
    POINTERS_EQUAL(protocol_table, node.protocol_table);
}

TEST(NetNodePublicAPI, HandleIncomingCallProtocolHandler)
{
    char frame[100];
    uint8_t prio = 3;
    uint8_t src_addr = 23;
    size_t len = sizeof(frame);
    uint8_t protocol = MY_PROTOCOL;
    _net_write_header(frame, node_addr, src_addr, prio, protocol);

    mock().expectOneCall("protocol_cb")
        .withParameter("node", &node)
        .withParameter("pkt", &frame[NET_HEADER_LEN]) // handler receives frame without header
        .withParameter("len", len - NET_HEADER_LEN)
        .withParameter("src_addr", src_addr)
        .withParameter("prio", prio)
        .withParameter("interface_idx", 0);

    net_handle_incoming_frame(&node, frame, len, 0, 0);

    mock().checkExpectations();
}

TEST(NetNodePublicAPI, HandleIncomingNoProtocolHandler)
{
    char frame[100];
    uint8_t prio = 3;
    uint8_t src_addr = 23;
    size_t len = sizeof(frame);
    uint8_t protocol = NON_EXISTENT_PROTOCOL;
    _net_write_header(frame, node_addr, src_addr, prio, protocol);

    net_handle_incoming_frame(&node, frame, len, 0, 0);

    // expect no calls
    mock().checkExpectations();
}

TEST(NetNodePublicAPI, PacketWithUnknownSourceRouteGetsAddedToRoutingTable)
{
    char frame[100];
    uint8_t prio = 3;
    uint8_t pkt_src_addr = 22;
    size_t len = sizeof(frame);
    uint8_t protocol = NON_EXISTENT_PROTOCOL;
    _net_write_header(frame, node_addr, pkt_src_addr, prio, protocol);
    uint8_t source_if = 2;
    uint8_t source_ll_addr = 88;

    net_handle_incoming_frame(&node, frame, len, source_if, source_ll_addr);

    CHECK_EQUAL(pkt_src_addr, routing_tab[1].dest_addr);
    CHECK_EQUAL(0xff, routing_tab[1].dest_mask);
    CHECK_EQUAL(source_if, routing_tab[1].link_layer_via_interface_idx);
    CHECK_EQUAL(source_ll_addr, routing_tab[1].link_layer_via_addr);
}

TEST(NetNodePublicAPI, PacketWithKnownSourceRouteDoesntGetAddedToRoutingTable)
{
    char frame[100];
    uint8_t prio = 3;
    uint8_t pkt_src_addr = 23;
    size_t len = sizeof(frame);
    uint8_t protocol = NON_EXISTENT_PROTOCOL;
    _net_write_header(frame, node_addr, pkt_src_addr, prio, protocol);
    uint8_t source_if = 2;
    uint8_t source_ll_addr = 88;

    net_handle_incoming_frame(&node, frame, len, source_if, source_ll_addr);

    CHECK_EQUAL(-1, routing_tab[1].link_layer_via_interface_idx); // nothing added
}

TEST(NetNodePublicAPI, PacketThatIsNotForNodeGetsRouted)
{
    char frame[100];
    uint8_t prio = 3;
    uint8_t pkt_src_addr = 22;
    size_t len = sizeof(frame);
    uint8_t protocol = MY_PROTOCOL;
    _net_write_header(frame, 23, pkt_src_addr, prio, protocol);
    uint8_t source_if = 2;
    uint8_t source_ll_addr = 88;

    mock().expectOneCall("if1")
        .withParameter("arg", &if1_arg)
        .withParameter("frame", frame)
        .withParameter("len", len)
        .withParameter("prio", prio)
        .withParameter("dest", 32);

    net_handle_incoming_frame(&node, frame, len, source_if, source_ll_addr);

    mock().checkExpectations();
}

TEST(NetNodePublicAPI, BroadcastPacketsAreHandledByNode)
{
    char frame[100];
    uint8_t prio = 3;
    uint8_t src_addr = 23;
    size_t len = sizeof(frame);
    uint8_t protocol = MY_PROTOCOL;
    uint8_t broadcast_addr = 0;
    _net_write_header(frame, broadcast_addr, src_addr, prio, protocol);
    uint8_t source_if = 2;
    uint8_t source_ll_addr = 88;

    mock().expectOneCall("protocol_cb")
        .withParameter("node", &node)
        .withParameter("pkt", &frame[NET_HEADER_LEN]) // handler receives frame without header
        .withParameter("len", len - NET_HEADER_LEN)
        .withParameter("src_addr", src_addr)
        .withParameter("prio", prio)
        .withParameter("interface_idx", source_if);


    net_handle_incoming_frame(&node, frame, len, source_if, source_ll_addr);

    mock().checkExpectations();
}

TEST(NetNodePublicAPI, SendPacket)
{
    char frame[100];
    uint8_t prio = 3;
    size_t len = sizeof(frame);
    uint8_t protocol = MY_PROTOCOL;
    uint8_t dest = 23;

    mock().expectOneCall("if1")
        .withParameter("arg", &if1_arg)
        .withParameter("frame", frame)
        .withParameter("len", len)
        .withParameter("prio", prio)
        .withParameter("dest", 32);

    bool ret = net_write_header_and_send_frame(&node, protocol, frame, len, dest, prio);

    CHECK_EQUAL(true, ret);

    mock().checkExpectations();
}

TEST(NetNodePublicAPI, SendPacketNoRouteFails)
{
    char frame[100];
    uint8_t prio = 3;
    size_t len = sizeof(frame);
    uint8_t protocol = MY_PROTOCOL;
    uint8_t dest = 22;

    bool ret = net_write_header_and_send_frame(&node, protocol, frame, len, dest, prio);

    CHECK_EQUAL(false, ret);

    // no calls
    mock().checkExpectations();
}



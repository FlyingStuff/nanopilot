#include "../net.h"
#include <mcucom_port_sync.h>

#include <stdint.h>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>




TEST_GROUP(NetLockingTest)
{
    static void discard_if(void *arg, const char *frame, size_t len, uint8_t prio, uint8_t dest)
    {
        (void)arg;
        (void)frame;
        (void)len;
        (void)prio;
        (void)dest;
    }

    static const int MY_PROTOCOL = 2;
    static void empy_protocol(net_node_t *node, const char *pkt, size_t len, uint8_t src_addr, uint8_t prio, uint8_t interface_idx)
    {
        (void)node;
        (void)pkt;
        (void)len;
        (void)src_addr;
        (void)prio;
        (void)interface_idx;
    }

    net_node_t node;
    static const int NODE_ADDR = 42;
    struct net_route_tab_entry_s routing_table[10];
    struct net_if_s interface_list[2] = {
        {.send_fn = discard_if, .arg = NULL},
        {.send_fn = NULL}
    };
    const struct net_protocol_table_entry_s protocol_list[2] = {
        {.protocol_nbr = MY_PROTOCOL, .protocol_rcv_cb = empy_protocol},
        {.protocol_nbr = -1}
    };

    void setup()
    {
        mock().strictOrder();

        routing_table[0].link_layer_via_addr = -1;
        net_node_init(&node, NODE_ADDR, routing_table, 10, interface_list, protocol_list);

        lock_mocks_enable(true);
    }

    void teardown()
    {
        lock_mocks_enable(false);
        synchronization_init_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};


TEST(NetLockingTest, node_init_inits_mutex)
{
    synchronization_init_mocks_enable(true);

    mock().expectOneCall("sync_mock_mutex_init")
          .withParameter("mutex", &node.node_lock);

    mock().expectOneCall("sync_mock_mutex_init")
          .withParameter("mutex", &interface_list[0].send_lock);

    net_node_init(&node, NODE_ADDR, routing_table, 10, interface_list, protocol_list);
}


TEST(NetLockingTest, net_handle_incoming_frame_for_node_is_locked)
{
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &node.node_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &node.node_lock);

    char frame[100];
    _net_write_header(frame, NODE_ADDR, 10, 0, MY_PROTOCOL);
    net_handle_incoming_frame(&node, frame, 100, 0, 0);
}


TEST(NetLockingTest, net_handle_incoming_frame_to_forward_is_locked)
{
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &node.node_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &node.node_lock);

    // driver send lock
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &interface_list[0].send_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &interface_list[0].send_lock);

    char frame[100];
    _net_write_header(frame, NODE_ADDR + 1, 10, 0, MY_PROTOCOL);
    net_handle_incoming_frame(&node, frame, 100, 0, 0);
}

TEST(NetLockingTest, net_write_header_and_send_frame_is_locked)
{
    // should be locked for routing table lookup
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &node.node_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &node.node_lock);
    // driver send lock
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &interface_list[0].send_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &interface_list[0].send_lock);

    char frame[100];
    net_write_header_and_send_frame(&node, MY_PROTOCOL, frame, 100, 23, 0);
}

TEST(NetLockingTest, net_route_lookup_is_locked)
{
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &node.node_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &node.node_lock);

    uint8_t _via_addr;
    net_route_lookup(&node, 10, &_via_addr);
}


TEST(NetLockingTest, net_route_add_is_locked)
{
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &node.node_lock);
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &node.node_lock);

    net_route_add(&node, 10, 0xff, 0, 0);
}


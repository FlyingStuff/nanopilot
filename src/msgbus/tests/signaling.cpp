#include "../msgbus.h"
#include "../messagebus.h"
#include "mocks/synchronization.hpp"
#include "types/test.h"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

TEST_GROUP(SignalingTestGroup)
{
    msgbus_t bus;
    msgbus_topic_t topic;
    uint8_t buffer[128];

    void setup()
    {
        mock().strictOrder();
        msgbus_init(&bus);
        msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
    }

    void teardown()
    {
        clear_condvar_wait_side_effect();
        lock_mocks_enable(false);
        condvar_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(SignalingTestGroup, TopicFindWithTimeoutImmediateDoesntBlockOnCond)
{
    condvar_mocks_enable(true);

    msgbus_find_topic(&bus, "notatopic", MSGBUS_TIMEOUT_IMMEDIATE);
}

TEST(SignalingTestGroup, TopicPublish)
{
    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", &topic.lock);

    mock().expectOneCall("messagebus_condvar_broadcast")
          .withPointerParameter("cond", &topic.condvar);

    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", &topic.lock);

    msgbus_topic_publish(&topic, buffer);
}

TEST(SignalingTestGroup, TopicWait)
{
    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", &topic.lock);

    mock().expectOneCall("messagebus_condvar_wait")
          .withPointerParameter("cond", &topic.condvar)
          .withParameter("timeout_us", MSGBUS_TIMEOUT_NEVER);

    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", &topic.lock);

    messagebus_topic_wait(&topic, buffer);
}

TEST(SignalingTestGroup, SubscriberWaitWithTimeoutImmediateDoesntBlockOnCond)
{
    msgbus_subscriber_t sub;
    msgbus_topic_subscribe(&sub, &bus, "topic", MSGBUS_TIMEOUT_IMMEDIATE);

    condvar_mocks_enable(true);
    CHECK_FALSE(msgbus_subscriber_wait_for_update(&sub, MSGBUS_TIMEOUT_IMMEDIATE));
}

TEST(SignalingTestGroup, SubscriberWaitBlockOnCond)
{
    msgbus_subscriber_t sub;
    msgbus_topic_subscribe(&sub, &bus, "topic", MSGBUS_TIMEOUT_IMMEDIATE);

    condvar_mocks_enable(true);
    mock().expectOneCall("messagebus_condvar_wait")
          .withPointerParameter("cond", &topic.condvar)
          .withParameter("timeout_us", 100);

    CHECK_FALSE(msgbus_subscriber_wait_for_update(&sub, 100));
}

TEST(SignalingTestGroup, SubscriberWaitBlockOnCondHasUpdate)
{
    msgbus_subscriber_t sub;
    msgbus_topic_subscribe(&sub, &bus, "topic", MSGBUS_TIMEOUT_IMMEDIATE);

    set_condvar_wait_side_effect([this](){msgbus_topic_publish(&topic, buffer);});

    CHECK_TRUE(msgbus_subscriber_wait_for_update(&sub, 100));
}

TEST(SignalingTestGroup, Advertise)
{
    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", &bus.lock);

    mock().expectOneCall("messagebus_condvar_broadcast")
          .withPointerParameter("cond", &bus.condvar);

    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", &bus.lock);

    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
}

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../msgbus.h"
#include "mocks/synchronization.hpp"
#include "types/test.h"


TEST_GROUP(BusTests)
{
    msgbus_t bus;
    msgbus_topic_t topic;
    uint8_t buffer[128];
    msgbus_topic_t second_topic;

    void setup()
    {
        msgbus_init(&bus);
    }

    void teardown()
    {
        condvar_init_mock_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};


TEST(BusTests, Initializer)
{
    mock().expectOneCall("messagebus_condvar_init").withParameter("cond", &bus.condvar);
    mock().expectOneCall("messagebus_lock_init").withParameter("lock", &bus.lock);
    condvar_init_mock_enable(true);
    msgbus_init(&bus);
    POINTERS_EQUAL(NULL, bus.topics.head);
}


TEST(BusTests, FirstTopicGoesToHead)
{
    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");

    POINTERS_EQUAL(&topic, bus.topics.head);
}

TEST(BusTests, NextofListIsOkToo)
{
    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "first");
    msgbus_topic_create(&second_topic, &bus, &simple_type, buffer, "second");

    POINTERS_EQUAL(&second_topic, bus.topics.head);
    POINTERS_EQUAL(&topic, bus.topics.head->next);
}

TEST(BusTests, TopicNotFound)
{
    msgbus_find_topic(&bus, "topic");
    POINTERS_EQUAL(NULL, msgbus_find_topic(&bus, "topic"));
}

TEST(BusTests, TopicFound)
{
    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
    POINTERS_EQUAL(&topic, msgbus_find_topic(&bus, "topic"));
}

TEST(BusTests, CanScanBus)
{
    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "first");
    msgbus_topic_create(&second_topic, &bus, &simple_type, buffer, "second");

    POINTERS_EQUAL(&topic, msgbus_find_topic(&bus, "first"));
    POINTERS_EQUAL(&second_topic, msgbus_find_topic(&bus, "second"));
}

TEST(BusTests, FindTopicBlocking)
{
    msgbus_topic_t *res;
    /* This is a partial test only: we cannot test that the behavior is correct
     * when the topic is not on the bus yes without additional thread and I
     * don't like threading in tests. */
    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
    res = msgbus_find_topic_blocking(&bus, "topic", MSGBUS_TIMEOUT_NEVER);
    POINTERS_EQUAL(&topic, res);
}

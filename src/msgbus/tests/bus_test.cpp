#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
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
        messagebus_init(&bus);
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
    messagebus_init(&bus);
    POINTERS_EQUAL(NULL, bus.topics.head);
}


TEST(BusTests, FirstTopicGoesToHead)
{
    messagebus_topic_create(&topic, &bus, &simple_type, buffer, "topic");

    POINTERS_EQUAL(&topic, bus.topics.head);
}

TEST(BusTests, NextofListIsOkToo)
{
    messagebus_topic_create(&topic, &bus, &simple_type, buffer, "first");
    messagebus_topic_create(&second_topic, &bus, &simple_type, buffer, "second");

    POINTERS_EQUAL(&second_topic, bus.topics.head);
    POINTERS_EQUAL(&topic, bus.topics.head->next);
}

TEST(BusTests, TopicNotFound)
{
    messagebus_find_topic(&bus, "topic");
    POINTERS_EQUAL(NULL, messagebus_find_topic(&bus, "topic"));
}

TEST(BusTests, TopicFound)
{
    messagebus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
    POINTERS_EQUAL(&topic, messagebus_find_topic(&bus, "topic"));
}

TEST(BusTests, CanScanBus)
{
    messagebus_topic_create(&topic, &bus, &simple_type, buffer, "first");
    messagebus_topic_create(&second_topic, &bus, &simple_type, buffer, "second");

    POINTERS_EQUAL(&topic, messagebus_find_topic(&bus, "first"));
    POINTERS_EQUAL(&second_topic, messagebus_find_topic(&bus, "second"));
}

TEST(BusTests, FindTopicBlocking)
{
    msgbus_topic_t *res;
    /* This is a partial test only: we cannot test that the behavior is correct
     * when the topic is not on the bus yes without additional thread and I
     * don't like threading in tests. */
    messagebus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
    res = messagebus_find_topic_blocking(&bus, "topic");
    POINTERS_EQUAL(&topic, res);
}


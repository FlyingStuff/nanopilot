#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
#include "mocks/synchronization.hpp"

TEST_GROUP(MessageBusTestGroup)
{
    messagebus_t bus;
    messagebus_topic_t topic;
    uint8_t buffer[128];
    messagebus_topic_t second_topic;

    void setup()
    {
        messagebus_init(&bus);
        messagebus_topic_init(&topic, buffer, sizeof buffer);
        messagebus_topic_init(&second_topic, NULL, 0);
    }

    void teardown()
    {
        condvar_init_mock_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(MessageBusTestGroup, CanCreateTopicWithBuffer)
{
    mock().expectOneCall("messagebus_condvar_init").withParameter("cond", &topic.condvar);
    mock().expectOneCall("messagebus_lock_init").withParameter("lock", &topic.lock);
    condvar_init_mock_enable(true);
    messagebus_topic_init(&topic, buffer, sizeof buffer);
    POINTERS_EQUAL(buffer, topic.buffer);
    CHECK_EQUAL(topic.buffer_len, sizeof(buffer));
}

TEST(MessageBusTestGroup, CanCreateBus)
{
    mock().expectOneCall("messagebus_condvar_init").withParameter("cond", &bus.condvar);
    mock().expectOneCall("messagebus_lock_init").withParameter("lock", &bus.lock);
    condvar_init_mock_enable(true);
    messagebus_init(&bus);
    POINTERS_EQUAL(NULL, bus.topics.head);
}

TEST(MessageBusTestGroup, AdvertiseTopicName)
{
    messagebus_advertise_topic(&bus, &topic, "/imu/raw");

    STRCMP_EQUAL("/imu/raw", topic.name);
}

TEST(MessageBusTestGroup, FirstTopicGoesToHead)
{
    messagebus_advertise_topic(&bus, &topic, "/imu/raw");

    POINTERS_EQUAL(&topic, bus.topics.head);
}

TEST(MessageBusTestGroup, NextofListIsOkToo)
{
    messagebus_advertise_topic(&bus, &topic, "first");
    messagebus_advertise_topic(&bus, &second_topic, "second");

    POINTERS_EQUAL(&second_topic, bus.topics.head);
    POINTERS_EQUAL(&topic, bus.topics.head->next);
}

TEST(MessageBusTestGroup, TopicNotFound)
{
    messagebus_find_topic(&bus, "topic");
    POINTERS_EQUAL(NULL, messagebus_find_topic(&bus, "topic"));
}

TEST(MessageBusTestGroup, TopicFound)
{
    messagebus_advertise_topic(&bus, &topic, "topic");
    POINTERS_EQUAL(&topic, messagebus_find_topic(&bus, "topic"));
}

TEST(MessageBusTestGroup, CanScanBus)
{
    messagebus_advertise_topic(&bus, &topic, "first");
    messagebus_advertise_topic(&bus, &second_topic, "second");

    POINTERS_EQUAL(&topic, messagebus_find_topic(&bus, "first"));
    POINTERS_EQUAL(&second_topic, messagebus_find_topic(&bus, "second"));
}

TEST(MessageBusTestGroup, FindTopicBlocking)
{
    messagebus_topic_t *res;
    /* This is a partial test only: we cannot test that the behavior is correct
     * when the topic is not on the bus yes without additional thread and I
     * don't like threading in tests. */
    messagebus_advertise_topic(&bus, &topic, "topic");
    res = messagebus_find_topic_blocking(&bus, "topic");
    POINTERS_EQUAL(&topic, res);
}

TEST(MessageBusTestGroup, CanPublish)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    res = messagebus_topic_publish(&topic, data, sizeof(data));

    MEMCMP_EQUAL(topic.buffer, data, sizeof(data));
    CHECK_TRUE(res);
}

TEST(MessageBusTestGroup, WontPublishTooBigMessage)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    topic.buffer_len = 1;
    res = messagebus_topic_publish(&topic, data, sizeof(data));

    CHECK_FALSE(res);
}

TEST(MessageBusTestGroup, CanRead)
{
    int tx=42, rx;
    bool res;

    messagebus_topic_publish(&topic, &tx, sizeof(int));
    res = messagebus_topic_read(&topic, &rx, sizeof(int));

    CHECK_TRUE(res);
    CHECK_EQUAL(tx, rx);
}

TEST(MessageBusTestGroup, WontReadUnpublishedtopic)
{
    int rx;
    bool res;

    res = messagebus_topic_read(&topic, &rx, sizeof(int));
    CHECK_FALSE(res);
}

TEST(MessageBusTestGroup, WaitForUpdate)
{
    int tx=42, rx;

    messagebus_topic_publish(&topic, &tx, sizeof(int));
    messagebus_topic_wait(&topic, &rx, sizeof(int));

    CHECK_EQUAL(tx, rx);
}


#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
#include "mocks/synchronization.hpp"


TEST_GROUP(TopicTests)
{
    messagebus_t bus;
    messagebus_topic_t topic;
    uint8_t buffer[128];

    void setup()
    {
        messagebus_init(&bus);
        messagebus_topic_create(&topic, &bus, buffer, sizeof buffer, "topic");
    }

    void teardown()
    {
        condvar_init_mock_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};



TEST(TopicTests, Initializer)
{
    mock().expectOneCall("messagebus_condvar_init").withParameter("cond", &topic.condvar);
    mock().expectOneCall("messagebus_lock_init").withParameter("lock", &topic.lock);
    condvar_init_mock_enable(true);

    messagebus_topic_create(&topic, &bus, buffer, sizeof buffer, "topic");

    POINTERS_EQUAL(buffer, topic.buffer);
    CHECK_EQUAL(sizeof(buffer), topic.buffer_len);
    CHECK_EQUAL(0, topic.pub_seq_nbr);
    STRCMP_EQUAL("topic", topic.name);
}

TEST(TopicTests, PublishCopiesData)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    res = messagebus_topic_publish(&topic, data, sizeof(data));

    MEMCMP_EQUAL(topic.buffer, data, sizeof(data));
    CHECK_TRUE(res);
}

TEST(TopicTests, PublishIncrementsSequenceNbr)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    res = messagebus_topic_publish(&topic, data, sizeof(data));

    CHECK_EQUAL(1, topic.pub_seq_nbr);
}

TEST(TopicTests, PublishSequenceNbrCorrectlyOverflows)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    topic.pub_seq_nbr = UINT32_MAX;
    res = messagebus_topic_publish(&topic, data, sizeof(data));

    CHECK_EQUAL(0, topic.pub_seq_nbr);
}


TEST(TopicTests, WontPublishTooBigMessage)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    topic.buffer_len = 1;
    res = messagebus_topic_publish(&topic, data, sizeof(data));

    CHECK_FALSE(res);
}

TEST(TopicTests, CanRead)
{
    int tx=42, rx;
    bool res;

    messagebus_topic_publish(&topic, &tx, sizeof(int));
    res = messagebus_topic_read(&topic, &rx, sizeof(int));

    CHECK_TRUE(res);
    CHECK_EQUAL(tx, rx);
}

TEST(TopicTests, WontReadUnpublishedtopic)
{
    int rx;
    bool res;

    res = messagebus_topic_read(&topic, &rx, sizeof(int));
    CHECK_FALSE(res);
}

TEST(TopicTests, WaitForUpdate)
{
    int tx=42, rx;

    messagebus_topic_publish(&topic, &tx, sizeof(int));
    messagebus_topic_wait(&topic, &rx, sizeof(int));

    CHECK_EQUAL(tx, rx);
}


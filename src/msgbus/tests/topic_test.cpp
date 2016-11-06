#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../msgbus.h"
#include "../messagebus.h"
#include "mocks/synchronization.hpp"
#include "types/test.h"


TEST_GROUP(TopicTests)
{
    msgbus_t bus;
    msgbus_topic_t topic;
    uint8_t buffer[128];

    void setup()
    {
        msgbus_init(&bus);
        msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");
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

    msgbus_topic_create(&topic, &bus, &simple_type, buffer, "topic");

    POINTERS_EQUAL(buffer, topic.buffer);
    POINTERS_EQUAL(&simple_type, topic.type);
    CHECK_EQUAL(0, topic.pub_seq_nbr);
    STRCMP_EQUAL("topic", topic.name);
}

TEST(TopicTests, PublishCopiesData)
{
    simple_t data = {42};

    msgbus_topic_publish(&topic, &data);

    MEMCMP_EQUAL(topic.buffer, &data, sizeof(data));
}

TEST(TopicTests, PublishIncrementsSequenceNbr)
{
    simple_t data = {42};

    msgbus_topic_publish(&topic, &data);

    CHECK_EQUAL(1, topic.pub_seq_nbr);
}

TEST(TopicTests, PublishSequenceNbrCorrectlyOverflows)
{
    simple_t data = {42};

    topic.pub_seq_nbr = UINT32_MAX;
    msgbus_topic_publish(&topic, &data);

    CHECK_EQUAL(0, topic.pub_seq_nbr);
}


TEST(TopicTests, PublishAndReadBack)
{
    simple_t tx = {42};
    simple_t rx;
    bool res;

    msgbus_topic_publish(&topic, &tx);
    res = messagebus_topic_read(&topic, &rx);

    CHECK_TRUE(res);
    CHECK_EQUAL(tx.x, rx.x);
}

TEST(TopicTests, WontReadUnpublishedtopic)
{
    simple_t rx = {42};
    bool res;

    res = messagebus_topic_read(&topic, &rx);
    CHECK_FALSE(res);
}

TEST(TopicTests, WaitForUpdate)
{
    simple_t tx = {42};
    simple_t rx;

    msgbus_topic_publish(&topic, &tx);
    messagebus_topic_wait(&topic, &rx);

    CHECK_EQUAL(tx.x, rx.x);
}


#include <CppUTest/TestHarness.h>
#include "../msgbus.h"
#include "types/test.h"


TEST_GROUP(MsgBus)
{
    msgbus_t bus;
    msgbus_topic_t topic;
    simple_t topic_buf;
    msgbus_subscriber_t sub;

    void setup(void)
    {
        msgbus_init(&bus);
        msgbus_topic_create(&topic, &bus, &simple_type, &topic_buf, "test");
    }
};


TEST(MsgBus, FindTopic)
{
    POINTERS_EQUAL(&topic, msgbus_find_topic(&bus, "test"));
}

TEST(MsgBus, IterateTopics)
{
    msgbus_topic_t topic2;
    simple_t topic_buf2;
    msgbus_topic_create(&topic2, &bus, &simple_type, &topic_buf2, "test2");

    msgbus_topic_t *i = msgbus_iterate_topics(&bus);
    POINTERS_EQUAL(&topic2, i);
    i = msgbus_iterate_topics_next(i);
    POINTERS_EQUAL(&topic, i);
    i = msgbus_iterate_topics_next(i);
    POINTERS_EQUAL(NULL, i);
}

TEST(MsgBus, GetTopicType)
{
    POINTERS_EQUAL(&simple_type, msgbus_topic_get_type(&topic));
}

TEST(MsgBus, GetTopicName)
{
    STRCMP_EQUAL("test", msgbus_topic_get_name(&topic));
}

TEST(MsgBus, Subscribe)
{
    CHECK_TRUE(msgbus_topic_subscribe(&sub, &bus, "test", MSGBUS_TIMEOUT_IMMEDIATE));
    CHECK_EQUAL(0, msgbus_subscriber_has_update(&sub));
    CHECK_FALSE(msgbus_subscriber_topic_is_valid(&sub));
}


TEST(MsgBus, Publish)
{
    CHECK_TRUE(msgbus_topic_subscribe(&sub, &bus, "test", MSGBUS_TIMEOUT_IMMEDIATE));
    simple_t t = {42};
    msgbus_topic_publish(&topic, &t);
    CHECK_EQUAL(1, msgbus_subscriber_has_update(&sub));
    CHECK_TRUE(msgbus_subscriber_topic_is_valid(&sub));
}


TEST(MsgBus, PublishAndRead)
{
    CHECK_TRUE(msgbus_topic_subscribe(&sub, &bus, "test", MSGBUS_TIMEOUT_IMMEDIATE));
    simple_t t = {42};
    msgbus_topic_publish(&topic, &t);
    simple_t s;
    msgbus_subscriber_read(&sub, &s);
    CHECK_EQUAL(42, s.x);
    // no update after read
    CHECK_EQUAL(0, msgbus_subscriber_has_update(&sub));
    // but still valid
    CHECK_TRUE(msgbus_subscriber_topic_is_valid(&sub));
}

TEST(MsgBus, SubscriberGetTopic)
{
    msgbus_topic_subscribe(&sub, &bus, "test", MSGBUS_TIMEOUT_IMMEDIATE);
    POINTERS_EQUAL(&topic, msgbus_subscriber_get_topic(&sub));
}

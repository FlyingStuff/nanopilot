#define MSGBUS_UNITTEST
#include "msgbus.hpp"

#include <CppUTest/TestHarness.h>
#include "CppUTestExt/MockSupport.h"
#include "CppUTest/CommandLineTestRunner.h"


// mock implementation of synchronization functions
static bool locked = false;
void reset_mock_lock_state()
{
    locked = false;
}

void msgbus::_lock()
{
    CHECK_FALSE(locked);
    locked = true;
    mock().actualCall("lock");
}

void msgbus::_unlock()
{
    CHECK_TRUE(locked);
    locked = false;
    mock().actualCall("unlock");
}

void msgbus::condvar_init(condvar_t *cv)
{
    (void)cv;
}

void msgbus::condvar_destroy(condvar_t *cv)
{
    (void)cv;
}

void msgbus::condvar_broadcast(condvar_t *cv)
{
    CHECK_TRUE(locked);
    mock().actualCall("broadcast").withParameter("cv", cv);
}

bool msgbus::condvar_wait(condvar_t *cv, float timeout)
{
    mock().actualCall("wait").withParameter("cv", cv).withParameter("timeout", timeout);
    return false;
}



// these tests check the logic of publishing & subscribing to a topic
TEST_GROUP(PublishSubscribe)
{
    void setup()
    {
        mock().disable();
        reset_mock_lock_state();
    }
};

TEST(PublishSubscribe, new_topic_is_not_published)
{
    msgbus::Topic<int> topic;
    CHECK_FALSE(topic.has_been_published());
}

TEST(PublishSubscribe, publish_sets_has_been_published_flag)
{
    msgbus::Topic<int> topic;
    topic.publish(123);
    CHECK_TRUE(topic.has_been_published());
}

TEST(PublishSubscribe, subscriber_on_unpublished_topic_has_no_update)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    CHECK_FALSE(sub.has_update());
}

TEST(PublishSubscribe, subscriber_has_update_after_publish)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    topic.publish(123);
    CHECK_TRUE(sub.has_update());
}

TEST(PublishSubscribe, subscriber_of_already_published_topic_has_update_after_creation)
{
    msgbus::Topic<int> topic;
    topic.publish(123);

    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    CHECK_TRUE(sub.has_update());
}

TEST(PublishSubscribe, subscriber_get_value_clears_update_flag)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    topic.publish(123);
    sub.get_value();
    CHECK_FALSE(sub.has_update());
}

TEST(PublishSubscribe, subscriber_on_unpublished_topic_has_no_value)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    CHECK_FALSE(sub.has_value());
}

TEST(PublishSubscribe, subscriber_on_published_topic_has_value)
{
    msgbus::Topic<int> topic;
    topic.publish(123);
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    CHECK_TRUE(sub.has_value());
}

TEST(PublishSubscribe, subscriber_on_published_topic_has_value_even_after_get_value_is_called)
{
    msgbus::Topic<int> topic;
    topic.publish(123);
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    sub.get_value();
    CHECK_TRUE(sub.has_value());
}

TEST(PublishSubscribe, subscriber_get_published_value)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    topic.publish(123);
    auto val = sub.get_value();
    CHECK_EQUAL(123, val);
}

TEST(PublishSubscribe, subscriber_get_same_published_value_multiple_times)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    topic.publish(123);
    auto val = sub.get_value();
    CHECK_EQUAL(123, val);
    val = sub.get_value();
    CHECK_EQUAL(123, val);
}

TEST(PublishSubscribe, subscriber_gets_last_published_value)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::Subscriber<msgbus::Topic<int>>(topic);
    topic.publish(123);
    topic.publish(456);
    auto val = sub.get_value();
    CHECK_EQUAL(456, val);
}

TEST(PublishSubscribe, subscribe_creates_a_subscriber)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::subscribe(topic);
    topic.publish(123);
    CHECK_TRUE(sub.has_update()); // subscriber is connected to the topic
}

// these tests make sure the functions that access shared state are locking/unlocking
TEST_GROUP(Locking)
{
    void setup()
    {
        reset_mock_lock_state();
    }
    void teardown()
    {
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(Locking, publish_locks)
{
    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    msgbus::Topic<int> topic;
    topic.publish(123);
}

TEST(Locking, has_been_published_locks)
{
    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    msgbus::Topic<int> topic;
    topic.has_been_published();
}

TEST(Locking, wait_for_update_locks)
{
    msgbus::Topic<int> topic;
    mock().disable();
    topic.publish(1);
    mock().enable();
    auto sub = msgbus::subscribe(topic);

    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    sub.wait_for_update();
}

TEST(Locking, has_update_locks)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::subscribe(topic);
    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    sub.has_update();
}

TEST(Locking, has_value_locks)
{
    msgbus::Topic<int> topic;
    auto sub = msgbus::subscribe(topic);
    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    sub.has_value();
}

TEST(Locking, get_value_locks)
{
    msgbus::Topic<int> topic;
    mock().disable();
    topic.publish(1);
    mock().enable();

    auto sub = msgbus::subscribe(topic);
    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    sub.get_value();
}


// these tests check the wait / wakeup logic
TEST_GROUP(Synchronization)
{
    void setup()
    {
        reset_mock_lock_state();
    }
    void teardown()
    {
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(Synchronization, signal_condition_variable_linking_and_unlinking_works)
{
    msgbus::Topic<int> topic;
    auto s1 = msgbus::subscribe(topic);
    auto s2 = msgbus::subscribe(topic);
    msgbus::condvar_t cv;
    s1._with_lock_register_publish_ev_link(&cv);
    s2._with_lock_register_publish_ev_link(&cv);
    mock().expectOneCall("lock");
    mock().expectOneCall("broadcast").withParameter("cv", &cv);
    mock().expectOneCall("broadcast").withParameter("cv", &cv);
    mock().expectOneCall("unlock");
    topic.publish(123);

    s1._with_lock_unregister_publish_ev_link();
    s2._with_lock_unregister_publish_ev_link();
    mock().expectOneCall("lock");
    mock().expectOneCall("unlock");
    topic.publish(123);
}


int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}

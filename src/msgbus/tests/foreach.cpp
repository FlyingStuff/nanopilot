#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include "../messagebus.h"
#include "mocks/synchronization.hpp"
#include "types/test.h"

TEST_GROUP(MsgBusForeachTestGroup)
{
   messagebus_t bus;
   messagebus_topic_t foo, bar;

   void setup()
   {
        messagebus_init(&bus);
        messagebus_topic_create(&foo, &bus, &simple_type, NULL, "foo");
        messagebus_topic_create(&bar, &bus, &simple_type, NULL, "bar");
   }

   void teardown()
   {
        lock_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
   }
};

TEST(MsgBusForeachTestGroup, CanIterate)
{
    mock().expectOneCall("loop").withParameter("topic", &foo);
    mock().expectOneCall("loop").withParameter("topic", &bar);

    MESSAGEBUS_TOPIC_FOREACH(&bus, topic) {
        mock().actualCall("loop").withParameter("topic", topic);
    }
}

TEST(MsgBusForeachTestGroup, CanLock)
{
    lock_mocks_enable(true);
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", &bus.lock);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", &bus.lock);

    MESSAGEBUS_TOPIC_FOREACH(&bus, topic) {
    }
}

TEST(MsgBusForeachTestGroup, CanBreak)
{
    lock_mocks_enable(true);
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", &bus.lock);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", &bus.lock);

    MESSAGEBUS_TOPIC_FOREACH(&bus, topic) {
        break;
    }
}

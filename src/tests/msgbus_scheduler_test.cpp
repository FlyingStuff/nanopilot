#include "../msgbus_scheduler.h"
#include <types/test.h>
#include <mocks/synchronization.hpp>

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>


TEST_GROUP(MsgBusSchedulerTests)
{
    msgbus_t bus;
    msgbus_scheduler_t sched;
    msgbus_scheduler_task_buffer_space_t buf[100];

    void setup()
    {
        msgbus_init(&bus);
        msgbus_scheduler_init(&sched, &bus, buf, 100);
    }
};

TEST(MsgBusSchedulerTests, TestInit)
{
    memset(&sched, 0xff, sizeof(sched));
    msgbus_scheduler_init(&sched, &bus, buf, 100);
    POINTERS_EQUAL(&bus, sched.bus);
    CHECK_EQUAL(100, sched.buffer_size);
    CHECK_EQUAL(0, sched.nb_entries);
    POINTERS_EQUAL(((char*)&buf) + 0, sched.subs);
    POINTERS_EQUAL(((char*)&buf) + sizeof(void*[100]), sched.callbacks);
    POINTERS_EQUAL(((char*)&buf) + 2* sizeof(void*[100]), sched.args);
}

TEST(MsgBusSchedulerTests, TestGetBus)
{
    POINTERS_EQUAL(&bus, msgbus_scheduler_get_bus(&sched));
}

TEST(MsgBusSchedulerTests, TestAdd)
{
    msgbus_subscriber_t sub;
    void (*fn)(void*) = [](void *arg) {};
    int arg;
    CHECK_TRUE(msgbus_scheduler_add_task(&sched, &sub, fn, &arg));
    CHECK_EQUAL(1, sched.nb_entries);
    POINTERS_EQUAL(&sub, sched.subs[0]);
    POINTERS_EQUAL(fn, sched.callbacks[0]);
    POINTERS_EQUAL(&arg, sched.args[0]);
}

TEST(MsgBusSchedulerTests, TestAddSecond)
{
    msgbus_subscriber_t sub;
    void (*fn)(void*) = [](void *arg) {};
    int arg;
    sched.nb_entries = 1;
    CHECK_TRUE(msgbus_scheduler_add_task(&sched, &sub, fn, &arg));
    CHECK_EQUAL(2, sched.nb_entries);
    POINTERS_EQUAL(&sub, sched.subs[1]);
    POINTERS_EQUAL(fn, sched.callbacks[1]);
    POINTERS_EQUAL(&arg, sched.args[1]);
}

TEST(MsgBusSchedulerTests, TestAddBufferFull)
{
    msgbus_subscriber_t sub;
    void (*fn)(void*) = [](void *arg) {};
    int arg;
    sched.nb_entries = sched.buffer_size;
    CHECK_FALSE(msgbus_scheduler_add_task(&sched, &sub, fn, &arg));
}



TEST_GROUP(MsgBusSchedulerSpinTests)
{
    msgbus_t bus;
    msgbus_topic_t topic1, topic2;
    test_t topic_buf;
    msgbus_subscriber_t sub1, sub2;
    int arg1, arg2;
    void (*cb1)(void*) = [](void *arg) {
        mock().actualCall("cb1").withPointerParameter("arg", arg);};
    void (*cb2)(void*) = [](void *arg) {
        mock().actualCall("cb2").withPointerParameter("arg", arg);};
    msgbus_scheduler_t sched;
    msgbus_scheduler_task_buffer_space_t buf[100];

    void setup()
    {
        msgbus_init(&bus);
        msgbus_topic_create(&topic1, &bus, &test_type, &topic_buf, "topic1");
        msgbus_topic_create(&topic2, &bus, &test_type, &topic_buf, "topic2");
        msgbus_topic_subscribe(&sub1, &bus, "topic1", MSGBUS_TIMEOUT_IMMEDIATE);
        msgbus_topic_subscribe(&sub2, &bus, "topic2", MSGBUS_TIMEOUT_IMMEDIATE);

        msgbus_scheduler_init(&sched, &bus, buf, 100);

        msgbus_scheduler_add_task(&sched, &sub1, cb1, &arg1);
        msgbus_scheduler_add_task(&sched, &sub2, cb2, &arg2);
    }

    void teardown()
    {
        condvar_mocks_enable(false);
        condvar_mocks_ignore_cv_pointer_arg(false);
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(MsgBusSchedulerSpinTests, TestSpinNoUpdateDoesntCall)
{
    msgbus_scheduler_spin(&sched, MSGBUS_TIMEOUT_IMMEDIATE);
}

TEST(MsgBusSchedulerSpinTests, TestSpinUpdateOneCall)
{
    msgbus_topic_publish(&topic1, &buf);
    mock().expectOneCall("cb1").withParameter("arg", &arg1);
    msgbus_scheduler_spin(&sched, MSGBUS_TIMEOUT_IMMEDIATE);
}

TEST(MsgBusSchedulerSpinTests, TestSpinUpdateMultipleCall)
{
    msgbus_topic_publish(&topic1, &buf);
    msgbus_topic_publish(&topic2, &buf);
    mock().expectOneCall("cb1").withParameter("arg", &arg1);
    mock().expectOneCall("cb2").withParameter("arg", &arg2);
    msgbus_scheduler_spin(&sched, MSGBUS_TIMEOUT_IMMEDIATE);
}


TEST(MsgBusSchedulerSpinTests, TestSpinBlocksIfNoUpdate)
{
    condvar_mocks_enable(true);
    condvar_mocks_ignore_cv_pointer_arg(true);
    mock().expectOneCall("msgbus_condvar_wait")
          .withPointerParameter("mutex", &bus.topic_update_lock)
          .withParameter("timeout_us", 1000);
    msgbus_scheduler_spin(&sched, 1000);
}

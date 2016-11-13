#include <functional>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../../messagebus.h"
#include <messagebus_port.h>


static bool lock_enabled = false;
static bool condvar_enabled = false;
static bool init_enabled = false;
std::function<void()> condvar_wait_side_effect = [](){};

void messagebus_lock_init(msgbus_mutex_t *lock)
{
    if (init_enabled) {
        mock().actualCall("messagebus_lock_init")
            .withPointerParameter("lock", lock);
    }
}

void messagebus_condvar_init(msgbus_cond_t *cond)
{
    if (init_enabled) {
        mock().actualCall("messagebus_condvar_init")
            .withPointerParameter("cond", cond);
    }
}


void messagebus_lock_acquire(msgbus_mutex_t *lock)
{
    if (lock_enabled) {
        mock().actualCall("messagebus_lock_acquire")
            .withPointerParameter("lock", lock);
    }
}

void messagebus_lock_release(msgbus_mutex_t *lock)
{
    if (lock_enabled) {
        mock().actualCall("messagebus_lock_release")
            .withPointerParameter("lock", lock);
    }
}

void messagebus_condvar_broadcast(msgbus_cond_t *cond)
{
    if (condvar_enabled) {
        mock().actualCall("messagebus_condvar_broadcast")
              .withPointerParameter("cond", cond);
    }
}

void messagebus_condvar_wait(msgbus_cond_t *cond, uint64_t timeout_us)
{
    if (condvar_enabled) {
        mock().actualCall("messagebus_condvar_wait")
              .withPointerParameter("cond", cond)
              .withParameter("timeout_us", (uint32_t)timeout_us);
    }
    condvar_wait_side_effect();
}

void set_condvar_wait_side_effect(std::function<void()> side_effect)
{
    condvar_wait_side_effect = side_effect;
}

void clear_condvar_wait_side_effect()
{
    condvar_wait_side_effect = [](){};
}

void lock_mocks_enable(bool enabled)
{
    lock_enabled = enabled;
}

void condvar_mocks_enable(bool enabled)
{
    condvar_enabled = enabled;
}

void condvar_init_mock_enable(bool enabled)
{
    init_enabled = enabled;
}

TEST_GROUP(LockTestGroup)
{
    int lock;

    void setup()
    {
        lock_mocks_enable(true);
        condvar_mocks_enable(true);
    }

    void teardown()
    {
        lock_mocks_enable(false);
        condvar_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(LockTestGroup, CanLock)
{
    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", &lock);

    messagebus_lock_acquire(&lock);
}

TEST(LockTestGroup, CanUnlock)
{
    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", &lock);

    messagebus_lock_release(&lock);
}

TEST(LockTestGroup, CanBroadcastCondVar)
{
    int cond;
    mock().expectOneCall("messagebus_condvar_broadcast")
          .withPointerParameter("cond", &cond);

    messagebus_condvar_broadcast(&cond);
}

TEST(LockTestGroup, CanWaitCondVar)
{
    int cond;

    mock().expectOneCall("messagebus_condvar_wait")
          .withPointerParameter("cond", &cond)
          .withParameter("timeout_us", 100);


    messagebus_condvar_wait(&cond, 100);
}

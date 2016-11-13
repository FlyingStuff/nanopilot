#include <functional>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../../msgbus.h"
#include <msgbus_port.h>

static bool lock_enabled = false;
static bool condvar_enabled = false;
static bool init_enabled = false;
std::function<void()> condvar_wait_side_effect = [](){};

void msgbus_mutex_init(msgbus_mutex_t *lock)
{
    if (init_enabled) {
        mock().actualCall("msgbus_mutex_init")
            .withPointerParameter("mutex", lock);
    }
}

void msgbus_condvar_init(msgbus_cond_t *cond)
{
    if (init_enabled) {
        mock().actualCall("msgbus_condvar_init")
            .withPointerParameter("cond", cond);
    }
}


void msgbus_mutex_acquire(msgbus_mutex_t *mutex)
{
    if (lock_enabled) {
        mock().actualCall("msgbus_mutex_acquire")
            .withPointerParameter("mutex", mutex);
    }
}

void msgbus_mutex_release(msgbus_mutex_t *mutex)
{
    if (lock_enabled) {
        mock().actualCall("msgbus_mutex_release")
            .withPointerParameter("mutex", mutex);
    }
}

void msgbus_condvar_broadcast(msgbus_cond_t *cond)
{
    if (condvar_enabled) {
        mock().actualCall("msgbus_condvar_broadcast")
              .withPointerParameter("cond", cond);
    }
}

void msgbus_condvar_wait(msgbus_cond_t *cond, msgbus_mutex_t *mutex, uint32_t timeout_us)
{
    if (condvar_enabled) {
        mock().actualCall("msgbus_condvar_wait")
              .withPointerParameter("cond", cond)
              .withPointerParameter("mutex", mutex)
              .withParameter("timeout_us", timeout_us);
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
    mock().expectOneCall("msgbus_mutex_acquire")
          .withPointerParameter("mutex", &lock);

    msgbus_mutex_acquire(&lock);
}

TEST(LockTestGroup, CanUnlock)
{
    mock().expectOneCall("msgbus_mutex_release")
          .withPointerParameter("mutex", &lock);

    msgbus_mutex_release(&lock);
}

TEST(LockTestGroup, CanBroadcastCondVar)
{
    int cond;
    mock().expectOneCall("msgbus_condvar_broadcast")
          .withPointerParameter("cond", &cond);

    msgbus_condvar_broadcast(&cond);
}

TEST(LockTestGroup, CanWaitCondVar)
{
    int cond, mutex;

    mock().expectOneCall("msgbus_condvar_wait")
          .withPointerParameter("cond", &cond)
          .withPointerParameter("mutex", &mutex)
          .withParameter("timeout_us", 100);


    msgbus_condvar_wait(&cond, &mutex, 100);
}

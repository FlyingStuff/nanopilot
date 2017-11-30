#include "mcucom_port_sync.h"

#include <functional>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

static bool lock_enabled = false;
static bool condvar_enabled = false;
static bool init_enabled = false;
static bool condvar_ignore_cv_pointer_arg = false;
static std::function<void()> condvar_wait_side_effect = [](){};

void mcucom_port_mutex_init(mcucom_port_mutex_t *lock)
{
    if (init_enabled) {
        mock().actualCall("sync_mock_mutex_init")
            .withParameter("mutex", lock);
    }
}

void mcucom_port_condvar_init(mcucom_port_cond_t *cond)
{
    if (init_enabled) {
        if (condvar_ignore_cv_pointer_arg) {
            mock().actualCall("sync_mock_condvar_init");
        } else {
            mock().actualCall("sync_mock_condvar_init")
                .withParameter("cond", cond);
        }
    }
}


void mcucom_port_mutex_acquire(mcucom_port_mutex_t *mutex)
{
    if (lock_enabled) {
        mock().actualCall("sync_mock_mutex_acquire")
            .withParameter("mutex", mutex);
    }
}

void mcucom_port_mutex_release(mcucom_port_mutex_t *mutex)
{
    if (lock_enabled) {
        mock().actualCall("sync_mock_mutex_release")
            .withParameter("mutex", mutex);
    }
}

void mcucom_port_condvar_broadcast(mcucom_port_cond_t *cond)
{
    if (condvar_enabled) {
        if (condvar_ignore_cv_pointer_arg) {
            mock().actualCall("sync_mock_condvar_broadcast");
        } else {
            mock().actualCall("sync_mock_condvar_broadcast")
                  .withParameter("cond", cond);
        }
    }
}

bool mcucom_port_condvar_wait(mcucom_port_cond_t *cond, mcucom_port_mutex_t *mutex, uint32_t timeout_us)
{
    if (condvar_enabled) {
        if (condvar_ignore_cv_pointer_arg) {
            mock().actualCall("sync_mock_condvar_wait")
                  .withParameter("mutex", mutex)
                  .withParameter("timeout_us", timeout_us);
        } else {
            mock().actualCall("sync_mock_condvar_wait")
                  .withParameter("cond", cond)
                  .withParameter("mutex", mutex)
                  .withParameter("timeout_us", timeout_us);
        }
    }
    condvar_wait_side_effect();
    return true;
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

void synchronization_init_mocks_enable(bool enabled)
{
    init_enabled = enabled;
}

void condvar_mocks_ignore_cv_pointer_arg(bool enabled)
{
    condvar_ignore_cv_pointer_arg = enabled;
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
    mock().expectOneCall("sync_mock_mutex_acquire")
          .withParameter("mutex", &lock);

    mcucom_port_mutex_acquire(&lock);
}

TEST(LockTestGroup, CanUnlock)
{
    mock().expectOneCall("sync_mock_mutex_release")
          .withParameter("mutex", &lock);

    mcucom_port_mutex_release(&lock);
}

TEST(LockTestGroup, CanBroadcastCondVar)
{
    int cond;
    mock().expectOneCall("sync_mock_condvar_broadcast")
          .withParameter("cond", &cond);

    mcucom_port_condvar_broadcast(&cond);
}

TEST(LockTestGroup, CanWaitCondVar)
{
    int cond, mutex;

    mock().expectOneCall("sync_mock_condvar_wait")
          .withParameter("cond", &cond)
          .withParameter("mutex", &mutex)
          .withParameter("timeout_us", 100);


    mcucom_port_condvar_wait(&cond, &mutex, 100);
}


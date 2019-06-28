#ifndef MSGBUS_HPP
#define MSGBUS_HPP

#include <cstddef>
#include <cstdint>
#include <cassert>
#include <utility>


#if defined (MSGBUS_UNITTEST)
// Plaform implmentation for unittests

namespace msgbus {

void _lock();
void _unlock();
typedef int condvar_t;
void condvar_init(condvar_t *cv);
void condvar_destroy(condvar_t *cv);
void condvar_broadcast(condvar_t *cv);
bool condvar_wait(condvar_t *cv, float timeout);

}

#elif defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
// Posix Platform

#include <pthread.h>
#include <time.h>
#include <errno.h>

namespace msgbus {

inline pthread_mutex_t *get_mutex()
{
    static pthread_mutex_t mutex;
    static bool is_initialized = false;
    if (!is_initialized) {
        pthread_mutex_init(&mutex, NULL);
        is_initialized = true;
    }
    return &mutex;
}

inline void _lock()
{
    assert(pthread_mutex_lock(get_mutex()) == 0);
}

inline void _unlock()
{
    assert(pthread_mutex_unlock(get_mutex()) == 0);
}

typedef pthread_cond_t condvar_t;

inline void condvar_init(condvar_t *cv)
{
    assert(pthread_cond_init(cv, NULL) == 0);
}

inline void condvar_destroy(condvar_t *cv)
{
    assert(pthread_cond_destroy(cv) == 0);
}

inline void condvar_broadcast(condvar_t *cv)
{
    assert(pthread_cond_broadcast(cv) == 0);
}


#ifdef __APPLE__ // OS X has no gettime with realtime clock
inline bool condvar_wait(condvar_t *cv, float timeout)
{
    if (timeout == 0) {
        return false;
    }
    if (timeout < 0) {
        assert(pthread_cond_wait(cv, get_mutex()) == 0);
    } else {
        struct timespec ts;
        ts.tv_sec = timeout;
        ts.tv_nsec = (timeout - ts.tv_sec) * 1000000000UL;
        int ret = pthread_cond_timedwait_relative_np(cv, get_mutex(), &ts);
        assert(ret == 0 || ret == ETIMEDOUT);
        if (ret == ETIMEDOUT) {
            return false;
        }
    }
    return true;
}
#else // posix
inline bool condvar_wait(condvar_t *cv, float timeout)
{
    if (timeout == 0) {
        return false;
    }
    if (timeout < 0) {
        assert(pthread_cond_wait(cv, get_mutex()) == 0);
    } else {
        struct timespec ts;
        assert(clock_gettime(CLOCK_REALTIME, &ts) == 0);
        uint32_t sec = timeout;
        ts.tv_nsec += (timeout - sec) * 1000000000UL;
        ts.tv_sec += sec + ts.tv_nsec / 1000000000UL;
        ts.tv_nsec = ts.tv_nsec % 1000000000UL;
        int ret = pthread_cond_timedwait(cv, get_mutex(), &ts);
        assert(ret == 0 || ret == ETIMEDOUT);
        if (ret == ETIMEDOUT) {
            return false;
        }
    }
    return true;
}
#endif

}

#elif defined(__CHIBIOS__)

#include <ch.h>
#include <osal.h>

namespace msgbus {

typedef condition_variable_t condvar_t;
typedef mutex_t mutex_t;

inline mutex_t *get_mutex() {
    static MUTEX_DECL(mutex);
    return &mutex;
}

inline void _lock()
{
    chMtxLock(get_mutex());
}

inline void _unlock()
{
    chMtxUnlock(get_mutex());
}

inline void condvar_init(condvar_t *cv)
{
    chCondObjectInit(cv);
}

inline void condvar_destroy(condvar_t *cv)
{
    (void)cv;
}

inline void condvar_broadcast(condvar_t *cv)
{
    chCondBroadcast(cv);
}

inline bool condvar_wait(condvar_t *cv, float timeout)
{
    if (timeout == 0) {
        return false;
    }
    systime_t timeout_sys;
    if (timeout < 0) {
        timeout_sys = TIME_INFINITE;
    } else {
        timeout_sys = TIME_US2I(timeout*1e6);
    }
    msg_t ret = chCondWaitTimeout(cv, timeout_sys);
    if (ret == MSG_TIMEOUT) {
        chMtxLock(get_mutex());
        return false;
    } else {
        return true;
    }
}

}

#else // add your platform here
#error undefined platform
#endif



namespace msgbus {


// forward declartation for friend class
class SubscriberBase;

class TopicBase {
    friend class SubscriberBase;
public:
    TopicBase() :  m_pub_seq_nbr(0), m_published(false), event_list(NULL) {}
    TopicBase(const TopicBase&) = delete; // don't copy

    bool has_been_published()
    {
        bool published;
        _lock();
        published = with_lock_has_been_published();
        _unlock();
        return published;
    }

protected:
    void with_lock_notify_published() {
        m_pub_seq_nbr++;
        m_published=true;
        // wake up waiting subscribers
        PublishEventLink *ev = event_list;
        while (ev) {
            condvar_broadcast(ev->cv);
            ev = ev->next;
        }
    }

private:
    bool with_lock_has_been_published() {
        return m_published;
    }

    uint32_t with_lock_get_seq_nbr() {
        return m_pub_seq_nbr;
    }

    struct PublishEventLink {
        PublishEventLink() : cv(NULL), prev(NULL), next(NULL) {}
        condvar_t *cv;
        PublishEventLink *prev;
        PublishEventLink *next;
    };

    void with_lock_register_event_link(PublishEventLink &ev) {
        ev.next = event_list;
        if (event_list) {
            event_list->prev = &ev;
        }
        ev.prev = NULL;
        event_list = &ev;
    }

    void with_lock_unregister_event_link(PublishEventLink &ev) {
        if (ev.prev == NULL) { // at head of list
            event_list = ev.next;
        } else {
            ev.prev->next = ev.next;
        }
        if (ev.next != NULL) {
            ev.next->prev = ev.prev;
        }
    }

private:
    uint32_t m_pub_seq_nbr;
    bool m_published;
    PublishEventLink *event_list;
};


template <typename T>
class Topic : public TopicBase {
public:
    typedef T MsgType;
    Topic() {}

    void publish(const MsgType &value) {
        _lock();
        m_buffer = value;
        with_lock_notify_published();
        _unlock();
    }

public: // internal
    void _with_lock_read(MsgType *val) const {
        *val = m_buffer;
    }

private:
    MsgType m_buffer;
};



class SubscriberBase {
public:
    SubscriberBase(TopicBase &topic) : m_topic(topic), m_seq_nbr(0) {}
    SubscriberBase(const SubscriberBase&) = delete; // don't copy
    SubscriberBase(SubscriberBase&&) = default; // allow move

    bool wait_for_update(float timeout=-1) {
        _lock();
        if (_with_lock_has_update()) {
            _unlock();
            return true;
        }
        condvar_t cv;
        condvar_init(&cv);
        _with_lock_register_publish_ev_link(&cv);
        bool ret = condvar_wait(&cv, timeout);
        _with_lock_unregister_publish_ev_link();
        condvar_destroy(&cv);
        _unlock();
        return ret;
    }

    bool has_update() {
        bool update;
        _lock();
        update = _with_lock_has_update();
        _unlock();
        return update;
    }

    bool has_value() {
        return m_topic.has_been_published();
    }

    TopicBase &get_topic() {
        return m_topic;
    }

public: // internal
    void _with_lock_register_publish_ev_link(condvar_t *cv)
    {
        _publish_ev_link.cv = cv;
        m_topic.with_lock_register_event_link(_publish_ev_link);
    }

    void _with_lock_unregister_publish_ev_link()
    {
        m_topic.with_lock_unregister_event_link(_publish_ev_link);
    }

    bool _with_lock_has_update()
    {
        return m_topic.with_lock_has_been_published() && with_lock_get_nb_updates();
    }

private:
    uint32_t with_lock_get_nb_updates()
    {
        return m_topic.with_lock_get_seq_nbr() - m_seq_nbr;
    }

protected:
    void with_lock_update_seq_nbr_from_topic()
    {
        m_seq_nbr = m_topic.with_lock_get_seq_nbr();
    }

    void with_lock_assert_published()
    {
        assert(m_topic.with_lock_has_been_published());
    }

private:
    TopicBase &m_topic;
    uint32_t m_seq_nbr;
    TopicBase::PublishEventLink _publish_ev_link;
};

template <class T>
class Subscriber : public SubscriberBase {
public:
    typedef typename T::MsgType MsgType;
    Subscriber(T &topic) : SubscriberBase(topic), m_topic(topic) {}

    const MsgType get_value() {
        MsgType val;
        _lock();
        with_lock_assert_published();
        m_topic._with_lock_read(&val);
        with_lock_update_seq_nbr_from_topic();
        _unlock();
        return val;
    }
private:
    T &m_topic;
};


template <class T>
Subscriber<T> subscribe(T &topic) {
    return std::move(Subscriber<T>{topic});
}

template <typename subscriber_iterator>
bool wait_for_update_on_any(subscriber_iterator begin,
    subscriber_iterator end, float timeout=-1)
{
    _lock();
    subscriber_iterator sub;
    for(sub = begin; sub != end; sub++) {
        if ((*sub)->_with_lock_has_update()) {
            _unlock();
            return true;
        }
    }

    condvar_t cv;
    condvar_init(&cv);
    for(sub = begin; sub != end; sub++) {
        (*sub)->_with_lock_register_publish_ev_link(&cv);
    }
    bool ret = condvar_wait(&cv, timeout);
    for(sub = begin; sub != end; sub++) {
        (*sub)->_with_lock_unregister_publish_ev_link();
    }
    condvar_destroy(&cv);
    _unlock();
    return ret;
}


} // namespace msgbus

#endif /* MSGBUS_HPP */

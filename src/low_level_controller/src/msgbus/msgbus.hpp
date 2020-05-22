#ifndef MSGBUS_HPP
#define MSGBUS_HPP

#include <cstddef>
#include <cstdint>
#include <cassert>
#include <utility>

#include "port.hpp"


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

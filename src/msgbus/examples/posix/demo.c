#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include "../../msgbus.h"
#include "example_type.h"

msgbus_t bus;

static void* producer(void *p)
{
    const char *name = "test";

    msgbus_topic_t topic;
    example_t topic_buffer;
    msgbus_topic_create(&topic, &bus, &example_type, &topic_buffer, name);

    example_t x = {.counter = 0};
    while (1) {
        printf("[publisher] writing %d on topic %s\n", x.counter, name);
        msgbus_topic_publish(&topic, &x);
        x.counter += 1;
        sleep(1);
    }
}

static void *consumer(void *p)
{
    int consumer_number = (int)p;

    msgbus_subscriber_t sub;

    printf("[consumer %d] waiting for topic\n", consumer_number);
    msgbus_topic_subscribe(&sub, &bus, "test", MSGBUS_TIMEOUT_NEVER);

    example_t x;
    while (1) {
        msgbus_subscriber_wait_for_update(&sub, MSGBUS_TIMEOUT_NEVER);
        msgbus_subscriber_read(&sub, &x);
        printf("[consumer %d] read %d on topic\n", consumer_number, x.counter);
    }

    return NULL;
}

int main(int argc, const char **argv)
{
    (void) argc;
    (void) argv;

    /* Create the message bus. */
    msgbus_init(&bus);


    /* Creates a few consumer threads. */
    pthread_t producer_thd, consumer_thd;
    pthread_create(&consumer_thd, NULL, consumer, (void *)1);
    pthread_create(&consumer_thd, NULL, consumer, (void *)2);
    pthread_create(&consumer_thd, NULL, consumer, (void *)3);

    sleep(1);

    /* Creates a producer thread */
    pthread_create(&producer_thd, NULL, producer, NULL);

    while(1) {
        sleep(1);
    }
}

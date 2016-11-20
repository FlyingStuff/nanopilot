#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "thread_prio.h"
#include "main.h"
#include "rate_limiter.h"
#include "parameter/parameter.h"
#include "msgbus_scheduler.h"
#include "msgbus/type_print.h"
#include "log.h"
#include <string.h>

#include "stream.h"


struct stream_arg_s {
    rate_limiter_t rate_limiter;
    msgbus_subscriber_t sub;
    BaseSequentialStream *out_fd;
};


void stream_cb(void *arg)
{
    struct stream_arg_s *s = (struct stream_arg_s*)arg;

    msgbus_topic_t *topic = msgbus_subscriber_get_topic(&s->sub);
    const msgbus_type_definition_t *type = msgbus_topic_get_type(topic);
    void *buf = malloc(type->struct_size);
    if (buf == NULL) {
        chprintf(s->out_fd, "malloc failed\n");
        return;
    }
    msgbus_subscriber_read(&s->sub, buf);
    if (rate_limiter_should_run(&s->rate_limiter)) {
        msgbus_print_type((void (*)(void *, const char *, ...))chprintf,
                      s->out_fd, type, buf);
    }
    free(buf);
}

void stream_scheduler_add(struct stream_arg_s *stream,
                          msgbus_scheduler_t *sched,
                          parameter_namespace_t *ns,
                          BaseSequentialStream *out_fd,
                          const char *name)
{
    char *pname = malloc(strlen(name)+1);
    if (pname == NULL) {
        log_warning("stream %s could not be allocated", name);
        return;
    }
    const char *first = &name[0];
    while (first[0] == '/') {
        first++;
    }
    strcpy(pname, first);
    char *c = pname;
    while (*c != '\0') {
        if (*c == '/') {
            *c = '.';
        }
        c++;
    }
    msgbus_t *bus = msgbus_scheduler_get_bus(sched);
    if (msgbus_topic_subscribe(&stream->sub, bus, name, 1000000)) {
        stream->out_fd = out_fd;
        rate_limiter_init(&stream->rate_limiter, pname, ns);
        if (!msgbus_scheduler_add_task(sched, &stream->sub, stream_cb, stream)) {
            log_warning("stream %s could not be allocated", name);
        }
    } else {
        log_warning("stream %s topic not found", name);
    }
}

static THD_WORKING_AREA(stream_wa, 1024);
static THD_FUNCTION(stream, arg)
{
    chRegSetThreadName("stream");
    BaseSequentialStream *out_fd = (BaseSequentialStream*)arg;

    msgbus_scheduler_t sched;
    static msgbus_scheduler_task_buffer_space_t buf[100];
    msgbus_scheduler_init(&sched, &bus, buf, sizeof(buf)/sizeof(buf[0]));

    parameter_namespace_t stream_ns;
    parameter_namespace_declare(&stream_ns, &parameters, "streams");

    static struct stream_arg_s streams[sizeof(buf)/sizeof(buf[0])];
    struct stream_arg_s *s = streams;

    stream_scheduler_add(s++, &sched, &stream_ns, out_fd, "/sensors/mpu6000");
    stream_scheduler_add(s++, &sched, &stream_ns, out_fd, "/sensors/hmc5883l");
    stream_scheduler_add(s++, &sched, &stream_ns, out_fd, "/sensors/h3lis331dl");
    stream_scheduler_add(s++, &sched, &stream_ns, out_fd, "/sensors/ms5611");

    while(1) {
        msgbus_scheduler_spin(&sched, MSGBUS_TIMEOUT_NEVER);
    }
}

void stream_start(BaseSequentialStream *out)
{
    chThdCreateStatic(stream_wa, sizeof(stream_wa), THD_PRIO_STREAM, stream, out);
}

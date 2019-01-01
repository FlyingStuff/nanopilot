#ifndef SHELL_CMDS_H
#define SHELL_CMDS_H

#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif

void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_cpu(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_panic(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_panic_get(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_parameter_list(BaseSequentialStream *stream, int argc, char *argv[]);
void cmd_parameter_set(BaseSequentialStream *stream, int argc, char *argv[]);
void cmd_stdout_log_lvl(BaseSequentialStream *stream, int argc, char *argv[]);
void cmd_parameter_load(BaseSequentialStream *stream, int argc, char *argv[]);
void cmd_parameter_save(BaseSequentialStream *stream, int argc, char *argv[]);
void cmd_parameter_erase(BaseSequentialStream *stream, int argc, char *argv[]);
void cmd_calibrate_esc(BaseSequentialStream *stream, int argc, char *argv[]);


#ifdef __cplusplus
}
#endif

#endif /* SHELL_CMDS_H */

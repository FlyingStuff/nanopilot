#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif


#define LOG_COLOR_BLUE "\033[1;34m"
#define LOG_COLOR_RED "\033[1;31m"
#define LOG_COLOR_CLEAR "\033[0m"

#define LOG_LVL_INFO ""
#define LOG_LVL_WARNING LOG_COLOR_RED "\033[0;33mWARNING " LOG_COLOR_CLEAR
#define LOG_LVL_ERROR LOG_COLOR_RED "ERROR " LOG_COLOR_CLEAR

#define log_info(...) log_message(LOG_LVL_INFO, __VA_ARGS__);
#define log_warning(...) log_message(LOG_LVL_WARNING, __VA_ARGS__);
#define log_error(...) log_message(LOG_LVL_ERROR, __VA_ARGS__);

void log_message(const char *lvl, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */

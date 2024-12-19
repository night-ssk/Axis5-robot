/*
 *
 *  使用方法:
 *
 *  1.  初始化:
 *      -   无需显式初始化。静态变量会自动初始化。
 *
 *  2.  设置日志级别:
 *      -   `void log_set_level(int level);`
 *          -   设置全局日志级别。低于此级别的日志消息将被忽略。
 *          -   `level` 可以是 `LOG_TRACE`, `LOG_DEBUG`, `LOG_INFO`, `LOG_WARN`, `LOG_ERROR`, `LOG_FATAL` 中的一个。
 *
 *  3.  添加日志输出目标:
 *      -   `int log_add_callback(log_LogFn fn, void *udata, int level);`
 *          -   添加一个自定义的回调函数作为日志输出目标。
 *          -   `fn` 是回调函数，类型为 `log_LogFn`。
 *          -   `udata` 是用户数据，会传递给回调函数。
 *          -   `level` 是此回调函数的日志级别，低于此级别的日志消息不会发送到此回调。
 *          -   返回 0 表示成功，-1 表示失败（达到最大回调数）。
 *      -   `int log_add_fp(FILE *fp, int level);`
 *          -   添加一个文件指针作为日志输出目标。
 *          -   `fp` 是文件指针，例如 `stdout`, `stderr` 或 `fopen` 返回的文件。
 *          -   `level` 是此文件输出的日志级别。
 *          -   实际上是调用 `log_add_callback`，并使用 `file_callback` 作为回调函数。
 *
 *  4.  设置锁:
 *      -  `void log_set_lock(log_LockFn fn, void *udata);`
 *          -   设置一个锁函数，用于多线程环境下的日志操作。
 *          -   `fn` 是锁函数，类型为 `log_LockFn`。
 *          -   `udata` 是用户数据，会传递给锁函数。
 *
 *  5.  设置静默模式:
 *      -   `void log_set_quiet(bool enable);`
 *          -   启用或禁用控制台输出。
 *          -   `enable` 为 `true` 时禁用控制台输出, 为 `false` 时启用控制台输出
 *
 *  6.  输出日志:
 *      -   `void log_log(int level, const char *file, int line, const char *fmt, ...);`
 *          -   输出日志消息。
 *          -   `level` 是日志级别。
 *          -   `file` 是文件名，通常使用 `__FILE__` 宏。
 *          -   `line` 是行号，通常使用 `__LINE__` 宏。
 *          -   `fmt` 是格式化字符串，与 `printf` 类似。
  *          -   可以通过在fmt中添加`LOG_FREQUENCY(n)`来设置此条日志的打印频率, n为打印频率
 *          -   `...` 是可变参数，传递给格式化字符串。
 *
 *  7.  获取日志级别字符串
 *      -   `const char* log_level_string(int level);`
 *          -   根据日志级别获取对应的字符串, 比如 `LOG_DEBUG` 会返回 `"DEBUG"`
 *
 *  示例:
 *
 *  ```c
 *  #include "log.h"
 *
 *  int main() {
 *      log_set_level(LOG_DEBUG);
 *      log_add_fp(stdout, LOG_DEBUG);
 *      log_info("This is an info message");
 *      log_debug("This is a debug message");
 *      log_warn("This is a warning message");
 *      log_error("This is an error message");
 *       log_info("LOG_FREQUENCY(5) This is an info message with frequency 5");
 *      log_fatal("This is a fatal message");
 *      return 0;
 *  }
 *  ```
 */
#include "log.h"
#include <string.h>
#include <stdlib.h>

#define MAX_CALLBACKS 32
#define LOG_FREQUENCY_STR "LOG_FREQUENCY("

typedef struct {
  log_LogFn fn;
  void *udata;
  int level;
    int print_frequency;
    int print_counter;
} Callback;

static struct {
  void *udata;
  log_LockFn lock;
  int level;
  bool quiet;
  Callback callbacks[MAX_CALLBACKS];
} L;


static const char *level_strings[] = {
  "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
};

#ifdef LOG_USE_COLOR
static const char *level_colors[] = {
  "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"
};
#endif


static void stdout_callback(log_Event *ev) {
  char buf[16];
  buf[strftime(buf, sizeof(buf), "%H:%M:%S", ev->time)] = '\0';
#ifdef LOG_USE_COLOR
  fprintf(
    ev->udata, "%s %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ",
    buf, level_colors[ev->level], level_strings[ev->level],
    ev->file, ev->line);
#else
  fprintf(
    ev->udata, "%s %-5s %s:%d: ",
    buf, level_strings[ev->level], ev->file, ev->line);
#endif
  vfprintf(ev->udata, ev->fmt, ev->ap);
  fprintf(ev->udata, "\n");
  fflush(ev->udata);
}


static void file_callback(log_Event *ev) {
  char buf[64];
  buf[strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ev->time)] = '\0';
  fprintf(
    ev->udata, "%s %-5s %s:%d: ",
    buf, level_strings[ev->level], ev->file, ev->line);
  vfprintf(ev->udata, ev->fmt, ev->ap);
  fprintf(ev->udata, "\n");
  fflush(ev->udata);
}


static void lock(void)   {
  if (L.lock) { L.lock(true, L.udata); }
}


static void unlock(void) {
  if (L.lock) { L.lock(false, L.udata); }
}


const char* log_level_string(int level) {
  return level_strings[level];
}


void log_set_lock(log_LockFn fn, void *udata) {
  L.lock = fn;
  L.udata = udata;
}


void log_set_level(int level) {
  L.level = level;
}


void log_set_quiet(bool enable) {
  L.quiet = enable;
}


int log_add_callback(log_LogFn fn, void *udata, int level) {
  for (int i = 0; i < MAX_CALLBACKS; i++) {
    if (!L.callbacks[i].fn) {
      L.callbacks[i] = (Callback) { fn, udata, level , .print_frequency = 1, .print_counter = 0};
      return 0;
    }
  }
  return -1;
}


int log_add_fp(FILE *fp, int level) {
  return log_add_callback(file_callback, fp, level);
}


static void init_event(log_Event *ev, void *udata) {
  if (!ev->time) {
    time_t t = time(NULL);
    ev->time = localtime(&t);
  }
  ev->udata = udata;
}

static int parse_log_frequency(const char **fmt) {
    int frequency = 1;
    const char *freq_str = strstr(*fmt, LOG_FREQUENCY_STR);
    if(freq_str){
        const char *start = freq_str + strlen(LOG_FREQUENCY_STR);
        char *end;
        frequency = strtol(start, &end, 10);
          if(end && *end == ')'){
               *fmt = end + 1;
          }
        if(frequency <= 0){
            frequency = 1;
        }
    }
    return frequency;

}


void log_log(int level, const char *file, int line, const char *fmt, ...) {
  log_Event ev = {
    .fmt   = fmt,
    .file  = file,
    .line  = line,
    .level = level,
  };

  lock();
    int frequency = parse_log_frequency(&ev.fmt);
  if (!L.quiet && level >= L.level) {
      static int stdout_counter = 0;
       stdout_counter++;
    if(stdout_counter % frequency == 0){
        init_event(&ev, stderr);
        va_start(ev.ap, fmt);
        stdout_callback(&ev);
        va_end(ev.ap);
    }

  }

  for (int i = 0; i < MAX_CALLBACKS && L.callbacks[i].fn; i++) {
    Callback *cb = &L.callbacks[i];
       if (level >= cb->level) {
            cb->print_counter++;
            if(cb->print_counter % cb->print_frequency == 0){
                init_event(&ev, cb->udata);
                va_start(ev.ap, fmt);
               cb->fn(&ev);
                va_end(ev.ap);
            }
       }
  }

  unlock();
}
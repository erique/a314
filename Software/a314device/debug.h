#define DEBUG 1

#include "kprintf.h"

extern void dbg_init();
extern void dbg(const char* fmt, ...);

#if !DEBUG

#define dbg_error(...)      do{} while(0)
#define dbg_warning(...)    do{} while(0)
#define dbg_info(...)       do{} while(0)
#define dbg_trace(...)      do{} while(0)

#else

extern void dbg_init();
extern void dbg(const char* fmt, ...);

#define dbg_error(...) dbg(__VA_ARGS__)
#define dbg_warning(...) dbg(__VA_ARGS__)
#define dbg_info(...) dbg(__VA_ARGS__)
#define dbg_trace(...) dbg(__VA_ARGS__)

#endif

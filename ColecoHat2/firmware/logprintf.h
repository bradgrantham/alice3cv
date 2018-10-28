#ifndef __LOGPRINTF_H__
#define __LOGPRINTF_H__

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

enum DebugLevels {
    DEBUG_SILENT = 0,
    DEBUG_ERRORS,
    DEBUG_WARNINGS,
    DEBUG_EVENTS,
    DEBUG_DATA,
    DEBUG_ALL,
    DEBUG_INSANE = 99,
};
extern int gDebugLevel;

void logprintf(int level, char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __LOGPRINTF_H__ */

#ifndef _ASSERT_H_INCLUDED_
#define _ASSERT_H_INCLUDED_
#include <stdint.h>

extern void spinDelayUs(uint32_t microseconds);
extern void spinDelayMs(uint32_t milliseconds);
extern void _assert_failed (const char *assertion, const char *file, unsigned int line);

#ifdef NDEBUG
#	define assert(expr)	(void (0))
#else
#	define assert(x)   ((x)?((void) 0):_assert_failed (#x, __FILE__, __LINE__))
#endif

#endif


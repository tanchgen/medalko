#ifndef _MAIN_H
#define _MAIN_H

#include <stddef.h>
#include <stdbool.h>

#include "stm32_it.h"

#include "bitbanding.h"
#include "types.h"
#include "tinyalloc.h"
#include "gpio.h"
#include "iface.h"
#include "list.h"
#include "nvic.h"
#include "pmbus.h"
#include "rcc.h"
#include "swab.h"
#include "times.h"

/** Тайм-аут переходов между состояниями выводов GPIO FPGA_EN_x или FPGA_EN_x. */
#define TOUT_50       50
#define TOUT_100      100
#define TOUT_200      200
#define TOUT_300      300
#define TOUT_500      500
#define TOUT_1000     1000
#define TOUT_1500     1500
#define TOUT_2000     2000

#define INIT_TOUT     5000

#define FPGA_DONE_TOUT (TOUT_1000 * 15)

#if defined(DEBUG)
#define SWO_ENABLE      1

#define DEBUG_TRACE     0
#define DEBUG_TRACE_D   1
#define DEBUG_TRACE_RUN 1
#define DEBUG_DATA      0
#define DEBUG_DATA_VP   0
#define DEBUG_ALARM     1
#define DEBUG_LOG       1
#else
#undef SWO_ENABLE

#define DEBUG_TRACE     0
#define DEBUG_TRACE_D   0
#define DEBUG_TRACE_RUN 0
#define DEBUG_DATA      0
#define DEBUG_DATA_VP   0
#define DEBUG_ALARM     0
#endif


#include "diag/Trace.h"

#if SWO_ENABLE
extern char prnbuf[];
#endif


#ifndef  __packed
#define __packed __attribute__ ((__packed__))
#endif

//enum {
//  RESET,
//  SET
//};

/** Сборка кода версии программного обеспечения. */
#define FW_VERSION(a, b, c)	(((a) << 13) + ((b) << 8) + (c))

/** Force a compilation error if condition is constant and true. */
#ifndef MAYBE_BUILD_BUG_ON
#	define MAYBE_BUILD_BUG_ON(cond)	  ((void)sizeof(char[1 - 2 * !!(cond)]))
#endif

/**
 * @brief	Cast a member of a structure out to the containing structure.
 *
 * @param[in]	ptr	the pointer to the member
 * @param[in]	type	the type of the container struct this is embedded in
 * @param[in]	member	the name of the member within the struct
 *
 */
#ifndef container_of
#	define container_of(ptr, type, member)	((type *)((size_t)(ptr) - offsetof(type, member)))
#endif

/** min() macro without type-checking. */
#ifndef min
#	define min(a, b)	((a) < (b) ? (a) : (b))
#endif

/** max() macro without type-checking. */
#ifndef max
#	define max(a, b)	((a) > (b) ? (a) : (b))
#endif

/** roundup() macro without type-checking. */
#ifndef roundup
#	define roundup(x, y)	((((x) + ((y) - 1)) / (y)) * (y))
#endif

/** DIV_ROUND_UP() macro without type-checking. */
#ifndef DIV_ROUND_UP
#	define DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))
#endif

/** ARRAY_SIZE() macro. */
#ifndef ARRAY_SIZE
#	define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))
#endif

/** FIELD_SIZEOF() macro. */
#ifndef FIELD_SIZEOF
#	define FIELD_SIZEOF(t, f)	(sizeof(((t *)0)->f))
#endif

#ifndef _STRINGIFY
#define _STRINGIFY_B(x)   #x
#define _STRINGIFY(x)   _STRINGIFY_B(x)
#endif  //_STRINGIFY

#define MOV_AVG_NUM   (5)
// Расчет скользящего среднего
static inline void fMovAvg( float *avg, float pt ){
  const float a = 2.0 / (1 + MOV_AVG_NUM);

  *avg = pt * a + (*avg * (1 - a));
}


#endif /* _MAIN_H */


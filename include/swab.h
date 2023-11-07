#ifndef _SWAB_H
#define _SWAB_H

#include <stdint.h>

#ifdef STM32F4XX
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#endif // STM32F4XX


#define constant_swab16(x) ((uint16_t)(				\
	(((uint16_t)(x) & (uint16_t)0x00ffU) << 8) |		\
	(((uint16_t)(x) & (uint16_t)0xff00U) >> 8)))

#define constant_swab32(x) ((uint32_t)(				\
	(((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) |	\
	(((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) |	\
	(((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) |	\
	(((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24)))

/**
  * @brief	Return a byteswapped 16-bit value.
  *
  * @param[in]	x	value to byteswap
  *
  * @retval	a byteswapped 16-bit value
  */
__STATIC_INLINE uint16_t swab16(uint16_t x)
{
	return (__REV(x) >> 16);
}

/**
  * @brief	Return a byteswapped 32-bit value.
  *
  * @param[in]	x	value to byteswap
  *
  * @retval	a byteswapped 32-bit value
  */
__STATIC_INLINE uint32_t swab32(uint32_t x)
{
	return __REV(x);
}

#endif /* _SWAB_H */


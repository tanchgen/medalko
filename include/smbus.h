#ifndef _SMBUS_H
#define _SMBUS_H

#include <stddef.h>
#include <stdint.h>

#include "stm32f10x.h"


#define PEC_DISABLE         RESET
#define PEC_ENABLE          SET


/**
  * @brief	Преобразование данных из формата Linear-11 в действительное значение величины.
  *
  * @param[in]	data	преобразуемые данные
  *
  * @retval	действительное значение величины
  */
float fromLinear11F(uint16_t data);

/**
  * @brief	Преобразование данных из действительного значения в формат Linear-11.
  *
  * @param[in]	data	преобразуемые данные
  *
  * @retval	данные в формате Linear-11
  */
uint16_t toLinea11F(float data);

/**
  * @brief  Преобразование данных в Linear_11
  *
  * @param[in] data  Преобразовываемые данные (ед.изм. = 1/k от основной ед.)
  *
  * @retval Данные в формате linear_11
  */
uint16_t toLinear11( int32_t data, uint16_t k );

/**
  * @brief  Преобразование данных из Linear_11
  *
  * @param[in] data  Данные в формате linear_11
  *
  * @retval Преобразованные данные (ед.изм. = 1/4096 от основной ед.)
  */
int32_t fromLinear11( int32_t data, uint16_t k  );

#endif /* _SMBUS_H */


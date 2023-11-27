/*
 * buffer.meas.h
 *
 *  Created on: Фев 20, 2020
 *      Author: jet
 */

#ifndef MEAS_BUF_H
#define MEAS_BUF_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "measur.h"
/**
 * @defgroup measBuf_Macros
 * @brief    Library defines
 * @{
 */

#define measBuf_INITIALIZED     0x01 /*!< Buffer initialized flag */
#define measBuf_MALLOC          0x02 /*!< Buffer uses malloc for memory */
#define measBuf_OVER            0x04

/* Custom allocation and free functions if needed */
#ifndef LIB_ALLOC_FUNC
#define LIB_ALLOC_FUNC         malloc
#endif
#ifndef LIB_FREE_FUNC
#define LIB_FREE_FUNC          free
#endif

#ifndef measBuf_FAST
#define measBuf_FAST            1
#endif

/**
 * @}
 */

/**
 * @defgroup measBuf_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Buffer structure
 */
typedef struct _measBuf_t {
  uint32_t Size;           /*!< Size of buffer in units of sizeof( sCanMsg ), DO NOT MOVE OFFSET, 0 */
  sMeasRec * In;             /*!< Input pointer to save next struct sCanMsg, DO NOT MOVE OFFSET, 1 */
  sMeasRec * Out;            /*!< Output pointer to read next value, DO NOT MOVE OFFSET, 2 */
  sMeasRec * Buffer;         /*!< Pointer to buffer data array, DO NOT MOVE OFFSET, 3 */
  uint8_t Flags;           /*!< Flags for buffer, DO NOT MOVE OFFSET, 4 */
//  size_t  recSize;         /*!< size of records */
} measBuf_t;


extern sMeasRec measRecBuff[MEAS_SEQ_NUM_MAX];
extern measBuf_t measBuf;

/**
 * @}
 */

/**
 * @defgroup measBuf_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes buffer structure for work
 * @param  *Buffer: Pointer to @ref measBuf_t structure to initialize
 * @param  Size: Size of buffer in units of bytes
 * @param  *BufferPtr: Pointer to array for buffer storage. Its length should be equal to @param Size parameter.
 *           If NULL is passed as parameter, @ref malloc will be used to allocate memory on heap.
 * @retval Buffer initialization status:
 *            - 0: Buffer initialized OK
 *            - > 0: Buffer initialization error. Malloc has failed with allocation
 */
uint8_t measBuf_Init(measBuf_t* Buffer, sMeasRec * BufferPtr, uint16_t Size, uint16_t recsize );

/**
 * @brief  Free memory for buffer allocated using @ref malloc
 * @note   This function has sense only if malloc was used for dynamic allocation
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @retval None
 */
void measBuf_Free(measBuf_t* Buffer);

/**
 * @brief  Writes data to buffer
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @param  *Data: Pointer to data to be written
 * @param  count: Number of elements of type unsigned char to write
 * @retval Number of elements written in buffer
 */
uint16_t measBuf_Write(measBuf_t* Buffer, sMeasRec * Data, uint16_t count);
uint16_t measBuf_WriteMsg(measBuf_t* Buffer, sMeasRec * msg);

/**
 * @brief  Reads data from buffer
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @param  *Data: Pointer to data where read values will be stored
 * @param  count: Number of elements of type unsigned char to read
 * @retval Number of elements read from buffer
 */
uint16_t measBuf_Read(measBuf_t* Buffer, sMeasRec * Data, uint16_t count);
uint16_t measBuf_ReadMsg(measBuf_t* Buffer, sMeasRec * msg);

/**
 * @brief  Check: buffer is free?
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @retval  1 - buffer is free
 *          0 - buffer isn't free
 */
static inline bool measBuf_IsFree(measBuf_t* Buffer) {
  return  Buffer->In == Buffer->Out;

}


/**
 * @brief  Gets number of free elements in buffer
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @retval Number of free elements in buffer
 */
uint16_t measBuf_GetFree(measBuf_t* Buffer);

/**
 * @brief  Gets number of elements in buffer
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @retval Number of elements in buffer
 */
uint16_t measBuf_GetFull(measBuf_t* Buffer);

/**
 * @brief  Resets (clears) buffer pointers
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @retval None
 */
void measBuf_Reset(measBuf_t* Buffer);

/**
 * @brief  Checks if specific element value is stored in buffer
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @param  uint8_t Element: Element to check
 * @retval Status of element:
 *            -  < 0: Element was not found
 *            - >= 0: Element found, location in buffer is returned
 *                   Ex: If value 1 is returned, it means 1 read from buffer and your element will be returned
 */
int16_t measBuf_FindElement(measBuf_t* Buffer, sMeasRec * Element);

/**
 * @brief  Sets string delimiter character when reading from buffer as string
 * @param  Buffer: Pointer to @ref measBuf_t structure
 * @param  StrDel: Character as string delimiter
 * @retval None
 */
#define measBuf_SetStringDelimiter(Buffer, StrDel)  ((Buffer)->StringDelimiter = (StrDel))

/**
 * @brief  Checks if character exists in location in buffer
 * @param  *Buffer: Pointer to @ref measBuf_t structure
 * @param  pos: Position in buffer, starting from 0
 * @param  *element: Pointer to save value at desired position to be stored into
 * @retval Check status:
 *            - 0: Buffer is not so long as position desired
 *            - > 0: Position to check was inside buffer data size
 */
int8_t measBuf_CheckElement(measBuf_t* Buffer, uint16_t pos, uint8_t* element);

/**
 * @}
 */

/**
 * @}
 */


#endif /* MEAS_BUF_H_ */

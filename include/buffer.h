/*
 * buffer.h
 *
 *  Created on: Фев 20, 2020
 *      Author: jet
 */

#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup buffer_Macros
 * @brief    Library defines
 * @{
 */

#define RX_BUFF_SIZE          256

#define buffer_INITIALIZED     0x01 /*!< Buffer initialized flag */
#define buffer_MALLOC          0x02 /*!< Buffer uses malloc for memory */
#define buffer_OVER            0x04

/* Custom allocation and free functions if needed */
#ifndef LIB_ALLOC_FUNC
#define LIB_ALLOC_FUNC         malloc
#endif
#ifndef LIB_FREE_FUNC
#define LIB_FREE_FUNC          free
#endif

#ifndef buffer_FAST
#define buffer_FAST            1
#endif

/**
 * @}
 */

/**
 * @defgroup buffer_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Buffer structure
 */
typedef struct _Buffer_t {
  uint32_t Size;           /*!< Size of buffer in units of sizeof( sCanMsg ), DO NOT MOVE OFFSET, 0 */
  uint8_t * In;             /*!< Input pointer to save next struct sCanMsg, DO NOT MOVE OFFSET, 1 */
  uint8_t * Out;            /*!< Output pointer to read next value, DO NOT MOVE OFFSET, 2 */
  uint8_t * Buffer;         /*!< Pointer to buffer data array, DO NOT MOVE OFFSET, 3 */
  uint8_t Flags;           /*!< Flags for buffer, DO NOT MOVE OFFSET, 4 */
//  size_t  recSize;         /*!< size of records */
} Buffer_t;


extern uint8_t receivBuff[256];
extern Buffer_t rxBuf;

/**
 * @}
 */

/**
 * @defgroup buffer_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes buffer structure for work
 * @param  *Buffer: Pointer to @ref Buffer_t structure to initialize
 * @param  Size: Size of buffer in units of bytes
 * @param  *BufferPtr: Pointer to array for buffer storage. Its length should be equal to @param Size parameter.
 *           If NULL is passed as parameter, @ref malloc will be used to allocate memory on heap.
 * @retval Buffer initialization status:
 *            - 0: Buffer initialized OK
 *            - > 0: Buffer initialization error. Malloc has failed with allocation
 */
uint8_t buffer_Init(Buffer_t* Buffer, uint8_t * BufferPtr, uint16_t Size );

/**
 * @brief  Free memory for buffer allocated using @ref malloc
 * @note   This function has sense only if malloc was used for dynamic allocation
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @retval None
 */
void buffer_Free(Buffer_t* Buffer);

/**
 * @brief  Writes data to buffer
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @param  *Data: Pointer to data to be written
 * @param  count: Number of elements of type unsigned char to write
 * @retval Number of elements written in buffer
 */
uint16_t buffer_Write(Buffer_t* Buffer, uint8_t * Data, uint16_t count);
uint16_t buffer_WriteMsg(Buffer_t* Buffer, uint8_t * msg);

/**
 * @brief  Reads data from buffer
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @param  *Data: Pointer to data where read values will be stored
 * @param  count: Number of elements of type unsigned char to read
 * @retval Number of elements read from buffer
 */
uint16_t buffer_Read(Buffer_t* Buffer, uint8_t * Data, uint16_t count);
uint16_t buffer_ReadMsg(Buffer_t* Buffer, uint8_t * msg);

/**
 * @brief  Check: buffer is free?
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @retval  1 - buffer is free
 *          0 - buffer isn't free
 */
static inline bool buffer_IsFree(Buffer_t* Buffer) {
  return  Buffer->In == Buffer->Out;

}


/**
 * @brief  Gets number of free elements in buffer
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @retval Number of free elements in buffer
 */
uint16_t buffer_GetFree(Buffer_t* Buffer);

/**
 * @brief  Gets number of elements in buffer
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @retval Number of elements in buffer
 */
uint16_t buffer_GetFull(Buffer_t* Buffer);

/**
 * @brief  Resets (clears) buffer pointers
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @retval None
 */
void buffer_Reset(Buffer_t* Buffer);

int16_t buffer_FindChar(Buffer_t* Buffer, uint8_t ch);

/**
 * @brief  Checks if specific element value is stored in buffer
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @param  uint8_t Element: Element to check
 * @retval Status of element:
 *            -  < 0: Element was not found
 *            - >= 0: Element found, location in buffer is returned
 *                   Ex: If value 1 is returned, it means 1 read from buffer and your element will be returned
 */
int16_t buffer_FindElement(Buffer_t* Buffer, uint8_t * Element);

/**
 * @brief  Sets string delimiter character when reading from buffer as string
 * @param  Buffer: Pointer to @ref Buffer_t structure
 * @param  StrDel: Character as string delimiter
 * @retval None
 */
#define buffer_SetStringDelimiter(Buffer, StrDel)  ((Buffer)->StringDelimiter = (StrDel))

/**
 * @brief  Checks if character exists in location in buffer
 * @param  *Buffer: Pointer to @ref Buffer_t structure
 * @param  pos: Position in buffer, starting from 0
 * @param  *element: Pointer to save value at desired position to be stored into
 * @retval Check status:
 *            - 0: Buffer is not so long as position desired
 *            - > 0: Position to check was inside buffer data size
 */
int8_t buffer_CheckElement(Buffer_t* Buffer, uint16_t pos, uint8_t* element);

/**
 * @}
 */

/**
 * @}
 */


#endif /* BUFFER_H_ */

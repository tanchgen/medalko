/*
 * buffer.c
 *
 *  Created on: Фев 20, 2020
 *      Author: jet
 */

#include "buffer.meas.h"

sMeasRec measRecBuff[MEAS_SEQ_NUM_MAX];
measBuf_t measBuf;

uint8_t measBuf_Init(measBuf_t* Buffer, sMeasRec * BufferPtr, uint16_t Size, uint16_t recsize ) {
  (void)recsize;
	/* Set buffer values to all zeros */
	memset(Buffer, 0, sizeof(measBuf_t));

	/* Set default values */
	Buffer->Size = Size;
	Buffer->Buffer = BufferPtr;

	/* Check if malloc should be used */
	if (!Buffer->Buffer) {
		/* Try to allocate */
		Buffer->Buffer = (sMeasRec *) LIB_ALLOC_FUNC(Size * sizeof(sMeasRec));

		/* Check if allocated */
		if (!Buffer->Buffer) {
			/* Reset size */
			Buffer->Size = 0;

			/* Return error */
			return 1;
		} else {
			/* Set flag for malloc */
			Buffer->Flags |= measBuf_MALLOC;
		}
	}

	Buffer->In = Buffer->Out = Buffer->Buffer;
	/* We are initialized */
	Buffer->Flags |= measBuf_INITIALIZED;
//  Buffer->recSize = recsize;

	/* Initialized OK */
	return 0;
}

void measBuf_Free(measBuf_t* Buffer) {
	/* Check buffer structure */
	if (Buffer == NULL) {
		return;
	}

	/* If malloc was used for allocation */
	if (Buffer->Flags & measBuf_MALLOC) {
		/* Free memory */
		LIB_FREE_FUNC(Buffer->Buffer);
	}

	/* Clear flags */
	Buffer->Flags = 0;
	Buffer->Size = 0;
}

uint16_t measBuf_Write(measBuf_t* Buffer, sMeasRec * pkt, uint16_t count) {
	uint32_t i = 0;
	uint32_t free;

	/* Check buffer structure */
	if (Buffer == NULL || count == 0) {
		return 0;
	}

//	/* Check input pointer */
//	if (Buffer->In >= (Buffer->Buffer + Buffer->Size) ){
//		Buffer->In = Buffer->Buffer;
//	}

	/* Get free memory */
	free = measBuf_GetFree(Buffer);

	/* Check available memory */
	if (free < count) {
		/* If no memory, stop execution */
		if (free == 0) {
			return 0;
		}

		/* Set values for write */
		count = free;
		Buffer->Flags |= measBuf_OVER;
	}
	else if (free == count){
    Buffer->Flags |= measBuf_OVER;
	}

	/* We have calculated memory for write */

	/* Go through all elements */
	while (count--) {
		/* Add to buffer */
		*Buffer->In++ = *pkt++;

		/* Increase pointers */
		i++;

		/* Check input overflow */
		if (Buffer->In >= (Buffer->Buffer + Buffer->Size) ) {
			Buffer->In = Buffer->Buffer;
		}
	}

	/* Return number of elements written */
	return i;
}

uint16_t measBuf_Read(measBuf_t* Buffer, sMeasRec * pkt, uint16_t count) {
	uint32_t i = 0, full;

	/* Check buffer structure */
	if (Buffer == NULL || count == 0) {
		return 0;
	}

//	/* Check output pointer */
//	if (Buffer->Out >= (Buffer->Buffer + Buffer->Size) ) {
//		Buffer->Out = Buffer->Buffer;
//	}

	/* Get free memory */
	full = measBuf_GetFull(Buffer);

	/* Check available memory */
	if (full < count) {
		/* If no memory, stop execution */
		if (full == 0) {
			return 0;
		}

		/* Set values for write */
		count = full;
	}

	/* We have calculated memory for write */

	/* Go through all elements */
	while (count--) {
		/* Read from buffer */
		*pkt++ = *Buffer->Out++;

		/* Increase pointers */
		i++;

		/* Check output overflow */
		if (Buffer->Out >= (Buffer->Buffer + Buffer->Size) ) {
			Buffer->Out = Buffer->Buffer;
		}
	}
	Buffer->Flags &= ~measBuf_OVER;
	/* Return number of elements stored in memory */
	return i;
}

uint16_t measBuf_ReadMsg(measBuf_t* Buffer, sMeasRec * pkt ) {
  uint32_t full;
  sMeasRec * pout = Buffer->Out;

  /* Check buffer structure */
  if (Buffer == NULL) {
    return 0;
  }

//  /* Check output pointer */
//  if (Buffer->Out >= (Buffer->Buffer + Buffer->Size) ) {
//    Buffer->Out = Buffer->Buffer;
//  }

  /* Get free memory */
  full = measBuf_GetFull(Buffer);

  /* Check available memory */
  if (full == 0) {
    /* If no memory, stop execution */
    return 0;
  }

  /* We have calculated memory for write */
  /* Read from buffer */
  *pkt = *pout++;

  /* Check output overflow */
  if (pout >= (Buffer->Buffer + Buffer->Size) ) {
    pout = Buffer->Buffer;
  }

  Buffer->Flags &= ~measBuf_OVER;

  Buffer->Out = pout;

  /* Return number of elements stored in memory */
  return 1;
}


uint16_t measBuf_WriteMsg(measBuf_t* Buffer, sMeasRec * pkt) {
	uint32_t i = 0;
	uint32_t free;
  sMeasRec * pIn = Buffer->In;

	/* Check buffer structure */
	if (Buffer == NULL) {
		return 0;
	}

//	/* Check input pointer */
//	if (Buffer->In >= (Buffer->Buffer + Buffer->Size) ) {
//		Buffer->In = Buffer->Buffer;
//	}

	/* Get free memory */
	free = measBuf_GetFree(Buffer);

	// Check available memory
	// If no memory, stop execution
	if (free == 0) {
		return i;
	}

	/* We have calculated memory for write */

	*pIn++ = *pkt;

  /* Increase pointers */
  i++;
	/* Check input overflow */
	if (pIn >= (Buffer->Buffer + Buffer->Size) ) {
		pIn = Buffer->Buffer;
	}

	if( i == free ){
	  Buffer->Flags |= measBuf_OVER;
	}

  Buffer->In = pIn;

	/* Return number of elements stored in memory */
	return i;
}

uint16_t measBuf_GetFree(measBuf_t* Buffer) {
	uint32_t size = 0;
	sMeasRec *in;
	sMeasRec * out;

	/* Check buffer structure */
	if (Buffer == NULL) {
		return 0;
	}

	/* Save values */
	in = Buffer->In;
	out = Buffer->Out;

	/* Check if the same */
	if (in == out) {
		size = (Buffer->Flags & measBuf_OVER)? 0: Buffer->Size;
	}
	else if (out > in) {     /* Check normal mode */
		size = out - in;
	}
	else if (in > out) {     /* Check if overflow mode */
		size = Buffer->Size - (in - out);
	}

	/* Return free memory */
	return size;
}

uint16_t measBuf_GetFull(measBuf_t* Buffer) {
	uint32_t size = 0;
	sMeasRec * in;
	sMeasRec * out;

	/* Check buffer structure */
	if (Buffer == NULL) {
		return 0;
	}

	/* Save values */
	in = Buffer->In;
	out = Buffer->Out;

  if (in == out) {
    size = (Buffer->Flags & measBuf_OVER)? Buffer->Size : 0;
  }
  else if (in > out) {      /* Buffer is not in overflow mode */
    size = in - out;
  }
  else if (out > in) {     /* Buffer is in overflow mode */
    size = Buffer->Size - (out - in);
  }

	/* Return number of elements in buffer */
	return size;
}

void measBuf_Reset(measBuf_t* Buffer) {
	/* Check buffer structure */
	if (Buffer == NULL) {
		return;
	}

	/* Reset values */
	Buffer->In = Buffer->Buffer;
	Buffer->Out = Buffer->Buffer;
}

int16_t measBuf_FindElement(measBuf_t* Buffer, sMeasRec * Element) {
	uint32_t Num, retval = 0;
	sMeasRec * Out;

	/* Check buffer structure */
	if (Buffer == NULL) {
		return -1;
	}

	/* Create temporary variables */
	Num = measBuf_GetFull(Buffer);
	Out = Buffer->Out;

	/* Go through input elements */
	while (Num > 0) {
		/* Check output overflow */
		if (Out >= (Buffer->Buffer + Buffer->Size) ){
			Out = Buffer->Buffer;
		}

		/* Check for element */
		if ( memcmp(Out, Element, sizeof(sMeasRec)) == 0 ) {
			/* Element found, return position in buffer */
			return retval;
		}

		/* Set new variables */
		Out++;
		Num--;
		retval++;
	}

	/* Element is not in buffer */
	return -1;
}

int8_t measBuf_CheckElement(measBuf_t* Buffer, uint16_t pos, uint8_t* element) {
	uint32_t i = 0;
	sMeasRec * In;
	sMeasRec * Out;

	/* Check value buffer */
	if (Buffer == NULL) {
		return 0;
	}

	/* Read current values */
	In = Buffer->In;
	Out = Buffer->Out;

	/* Set pointers to right location */
	while (i < pos && (In != Out)) {
		/* Increase output pointer */
		Out++;
		i++;

		/* Check overflow */
		if (Out >= (Buffer->Buffer + Buffer->Size) ){
			Out = Buffer->Buffer;
		}
	}

	/* If positions match */
	if (i == pos) {
		/* Save element */
		if( memcmp( element, Out, sizeof(sMeasRec) ) == 0 ){
		  /* Return OK */
		  return 1;
		}
	}

	/* Return zero */
	return 0;
}




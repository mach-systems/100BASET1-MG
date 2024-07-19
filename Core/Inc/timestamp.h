/*
 * timestamp.h
 *
 *  Created on: Mar 28, 2023
 *      Author: Karel Hevessy
 *
 * Module for timestamp generation. Note that timestamp structures belong to this
 * module and other modules can access only its indexes.
 */

#ifndef INC_TIMESTAMP_H_
#define INC_TIMESTAMP_H_

#include <stdint.h>

#define MESSAGE_TIMESTAMP_SIZE          8


typedef struct
{
    uint8_t Active : 1;         /* Set to 1 if timeout was initialized */
    uint32_t InitialValue;      /* Initial value of the counter */
    uint32_t OverflowCounter;   /* How many times the counter overflowed */
} TimerTimestamp;

/*
 * Initialize the timeout timer - must be called at the beginning of the
 * application.
 */
void TimestampTimerInit(void);

/*
 * Get timestamp index.
 */
uint8_t TimestampInit(void);

/*
 * Initialize the timestamp structure.
 */
void TimestampStart(uint8_t index);

/*
 * Initialize all the initial values to the same value.
 */
void TimestampAlignAll(void);

/*
 * De-initialize the timestamp structure (meaning that subsequent reads of the
 * value will return 0).
 */
void TimestampDeinit(uint8_t index);

/*
 * Get current microsecond timestamp.
 */
uint64_t GetTimestamp(uint8_t index);

/*
 * Remember how many times the timestamp timer overflowed.
 */
void TimestampTimerOverflow(void);

/*
 * Transfer 64-bit timestamp to data array of length 8 (Intel / LSB first order).
 */
void FillTimestampToData(uint8_t* pData, uint64_t* pTimestamp);

/*
 * Fill 64-but timestamp to a supplied buffer (Intel / LSB first order).
 * If pDatalength is not NULL, increase it accordingly.
 */
void FillTimestamp(uint8_t* pBuf, uint8_t* pDatalength, uint64_t* pTimestamp);



#endif /* INC_TIMESTAMP_H_ */

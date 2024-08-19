/*
 * timestamp.c
 *
 *  Created on: Mar 28, 2023
 *      Author: Karel Hevessy
 */

#include "timestamp.h"
#include "main.h"


#define TIMESTAMP_COUNTER               TIM24
#define TIMESTAMP_COUNTER_MAX           4294967295U
#define TIMESTAMP_ARRAY_CNT             10

TimerTimestamp timestampArray[TIMESTAMP_ARRAY_CNT];
static uint8_t timestampsUsed = 0;

/*
 * Get current counter value from the timestamp timer.
 */
static uint32_t getTimestampTimerCurrentValue(void);

void TimestampTimerInit(void)
{
    LL_TIM_EnableCounter(TIMESTAMP_COUNTER);
    LL_TIM_EnableIT_UPDATE(TIMESTAMP_COUNTER);
}

uint8_t TimestampInit(void)
{
    uint8_t ret = UINT8_MAX;
    if (timestampsUsed < TIMESTAMP_ARRAY_CNT)
    {
        ret = timestampsUsed;
        timestampArray[timestampsUsed].InitialValue = timestampArray[timestampsUsed].OverflowCounter = 0;
        timestampsUsed++;
    }
    return ret;
}

void TimestampStart(uint8_t index)
{
    if (index < TIMESTAMP_ARRAY_CNT)
    {
        timestampArray[index].Active = 1;
        timestampArray[index].InitialValue = getTimestampTimerCurrentValue();
        timestampArray[index].OverflowCounter = 0;
    }
}

void TimestampAlignAll(void)
{
    uint32_t value = getTimestampTimerCurrentValue();
    for (uint8_t i = 0; i < TIMESTAMP_ARRAY_CNT; i++)
        if (timestampArray[i].Active)
            timestampArray[i].InitialValue = value;
}

void TimestampDeinit(uint8_t index)
{
    if (index < TIMESTAMP_ARRAY_CNT)
    {
        timestampArray[index].Active = timestampArray[index].InitialValue
        = timestampArray[index].OverflowCounter = 0;
    }
}

uint64_t GetTimestamp(uint8_t index)
{
    uint64_t timestamp = 0;
    if (index < TIMESTAMP_ARRAY_CNT && timestampArray[index].Active)
    {
        uint32_t value = getTimestampTimerCurrentValue();

        // Values are 32bit, maximum is 2^32-1 => always correct
        uint32_t tmp = value - timestampArray[index].InitialValue;
        timestamp = tmp;
        uint32_t cnt = timestampArray[index].OverflowCounter;
        if (value < timestampArray[index].InitialValue)
        {
            if (cnt > 0)
            {
                --cnt;
            }
        }
        timestamp += cnt * ((uint64_t) TIMESTAMP_COUNTER_MAX + 1);

    }
    return timestamp;
}

void TimestampTimerOverflow(void)
{
    for (uint8_t i = 0; i < TIMESTAMP_ARRAY_CNT; i++)
        if (timestampArray[i].Active)
            timestampArray[i].OverflowCounter++;
}

uint32_t getTimestampTimerCurrentValue(void)
{
    return LL_TIM_GetCounter(TIMESTAMP_COUNTER);
}

void FillTimestamp(uint8_t* pBuf, uint8_t* pDatalength, uint64_t* pTimestamp)
{
    if (NULL != pBuf && NULL != pTimestamp)
    {
        pBuf[0] = *pTimestamp & 0xff;
        pBuf[1] = ((*pTimestamp >> 8) & 0xff);
        pBuf[2] = ((*pTimestamp >> 16) & 0xff);
        pBuf[3] = ((*pTimestamp >> 24) & 0xff);
        pBuf[4] = ((*pTimestamp >> 32) & 0xff);
        pBuf[5] = ((*pTimestamp >> 40) & 0xff);
        pBuf[6] = ((*pTimestamp >> 48) & 0xff);
        pBuf[7] = ((*pTimestamp >> 56) & 0xff);
        if (NULL != pDatalength)
            *pDatalength += 8;
    }
}

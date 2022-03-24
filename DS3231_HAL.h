/*
 * Copyright (c) 2022 Federico Zuccardi Merli.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DS3231_HAL_H_
#define DS3231_HAL_H_

#include <stdbool.h>

#include "pico/time.h"

typedef enum RtcIrqRate_
{
    RtcSqw1Hz    = 0x00u, /* These are the RS1 and RS2 values, square wawe */
    RtcSqw1024Hz = 0x08u, /* These are the RS1 and RS2 values, square wawe */
    RtcSqw4096Hz = 0x10u, /* These are the RS1 and RS2 values, square wawe */
    RtcSqw8192Hz = 0x18u, /* These are the RS1 and RS2 values, square wawe */
    RtcAla1s     = 0x80u, /* Uses Alarm 1, output low until cleared by SW */
    RtcAla60s    = 0x81u, /* Uses Alarm 2, output low until cleared by SW */
    RtcIntSqwOff = 0x0Fu, /* INT/SQW output is disabled */
} RtcIntSqwRate;

/**
 * Initialize RTC
 * - The oscillator is started, if needed.
 * - Alarms are disabled, unless irqRate is one of RtcAla<n>s values.
 *   In that case, it's up to the user to clear the interrupt condition in
 *   the Status register using ClearRtcAlarm.
 * - 32 kHz out is disabled
 * - Int/Sqw output is disabled when on battery
 * - 24 hours mode is set.
 * - if dateTime is NULL current RTC time is kept, if not, the time is set only
 *   if the RTC oscillar was stopped any time before the call.
 *   Use SetRtcTime() to force.
 * - i2cInst is the pico i2c peripheral instance pointer, i2c0 or i2c1
 * - Return value is true if RTC running, false if oscillar was stopped */
extern bool InitRtc(
    datetime_t   *d,
    RtcIntSqwRate irqRate,
    void         *i2cInst);

/* Reads date and time, returns success (true) or failure (false) */
extern bool GetRtcTime(datetime_t *d);

/* Sets date and time, returns success (true) or failure (false) */
extern bool SetRtcTime(datetime_t *d);

/* Sets a new mode for the SQW/INTIRQ rate, see description in InitRtc */
extern bool SetIntSqwRate(RtcIntSqwRate irqRate);

/**
 *  Reads temperature sensor, LSb = 0,25°
 *  if conv is true starts and waits a new conversion,
 *  the calling task will be suspended until the conversion ends
 *  or a 250 ms timeout expires */
extern int32_t GetRtcTemp(bool conv);

/* Enables or disables the 32 kHz output, returns success (true) or failure (false) */
extern bool EnableRtc32kHz(bool enable);

/* Clears the alarm interrupt flags */
extern bool ClearRtcAlarm(void);

#endif

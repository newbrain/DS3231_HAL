/*
 * Copyright (c) 2022 Federico Zuccardi Merli.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "DS3231_HAL.h"
#include "DS3231_def.h"
#include "memI2C.h"

/* Static buffer to hold registers */
static DS3231 *ds3231 = &(DS3231){};

/* I2C instance, defaults to i2c0 */
i2c_inst_t *i2c = i2c0;

/* The current IRQ rate */
RtcIntSqwRate curRate = RtcIntSqwOff;
/* 32768 Hz clock status */
bool rtc32kHzEnabled = false;

/* Curried multi-byte Write and Read  for DS3231 */
static inline bool DS3231Read(uint8_t addr, uint8_t *data, uint8_t bytes)
{
    return 0 < i2c_read_mem_blocking(i2c, DS3231_I2C_ADDRESS, addr, 1, data, bytes);
}

static inline bool DS3231Write(uint8_t addr, uint8_t *data, uint8_t bytes)
{
    return 0 < i2c_write_mem_blocking(i2c, DS3231_I2C_ADDRESS, addr, 1, data, bytes);
}

/* Double Curried Write and Read to handle a single byte for DS3231 */
static inline bool DS3231Write1(uint8_t addr, uint8_t data)
{
    return DS3231Write(addr, &data, 1);
}

static inline bool DS3231Read1(uint8_t addr, uint8_t *data)
{
    return DS3231Read(addr, data, 1);
}

/* BCD to binary, with mask */
static inline uint8_t BCDtoBin(uint8_t bits, uint8_t mask)
{
    bits &= mask;
    return (bits & 0x0fu) + 10 * (bits >> 4);
}

/* binary to BCD */
static inline uint8_t BinToBCD(uint8_t bits)
{
    return ((bits / 10) << 4) | (bits % 10);
}

/**
 * Initialize RTC
 * - The oscillator is started, if needed.
 * - Alarms are disabled, unless intSqwRate is one of RtcAla<n>s values.
 *   In that case, it's up to the user to clear the interrupt condition in
 *   the Status register using ClearAlarmIrq.
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
    RtcIntSqwRate intSqwRate,
    void         *i2cInst)
{
    /* Make sure the handle is stored, if not NULL, otherwise use i2c0 (or last used) */
    if (i2cInst) i2c = i2cInst;
    /* Make sure alarms are disabled, read oscillator stopped flag */
    if (!DS3231Read(DS3231_Control_Status, ds3231->Control_Status, CNTRLSTS_SIZE))
        return false;
    /* Set time only if needed (oscillator was stopped) */
    if (d && ds3231->Status & DS3231_OSF) SetRtcTime(d);
    /* Start oscillator, set alarm interrupt, disable output when on battery, disable Alarms */
    ds3231->Control = DS3231_INTCN;
    /* Clear oscillator status, disable 32 kHz output and clear Alarm flags */
    ds3231->Status = 0x00u;
    if (!DS3231Write(DS3231_Control_Status, ds3231->Control_Status, CNTRLSTS_SIZE))
        return false;

    /* Set rate and callback, enable interrupt if needed */
    return SetIntSqwRate(intSqwRate);
}

/* Reads date and time, returns success (true) or failure (false) */
bool GetRtcTime(datetime_t *d)
{
    /* Read regs */
    if (!DS3231Read(DS3231_TimeDate, ds3231->TimeDate, TIMEDATE_SIZE)) return false;
    /* Convert to binary and store in *d */
    d->sec   = BCDtoBin(ds3231->Seconds, SECONDS_MASK);
    d->min   = BCDtoBin(ds3231->Minutes, MINUTES_MASK);
    d->hour  = BCDtoBin(ds3231->Hours, HOURS_MASK);
    d->day   = BCDtoBin(ds3231->Date, DATE_MASK);
    d->dotw  = BCDtoBin(ds3231->Day, DAY_MASK) % 7; /* Stupid SDK/RTC... */
    d->month = BCDtoBin(ds3231->Month_Century, MONTH_MASK);
    d->year  = BCDtoBin(ds3231->Year, YEAR_MASK);
    d->year += 2000 + 100 * ((ds3231->Month_Century & DS3231_CENTURY) != 0);
    return true;
}

/* Sets date and time, returns success (true) or failure (false) */
bool SetRtcTime(datetime_t *d)
{
    /* Convert *d to BCD and store in regs */
    ds3231->Seconds       = BinToBCD(d->sec);
    ds3231->Minutes       = BinToBCD(d->min);
    ds3231->Hours         = BinToBCD(d->hour);
    ds3231->Date          = BinToBCD(d->day);
    uint8_t dw            = BinToBCD(d->dotw);
    ds3231->Day           = dw ? dw : 7; /* Stupid SDK/RTC... */
    ds3231->Month_Century = BinToBCD(d->month);
    ds3231->Year          = BinToBCD(d->year - 2000);
    if (d->year >= 2100) ds3231->Year |= DS3231_CENTURY;

    /* Write regs */
    return DS3231Write(DS3231_TimeDate, ds3231->TimeDate, TIMEDATE_SIZE);
}

/* Sets a new INT/SQW rate, see description in InitRtc */
bool SetIntSqwRate(RtcIntSqwRate intSqwRate)
{
    /* Prepare Status to clear alarm flags */
    ds3231->Status = 0x00u;

    switch (intSqwRate)
    {
    case RtcIntSqwOff:
        /* Disable alarms and set INTCN */
        if (!DS3231Write1(DS3231_Control, DS3231_INTCN)) return false;
        break;

    case RtcAla1s:
        /* Set up Alarm 1 for periodic alarms every second */
        ds3231->Alarm1_Seconds = DS3231_ALRMRPT;
        ds3231->Alarm1_Minutes = DS3231_ALRMRPT;
        ds3231->Alarm1_Hours   = DS3231_ALRMRPT;
        ds3231->Alarm1_DayDate = DS3231_ALRMRPT + 1; /* Use a reasonable value for the date */
        if (!DS3231Write(DS3231_Alarm1, ds3231->Alarm1, ALARM1_SIZE)) return false;
        /* Enable Alarm1 in control register */
        ds3231->Control = DS3231_INTCN | DS3231_A1IE;
        break;

    case RtcAla60s:
        /* Set up Alarm 2 for periodic alarms every minute */
        ds3231->Alarm2_Minutes = DS3231_ALRMRPT;
        ds3231->Alarm2_Hours   = DS3231_ALRMRPT;
        ds3231->Alarm2_DayDate = DS3231_ALRMRPT + 1; /* Use a reasonable value for the date */
        if (!DS3231Write(DS3231_Alarm2, ds3231->Alarm2, ALARM2_SIZE)) return false;
        /* Enable Alarm2 in control register */
        ds3231->Control = DS3231_INTCN | DS3231_A2IE;
        break;

    case RtcSqw1Hz:
    case RtcSqw1024Hz:
    case RtcSqw4096Hz:
    case RtcSqw8192Hz:
        /* In this case it's enough to write RS1/2, clear INTCN, A1IE, A2IE */
        ds3231->Control = intSqwRate;
        break;

    default:
        /* Unknown option, error, just return */
        return false;
    }

    /* Write new configuration and clear alarm flags */
    if (!DS3231Write(DS3231_Control_Status, ds3231->Control_Status, CNTRLSTS_SIZE))
        return false;

    /* Remember the current IRQ rate, to clear the alarm flags if needed */
    curRate = intSqwRate;

    return true;
}

/**
 *  Reads temperature sensor, LSb = 0,25°
 *  if conv is true starts and waits a new conversion,
 *  the calling task will be suspended until the conversion ends
 *  or a 250 ms timeout expires */
int32_t GetRtcTemp(bool conv)
{
    if (conv)
    {
        /* Check if conversion already busy and caches Control reg */
        DS3231Read(DS3231_Control_Status, ds3231->Control_Status, CNTRLSTS_SIZE);
        /* If not, start a new conversion */
        if (!(ds3231->Control & DS3231_CONV || ds3231->Status & DS3231_BSY))
            DS3231Write1(DS3231_Control, ds3231->Control & DS3231_CONV);
        /* check every 50 ms if done, using BSY and CONV, for at most 250 ms */
        for (uint32_t i = 0; i < 5; i++)
        {
            sleep_ms(50);
            DS3231Read(DS3231_Control_Status, ds3231->Control_Status, CNTRLSTS_SIZE);
            if (!(ds3231->Control & DS3231_CONV || ds3231->Status & DS3231_BSY)) break;
        }
    }
    /* Read temp registers */
    DS3231Read(DS3231_Temp, ds3231->Temp, TEMP_SIZE);
    /* Transform into little endian */
    uint16_t t = (ds3231->Temp_MSB << 8) + (ds3231->Temp_LSB);
    /* Return scaled value with sign */
    return (int16_t)t / 64;
}

/* Enables or disables the 32 kHz output, returns success (true) or failure (false) */
bool EnableRtc32kHz(bool enable)
{
    /* Write Control and Status register EN32kHz bit */
    if (!DS3231Read1(DS3231_Status, &ds3231->Status)) return false;
    if (enable)
        ds3231->Status |= DS3231_EN32kHz;
    else
        ds3231->Status &= ~DS3231_EN32kHz;
    if (!DS3231Write1(DS3231_Status, ds3231->Status)) return false;
    rtc32kHzEnabled = enable;
    return true;
}

/* Clears the alarm interrupt flags */
bool ClearRtcAlarm(void)
{
    uint8_t sts = rtc32kHzEnabled ? DS3231_EN32kHz : 0x00u;
    return DS3231Write1(DS3231_Status, sts);
}

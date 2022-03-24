/**************************************************************************/
/*!
    @file     DS3231_def.h
    @author   F. Zuccardi Merli

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2020, Federico Zuccardi Merli
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef DS3231_DEF_H_
#define DS3231_DEF_H_

#include <assert.h>
#include <stddef.h>

/* Reading/Writing size constants */
#define TIMEDATE_SIZE 7
#define ALARM1_SIZE   4
#define ALARM2_SIZE   3
#define CNTRLSTS_SIZE 2
#define TEMP_SIZE     2

/* Default DS3231 I2C address */
#define DS3231_I2C_ADDRESS (0x68U)

/* Use C11 anonymous unions and structures */
typedef struct DS3231_
{
    union
    {
        struct
        {
            uint8_t Seconds;       /* BCD 3+4 bits*/
            uint8_t Minutes;       /* BCD 3+4 bits*/
            uint8_t Hours;         /* BCD 2+4 or 1+4 bits*/
            uint8_t Day;           /* BCD 3 bits */
            uint8_t Date;          /* BCD 2+4 bits */
            uint8_t Month_Century; /* Month: BCD 1+4 bits */
            uint8_t Year;          /* BCD 4+4 bits */
        };
        uint8_t TimeDate[TIMEDATE_SIZE];
    };
    union
    {
        struct
        {
            uint8_t Alarm1_Seconds; /* BCD 3+4 bits*/
            uint8_t Alarm1_Minutes; /* BCD 3+4 bits*/
            uint8_t Alarm1_Hours;   /* BCD 2+4 or 1+4 bits*/
            uint8_t Alarm1_DayDate; /* BCD 2+4 or 3 bits */
        };
        uint8_t Alarm1[ALARM1_SIZE];
    };
    union
    {
        struct
        {
            uint8_t Alarm2_Minutes; /* BCD 3+4 bits*/
            uint8_t Alarm2_Hours;   /* BCD 2+4 or 1+4 bits*/
            uint8_t Alarm2_DayDate; /* BCD 2+4 or 3 bits */
        };
        uint8_t Alarm2[ALARM2_SIZE];
    };
    union
    {
        struct
        {
            uint8_t Control;
            uint8_t Status;
        };
        uint8_t Control_Status[CNTRLSTS_SIZE];
    };
    uint8_t Aging;
    union
    {
        struct
        {
            uint8_t Temp_MSB;
            uint8_t Temp_LSB;
        };
        uint8_t Temp[TEMP_SIZE];
    };

} DS3231;

typedef enum DS3231Regs_
{
    DS3231_TimeDate       = offsetof(DS3231, TimeDate),
    DS3231_Seconds        = offsetof(DS3231, Seconds),
    DS3231_Minutes        = offsetof(DS3231, Minutes),
    DS3231_Hours          = offsetof(DS3231, Hours),
    DS3231_Day            = offsetof(DS3231, Day),
    DS3231_Date           = offsetof(DS3231, Date),
    DS3231_Month_Century  = offsetof(DS3231, Month_Century),
    DS3231_Year           = offsetof(DS3231, Year),
    DS3231_Alarm1         = offsetof(DS3231, Alarm1),
    DS3231_Alarm1_Seconds = offsetof(DS3231, Alarm1_Seconds),
    DS3231_Alarm1_Minutes = offsetof(DS3231, Alarm1_Minutes),
    DS3231_Alarm1_Hours   = offsetof(DS3231, Alarm1_Hours),
    DS3231_Alarm1_DayDate = offsetof(DS3231, Alarm1_DayDate),
    DS3231_Alarm2         = offsetof(DS3231, Alarm2),
    DS3231_Alarm2_Minutes = offsetof(DS3231, Alarm2_Minutes),
    DS3231_Alarm2_Hours   = offsetof(DS3231, Alarm2_Hours),
    DS3231_Alarm2_DayDate = offsetof(DS3231, Alarm2_DayDate),
    DS3231_Control_Status = offsetof(DS3231, Control_Status),
    DS3231_Control        = offsetof(DS3231, Control),
    DS3231_Status         = offsetof(DS3231, Status),
    DS3231_Aging          = offsetof(DS3231, Aging),
    DS3231_Temp           = offsetof(DS3231, Temp),
    DS3231_Temp_MSB       = offsetof(DS3231, Temp_MSB),
    DS3231_Temp_LSB       = offsetof(DS3231, Temp_LSB),
} DS3231Regs;

/* BCD masks */
#define SECONDS_MASK 0x7Fu
#define MINUTES_MASK 0x7Fu
#define HOURS_MASK   0x3Fu
#define DAY_MASK     0x07u
#define DATE_MASK    0x3Fu
#define MONTH_MASK   0x1Fu
#define YEAR_MASK    0xFFu

/* Time keeping and alarm registers flags */
#define DS3231_HOURS12 0x40u
#define DS3231_HOURSPM 0x20u
#define DS3231_HOURS24 0x00u
#define DS3231_CENTURY 0x80u
#define DS3231_ALRMRPT 0x80u
#define DS3231_DAYDATE 0x40u

/* Control register (0x0E) flags */
#define DS3231_A1IE  0x01u
#define DS3231_A2IE  0x02u
#define DS3231_INTCN 0x04u
#define DS3231_RS1   0x08u
#define DS3231_RS2   0x10u
#define DS3231_CONV  0x20u
#define DS3231_BBSQW 0x40u
#define DS3231_NEOSC 0x80u

/* Control and status register (0x0F) flags */
#define DS3231_A1F     0x01u
#define DS3231_A2F     0x02u
#define DS3231_BSY     0x04u
#define DS3231_EN32kHz 0x08u
#define DS3231_OSF     0x80u

#endif
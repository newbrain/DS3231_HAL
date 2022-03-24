# DS3231_HAL
A simple abstraction layer for DS3231 RTC based on pico-sdk and MemI2C.

Provided functions:
* InitRtc()
* GetRtcTime()
* SetRtcTime()
* SetIntSqwRate()
* GetRtcTemp()
* EnableRtc32kHz()
* ClearRtcAlarm()
TODO:
* SetRtcAlarm()

See `DS3231_HAL.h`for the description of parameters and use.

To use this in your pico project:
* copy this directory (DS3231_HAL) as a subdirectory of the base project directory or add it as a git submodule
* add the following line to the main CMakeLists.txt, after the `add_executable(...)` statement:<br>
`add_subdirectory( DS3231_HAL )`
* [MemI2C](https://github.com/newbrain/MemI2C) also needs to be added your project.
* At least version 3.13 of CMake is needed, make sure the main CMakeLists.txt begins with a<br>
`cmake_minimum_required(VERSION 3.13)` (or later) line to correctly set policies.
* use `#include "DS3231_HAL.h"` as needed.

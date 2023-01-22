# MT XSens Xbus MTData2 Data Stream Decoder

Date: 2022-12-29

## Introduction

- Not official library
- This is an Arduino library for decoding the `MT Xsens MTi-670` sensor's binary data stream encoded by the protocol `MTData2`.
- Available measurements: (the ID of the following measurements has been added in this library)
  - quaternion (float/double)
  - Euler angle (float/double)
  - rate of turn (float/double)
  - acceleration (float/double)
  - magnetometer (float/double)
  - baro pressure (float)
  - elliptic altitude (float/double)
  - latitude, longitude (float/double)
  - pvtData (float/double)
- The boards tested and can be run:
  - Teensy 3.2/4.0


## Name Space

This library uses the name space `xsens`.

## How to use

```cpp
// 2023-01-16
// Example: library utility
// This simple example illustrates using this library to conduct data stream polling and decoded measurement data accessing.
// Yang-Rui Li

#include <Arduino.h>

#include <xbus.h> 
// Include the xbus data decoding core library

#include <xbus_utility.hpp> 
// It can be included or not for users' demands.
// For different usages, the macros in "xbus_utility.hpp":
//      #define XBUS_DATA_CONFIG_GNSS_PVT 0     // not used
//      #define XBUS_DATA_CONFIG_INS 0          // not used
//      #define XBUS_DATA_CONFIG_ORIENTATION 1  // used
//      #define XBUS_DATA_CONFIG_IMU 1          // used
//      #define XBUS_DATA_CONFIG_BARO 1         // used
// can be set to 1 or 0 to use or not use the corresponding measurement.


#define HW_SERIAL_INTERFACE_XBUS Serial1 // Suppose that the headward serial `Serial1` is used
#define HW_SERIAL_BAUD_RATE_XBUS 2000000 // Suppose that the baud rate is set as 2000000

xsens::Xbus xbus;
xsens::xbus_motion_data_double data;

void setup()
{
    // Begin USB serial
    Serial.begin(115200);

    // Begin Xbus data stream
    xbus.begin(HW_SERIAL_INTERFACE_XBUS, HW_SERIAL_BAUD_RATE_XBUS);
}

void loop()
{
    /* Polling Xbus serial data */
    if (xbus.read() == Xbus::XD_OK)
    {
        /* Do something when the data packets has been decoded. */
        
        // Get all data
        xsens::xbus_get_all_data(xbus, data);

        // Print/Access data routine
        xsens::print_quaternion(data);
        xsens::print_imu(data);
        xsens::print_ins(data);
        xsens::print_gnss(data);
    }
}
```

/**
 * @file xbus_consts.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-01-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <Arduino.h>

namespace xsens
{
    enum XBUS_HEADER : uint8_t
    {
        XBUS_HEADER_PREAMBLE = 0xFA,
        XBUS_HEADER_BID = 0xFF,
        XBUS_HEADER_MID = 0x36
    };

    enum XBUS_DATA_ID : uint16_t
    {
        XBUS_DATA_ID_QUATERNION_FLOAT_ENU = 0x2010,
        XBUS_DATA_ID_QUATERNION_DOUBLE_ENU = 0x2013,
        XBUS_DATA_ID_EULER_FLOAT_ENU = 0x2030,
        XBUS_DATA_ID_EULER_DOUBLE_ENU = 0x2033,
        XBUS_DATA_ID_ACCEL_FLOAT_ENU = 0x4020,
        XBUS_DATA_ID_ACCEL_DOUBLE_ENU = 0x4023,
        XBUS_DATA_ID_GYRO_FLOAT_ENU = 0x8020,
        XBUS_DATA_ID_GYRO_DOUBLE_ENU = 0x8023,
        XBUS_DATA_ID_LATLON_FLOAT_ENU = 0x5040,
        XBUS_DATA_ID_LATLON_DOUBLE_ENU = 0x5043,
        XBUS_DATA_ID_ALTITUDE_FLOAT_ENU = 0x5020,
        XBUS_DATA_ID_ALTITUDE_DOUBLE_ENU = 0x5023,
        XBUS_DATA_ID_VELOCITY_FLOAT_ENU = 0xD010,
        XBUS_DATA_ID_VELOCITY_DOUBLE_ENU = 0xD013,
        XBUS_DATA_ID_MAG_FLOAT_ENU = 0xC020,
        XBUS_DATA_ID_MAG_DOUBLE_ENU = 0xC023,
        XBUS_DATA_ID_GNSS_PVTDATA_ENU = 0x7010
    };

    enum XBUS_EVENT : int
    {
        XBUS_EVT_WAIT_PREAMBLE,
        XBUS_EVT_WAIT_PACKETS
    };
}
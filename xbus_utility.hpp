/**
 * @file xbus_utilize.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <Arduino.h>
#include <xbus.h>
#include <attitude_kinematics.hpp>

#define XBUS_DATA_CONFIG_GNSS_PVT 0
#define XBUS_DATA_CONFIG_INS 0
#define XBUS_DATA_CONFIG_ORIENTATION 1
#define XBUS_DATA_CONFIG_IMU 1

template <class T>
struct xbus_motion_data
{
    xbus_motion_data() { memset(this, 0, sizeof(xbus_motion_data)); }
#if XBUS_DATA_CONFIG_GNSS_PVT
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    int32_t lon;
    int32_t lat;
    int32_t height;
#endif

#if XBUS_DATA_CONFIG_ORIENTATION
    T quat[4];
    T euler[3];
#endif

#if XBUS_DATA_CONFIG_IMU
    T gyro[3];
    T accel[3];
    T mag[3];
#endif

#if XBUS_DATA_CONFIG_INS
    T latlon[2];
    T altitude;
    T velocity[3];
#endif
};
typedef struct xbus_motion_data<float> xbus_motion_data_float;
typedef struct xbus_motion_data<double> xbus_motion_data_double;

void xbus_get_all_data(xsens::Xbus &xbus, xbus_motion_data_double &data)
{
#if XBUS_DATA_CONFIG_ORIENTATION
    memcpy(data.quat, xbus.quat.f64, sizeof(data.quat));
    Attitude::quaternion_to_euler(data.quat, data.euler);
#endif

#if XBUS_DATA_CONFIG_IMU
    memcpy(data.gyro, xbus.gyro.f64, sizeof(data.gyro));
    memcpy(data.accel, xbus.accel.f64, sizeof(data.accel));
    memcpy(data.mag, xbus.mag.f64, sizeof(data.mag));
#endif

#if XBUS_DATA_CONFIG_INS
    memcpy(data.latlon, xbus.latlon.f64, sizeof(data.latlon));
    memcpy(&data.altitude, &xbus.altitude.f64, sizeof(data.altitude));
    memcpy(data.velocity, &xbus.velocity.f64, sizeof(data.velocity));
#endif

#if XBUS_DATA_CONFIG_GNSS_PVT
    data.year = xbus.get_gnss().get_pvt().year;
    data.month = xbus.get_gnss().get_pvt().month;
    data.day = xbus.get_gnss().get_pvt().day;
    data.hour = xbus.get_gnss().get_pvt().hour;
    data.min = xbus.get_gnss().get_pvt().min;
    data.sec = xbus.get_gnss().get_pvt().sec;
    data.lat = xbus.get_gnss().get_pvt().lat;
    data.lon = xbus.get_gnss().get_pvt().lon;
    data.height = xbus.get_gnss().get_pvt().height;
#endif
}
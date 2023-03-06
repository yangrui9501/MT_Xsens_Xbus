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



#define XBUS_DATA_CONFIG_QUATERNION 1
#define XBUS_DATA_CONFIG_RATE_OF_TURN 0
#define XBUS_DATA_CONFIG_ACCEL 0
#define XBUS_DATA_CONFIG_FREE_ACCEL 1
#define XBUS_DATA_CONFIG_MAGNETIC_FIELD 1
#define XBUS_DATA_CONFIG_TEMPERATURE 1
#define XBUS_DATA_CONFIG_BARO 1
#define XBUS_DATA_CONFIG_ACCEL_HR 1
#define XBUS_DATA_CONFIG_RATE_OF_TURN_HR 1
#define XBUS_DATA_CONFIG_POSITION_AND_VELOCITY 1
#define XBUS_DATA_CONFIG_GNSS_DATA 1


namespace xsens
{
template <class T>
struct xbus_motion_data
{
    xbus_motion_data() { memset(this, 0, sizeof(xbus_motion_data)); }

#if XBUS_DATA_CONFIG_QUATERNION
    T quat[4];
    T euler[3];
#endif

#if XBUS_DATA_CONFIG_RATE_OF_TURN
    T gyro[3];
#endif
#if XBUS_DATA_CONFIG_ACCEL
    T accel[3];
#endif

#if XBUS_DATA_CONFIG_MAGNETIC_FIELD
    T mag[3];
#endif

#if XBUS_DATA_CONFIG_TEMPERATURE
    T temperature;
#endif

#if XBUS_DATA_CONFIG_BARO
    uint32_t baro;
#endif

#if XBUS_DATA_CONFIG_ACCEL_HR
    float accel_hr[3];
#endif

#if XBUS_DATA_CONFIG_RATE_OF_TURN_HR
    float gyro_hr[3];
#endif

#if XBUS_DATA_CONFIG_FREE_ACCEL
    T free_accel[3];
#endif

#if XBUS_DATA_CONFIG_POSITION_AND_VELOCITY
    T latlon[2];
    T altitude;
    T velocity[3];
#endif

#if XBUS_DATA_CONFIG_GNSS_DATA
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
};
typedef struct xbus_motion_data<float> xbus_motion_data_float;
typedef struct xbus_motion_data<double> xbus_motion_data_double;

void quat_to_euler(double(&quat)[4], double(&euler321)[3])
{
    double& q0 = quat[0];
    double& q1 = quat[1];
    double& q2 = quat[2];
    double& q3 = quat[3];

    double q0q1 = q0 * q1;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q1q1 = q1 * q1;
    double q0q2 = q0 * q2;
    double q1q3 = q1 * q3;
    double q0q3 = q0 * q3;
    double q1q2 = q1 * q2;
    double q3q3 = q3 * q3;

    euler321[0] = atan2(2.0 * (q0q1 + q2q3), 1.0 - 2.0 * (q1q1 + q2q2));
    euler321[1] = asin(2.0 * (q0q2 - q1q3));
    euler321[2] = atan2(2.0 * (q0q3 + q1q2), 1.0 - 2.0 * (q2q2 + q3q3));
}

void euler_to_quat(double(&euler321)[3], double(&quat)[4])
{
    double& phi = euler321[0];
    double& theta = euler321[1];
    double& psi = euler321[2];

    double cphi = cos(phi / 2.0);
    double ctheta = cos(theta / 2.0);
    double cpsi = cos(psi / 2.0);
    double sphi = sin(phi / 2.0);
    double stheta = sin(theta / 2.0);
    double spsi = sin(psi / 2.0);

    quat[0] = cphi * ctheta * cpsi + sphi * stheta * spsi;
    quat[1] = sphi * ctheta * cpsi - cphi * stheta * spsi;
    quat[2] = cphi * stheta * cpsi + sphi * ctheta * spsi;
    quat[3] = cphi * ctheta * spsi - sphi * stheta * cpsi;
}

void xbus_get_all_data(xsens::Xbus& xbus, xsens::xbus_motion_data_double& data)
{

#if XBUS_DATA_CONFIG_QUATERNION
    memcpy(data.quat, xbus.quat.f64, sizeof(data.quat));
    quat_to_euler(data.quat, data.euler);
#endif

#if XBUS_DATA_CONFIG_RATE_OF_TURN
    memcpy(data.gyro, xbus.gyro.f64, sizeof(data.gyro));
#endif
#if XBUS_DATA_CONFIG_ACCEL
    memcpy(data.accel, xbus.accel.f64, sizeof(data.accel));
#endif
#if XBUS_DATA_CONFIG_FREE_ACCEL
    memcpy(data.free_accel, xbus.free_accel.f64, sizeof(data.free_accel));
#endif
#if XBUS_DATA_CONFIG_MAGNETIC_FIELD
    memcpy(data.mag, xbus.mag.f64, sizeof(data.mag));
#endif

#if XBUS_DATA_CONFIG_TEMPERATURE
    data.temperature = xbus.temperature.f64;
#endif

#if XBUS_DATA_CONFIG_BARO
    data.baro = xbus.baro;
#endif

#if XBUS_DATA_CONFIG_ACCEL_HR
    memcpy(data.accel_hr, xbus.accel_hr, sizeof(data.accel_hr));
#endif

#if XBUS_DATA_CONFIG_RATE_OF_TURN_HR
    memcpy(data.gyro_hr, xbus.gyro_hr, sizeof(data.gyro_hr));
#endif

#if XBUS_DATA_CONFIG_POSITION_AND_VELOCITY
    memcpy(data.latlon, xbus.latlon.f64, sizeof(data.latlon));
    memcpy(&data.altitude, &xbus.altitude.f64, sizeof(data.altitude));
    memcpy(data.velocity, &xbus.velocity.f64, sizeof(data.velocity));
#endif

#if XBUS_DATA_CONFIG_GNSS_DATA
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

void print_quaternion(xsens::xbus_motion_data_double& data)
{
    Serial.print("Quaternion (w,x,y,z): (");
    Serial.print(data.quat[0]);
    Serial.print(", ");
    Serial.print(data.quat[1]);
    Serial.print(", ");
    Serial.print(data.quat[2]);
    Serial.print(", ");
    Serial.print(data.quat[3]);
    Serial.println(")");
}

#if BUS_DATA_CONFIG_RATE_OF_TURN
void print_rate_of_turn(xsens::xbus_motion_data_double& data)
{
    Serial.print("Rate of Turn (rad/s): (");
    Serial.print(data.gyro[0]);
    Serial.print(", ");
    Serial.print(data.gyro[1]);
    Serial.print(", ");
    Serial.print(data.gyro[2]);
    Serial.print(")");
}
#endif 

#if XBUS_DATA_CONFIG_ACCEL
void print_accel(xsens::xbus_motion_data_double& data)
{
    Serial.print("Accel (m/ss): (");
    Serial.print(data.accel[0]);
    Serial.print(", ");
    Serial.print(data.accel[1]);
    Serial.print(", ");
    Serial.print(data.accel[2]);
    Serial.println(")");
}
#endif

#if XBUS_DATA_CONFIG_FREE_ACCEL
void print_free_accel(xsens::xbus_motion_data_double& data)
{
    Serial.print("Free Accel (m/ss): (");
    Serial.print(data.free_accel[0]);
    Serial.print(", ");
    Serial.print(data.free_accel[1]);
    Serial.print(", ");
    Serial.print(data.free_accel[2]);
    Serial.println(")");
}
#endif

#if XBUS_DATA_CONFIG_MAGNETIC_FIELD
void print_magnetic_field(xsens::xbus_motion_data_double& data)
{
    Serial.print("Mag: (");
    Serial.print(data.mag[0]);
    Serial.print(", ");
    Serial.print(data.mag[1]);
    Serial.print(", ");
    Serial.print(data.mag[2]);
    Serial.println(")");
}
#endif

#if XBUS_DATA_CONFIG_TEMPERATURE 
void print_temperature(xsens::xbus_motion_data_double& data)
{
    Serial.print("Temperature (C): ");
    Serial.print(data.temperature);
    Serial.println();
}
#endif 

#if XBUS_DATA_CONFIG_BARO
void print_baro(xsens::xbus_motion_data_double& data)
{
    Serial.print("Barometric Pressure (Pa): ");
    Serial.print(data.baro);
    Serial.println();
}
#endif

#if XBUS_DATA_CONFIG_ACCEL_HR 
void print_accel_hr(xsens::xbus_motion_data_double& data)
{
    Serial.print("Accel (HR)(m/ss): (");
    Serial.print(data.accel_hr[0]);
    Serial.print(", ");
    Serial.print(data.accel_hr[1]);
    Serial.print(", ");
    Serial.print(data.accel_hr[2]);
    Serial.println(")");
}
#endif 

#if XBUS_DATA_CONFIG_RATE_OF_TURN_HR 
void print_rate_of_turn_hr(xsens::xbus_motion_data_double& data)
{
    Serial.print("Rate of Turn (HR)(m/ss): (");
    Serial.print(data.gyro_hr[0]);
    Serial.print(", ");
    Serial.print(data.gyro_hr[1]);
    Serial.print(", ");
    Serial.print(data.gyro_hr[2]);
    Serial.println(")");
}
#endif 

#if XBUS_DATA_CONFIG_POSITION_AND_VELOCITY
void print_ins(xsens::xbus_motion_data_double& data)
{
    Serial.print("INS Data | LLH: (");
    Serial.print(data.latlon[0]);
    Serial.print(", ");
    Serial.print(data.latlon[1]);
    Serial.print(", ");
    Serial.print(data.altitude);
    Serial.print(") | VelXYZ: (");
    Serial.print(data.velocity[0]);
    Serial.print(",");
    Serial.print(data.velocity[1]);
    Serial.print(", ");
    Serial.print(data.velocity[2]);
    Serial.println(")");
}
#endif

#if XBUS_DATA_CONFIG_GNSS_DATA
void print_gnss(xsens::xbus_motion_data_double& data)
{
    Serial.print("Pvt Data | ");
    Serial.print(data.year);
    Serial.print("-");
    Serial.print(data.month);
    Serial.print("-");
    Serial.print(data.day);
    Serial.print(" | ");
    Serial.print(data.hour);
    Serial.print(":");
    Serial.print(data.min);
    Serial.print(":");
    Serial.print(data.sec);
    Serial.print(" | LLH: (");
    Serial.print(data.lat);
    Serial.print(", ");
    Serial.print(data.lon);
    Serial.print(", ");
    Serial.print(data.height);
    Serial.print(")");
    Serial.println();
}
#endif
}
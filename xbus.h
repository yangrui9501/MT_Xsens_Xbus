/**
 * @file xbus.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Arduino.h>
#include <vector>
#include "xbus_consts.hpp"
#include "xbus_swap.h"
#include "xbus_gnss_data.hpp"
#include "simple_timer.hpp"

#define XBUS_PACKET_PAYLOAD_SIZE 512
#define ADDITION_BUFFER_SIZE 512
namespace xsens
{
class Xbus
{
public:
    Xbus()
    {
        memset(this, 0, sizeof(Xbus));
        bytes_consumed = 0x00;
    }
    enum XBUS_DATA_PARSE_STATUS
    {
        XD_WAIT_BEGIN_BYTE,
        XD_WAIT_PACKET_BYTE,
        XD_FAIL,
        XD_OK
    };

    // Begin Xbus data streaming
    void begin(HardwareSerial* _pSerial, int _baud_rate);
    void begin(HardwareSerial& _pSerial, int _baud_rate);
    // Read all data in the serial buffer
    int read();

    union
    {
        float f32;
        double f64;
    } altitude, temperature;
    union
    {
        float f32[2];
        double f64[2];
    } latlon;
    union
    {
        float f32[3];
        double f64[3];
    } gyro, accel, mag, velocity, euler, delta_v, free_accel;

    union
    {
        float f32[4];
        double f64[4];
    } quat;

    float accel_hr[3];
    float gyro_hr[3];
    uint32_t baro;
    inline const GnssData& get_gnss() const { return gnss_data; }
    const bool get_gyro_hr_double(double* _gyro_hr)
    {
        if (is_update.gyro_hr)
        {
            is_update.gyro_hr = false;
            for (int i = 0; i < 3; i++)
            {
                _gyro_hr[i] = (double)(gyro_hr[i]);
            }
            return true;
        }
        return false;
    }
    const bool get_accel_hr_double(double* _accel_hr)
    {
        if (is_update.accel_hr)
        {
            is_update.accel_hr = true;
            for (int i = 0; i < 3; i++)
            {
                _accel_hr[i] = (double)(accel_hr[i]);
            }
            return true;
        }
        return false;
    }
    const bool get_free_accel(double* _free_accel)
    {
        if (is_update.free_accel)
        {
            is_update.free_accel = false;
            memcpy(_free_accel, free_accel.f64, sizeof(double) * 3U);
            return true;
        }
        return false;
    }
protected:
    // bool gyro_hr_is_updated, accel_hr_is_updated;
    struct DataIsUpdate
    {
        bool gyro_hr;
        bool accel_hr;
        bool free_accel;
    } is_update;
    struct
    {
        uint8_t preamble;
        uint8_t bid;
        uint8_t mid;
        uint8_t length;
    } header;
    struct
    {
        uint8_t id[2];
        uint8_t length;
        uint8_t payload[XBUS_PACKET_PAYLOAD_SIZE];
    } packet;
    uint8_t checksum;
    GnssData gnss_data;

    int event_flag;
    int status;
    uint8_t bytes_consumed;

    HardwareSerial* MySerial;
    uint8_t serial_read_buf[ADDITION_BUFFER_SIZE];

    // protected function prototype
    void parse_data();
    uint8_t read_buffer();
    void calculate_checksum(uint8_t& cs, uint8_t* data, int counts);
    void read_payload();
};
}
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
#include <deque>

#define XBUS_PACKET_PAYLOAD_SIZE 512
namespace xsens
{
    class Xbus
    {
    public:
        Xbus() : altitude{}, temperature{}, latlon{}, gyro{}, accel{}, mag{}, velocity{}, euler{}, delta_v{}, free_accel{},
                 quat{}, accel_hr{}, gyro_hr{}, baro(0), is_update{false, false, false}, header{}, packet{}, checksum(0), gnss_data{}, event_flag(0), status(0), bytes_consumed(0)
        {
            // memset(this, 0, sizeof(Xbus)); // 2023-10-01
            bytes_consumed = 0x00;
            event_flag = XBUS_EVT_WAIT_PREAMBLE;
        }
        enum XBUS_DATA_PARSE_STATUS
        {
            XD_WAIT_BEGIN_BYTE,
            XD_WAIT_PACKET_BYTE,
            XD_FAIL,
            XD_OK
        };

        // Read all data in the serial buffer
        int parse(uint8_t *_buffer, int length)
        {
            for (int i = 0; i < length; i++)
            {
                buffer.push_back(_buffer[i]);
            }
            return read();
        }
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
        const bool get_gnss(xsens::GnssData::PvtData &_pvt_data)
        {
            if (is_update.gnss_pvt_data)
            {
                is_update.gnss_pvt_data = false;
                // memcpy(&_pvt_data, &gnss_data.get_pvt(), sizeof(_pvt_data));
                _pvt_data = gnss_data.get_pvt();
                return true;
            }
            return false;
        }
        const bool get_gyro_hr_double(double *_gyro_hr)
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
        const bool get_accel_hr_double(double *_accel_hr)
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
        const bool get_free_accel(double *_free_accel)
        {
            if (is_update.free_accel)
            {
                is_update.free_accel = false;
                memcpy(_free_accel, free_accel.f64, sizeof(double) * 3U);
                return true;
            }
            return false;
        }
        const bool get_mag(double *_mag)
        {
            if (is_update.mag)
            {
                is_update.mag = false;
                memcpy(_mag, mag.f64, sizeof(double) * 3U);
                return true;
            }
            return false;
        }

    protected:
        std::deque<uint8_t> buffer;
        // bool gyro_hr_is_updated, accel_hr_is_updated;
        struct DataIsUpdate
        {
            bool gyro_hr;
            bool accel_hr;
            bool free_accel;
            bool mag;
            bool gnss_pvt_data;
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

        // protected function prototype
        void parse_data();
        uint8_t read_buffer();
        void calculate_checksum(uint8_t &cs, uint8_t *data, int counts);
        void read_payload();
    };
}
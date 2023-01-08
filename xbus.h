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
        void begin(HardwareSerial *_pSerial, int _baud_rate);

        // Read all data in the serial buffer
        int read();

        union
        {
            float f32;
            double f64;
        } altitude;
        union
        {
            float f32[2];
            double f64[2];
        } latlon;
        union
        {
            float f32[3];
            double f64[3];
        } gyro, accel, mag, velocity, euler;

        union
        {
            float f32[4];
            double f64[4];
        } quat;
        inline void get_quat(float (&_quat)[4]) { memcpy(_quat, quat.f32, sizeof(_quat)); }
        inline void get_quat(double (&_quat)[4]) { memcpy(_quat, quat.f64, sizeof(_quat)); }
        inline const GnssData& get_gnss() const { return gnss_data; }

    protected:
        struct Timers
        {
            SimpleTimer euler;
            SimpleTimer gyro;
            SimpleTimer mag;
        } timer; // It will be removed in future release.
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

        HardwareSerial *MySerial;
        uint8_t serial_read_buf[ADDITION_BUFFER_SIZE];
        
        // protected function prototype
        void parse_data();
        uint8_t read_buffer();
        void calculate_checksum(uint8_t &cs, uint8_t *data, int counts);
        void read_payload();
    };
}
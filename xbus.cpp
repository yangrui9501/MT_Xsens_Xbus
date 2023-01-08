/**
 * @file xbus.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "xbus.h"

// #define XBUS_DEBUG_MODE

namespace xsens
{
    void Xbus::begin(HardwareSerial *_pSerial, int _baud_rate)
    {
        memset(serial_read_buf, 0, sizeof(serial_read_buf));
        event_flag = XBUS_EVT_WAIT_PREAMBLE;

        MySerial = _pSerial;
        MySerial->begin(_baud_rate);
        MySerial->addMemoryForRead(serial_read_buf, sizeof(serial_read_buf));
    }

    int Xbus::read()
    {
        while (event_flag == XBUS_EVT_WAIT_PREAMBLE && MySerial->available() >= 4) // Search for XBUS_HEADER_PREAMBLE
        {
            checksum = 0x00;

            // Read 1 byte: preamble
            header.preamble = read_buffer();

            if (header.preamble == XBUS_HEADER_PREAMBLE) // Check for first start byte
            {
                // Read 3 bytes: bid, mid, data length
                header.bid = read_buffer();
                header.mid = read_buffer();
                header.length = read_buffer();

                if ((header.bid == XBUS_HEADER_BID) && (header.mid == XBUS_HEADER_MID)) // Check for second and third start bytes
                {
                    event_flag = XBUS_EVT_WAIT_PACKETS;
                    bytes_consumed = 0x00;
                    calculate_checksum(checksum, &header.bid, 3);

#ifdef XBUS_DEBUG_MODE
                    Serial.print("Header: ");
                    Serial.print(header.preamble, HEX);
                    Serial.print(" ");
                    Serial.print(header.bid, HEX);
                    Serial.print(" ");
                    Serial.print(header.mid, HEX);
                    Serial.print(" ");
                    Serial.print(header.length, HEX);
                    Serial.println(" ");
#endif
                }
            }
        }

        if (event_flag == XBUS_EVT_WAIT_PREAMBLE)
        {
            status = XD_WAIT_BEGIN_BYTE;
        }
        else
        {
            status = XD_WAIT_PACKET_BYTE;
        }

        while (event_flag == XBUS_EVT_WAIT_PACKETS && MySerial->available() >= (int)(header.length - bytes_consumed) + 1)
        {
            // Read 3 bytes: 2 bytes ID, 1 byte data length
            packet.id[0] = read_buffer();
            packet.id[1] = read_buffer();
            packet.length = read_buffer();
            bytes_consumed += 0x03;

#ifdef XBUS_DEBUG_MODE
            Serial.print("Packet: ");
            Serial.print(packet.id[0], HEX);
            Serial.print(" ");
            Serial.print(packet.id[1], HEX);
            Serial.print(" ");
            Serial.print(packet.length, HEX);
            Serial.print(" ");
#endif

            // Check ID byte and parse data buffer
            parse_data();
            calculate_checksum(checksum, &packet.id[0], 0x03 + packet.length);

            if (bytes_consumed == header.length)
            {
                event_flag = XBUS_EVT_WAIT_PREAMBLE;
                bytes_consumed = 0x00;

                uint8_t checksum_read = read_buffer(); // Read 1 byte (checksum)

#ifdef XBUS_DEBUG_MODE
                Serial.print("Checksum: ");
                Serial.print(checksum, HEX);
                Serial.print(" ");
                Serial.print(checksum_read, HEX);
                Serial.println();
                Serial.flush();
#endif

                if (checksum == checksum_read)
                {
                    status = XD_OK;
                }
                else
                {
                    status = XD_FAIL;
                }
            }
        }

        return status;
    }

    void Xbus::parse_data()
    {
        uint16_t id16 = (uint16_t)(((uint16_t)(packet.id[0]) << 8U) | ((uint16_t)packet.id[1]));
        switch (id16)
        {
        case XBUS_DATA_ID_EULER_FLOAT_ENU:

            read_payload();
            memcpy(euler.f32, packet.payload, packet.length);
            for (auto &s : euler.f32)
            {
                xbus_swap_uint32(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_EULER_DOUBLE_ENU:

            read_payload();
            memcpy(euler.f64, packet.payload, packet.length);
            for (auto &s : euler.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_QUATERNION_FLOAT_ENU:

            read_payload();
            memcpy(quat.f32, packet.payload, packet.length);
            for (auto &s : quat.f32)
            {
                xbus_swap_uint32(&s, &s);
            }
            break; // 2023-01-06 ok

        case XBUS_DATA_ID_QUATERNION_DOUBLE_ENU:

            read_payload();
            memcpy(quat.f64, packet.payload, packet.length);
            for (auto &s : quat.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2023-01-06 ok

        case XBUS_DATA_ID_GYRO_FLOAT_ENU:
            timer.gyro.elapsed();
            timer.gyro.begin();

            read_payload();
            memcpy(gyro.f32, packet.payload, packet.length);
            for (auto &s : gyro.f32)
            {
                xbus_swap_uint32(&s, &s);
            }
            break; // 2023-01-06 ok

        case XBUS_DATA_ID_GYRO_DOUBLE_ENU:
            read_payload();
            memcpy(gyro.f64, packet.payload, packet.length);
            for (auto &s : gyro.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2223-01-06 ok

        case XBUS_DATA_ID_ACCEL_FLOAT_ENU:

            read_payload();
            memcpy(accel.f32, packet.payload, packet.length);
            for (auto &s : accel.f32)
            {
                xbus_swap_uint32(&s, &s);
            }

            break; // 2023-01-06 ok

        case XBUS_DATA_ID_ACCEL_DOUBLE_ENU:
            read_payload();
            memcpy(accel.f64, packet.payload, packet.length);
            for (auto &s : accel.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_MAG_FLOAT_ENU:
            timer.mag.elapsed();
            timer.mag.begin();

            read_payload();
            memcpy(mag.f32, packet.payload, packet.length);
            for (auto &s : mag.f32)
            {
                xbus_swap_uint32(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_MAG_DOUBLE_ENU:
            read_payload();
            memcpy(mag.f64, packet.payload, packet.length);
            for (auto &s : mag.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_LATLON_FLOAT_ENU:
            read_payload();
            memcpy(latlon.f32, packet.payload, packet.length);
            for (auto &s : latlon.f32)
            {
                xbus_swap_uint32(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_LATLON_DOUBLE_ENU:
            read_payload();
            memcpy(latlon.f64, packet.payload, packet.length);
            for (auto &s : latlon.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_ALTITUDE_FLOAT_ENU:

            read_payload();
            memcpy(&altitude.f32, packet.payload, packet.length);
            xbus_swap_uint32(&altitude.f32, &altitude.f32);
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_ALTITUDE_DOUBLE_ENU:
            read_payload();
            memcpy(&altitude.f64, packet.payload, packet.length);
            xbus_swap_uint64(&altitude.f64, &altitude.f64);
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_VELOCITY_FLOAT_ENU:
            read_payload();
            memcpy(velocity.f32, packet.payload, packet.length);
            for (auto &s : velocity.f32)
            {
                xbus_swap_uint32(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_VELOCITY_DOUBLE_ENU:
            read_payload();
            memcpy(velocity.f64, packet.payload, packet.length);
            for (auto &s : velocity.f64)
            {
                xbus_swap_uint64(&s, &s);
            }
            break; // 2023-01-06 ok
        case XBUS_DATA_ID_GNSS_PVTDATA_ENU:
            read_payload();
            memcpy(&gnss_data.pvt_data, packet.payload, packet.length);
            gnss_data.swap();

            // double gnss_data[2];
            // gnss_data[0] = (double)(pvt_data.lat) / 10000000.0;
            // gnss_data[1] = (double)(pvt_data.lon) / 10000000.0;

#ifdef XBUS_DEBUG_MODE
            Serial.print(gnss_data.get_pvt().year);
            Serial.print("-");
            Serial.print(gnss_data.pvt_data.month);
            Serial.print("-");
            Serial.print(gnss_data.pvt_data.day);
            Serial.print(" | ");
            Serial.print(gnss_data.pvt_data.hour);
            Serial.print(":");
            Serial.print(gnss_data.pvt_data.min);
            Serial.print(":");
            Serial.print(gnss_data.pvt_data.sec);
            Serial.print(" | ");
            Serial.print(gnss_data.pvt_data.lat);
            Serial.print(" ");
            // Serial.print(gnss_data[0], 7);
            // Serial.print(" ");
            Serial.print(gnss_data.pvt_data.lon);
            // Serial.print(" ");
            // Serial.print(gnss_data[1], 7);
            Serial.print(" ");
            Serial.println();
#endif
            break;
        default:
            Serial.print("Error: No matched ID: ");
            Serial.print(packet.id[0], HEX);
            Serial.print(" ");
            Serial.print(packet.id[1], HEX);
            Serial.print(" ");
            Serial.print(packet.length);
            Serial.println();

            bytes_consumed = 0x00;
            event_flag = XBUS_EVT_WAIT_PREAMBLE;
            break;
        }
#ifdef XBUS_DEBUG_MODE
        Serial.print(bytes_consumed, HEX);
        Serial.print(" ");
        for (uint8_t i = 0; i < packet.length; i++)
        {
            Serial.print("0x");
            Serial.print(packet.payload[i], HEX);
            Serial.print(", ");
        }
        Serial.println();
#endif
    }

    uint8_t Xbus::read_buffer()
    {
        // Calculate checksum
        return (uint8_t)(MySerial->read());
    }

    void Xbus::calculate_checksum(uint8_t &cs, uint8_t *data, int counts)
    {
        for (int i = 0; i < counts; i++)
        {
            cs -= data[i];
        }
    }

    void Xbus::read_payload()
    {
        for (uint8_t i = 0; i < packet.length; i++)
        {
            packet.payload[i] = read_buffer();
        }
        bytes_consumed += packet.length;
    }
}

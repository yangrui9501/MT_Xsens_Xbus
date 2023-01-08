/**
 * @file Xbus_typedefs.h
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

#define XBUS_PACKET_PAYLOAD_SIZE 128

namespace xsens
{
    typedef struct
    {
        uint8_t preamble;
        uint8_t bid;
        uint8_t mid;
        uint8_t length;
    } xbus_header_t;

    typedef struct
    {
        uint8_t id[2];
        uint8_t length;
        uint8_t payload[XBUS_PACKET_PAYLOAD_SIZE];
    } xbus_packet_t;

}
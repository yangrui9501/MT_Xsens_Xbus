/**
 * @file xbus_gnss_data.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-01-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <Arduino.h>
#include "xbus_swap.h"

namespace xsens
{
    class GnssData
    {
    protected:
        struct PvtData
        {
            uint32_t iTOW;
            uint16_t year; //
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t min;
            uint8_t sec;
            uint8_t valid;
            uint32_t tAcc;
            int32_t nano;
            uint8_t fixType;
            uint8_t flags;
            uint8_t numSv;
            uint8_t reserved;
            int32_t lon; //
            int32_t lat; //
            int32_t height; //
            int32_t hMsl;
            uint32_t hAcc;
            uint32_t vAcc;
            int32_t velN;
            int32_t velE;
            int32_t velD;
            int32_t gSpeed;
            int32_t headMot;
            uint32_t sAcc;
            uint32_t headAcc;
            int32_t headVeh;
            uint16_t gDop;
            uint16_t pDop;
            uint16_t tDop;
            uint16_t vDop;
            uint16_t hDop;
            uint16_t nDop;
            uint16_t eDop;
        } pvt_data; // 要注意這個結構會多 2 bytes

        void swap()
        {
            xbus_swap_uint16(&pvt_data.year, &pvt_data.year);
            xbus_swap_uint32(&pvt_data.lon, &pvt_data.lon);
            xbus_swap_uint32(&pvt_data.lat, &pvt_data.lat);
            xbus_swap_uint32(&pvt_data.height, &pvt_data.height);
        }

    public:
        GnssData() { memset(this, 0, sizeof(GnssData)); }
        const struct PvtData &get_pvt() const { return pvt_data; }
        friend class Xbus;
    }; 
}
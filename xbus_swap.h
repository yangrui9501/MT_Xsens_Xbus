/**
 * @file xbus_swap.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-01-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <Arduino.h>

namespace xsens
{
    void xbus_swap_uint16(void *_dest, void *_src);
    void xbus_swap_uint32(void *_dest, void *_src);
    void xbus_swap_uint64(void *_dest, void *_src);
}

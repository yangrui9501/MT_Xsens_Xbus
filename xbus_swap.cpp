#include "xbus_swap.h"

#define SHIFT_UINT8_IN_UINT64(x, bits) ((uint64_t)(x) << bits)
#define SHIFT_UINT8_IN_UINT32(x, bits) ((uint32_t)(x) << bits)

namespace xsens
{
    void xbus_swap_uint16(void *_dest, void *_src)
    {
        uint16_t *dest = (uint16_t *)_dest;
        uint8_t *src = (uint8_t *)_src;
        *dest = (uint16_t)(src[0]) << 8U | (uint16_t)(src[1]);
    }
    void xbus_swap_uint32(void *_dest, void *_src)
    {
        // 因為 Xbus 傳送浮點數資料是先傳送低位字節，
        // 所以這裡進行位移運算將低位字節移至記憶體開頭處
        // 假設說現在接收到的 32 位元浮點數資料為 0x1A2B3C4D
        // 則接收端所接收到的位元組順序為
        //  byte[0] byte[1] byte[2] byte[3]
        //     0x1A    0x2B    0x3C    0x4D
        // 也就是說，Xbus 預設先傳送「高位字節」(從左邊傳到右邊)
        // 這樣的 4 個位元組在 32 位元記憶體空間的實際排列順序是 0x4D3C2B1A
        // 也就是說排列順序預設是從低位排到高位
        // 為了要將資料以正確的順序排列，
        // 所以我們這裡要主動把高位字節移到 32 位元記憶體空間的最左邊，
        // 也就是 0x1A << 24U
        // 其它人分別是 0x2B << 16U, 0x3C << 8U, 0x4D
        // 最後 0x1A << 24U | 0x2B << 16U | 0x3C << 8U | 0x4D
        // 這樣在 32 位元記憶體空間的排列順序就會是 0x1A2B3C4D
        // 2023-01-07
        uint32_t *dest = (uint32_t *)_dest;
        uint8_t *src = (uint8_t *)_src;
        *dest = SHIFT_UINT8_IN_UINT32(src[0], 24U) |
                SHIFT_UINT8_IN_UINT32(src[1], 16U) |
                SHIFT_UINT8_IN_UINT32(src[2], 8U) |
                SHIFT_UINT8_IN_UINT32(src[3], 0U);
    }

    void xbus_swap_uint64(void *_dest, void *_src)
    {
        uint64_t *dest = (uint64_t *)_dest;
        uint8_t *src = (uint8_t *)_src;
        *dest = SHIFT_UINT8_IN_UINT64(src[0], 56U) |
                SHIFT_UINT8_IN_UINT64(src[1], 48U) |
                SHIFT_UINT8_IN_UINT64(src[2], 40U) |
                SHIFT_UINT8_IN_UINT64(src[3], 32U) |
                SHIFT_UINT8_IN_UINT64(src[4], 24U) |
                SHIFT_UINT8_IN_UINT64(src[5], 16U) |
                SHIFT_UINT8_IN_UINT64(src[6], 8U) |
                SHIFT_UINT8_IN_UINT64(src[7], 0U);
        // uint8_t *_dest = (uint8_t *)dest;
        // static uint8_t temp[8];
        // memcpy(temp, _src, 8U);
        // _dest[0] = temp[7];
        // _dest[1] = temp[6];
        // _dest[2] = temp[5];
        // _dest[3] = temp[4];
        // _dest[4] = temp[3];
        // _dest[5] = temp[2];
        // _dest[6] = temp[1];
        // _dest[7] = temp[0];
    }
}

// Jan 07, 2023
#include "xbus_swap.h"

#define SHIFT_UINT8_IN_UINT64(x, bits) ((uint64_t)(x) << bits)
#define SHIFT_UINT8_IN_UINT32(x, bits) ((uint32_t)(x) << bits)

namespace xsens
{

void xbus_swap_uint16(void* _dest, void* _src)
{
    uint16_t* dest = (uint16_t*)_dest;
    uint8_t* src = (uint8_t*)_src;
    *dest = (uint16_t)(src[0]) << 8U | (uint16_t)(src[1]);
}

void xbus_swap_uint32(void* _dest, void* _src)
{
    // 因為 Xbus 傳送資料是先傳送「高位字元」，
    // 所以這裡透過位移運算，將所接收的位元組資料「反序」才能重建為原本的物理資料
    //
    // 舉例來說，一個 32 位元浮點數資料為 0x1A2B3C4D，
    // Xbus 傳過來，接收端所接收到的位元組順序為
    //      記憶體順序  byte_buf[0] byte_buf[1] byte_buf[2] byte_buf[3]
    //      資料        0x1A        0x2B        0x3C        0x4D
    //                 高位字元                 低位字元
    // 也就是說，Xbus 預設先傳送「高位字元」(0x1A2B3C4D 從左邊傳到右邊)
    //
    // 但 Teensy 預設的儲存方式是先儲存低位字元。
    // 也就是說，要儲存 0x1A2B3C4D 這筆資料時，排列順序必須是
    //      記憶體順序  byte[0] byte[1] byte[2] byte[3]
    //      資料        0x4D    0x3C    0x2B    0x1A
    //                 低位字元                 高位字元
    // 但現在記憶體排列順序是相反的，
    // 如果什麼都不做，0x1A, 0x2B, 0x3C, 0x4D 這個順序，
    // 重建出來的 32 位元組資料就是 0x4D3C2B1A，
    // 因為 Teensy 會認為最先接收到的 0x1A 是低位字元，但實際上是高位字元。
    // 
    // 因此，為了要將資料正確地重建，要進行記憶體排列反序，
    // 像是這樣：
    //      byte[3] = byte_buf[0];
    //      byte[2] = byte_buf[1];
    //      byte[1] = byte_buf[2];
    //      byte[0] = byte_buf[3];
    // 或是可以透過位移運算，首先是 0x1A << 24U
    // 其它分別是 0x2B << 16U, 0x3C << 8U, 0x4D
    // 像是這樣
    //          0x1A000000
    //          0x002B0000
    //          0x00003C00
    //          0x0000004D
    // 最後再將上述四個 32 位元組，進行 or 運算，也就是 
    //      0x1A << 24U | 0x2B << 16U | 0x3C << 8U | 0x4D
    // 上述二種方法，在 32 位元記憶體空間的排列順序就會是 
    //      記憶體順序  byte[0] byte[1] byte[2] byte[3]
    //      資料        0x4D    0x3C    0x2B    0x1A
    //                 低位字元                 高位字元
    // 而組成的 32 位元資料會是 0x1A2B3C4D
    // 2023-01-07

    uint32_t* dest = (uint32_t*)_dest;
    uint8_t* src = (uint8_t*)_src;
    *dest = SHIFT_UINT8_IN_UINT32(src[0], 24U) |
        SHIFT_UINT8_IN_UINT32(src[1], 16U) |
        SHIFT_UINT8_IN_UINT32(src[2], 8U) |
        SHIFT_UINT8_IN_UINT32(src[3], 0U);
}

void xbus_swap_uint64(void* _dest, void* _src)
{
    uint64_t* dest = (uint64_t*)_dest;
    uint8_t* src = (uint8_t*)_src;
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

} // namespace xsens

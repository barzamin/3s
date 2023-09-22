#include <cstdint>
#include <cstdio>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// -- from MAME
u16 rotate_left(u16 value, int n) {
    int aux = value>>(16-n);
    return ((value<<n)|aux)%0x10000;
}

u16 rotxor(u16 val, u16 xorval) {
    u16 res = val + rotate_left(val,2);

    res = rotate_left(res,4) ^ (res & (val ^ xorval));

    return res;
}

u32 cps3_mask(u32 address, u32 key1, u32 key2) {
    // // ignore all encryption
    // if (m_altEncryption == 2)
    // 	return 0;

    address ^= key1;

    u16 val = (address & 0xffff) ^ 0xffff;

    val = rotxor(val, key2 & 0xffff);

    val ^= (address >> 16) ^ 0xffff;

    val = rotxor(val, key2 >> 16);

    val ^= (address & 0xffff) ^ (key2 & 0xffff);

    return val | (val << 16);
}

int main(int argc, char* argv[]) {
    // 3s = sfiii3nr1
    // 0xa55432b4, 0x0c129981
    u32 key1 = 0xa55432b4;
    u32 key2 = 0x0c129981;

    // test vectors:
    for (u32 addr = 0; addr < 0x080000; addr++) {
        printf("%08x: mask = %08x\n", addr, cps3_mask(addr, key1, key2));
    }

    return 0;
}
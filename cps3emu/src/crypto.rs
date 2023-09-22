//! DECRYPTION LOGIC
//! thanks to Andreas Naive via MAME/FBNeo code.
//! written while drunk as fuck at [Nightclub](https://start.gg/nightclub)

fn rotate_left(val: u16, n: i32) -> u16 {
    let aux: u32 = (val >> (16 - n)) as u32;
    ((((val as u32) << n) | aux) % 0x10000) as u16
}

fn rotxor(val: u16, xorval: u16) -> u16 {
    let mut res: u16 = val.wrapping_add(rotate_left(val, 2));

    rotate_left(res, 4) ^ (res & (val ^ xorval))
}

pub fn cps3_kdf(addr: u32, key: [u32; 2]) -> u32 {
    let mut addr = addr;

    addr ^= key[0];
    let mut val: u16 = ((addr & 0xffff) as u16) ^ 0xffff;
    val = rotxor(val, (key[1] & 0xffff) as u16);
    val ^= ((addr >> 16) as u16) ^ 0xffff;
    val = rotxor(val, (key[1] >> 16) as u16);
    val ^= ((addr & 0xffff) as u16) ^ ((key[1] & 0xffff) as u16);

    (val as u32) | ((val as u32) << 16)
}

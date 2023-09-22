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

#[cfg(test)]
mod tests {
    use std::{path::{Path, PathBuf}, fs::{self, File}, io::{BufReader, BufRead}};
    use regex::Regex;

    use super::*;

    #[test]
    fn mame_golden_check_kdf() {
        let KEY: [u32; 2] = [0xa55432b4, 0x0c129981];
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("test/data/kdf_a55432b4_0c129981");
        let rdr = BufReader::new(File::open(path).unwrap());

        let kdf_re = Regex::new(r"([0-9a-fA-F]+): mask = ([0-9a-fA-F]+)").unwrap();
        for line in rdr.lines().take(0x1000) { // only do a small subset for speed
            if let Ok(line) = line {
                let caps = kdf_re.captures(&line).unwrap();
                println!("caps={:?}", caps);
                let addr: u32 = u32::from_str_radix(&caps[1], 16).unwrap();
                let gold_mask: u32 = u32::from_str_radix(&caps[2], 16).unwrap();

                assert_eq!(gold_mask, cps3_kdf(addr, KEY));
            }
        }
    }
}

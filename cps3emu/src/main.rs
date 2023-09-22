// use yaxpeax_superh::SuperHDecoder;

use crate::crypto::cps3_kdf;

mod crypto;

fn main() {
    for addr in 0..0x080000 {
        println!("{:08x}: mask = {:08x}", addr, cps3_kdf(addr, [0xa55432b4, 0x0c129981]));
    }
}

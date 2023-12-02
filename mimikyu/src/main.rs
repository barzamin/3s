use std::{
    fs::{self, File},
    io::{Read, Write},
    path::{Path, PathBuf},
    sync::{
        mpsc::{self, channel},
        Arc, RwLock,
    },
    thread,
};

use anyhow::Result;
use clap::Parser;
use log::info;
use serde::{Deserialize, Serialize};

mod cps3;
mod cpu;
mod gui;

#[derive(Debug, Parser)]
struct Args {
    // game: PathBuf,
    #[arg(long)]
    trace_pc: Option<PathBuf>,
}

#[derive(Debug, Serialize, Deserialize)]
struct GameConfig {
    key: [u32; 2],
}

fn draw(w: usize, h: usize, fb: &mut [u8]) {
    for x in 0..w {
        for y in 0..h {
            // rgba8
            let p = (y * w + x) * 4;
            fb[p + 0] = 0xff;
            fb[p + 1] = 0x00;
            fb[p + 2] = 0xff;
            fb[p + 3] = 0xff;
        }
    }
}

fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();

    // let game_config: GameConfig = serde_yaml::from_reader(File::open(args.game)?)?;
    // println!("game_config={:#?}", game_config);

    let fb_size: [u32; 2] = [160, 144];
    let framebuf: Arc<RwLock<Vec<u8>>> = Arc::new(RwLock::new(vec![
        0;
        fb_size[0] as usize
            * fb_size[1] as usize
            * 4
    ]));

    // let fb = framebuf.clone();
    // let simthread = thread::spawn(move || {

    //     // loop {
    //     // //     cpu.execute(16);
    //     // }

    //     draw(fb_size[0] as usize, fb_size[1] as usize, &mut fb.write().unwrap());
    // });

    let mut pc_trace_f = args.trace_pc.map(File::create).transpose()?;
    let mut cpu = cpu::lr35902::Lr35902::new(move |it| {
        if let Some(ref mut f) = pc_trace_f {
            writeln!(f, "{:04x} -> {:04x} | {}", it.from_pc, it.to_pc, it.instr).unwrap();
        }
    });
    info!("reading DMG bootstrap rom into memory");
    let mut f =
        fs::File::open(Path::new(env!("CARGO_MANIFEST_DIR")).join("data/DMG_ROM.bin")).unwrap();
    f.read(&mut cpu.memory).unwrap();
    loop {
        cpu.execute(16);
    }

    // on macOS, the winit::EventLoop has to live on main thread
    // gui::run(fb_size)?;

    // simthread.join().unwrap();

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verify_cli() {
        use clap::CommandFactory;
        Args::command().debug_assert();
    }
}

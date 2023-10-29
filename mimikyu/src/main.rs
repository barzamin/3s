use std::{
    fs,
    io::Read,
    path::{Path, PathBuf},
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
    game: PathBuf,
}

#[derive(Debug, Serialize, Deserialize)]
struct GameConfig {
    key: [u32; 2],
}

fn main() -> Result<()> {
    env_logger::init();
    // let args = Args::parse();

    // let game_config: GameConfig = serde_yaml::from_reader(File::open(args.game)?)?;
    // println!("game_config={:#?}", game_config);
    // gui::run()?;

    let mut cpu = cpu::lr35902::Lr35902::new();
    info!("reading DMG bootstrap rom into memory");
    let mut f = fs::File::open(Path::new(env!("CARGO_MANIFEST_DIR")).join("data/DMG_ROM.bin"))?;
    f.read(&mut cpu.memory)?;
    loop {
        cpu.execute(16);
    }

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

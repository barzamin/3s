use std::{
    fs::{self, File},
    path::PathBuf,
};

use anyhow::Result;
use clap::Parser;
use serde::{Deserialize, Serialize};

mod cps3;
mod gui;
mod cpu;

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
    let args = Args::parse();

    let game_config: GameConfig = serde_yaml::from_reader(File::open(args.game)?)?;
    println!("game_config={:#?}", game_config);
    // gui::run()?;


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

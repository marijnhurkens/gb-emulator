#![allow(dead_code)]

extern crate core;

use std::ffi::OsString;
use std::fs::File;
use std::io::Read;
use std::sync::{Arc, Mutex};
use std::thread;

use clap::Parser;
use ggez::conf::WindowSetup;
use ggez::event::EventHandler;
use ggez::graphics::{Color, DrawParam, Image, ImageFormat};
use ggez::mint::Vector2;
use ggez::{event, graphics, Context, ContextBuilder, GameResult};

use crate::cartridge::Cartridge;
use crate::cpu::Cpu;

mod cartridge;
mod cpu;
mod helpers;
mod memory;
mod instructions;

const SCREEN_WIDTH: u32 = 160;
const SCREEN_HEIGHT: u32 = 144;
const SCREEN_BUFFER_SIZE: usize = (SCREEN_WIDTH * SCREEN_HEIGHT) as usize;
type ScreenBuffer = Arc<Mutex<[u8; SCREEN_BUFFER_SIZE]>>;

#[derive(Parser, Debug)]
struct Args {
    rom_file: OsString,
    break_point: Option<String>,
}

struct State {
    screen_buffer: ScreenBuffer,
}

fn main() {
    let args = Args::parse();

    // read rom file
    let mut rom = File::open(&args.rom_file).unwrap();
    let mut data = vec![];
    rom.read_to_end(&mut data).unwrap();

    let cartridge = Cartridge::load_rom(data);
    let mut cpu = Cpu::load_cartridge(cartridge);

    let break_point = args
        .break_point
        .map(|break_point| u16::from_str_radix(break_point.trim_start_matches("0x"), 16).unwrap());

    let screen_buffer: ScreenBuffer = Arc::new(Mutex::new([0; SCREEN_BUFFER_SIZE]));

    let cpu_screen_buffer = screen_buffer.clone();
    thread::spawn(move || {
        cpu.run(cpu_screen_buffer.clone(), break_point);
    });

    let (mut ctx, event_loop) = ContextBuilder::new("gb_emu", "")
        .window_setup(WindowSetup::default().title("GB Emulator"))
        .build()
        .expect("Error creating context.");

    let state = State::new(&mut ctx, screen_buffer);

    event::run(ctx, event_loop, state);
}

impl State {
    pub fn new(_ctx: &mut Context, screen_buffer: ScreenBuffer) -> State {
        State { screen_buffer }
    }
}

impl EventHandler for State {
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        // Update code here...
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(ctx, Color::WHITE);

        let guard = self.screen_buffer.lock().unwrap();
        let image = Image::from_pixels(
            ctx,
            &*guard,
            ImageFormat::R8Unorm,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
        );
        drop(guard);

        let scale = Vector2::<f32> { x: 4.0, y: 4.0 };
        canvas.draw(&image, DrawParam::new().scale(scale));
        canvas.finish(ctx)
    }
}

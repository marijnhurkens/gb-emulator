#![allow(dead_code)]

extern crate core;

use std::ffi::OsString;
use std::fs::File;
use std::io::{stdout, Read};
use std::sync::{Arc, Mutex};
use std::thread;

use clap::Parser;
use ggez::conf::WindowSetup;
use ggez::event::EventHandler;
use ggez::graphics::{Color, DrawParam, Image, ImageFormat};
use ggez::mint::Vector2;
use ggez::{event, graphics, Context, ContextBuilder, GameResult};
use tracing::Level;
use tracing_subscriber::fmt::writer::MakeWriterExt;
use tracing_subscriber::prelude::*;
use tracing_subscriber::Registry;

use crate::cartridge::Cartridge;
use crate::cpu::Cpu;

mod cartridge;
mod cpu;
mod helpers;
mod instructions;
mod memory;

const SCREEN_WIDTH: u32 = 160;
const SCREEN_HEIGHT: u32 = 144;
const SCREEN_BUFFER_SIZE: usize = (SCREEN_WIDTH * SCREEN_HEIGHT) as usize;
type ScreenBuffer = Arc<Mutex<[u8; SCREEN_BUFFER_SIZE]>>;

#[derive(Parser, Debug)]
struct Args {
    rom_file: OsString,

    #[arg(short, long)]
    break_point: Option<String>,

    #[arg(short, long)]
    log: Option<Level>,
}

struct State {
    screen_buffer: ScreenBuffer,
}

fn main() {
    let args = Args::parse();

    let stdout_log = tracing_subscriber::fmt::layer()
        .with_writer(stdout.with_max_level(args.log.unwrap_or(Level::ERROR)))
        .with_file(false)
        .with_line_number(false)
        .without_time()
        .pretty();


    let file = File::create("cpu.log").unwrap();


    let cpu_log = tracing_subscriber::fmt::layer()
        .with_writer(file.with_max_level(Level::TRACE).with_min_level(Level::TRACE))
        .with_line_number(false)
        .without_time()
        .with_ansi(false)
        .with_level(false)
        .with_file(false)
        .with_target(false)
        ;

    let my_subscriber = Registry::default().with(stdout_log).with(cpu_log);
    tracing::subscriber::set_global_default(my_subscriber).expect("setting tracing default failed");

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
        cpu.run(Some(cpu_screen_buffer.clone()), break_point);
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

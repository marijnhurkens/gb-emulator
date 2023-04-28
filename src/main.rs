#![allow(dead_code, clippy::upper_case_acronyms)]

extern crate core;

use std::ffi::OsString;
use std::fs::File;
use std::io::{stdout, Read};
use std::sync::{Arc, Mutex};
use std::thread;

use clap::{arg, Parser};
use ggez::conf::WindowSetup;
use ggez::event::EventHandler;
use ggez::graphics::{Color, DrawParam, Image, ImageFormat};
use ggez::input::keyboard::KeyCode;
use ggez::mint::Vector2;
use ggez::{event, graphics, Context, ContextBuilder, GameResult};
use tracing::Level;
use tracing_subscriber::fmt::writer::MakeWriterExt;
use tracing_subscriber::prelude::*;
use tracing_subscriber::Registry;

use crate::cartridge::Cartridge;
use crate::cpu::Cpu;
use crate::memory::Memory;

mod cartridge;
mod cpu;
mod helpers;
mod instructions;
mod memory;
mod video;

const SCREEN_WIDTH: u32 = 160;
const SCREEN_HEIGHT: u32 = 144;
const SCREEN_BUFFER_SIZE: usize = (SCREEN_WIDTH * SCREEN_HEIGHT) as usize;

type ScreenBuffer = [u8; SCREEN_BUFFER_SIZE];

#[derive(Parser, Debug)]
struct Args {
    rom_file: OsString,

    #[arg(short, long)]
    break_point: Option<String>,

    #[arg(short, long)]
    log: Option<Level>,

    #[arg(short, long)]
    cpu_log: bool,
}

#[derive(Debug, Default)]
pub struct KeyState {
    up: bool,
    down: bool,
    left: bool,
    right: bool,
    start: bool,
    select: bool,
    a: bool,
    b: bool,
}

struct State {
    screen_buffer: Arc<Mutex<ScreenBuffer>>,
    key_state: Arc<Mutex<KeyState>>,
}

fn main() {
    let args = Args::parse();

    let stdout_log = tracing_subscriber::fmt::layer()
        .with_writer(stdout.with_max_level(args.log.unwrap_or(Level::ERROR)))
        .with_line_number(false)
        .without_time()
        .with_file(false)
        .with_level(false)
        .with_target(false);

    let file = File::create("cpu.log").unwrap();

    let cpu_log = if args.cpu_log {
        Some(
            tracing_subscriber::fmt::layer()
                .with_writer(
                    file.with_max_level(Level::TRACE)
                        .with_min_level(Level::TRACE),
                )
                .with_line_number(false)
                .without_time()
                .with_ansi(false)
                .with_level(false)
                .with_file(false)
                .with_target(false),
        )
    } else {
        None
    };

    let my_subscriber = Registry::default().with(stdout_log).with(cpu_log);

    tracing::subscriber::set_global_default(my_subscriber).expect("setting tracing default failed");

    // read rom file
    let mut rom = File::open(&args.rom_file).unwrap();
    let mut data = vec![];
    rom.read_to_end(&mut data).unwrap();
    let cartridge = Cartridge::load_rom(data.clone());

    let screen_buffer: Arc<Mutex<ScreenBuffer>> = Arc::new(Mutex::new([0; SCREEN_BUFFER_SIZE]));
    let cpu_screen_buffer = screen_buffer.clone();
    let key_state = Arc::new(Mutex::new(KeyState::default()));

    let memory = Memory::new(data, key_state.clone());
    let mut cpu = Cpu::load_cartridge(cartridge, memory);

    let break_point = args
        .break_point
        .map(|break_point| u16::from_str_radix(break_point.trim_start_matches("0x"), 16).unwrap());

    thread::spawn(move || {
        cpu.run(Some(cpu_screen_buffer), break_point);
    });

    let (mut ctx, event_loop) = ContextBuilder::new("gb_emu", "")
        .window_setup(WindowSetup::default().title("GB Emulator"))
        .build()
        .expect("Error creating context.");

    let state = State::new(&mut ctx, screen_buffer, key_state);

    event::run(ctx, event_loop, state);
}

impl State {
    pub fn new(
        _ctx: &mut Context,
        screen_buffer: Arc<Mutex<ScreenBuffer>>,
        key_state: Arc<Mutex<KeyState>>,
    ) -> State {
        State {
            screen_buffer,
            key_state,
        }
    }
}

impl EventHandler for State {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        let mut key_state = self.key_state.lock().unwrap();

        key_state.a = ctx.keyboard.is_key_pressed(KeyCode::Z);
        key_state.b = ctx.keyboard.is_key_pressed(KeyCode::X);

        key_state.up = ctx.keyboard.is_key_pressed(KeyCode::Up);
        key_state.down = ctx.keyboard.is_key_pressed(KeyCode::Down);
        key_state.left = ctx.keyboard.is_key_pressed(KeyCode::Left);
        key_state.right = ctx.keyboard.is_key_pressed(KeyCode::Right);

        key_state.select = ctx.keyboard.is_key_pressed(KeyCode::Backslash);
        key_state.start = ctx.keyboard.is_key_pressed(KeyCode::Return);

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(ctx, Color::WHITE);

        let guard = self.screen_buffer.lock().unwrap();
        let image_pixels = guard
            .into_iter()
            .flat_map(|f| std::iter::repeat(f).take(3).chain(vec![255]))
            .collect::<Vec<_>>();
        let image = Image::from_pixels(
            ctx,
            &image_pixels,
            ImageFormat::Rgba8Unorm,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
        );
        drop(guard);

        let scale = Vector2::<f32> { x: 4.0, y: 4.0 };
        canvas.draw(&image, DrawParam::new().scale(scale));
        canvas.finish(ctx)
    }
}

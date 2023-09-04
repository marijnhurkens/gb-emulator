#![allow(dead_code, clippy::upper_case_acronyms, clippy::bad_bit_mask)]

extern crate core;

use std::collections::VecDeque;
use std::ffi::OsString;
use std::fs::File;
use std::io::{stdout, Read};
use std::sync::{Arc, Mutex};
use std::thread;

use clap::{arg, Parser};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{SampleFormat, SampleRate, Stream};
use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::EventHandler;
use ggez::graphics::{Color, DrawParam, Image, ImageFormat, Sampler};
use ggez::input::keyboard::KeyCode;
use ggez::mint::Vector2;
use ggez::{event, graphics, Context, ContextBuilder, GameResult};
use tracing::Level;
use tracing_subscriber::fmt::writer::MakeWriterExt;
use tracing_subscriber::prelude::*;
use tracing_subscriber::Registry;

use crate::apu::Apu;
use crate::cartridge::Cartridge;
use crate::cpu::Cpu;
use crate::mmu::MMU;

mod apu;
mod cartridge;
mod cpu;
mod helpers;
mod instructions;
mod mbc;
mod mmu;
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
    redraw: bool,
    audio_buffer: Arc<Mutex<VecDeque<i16>>>,
    stream: Stream,
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

    let (audio_stream, audio_buffer) = setup_audio();
    let audio_buffer_apu = audio_buffer.clone();

    let mbc = mbc::from_cartridge(cartridge);
    let apu = Apu::new_with_buffer(audio_buffer_apu);
    let mmu = MMU::new(mbc, apu, key_state.clone());
    let mut cpu = Cpu::new(mmu);

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

    ctx.gfx
        .set_mode(WindowMode::default().resizable(true))
        .unwrap();

    let state = State::new(
        &mut ctx,
        screen_buffer,
        key_state,
        audio_buffer,
        audio_stream,
    );

    event::run(ctx, event_loop, state);
}

impl State {
    pub fn new(
        _ctx: &mut Context,
        screen_buffer: Arc<Mutex<ScreenBuffer>>,
        key_state: Arc<Mutex<KeyState>>,
        audio_buffer: Arc<Mutex<VecDeque<i16>>>,
        stream: Stream,
    ) -> State {
        State {
            screen_buffer,
            key_state,
            redraw: false,
            audio_buffer,
            stream,
        }
    }
}

impl EventHandler for State {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ctx.time.check_update_time(60) {
            self.redraw = true;

            let mut key_state = self.key_state.lock().unwrap();

            key_state.a = ctx.keyboard.is_key_pressed(KeyCode::Z);
            key_state.b = ctx.keyboard.is_key_pressed(KeyCode::X);

            key_state.up = ctx.keyboard.is_key_pressed(KeyCode::Up);
            key_state.down = ctx.keyboard.is_key_pressed(KeyCode::Down);
            key_state.left = ctx.keyboard.is_key_pressed(KeyCode::Left);
            key_state.right = ctx.keyboard.is_key_pressed(KeyCode::Right);

            key_state.select = ctx.keyboard.is_key_pressed(KeyCode::Backslash);
            key_state.start = ctx.keyboard.is_key_pressed(KeyCode::Return);
        }

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        if !self.redraw {
            return Ok(());
        }

        self.redraw = false;
        let mut canvas = graphics::Canvas::from_frame(ctx, Color::WHITE);

        let guard = self.screen_buffer.lock().unwrap();
        let image_pixels = guard
            .into_iter()
            .flat_map(|f| std::iter::repeat(f).take(3).chain(vec![255]))
            .collect::<Vec<_>>();
        drop(guard);
        let image = Image::from_pixels(
            ctx,
            &image_pixels,
            ImageFormat::Rgba8Unorm,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
        );

        let window_size = ctx.gfx.window().inner_size();
        let scale = if window_size.width as f32 / window_size.height as f32
            > SCREEN_WIDTH as f32 / SCREEN_HEIGHT as f32
        {
            window_size.height as f32 / SCREEN_HEIGHT as f32
        } else {
            window_size.width as f32 / SCREEN_WIDTH as f32
        };

        let scale = Vector2::<f32> { x: scale, y: scale };
        canvas.set_sampler(Sampler::nearest_clamp());
        canvas.draw(&image, DrawParam::new().scale(scale));

        canvas.finish(ctx)
    }
}

fn setup_audio() -> (Stream, Arc<Mutex<VecDeque<i16>>>) {
    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .expect("no output device available");

    let mut supported_configs_range = device
        .supported_output_configs()
        .expect("error while querying configs");
    let supported_config = supported_configs_range
        .next()
        .expect("no supported config?!")
        .with_sample_rate(SampleRate(44_100));
    let sample_format = supported_config.sample_format();
    let config = supported_config.into();

    let audio_buffer = Arc::new(Mutex::new(VecDeque::<i16>::new()));
    let audio_buffer_closure = audio_buffer.clone();

    let err_fn = |err| eprintln!("an error occurred on the output audio stream: {}", err);
    let write_buffer_fn = move |output_buffer: &mut [f32], _info: &cpal::OutputCallbackInfo| {
        let mut input_buffer = audio_buffer_closure.lock().unwrap();
        let out_len = input_buffer.len().min(output_buffer.len());

        if input_buffer.len() == 0 {
            dbg!("Buffer under run");
        }

        for (i, sample) in input_buffer.drain(..out_len).enumerate() {
            output_buffer[i] = sample as f32 / 25.0
        }
    };

    let stream = match sample_format {
        SampleFormat::F32 => device.build_output_stream(&config, write_buffer_fn, err_fn, None),
        sample_format => panic!("Unsupported sample format '{sample_format}'"),
    }
    .unwrap();

    stream.play().unwrap();

    (stream, audio_buffer)
}

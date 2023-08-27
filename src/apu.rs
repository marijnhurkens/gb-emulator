use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use bitflags::bitflags;
use blip_buf::BlipBuf;

const CLOCK_RATE: u32 = 4_194_304;
const SAMPLE_RATE: u32 = 44_100;
const WAVE_PATTERN: [[i32; 8]; 4] = [
    [-1, -1, -1, -1, 1, -1, -1, -1],
    [-1, -1, -1, -1, 1, 1, -1, -1],
    [-1, -1, 1, 1, 1, 1, -1, -1],
    [1, 1, 1, 1, -1, -1, 1, 1],
];

pub struct Apu {
    audio_buffer: Arc<Mutex<VecDeque<i16>>>,
    channel_status: ChannelStatus,
    channel_square_one: ChannelSquareOne,
}

impl Apu {
    pub fn new() -> Apu {
        Apu {
            audio_buffer: Arc::new(Mutex::new(VecDeque::new())),
            channel_status: ChannelStatus::empty(),
            channel_square_one: ChannelSquareOne::new(),
        }
    }

    pub fn new_with_buffer(audio_buffer: Arc<Mutex<VecDeque<i16>>>) -> Apu {
        Apu {
            audio_buffer,
            channel_status: ChannelStatus::empty(),
            channel_square_one: ChannelSquareOne::new(),
        }
    }

    pub fn step(&mut self) {
        self.channel_square_one.step();
    }

    pub fn write_register(&mut self, reg: u16, value: u8) {
        match reg {
            0xff26 => {
                self.channel_status =
                    ChannelStatus::from_bits(value & ChannelStatus::ALL_ENABLED.bits).unwrap()
            }
            _ => panic!("Unknown audio register"),
        };
    }

    pub fn read_register(&self, reg: u16) -> u8 {
        match reg {
            0xff26 => self.channel_status.bits,
            _ => panic!("Unknown audio register"),
        }
    }

    pub fn output(&mut self) {
        self.mix_buffers();
        self.channel_square_one.time = 0;
    }

    fn mix_buffers(&mut self) {
        self.channel_square_one
            .blip_buf
            .end_frame(self.channel_square_one.time);
        let samples_available = self.channel_square_one.blip_buf.samples_avail() as usize;
        let mut buffer = [0; 2000 + 60];
        self.channel_square_one
            .blip_buf
            .read_samples(&mut buffer, false);

        let mut buffer_lock = self.audio_buffer.lock().unwrap();
        for sample in buffer.iter().take(samples_available) {
            buffer_lock.push_back(*sample);
            buffer_lock.push_back(*sample);
        }
    }
}

struct ChannelSquareOne {
    time: u32,
    timer_div: u8,
    timer: Timer,
    phase: usize,
    volume: i32,
    amplitude: i32,
    pub blip_buf: BlipBuf,
}

impl ChannelSquareOne {
    pub fn new() -> ChannelSquareOne {
        let mut blip = BlipBuf::new(4000 + 10);
        blip.set_rates(CLOCK_RATE as f64, 44_100.0);
        ChannelSquareOne {
            time: 0,
            timer_div: 0,
            timer: Timer::new(),
            phase: 0,
            volume: 1,
            amplitude: 0,
            blip_buf: blip,
        }
    }

    pub fn step(&mut self) {
        self.timer_div += 1;

        let timer_clocked = if self.timer_div == 4 {
            self.timer_div = 0;
            self.timer.step()
        } else {
            false
        };

        if timer_clocked {
            let time_next = self.time + ((2048 - self.timer.period) * 4);

            while self.time < time_next {
                let goal_amp = WAVE_PATTERN[2][self.phase] * self.volume;
                if self.amplitude != goal_amp {
                    let delta = goal_amp - self.amplitude;
                    self.blip_buf.add_delta(self.time, delta);
                    self.amplitude += delta;
                }
                self.time += 1;
            }

            self.phase = (self.phase + 1) % 8;
        }
    }
}

struct Timer {
    period: u32,
    period_div: u32,
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            period: 0x400,
            period_div: 0x400,
        }
    }

    pub fn step(&mut self) -> bool {
        self.period_div += 1;

        if self.period_div == 0x800 {
            self.period_div = self.period;
            return true;
        }

        false
    }
}

bitflags! {
    pub struct ChannelStatus: u8 {
        const ALL_ENABLED = 0b10000000;
        const CHANNEL_4_ENABLED = 0b00001000;
        const CHANNEL_3_ENABLED = 0b00000100;
        const CHANNEL_2_ENABLED = 0b00000010;
        const CHANNEL_1_ENABLED = 0b00000001;
    }

}

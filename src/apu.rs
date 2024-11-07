use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use bitflags::bitflags;
use bitvec::macros::internal::funty::Fundamental;
use blip_buf::BlipBuf;

const CLOCK_RATE: u32 = 4_194_304;
const SAMPLE_RATE: u32 = 44_100;
const FRAME_SEQUENCER_PERIOD: u32 = CLOCK_RATE / 512;
const SAMPLES_PER_FRAME: usize = 500;
const STEPS_PER_FRAME: u32 =
    ((SAMPLES_PER_FRAME as f32 / SAMPLE_RATE as f32) * CLOCK_RATE as f32) as u32;
const WAVE_PATTERN: [[i32; 8]; 4] = [
    [-1, -1, -1, -1, 1, -1, -1, -1],
    [-1, -1, -1, -1, 1, 1, -1, -1],
    [-1, -1, 1, 1, 1, 1, -1, -1],
    [1, 1, 1, 1, -1, -1, 1, 1],
];

pub struct Apu {
    pub audio_buffer: Arc<Mutex<VecDeque<i16>>>,
    master_control: MasterControl,
    master_volume: MasterVolume,
    channel_panning: ChannelPanning,
    channel_square_one: ChannelSquare,
    channel_square_two: ChannelSquare,
    time: u32,
}

impl Apu {
    pub fn new() -> Apu {
        Apu {
            audio_buffer: Arc::new(Mutex::new(VecDeque::new())),
            master_control: MasterControl::empty(),
            master_volume: MasterVolume {
                volume_left: 0,
                volume_right: 0,
            },
            channel_panning: ChannelPanning::empty(),
            channel_square_one: ChannelSquare::new(),
            channel_square_two: ChannelSquare::new(),
            time: 0,
        }
    }

    pub fn new_with_buffer(audio_buffer: Arc<Mutex<VecDeque<i16>>>) -> Apu {
        Apu {
            audio_buffer,
            master_control: MasterControl::empty(),
            master_volume: MasterVolume {
                volume_left: 0,
                volume_right: 0,
            },
            channel_panning: ChannelPanning::empty(),
            channel_square_one: ChannelSquare::new(),
            channel_square_two: ChannelSquare::new(),
            time: 0,
        }
    }

    /// This function runs the audio channels, while also syncing the system to the audio buffer.
    /// This is easier than syncing the PPU to 59.7 fps while producing the samples we need.
    pub fn step(&mut self) {
        self.time += 1;

        if self.time > STEPS_PER_FRAME {
            self.sleep_check();

            self.channel_square_one.run(STEPS_PER_FRAME);
            self.channel_square_two.run(STEPS_PER_FRAME);

            self.time = 0;

            self.mix_buffers();
        }
    }

    pub fn write_register(&mut self, reg: u16, value: u8) {
        match reg {
            0xff10 => {
                self.channel_square_one.sweep_pace = (value & 0b01110000) >> 4;
                self.channel_square_one.sweep_direction = if (value & 0b00001000) >> 4 == 0 {
                    SweepDirection::Up
                } else {
                    SweepDirection::Down
                };
                self.channel_square_one.sweep_individual_step = value & 0b00000111;
            }
            0xff11 => {
                self.channel_square_one.duty = (value & 0b11000000) >> 6;
                self.channel_square_one.length_timer = value & 0b00111111;
            }
            0xff12 => {
                self.channel_square_one.envelope_pace = value & 0b00000111;
                self.channel_square_one.envelope_direction = ((value & 0b00001000) >> 3).as_bool();
                self.channel_square_one.envelope_initial = (value & 0b11110000) >> 4;
            }
            0xff13 => {
                self.channel_square_one.timer.period =
                    (self.channel_square_one.timer.period & 0x700) | (value as u32)
            }
            0xff14 => {
                if value & 0x80 == 0x80 {
                    self.channel_square_one.trigger();
                }

                self.channel_square_one.length_timer_enabled = value & 0x20 == 0x20;

                let period_high = value & 0b00000111;
                self.channel_square_one.timer.period =
                    (self.channel_square_one.timer.period & 0xff) | ((period_high as u32) << 8)
            }
            0xff1a..=0xff1e => (),
            0xff16 => {
                self.channel_square_two.duty = (value & 0b11000000) >> 6;
                self.channel_square_two.length_timer = value & 0b00111111;
            }
            0xff17 => {
                self.channel_square_two.envelope_pace = value & 0b00000111;
                self.channel_square_two.envelope_direction = ((value & 0b00001000) >> 3).as_bool();
                self.channel_square_two.envelope_initial = (value & 0b11110000) >> 4;
            }
            0xff18 => {
                self.channel_square_two.timer.period =
                    (self.channel_square_two.timer.period & 0x700) | (value as u32)
            }
            0xff19 => {
                if value & 0x80 == 0x80 {
                    self.channel_square_two.trigger();
                }

                self.channel_square_two.length_timer_enabled = value & 0x20 == 0x20;

                let period_high = value & 0b00000111;
                self.channel_square_two.timer.period =
                    (self.channel_square_two.timer.period & 0xff) | ((period_high as u32) << 8)
            }
            0xff20..=0xff23 => (),
            0xff24 => {
                self.master_volume.volume_left = (value & 0b01110000) >> 4;
                self.master_volume.volume_right = value & 0b00000111;
            }
            0xff25 => self.channel_panning = ChannelPanning::from_bits(value).unwrap(),
            0xff26 => {
                // only audio enabled is writable
                self.master_control =
                    MasterControl::from_bits(value & MasterControl::AUDIO_ENABLED.bits()).unwrap()
            }
            register => panic!("Write to unknown audio register {:#04X}", register),
        };
    }

    pub fn read_register(&self, reg: u16) -> u8 {
        match reg {
            0xff10 => {
                (self.channel_square_one.sweep_pace << 4)
                    | (if self.channel_square_one.sweep_direction == SweepDirection::Up {
                        0b00000000
                    } else {
                        0b00001000
                    })
                    | (self.channel_square_one.sweep_individual_step)
            }
            0xff11 => {
                (self.channel_square_one.duty << 6)
                    | (self.channel_square_one.length_timer & 0b00111111)
            }
            0xff25 => self.channel_panning.bits(),
            0xff26 => {
                self.master_control.bits()
                    & self.channel_square_one.enabled.as_u8()
                    & self.channel_square_two.enabled.as_u8() << 1
            }
            register => panic!("Read from unknown audio register {:#04X}", register),
        }
    }

    fn mix_buffers(&mut self) {
        if !self.master_control.contains(MasterControl::AUDIO_ENABLED) {
            return;
        }

        let (channel_1_count, channel_1_buffer) = self.channel_square_one.mix_buffer();
        let (channel_2_count, channel_2_buffer) = self.channel_square_two.mix_buffer();
        assert_eq!(channel_1_count, channel_2_count);

        let mut channel_left: [i16; SAMPLES_PER_FRAME + 100] = [0; SAMPLES_PER_FRAME + 100];
        let mut channel_right: [i16; SAMPLES_PER_FRAME + 100] = [0; SAMPLES_PER_FRAME + 100];

        for ((mixed_sample, channel_1_sample), channel_2_sample) in channel_left
            .iter_mut()
            .zip(channel_1_buffer)
            .zip(channel_2_buffer)
        {
            if self
                .channel_panning
                .contains(ChannelPanning::CHANNEL_1_LEFT)
            {
                *mixed_sample += channel_1_sample;
            }

            if self
                .channel_panning
                .contains(ChannelPanning::CHANNEL_2_LEFT)
            {
                *mixed_sample += channel_2_sample;
            }
        }

        for ((mixed_sample, channel_1_sample), channel_2_sample) in channel_right
            .iter_mut()
            .zip(channel_1_buffer)
            .zip(channel_2_buffer)
        {
            if self
                .channel_panning
                .contains(ChannelPanning::CHANNEL_1_RIGHT)
            {
                *mixed_sample += channel_1_sample;
            }

            if self
                .channel_panning
                .contains(ChannelPanning::CHANNEL_2_RIGHT)
            {
                *mixed_sample += channel_2_sample;
            }
        }

        let mut buffer_lock = self.audio_buffer.lock().unwrap();
        for i in 0..channel_1_count.min(channel_2_count) {
            buffer_lock.push_back(channel_left[i]);
            buffer_lock.push_back(channel_right[i]);
        }
    }

    fn sleep_check(&self) {
        // Samples are produces per batch of SAMPLES_PER_FRAME, lets try to keep a buffer of
        // 2 * SAMPLES_PER_FRAME, for 2 channels of audio. If we have this buffer available
        // we should be able to sleep for SAMPLES_PER_FRAME / SAMPLE_RATE seconds.
        while self.audio_buffer.lock().unwrap().len() > SAMPLES_PER_FRAME * 2 * 2 {
            spin_sleep::sleep(Duration::from_secs_f64(
                SAMPLES_PER_FRAME as f64 / SAMPLE_RATE as f64,
            ));
        }
    }
}

struct ChannelSquare {
    enabled: bool,
    timer_div: u8,
    timer: Timer,
    frame_sequencer: FrameSequencer,
    frame_sequencer_div: u32,
    phase: usize,
    volume: u8,
    amplitude: i32,
    duty: u8,
    length_timer: u8,
    length_timer_enabled: bool,
    envelope_initial: u8,
    envelope_direction: bool,
    envelope_pace: u8,
    envelope_div: u8,
    sweep_pace: u8,
    sweep_direction: SweepDirection,
    sweep_individual_step: u8,
    sweep_div: u8,
    pub blip_buf: BlipBuf,
}

impl ChannelSquare {
    pub fn new() -> ChannelSquare {
        let mut blip = BlipBuf::new(SAMPLES_PER_FRAME as u32 * 2 + 100);
        blip.set_rates(CLOCK_RATE as f64, SAMPLE_RATE as f64);

        ChannelSquare {
            enabled: false,
            timer_div: 0,
            timer: Timer::new(),
            frame_sequencer: FrameSequencer::new(),
            frame_sequencer_div: 0,
            phase: 0,
            volume: 1,
            amplitude: 0,
            duty: 0x0,
            length_timer: 0,
            length_timer_enabled: false,
            envelope_initial: 0,
            envelope_direction: false,
            envelope_pace: 0,
            envelope_div: 0,
            sweep_pace: 0,
            sweep_direction: SweepDirection::Up,
            sweep_individual_step: 0,
            sweep_div: 0,
            blip_buf: blip,
        }
    }

    /// Run the audio for $cycles, this means that changes in the audio registers during these
    /// cycles aren't reflected.
    pub fn run(&mut self, cycles: u32) {
        for i in 0..cycles {
            self.timer_div += 1;

            // The period divider is clocked at once per 4 dots (cycles)
            let timer_clocked = if self.timer_div == 4 {
                self.timer_div = 0;
                self.timer.step()
            } else {
                false
            };

            // always write a sample if the timer has clocked, write amplitude 0 when disabled.
            if timer_clocked {
                self.write_sample(i);
            }

            if !self.enabled {
                return;
            }

            self.frame_sequencer_div += 1;

            let frame_sequencer_clocked = if self.frame_sequencer_div == FRAME_SEQUENCER_PERIOD {
                self.frame_sequencer_div = 0;
                self.frame_sequencer.step()
            } else {
                None
            };

            match frame_sequencer_clocked {
                None => {}
                Some(FrameSequencerClock::Length) => {
                    if self.length_timer_enabled {
                        self.length_timer += 1;
                    }
                }
                Some(FrameSequencerClock::LengthSweep) => {
                    if self.length_timer_enabled {
                        self.length_timer += 1;
                    }

                    self.step_sweep();
                }
                Some(FrameSequencerClock::Volume) => self.step_envelope(),
            }

            if self.length_timer == 64 {
                self.enabled = false;
                self.length_timer = 0;
            }
        }
    }

    pub fn mix_buffer(&mut self) -> (usize, [i16; SAMPLES_PER_FRAME + 100]) {
        self.blip_buf.end_frame(STEPS_PER_FRAME);
        let samples_available = self.blip_buf.samples_avail() as usize;
        let mut buffer = [0; SAMPLES_PER_FRAME + 100];
        self.blip_buf.read_samples(&mut buffer, false);

        (samples_available.min(SAMPLES_PER_FRAME + 100), buffer)
    }

    pub fn trigger(&mut self) {
        self.enabled = true;
        self.volume = self.envelope_initial;
        self.envelope_div = 0;
    }

    fn write_sample(&mut self, time: u32) {
        let goal_amp = if self.enabled {
            WAVE_PATTERN[self.duty as usize][self.phase] * self.volume as i32
        } else {
            0
        };

        if self.amplitude != goal_amp {
            let delta = goal_amp - self.amplitude;
            self.blip_buf.add_delta(time, delta);
            self.amplitude += delta;
        }

        self.phase = (self.phase + 1) % 8;
    }

    fn step_envelope(&mut self) {
        if self.envelope_pace == 0 {
            return;
        }

        self.envelope_div += 1;

        if self.envelope_div == self.envelope_pace {
            self.envelope_div = 0;

            if self.envelope_direction && self.volume < 15 {
                self.volume += 1;
            } else if !self.envelope_direction && self.volume > 0 {
                self.volume -= 1;
            }
        }
    }

    /// Called at 128 Hz ticks
    fn step_sweep(&mut self) {
        if self.sweep_pace == 0 {
            return;
        }

        self.sweep_div += 1;

        if self.sweep_div >= self.sweep_pace {
            self.sweep_div = 0;

            let step = self.timer.period / 2u32.pow(self.sweep_individual_step as u32);
            if self.sweep_direction == SweepDirection::Up {
                self.timer.period += step;
            } else {
                self.timer.period -= step;
            }
        }
    }
}

#[derive(Default)]
struct Timer {
    period: u32,
    period_div: u32,
}

/// The timer divider counts up to 0x800 and then resets to the value of period.
///
/// It should be clocked every 4 dots.
impl Timer {
    pub fn new() -> Timer {
        Timer::default()
    }

    pub fn step(&mut self) -> bool {
        self.period_div += 1;

        if self.period_div >= 0x800 {
            self.period_div = self.period;
            return true;
        }

        false
    }

    pub fn period_length(&self) -> u32 {
        0x800 - self.period
    }
}

#[derive(Default)]
struct FrameSequencer {
    step: u8,
}

/// The frame sequences generates clocks for the length, volume and sweep functions
///
/// It should be clocked at 512Hz.
impl FrameSequencer {
    pub fn new() -> FrameSequencer {
        FrameSequencer::default()
    }

    pub fn step(&mut self) -> Option<FrameSequencerClock> {
        self.step = (self.step + 1) % 8;

        if self.step == 7 {
            return Some(FrameSequencerClock::Volume);
        }

        if self.step % 4 == 0 {
            return Some(FrameSequencerClock::Length);
        }

        if self.step >= 2 && (self.step - 2) % 4 == 0 {
            return Some(FrameSequencerClock::LengthSweep);
        }

        None
    }
}

#[derive(Debug, Copy, Clone)]
enum FrameSequencerClock {
    Volume,
    Length,
    LengthSweep,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum SweepDirection {
    Up,
    Down,
}

bitflags! {
    #[derive(Debug)]
    pub struct MasterControl: u8 {
        const AUDIO_ENABLED = 0b10000000;
        const CHANNEL_4_ENABLED = 0b00001000;
        const CHANNEL_3_ENABLED = 0b00000100;
        const CHANNEL_2_ENABLED = 0b00000010;
        const CHANNEL_1_ENABLED = 0b00000001;
    }

    #[derive(Debug)]
    pub struct ChannelPanning: u8 {
        const CHANNEL_4_LEFT =  0b10000000;
        const CHANNEL_3_LEFT =  0b01000000;
        const CHANNEL_2_LEFT =  0b00100000;
        const CHANNEL_1_LEFT =  0b00010000;
        const CHANNEL_4_RIGHT = 0b00001000;
        const CHANNEL_3_RIGHT = 0b00000100;
        const CHANNEL_2_RIGHT = 0b00000010;
        const CHANNEL_1_RIGHT = 0b00000001;
    }

}

struct MasterVolume {
    volume_left: u8,
    volume_right: u8,
}

use std::io::Cursor;

use bitflags::bitflags;

use crate::memory::InterruptFlags;
use crate::SCREEN_BUFFER_SIZE;

#[derive(Debug)]
pub struct Video {
    mode_step: usize,
    pub vram: Cursor<Vec<u8>>,
    pub line: u8,
    pub lyc: u8,
    pub scy: u8,
    pub scx: u8,
    pub mode: VideoMode,
    // lcdc
    pub lcd_control: LcdControl,
    // stat
    pub lcd_status: LcdStatus,
    pub bg_palette: u8,
    pub obj_0_palette: u8,
    pub obj_1_palette: u8,
    pub window_y: u8,
    pub window_x: u8,
    pub bank_select: u8,
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum VideoMode {
    OamRead = 2,
    VramRead = 3,
    HBlank = 0,
    VBlank = 1,
}

bitflags! {
    pub struct LcdStatus: u8 {
        const HBLANK            =  0b00000000;
        const VBLANK            =  0b00000001;
        const OAM               =  0b00000010;
        const VRAM_READ         =  0b00000011;
        const ALL_MODE_FLAGS    =  0b00000011;

        const LYC               =  0b00000100;
        const HBLANK_INTERRUPT  =  0b00001000;
        const VBLANK_INTERRUPT  =  0b00010000;
        const OAM_INTERRUPT     =  0b00100000;
        const LYC_INTERRUPT     =  0b01000000;
    }

    pub struct LcdControl: u8 {
        const LCD_ENABLE =              0b10000000;
        const WINDOW_TILE_MAP =         0b01000000;
        const WINDOW_ENABLE =           0b00100000;
        const WINDOW_BG_ADDRES_MODE =   0b00010000;
        const BG_TILE_MAP =             0b00001000;
        const OBJ_SIZE =                0b00000100;
        const OBJ_ENABLE =              0b00000010;
        const WINDOW_BG_DISPLAY =       0b00000001;
    }
}

impl Video {
    pub fn new() -> Self {
        Self {
            mode: VideoMode::OamRead,
            vram: Cursor::new(vec![0; SCREEN_BUFFER_SIZE]),
            mode_step: 0,
            line: 0x91,
            lyc: 0,
            scy: 0,
            scx: 0,
            lcd_control: LcdControl::empty(),
            lcd_status: LcdStatus::empty(),
            bg_palette: 0,
            obj_0_palette: 0,
            obj_1_palette: 0,
            window_y: 0,
            window_x: 0,
            bank_select: 0,
        }
    }

    pub fn step(&mut self, interrupt_flags: InterruptFlags) -> InterruptFlags {
        let mut interrupt_flags = interrupt_flags;
        self.mode_step += 1;

        if self.line == self.lyc {
            self.lcd_status |= LcdStatus::LYC;
            self.lcd_status |= LcdStatus::LYC_INTERRUPT;
        } else {
            self.lcd_status -= LcdStatus::LYC
        }

        match self.mode {
            VideoMode::OamRead => {
                if self.mode_step >= 80 {
                    self.mode_step = 0;
                    self.mode = VideoMode::VramRead;
                    self.lcd_status -= LcdStatus::ALL_MODE_FLAGS;
                    self.lcd_status |= LcdStatus::VRAM_READ;
                }
            }
            VideoMode::VramRead => {
                self.lcd_status |= LcdStatus::OAM;
                if self.mode_step >= 172 {
                    self.mode_step = 0;
                    self.mode = VideoMode::HBlank;
                    self.lcd_status -= LcdStatus::ALL_MODE_FLAGS;
                    self.lcd_status |= LcdStatus::HBLANK;
                    self.lcd_status |= LcdStatus::HBLANK_INTERRUPT;
                }
            }
            VideoMode::HBlank => {
                if self.mode_step >= 204 {
                    self.mode_step = 0;
                    self.line += 1;

                    if self.line >= 143 {
                        self.mode = VideoMode::VBlank;
                        // set the lcd stat register and set the vblank interrupt
                        self.lcd_status -= LcdStatus::ALL_MODE_FLAGS;
                        self.lcd_status |= LcdStatus::VBLANK;
                        self.lcd_status |= LcdStatus::VBLANK_INTERRUPT;
                        interrupt_flags |= InterruptFlags::VBLANK;
                    } else {
                        self.mode = VideoMode::OamRead;
                        self.lcd_status -= LcdStatus::ALL_MODE_FLAGS;
                        self.lcd_status |= LcdStatus::OAM;
                        self.lcd_status |= LcdStatus::OAM_INTERRUPT;
                    }
                }
            }
            VideoMode::VBlank => {
                if self.mode_step >= 456 {
                    self.mode_step = 0;
                    self.line += 1;

                    if self.line > 153 {
                        self.mode = VideoMode::OamRead;
                        self.line = 0;
                        self.lcd_status -= LcdStatus::ALL_MODE_FLAGS;
                        self.lcd_status |= LcdStatus::OAM;
                        self.lcd_status |= LcdStatus::OAM_INTERRUPT;
                    }
                }
            }
        };

        // check if we need to trigger the STAT interrupt
        if !interrupt_flags.contains(InterruptFlags::LCD_STAT)
            && (self.lcd_status.contains(LcdStatus::HBLANK_INTERRUPT)
                || self.lcd_status.contains(LcdStatus::VBLANK_INTERRUPT)
                || self.lcd_status.contains(LcdStatus::OAM_INTERRUPT)
                || self.lcd_status.contains(LcdStatus::LYC_INTERRUPT))
        {
            interrupt_flags |= InterruptFlags::LCD_STAT;
        }

        return interrupt_flags;
    }
}

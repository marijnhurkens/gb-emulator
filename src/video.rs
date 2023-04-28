use std::io::{Cursor, Read, Write};

use bitflags::bitflags;
use byteorder::{ReadBytesExt, WriteBytesExt};

use crate::memory::InterruptFlags;
use crate::{helpers, SCREEN_BUFFER_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH};

pub const VRAM_START: u16 = 0x8000;
const VRAM_END: u16 = 0xA000;
const VRAM_SIZE: u16 = VRAM_END + 1 - VRAM_START;

const OAM_START: u16 = 0xFE00;
const OAM_END: u16 = 0xFE9F;
const OAM_SIZE: u16 = OAM_END + 1 - OAM_START;

const BACKGROUND_MAP_START: u16 = 0x9800;
const BACKGROUND_MAP_END: u16 = 0x9BFF;
const BACKGROUND_MAP_SIZE: u16 = BACKGROUND_MAP_END + 1 - BACKGROUND_MAP_START;
const WINDOW_MAP_START: u16 = 0x9C00;
const WINDOW_MAP_END: u16 = 0x9FFF;
const WINDOW_MAP_SIZE: u16 = WINDOW_MAP_END + 1 - WINDOW_MAP_START;

#[derive(Debug)]
pub struct Video {
    mode_step: usize,
    pub vram: Cursor<Vec<u8>>,
    pub oam: Cursor<Vec<u8>>,
    pub screen_buffer: Cursor<Vec<u8>>,
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

    pub struct ObjectAttributes: u8 {
        const BG_WINDOW_OVER_OBJ    = 0b10000000;
        const Y_FLIP                = 0b01000000;
        const X_FLIP                = 0b00100000;
        const PALETTE               = 0b00010000;
    }
}

impl Video {
    pub fn new() -> Self {
        Self {
            mode: VideoMode::OamRead,
            vram: Cursor::new(vec![0; VRAM_SIZE as usize]),
            oam: Cursor::new(vec![0; OAM_SIZE as usize]),
            screen_buffer: Cursor::new(vec![0; SCREEN_BUFFER_SIZE]),
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
                        self.draw_line();
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

        interrupt_flags
    }

    pub fn read_screen_buffer(&mut self) -> [u8; SCREEN_BUFFER_SIZE] {
        self.screen_buffer.set_position(0);
        let mut buffer = [0; SCREEN_BUFFER_SIZE];
        self.screen_buffer.read_exact(&mut buffer).unwrap();

        buffer
    }

    pub fn read_byte(&mut self, pos: u16) -> u8 {
        self.vram.set_position((pos - VRAM_START) as u64);
        self.vram.read_u8().unwrap()
    }

    pub fn write_byte(&mut self, pos: u16, byte: u8) {
        self.vram.set_position((pos - VRAM_START) as u64);
        self.vram.write_u8(byte).unwrap();
    }

    pub fn read_byte_oam(&mut self, pos: u16) -> u8 {
        self.oam.set_position((pos - OAM_START) as u64);
        self.oam.read_u8().unwrap()
    }

    pub fn write_byte_oam(&mut self, pos: u16, byte: u8) {
        self.oam.set_position((pos - OAM_START) as u64);
        self.oam.write_u8(byte).unwrap();
    }

    pub fn write_oam_transfer(&mut self, buf: &[u8]) {
        self.oam.set_position(0);
        self.oam.write_all(buf).unwrap();
    }

    fn draw_line(&mut self) {
        if self.lcd_control.contains(LcdControl::WINDOW_BG_DISPLAY) {
            self.draw_background();
            if self.lcd_control.contains(LcdControl::WINDOW_ENABLE) {
                self.draw_window();
            }
        }

        if self.lcd_control.contains(LcdControl::OBJ_ENABLE) {
            self.draw_oam();
        }
    }


    fn draw_oam(&mut self) {
        let mut data = [0; OAM_SIZE as usize];
        self.oam.set_position(0);
        self.oam.read_exact(&mut data).unwrap();

        data.chunks(4).for_each(|object| {
            let y_position = object[0] as i64 - 16;
            let x_position = object[1] as i64 - 8;
            let _attributes = ObjectAttributes::from_bits(object[3]);

            if self.lcd_control.contains(LcdControl::OBJ_SIZE) {
                let tile1 = self.get_tile(object[2] & 0xFE, false);
                let tile2 = self.get_tile(object[2] | 0x01, false);

                self.draw_tile(tile1, x_position, y_position);
                self.draw_tile(tile2, x_position, y_position + 8);
            } else {
                let tile = self.get_tile(object[2], false);

                self.draw_tile(tile, x_position, y_position);
            };
        });
    }

    /// This method draws the background tile map. The background is scrollable and will wrap
    /// around if the visible portion goes outside of the map.
    ///
    /// todo: implement wrapping
    fn draw_background(&mut self) {
        self.vram
            .set_position((BACKGROUND_MAP_START - VRAM_START) as u64);
        let mut tile_map = [0; BACKGROUND_MAP_SIZE as usize];
        self.vram.read_exact(&mut tile_map).unwrap();

        tile_map.iter().enumerate().for_each(|(i, tile_index)| {
            let tile = self.get_tile(
                *tile_index,
                !self.lcd_control.contains(LcdControl::WINDOW_BG_ADDRES_MODE),
            );
            let anchor_x = ((i as u64 % 32) * 8) + self.scx as u64;
            let anchor_y = ((i as u64 / 32) * 8) + self.scy as u64;

            self.draw_tile(tile, anchor_x as i64, anchor_y as i64);
        });
    }

    /// The window is just tile map which can be drawn as a rectangle. It can be positioned
    /// but it will not wrap around. It has no transparency so the use is limited.
    fn draw_window(&mut self) {
        self.vram
            .set_position((WINDOW_MAP_START - VRAM_START) as u64);
        let mut tile_map = [0; WINDOW_MAP_SIZE as usize];
        self.vram.read_exact(&mut tile_map).unwrap();

        tile_map.iter().enumerate().for_each(|(i, tile_index)| {
            let tile = self.get_tile(
                *tile_index,
                !self.lcd_control.contains(LcdControl::WINDOW_BG_ADDRES_MODE),
            );
            let anchor_x = ((i as u64 % 32) * 8) + self.window_x as u64 - 7;
            let anchor_y = (i as u64 / 32) * 8 + self.window_y as u64;

            self.draw_tile(tile, anchor_x as i64, anchor_y as i64);
        });
    }

    fn draw_tiles(&mut self) {
        for x in 0u64..=384 {
            let tile = self.get_tile(x as u8, false);
            let anchor_x = (x % (SCREEN_WIDTH as u64 / 8)) * 8;
            let anchor_y = (x / (SCREEN_WIDTH as u64 / 8)) * 8;
            self.draw_tile(tile, anchor_x as i64, anchor_y as i64);
        }
    }

    /// This method draws a tile on the screen buffer. A tile is 8x8 pixels in size, and each pixel
    /// can have 4 values. The values have different meanings when drawn as the BG/window or when
    /// drawn as an object. Objects will use the value 0 for transparent pixels, whereas the BG and
    /// window have no transparency.
    ///
    /// todo: implement transparency
    fn draw_tile(&mut self, tile: Tile, anchor_x: i64, anchor_y: i64) {
        if anchor_x < -8
            || anchor_y < -8
            || anchor_x > SCREEN_WIDTH as i64
            || anchor_y > SCREEN_HEIGHT as i64
        {
            return;
        }

        tile.chunks(8)
            .skip(anchor_y.min(0).unsigned_abs() as usize)
            .enumerate()
            .for_each(|(i, row)| {
                let y = anchor_y as u64 + i as u64;
                self.screen_buffer
                    .set_position(y * SCREEN_WIDTH as u64 + anchor_x.max(0) as u64);
                let row_scaled: [u8; 8] = row
                    .iter()
                    .skip(anchor_x.min(0).unsigned_abs() as usize)
                    .map(|f| f * 60)
                    .collect::<Vec<u8>>()
                    .try_into()
                    .unwrap();
                let _ = self.screen_buffer.write(&row_scaled).unwrap();
            })
    }

    fn get_tile(&mut self, number: u8, signed: bool) -> Tile {
        let pos = if signed {
            (0x9000 + (helpers::u8_to_i8(number) as u64 * 16)) - VRAM_START as u64
        } else {
            0x8000 + (number as u64 * 16) - VRAM_START as u64
        };

        self.vram.set_position(pos);
        let mut data = [0; 16];
        self.vram.read_exact(&mut data).unwrap();

        let pixels: Tile = data
            .chunks(2)
            .flat_map(|chunk| {
                let mut pixels: [u8; 8] = [0; 8];
                for (i, pixel) in pixels.iter_mut().enumerate() {
                    // get the nth bit from both bytes
                    let least = (chunk[0] >> (7 - i)) & 0x1;
                    let most = (chunk[1] >> (7 - i)) & 0x1;

                    // combine, first bit is the least significant bit and the second the most significant
                    *pixel = least + (most << 1);
                }

                pixels
            })
            .collect::<Vec<u8>>()
            .try_into()
            .unwrap();

        pixels
    }
}

type Tile = [u8; 8 * 8];

use bitflags::bitflags;
use byteorder::{ReadBytesExt, WriteBytesExt};
use std::io::{Cursor, Read, Write};
use std::sync::{Arc, Mutex};

use crate::mmu::InterruptFlags;
use crate::{helpers, ScreenBuffer, SCREEN_BUFFER_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH};

pub const VRAM_START: u16 = 0x8000;
const VRAM_END: u16 = 0xA000;
const VRAM_SIZE: u16 = VRAM_END + 1 - VRAM_START;

const OAM_START: u16 = 0xFE00;
const OAM_END: u16 = 0xFE9F;
const OAM_SIZE: u16 = OAM_END + 1 - OAM_START;

const TILE_MAP_1_START: u16 = 0x9800;
const TILE_MAP_1_END: u16 = 0x9BFF;
const TILE_MAP_2_START: u16 = 0x9C00;
const TILE_MAP_2_END: u16 = 0x9FFF;

#[derive(Debug)]
pub struct Ppu {
    mode_step: usize,
    pub vram: Cursor<Vec<u8>>,
    pub oam: Cursor<Vec<u8>>,
    screen_buffer: Cursor<Vec<u8>>,
    external_screen_buffer: Arc<Mutex<ScreenBuffer>>,
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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum VideoMode {
    OamRead = 2,
    VramRead = 3,
    HBlank = 0,
    VBlank = 1,
}

#[derive(Debug, Copy, Clone)]
enum Palette {
    Obj0,
    Obj1,
    Bg,
}

bitflags! {
    #[derive(PartialEq, Debug, Copy, Clone)]
    pub struct LcdStatus: u8 {
        const HBLANK            =  0b00000000;
        const VBLANK            =  0b00000001;
        const OAM               =  0b00000010;
        const VRAM_READ         =  0b00000011;
        const ALL_MODE_FLAGS    =  0b00000011;

        const LYC               =  0b00000100;

        const HBLANK_INTERRUPT      =  0b00001000;
        const VBLANK_INTERRUPT      =  0b00010000;
        const OAM_INTERRUPT         =  0b00100000;
        const LYC_INTERRUPT         =  0b01000000;
        const ALL_INTERRUPT_FLAGS   =  0b01111000;
    }

    #[derive(PartialEq, Debug, Copy, Clone)]
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

    #[derive(PartialEq, Debug, Copy, Clone)]
    pub struct ObjectAttributes: u8 {
        const BG_WINDOW_OVER_OBJ    = 0b10000000;
        const Y_FLIP                = 0b01000000;
        const X_FLIP                = 0b00100000;
        const PALETTE               = 0b00010000;
    }
}

impl LcdStatus {
    pub fn get_video_mode(self) -> u8 {
        (self & LcdStatus::ALL_MODE_FLAGS).bits()
    }
}

impl Ppu {
    pub fn new(external_screen_buffer: Arc<Mutex<ScreenBuffer>>) -> Self {
        Self {
            mode: VideoMode::OamRead,
            vram: Cursor::new(vec![0; VRAM_SIZE as usize]),
            oam: Cursor::new(vec![0; OAM_SIZE as usize]),
            screen_buffer: Cursor::new(vec![0; SCREEN_BUFFER_SIZE]),
            external_screen_buffer,
            mode_step: 0,
            line: 0x91,
            lyc: 0,
            scy: 0,
            scx: 0,
            lcd_control: LcdControl::from_bits(0x91).unwrap(),
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
        if !self.lcd_control.contains(LcdControl::LCD_ENABLE) {
            return interrupt_flags;
        }

        let mut interrupt_flags = interrupt_flags;
        // keep track of the various interrupt which can trigger a STAT interrupt
        let mut stat_interrupt = LcdStatus::empty();

        self.mode_step += 1;

        match self.mode {
            VideoMode::OamRead => {
                if self.mode_step >= 80 {
                    self.mode_step = 0;
                    self.mode = VideoMode::VramRead;
                    self.lcd_status &= !LcdStatus::ALL_MODE_FLAGS;
                    self.lcd_status |= LcdStatus::VRAM_READ;
                }
            }
            VideoMode::VramRead => {
                if self.mode_step >= 172 {
                    self.draw_line();
                    self.mode_step = 0;
                    self.mode = VideoMode::HBlank;
                    self.lcd_status &= !LcdStatus::ALL_MODE_FLAGS;
                    self.lcd_status |= LcdStatus::HBLANK;
                    stat_interrupt |= LcdStatus::HBLANK_INTERRUPT;
                }
            }
            VideoMode::HBlank => {
                if self.mode_step >= 204 {
                    self.mode_step = 0;
                    self.line += 1;

                    if self.line >= 143 {
                        self.mode = VideoMode::VBlank;
                        // set the lcd stat register and set the vblank interrupt
                        self.lcd_status &= !LcdStatus::ALL_MODE_FLAGS;
                        self.lcd_status |= LcdStatus::VBLANK;
                        stat_interrupt |= LcdStatus::VBLANK_INTERRUPT;
                        interrupt_flags |= InterruptFlags::VBLANK;
                    } else {
                        self.mode = VideoMode::OamRead;
                        self.lcd_status &= !LcdStatus::ALL_MODE_FLAGS;
                        self.lcd_status |= LcdStatus::OAM;
                        stat_interrupt |= LcdStatus::OAM_INTERRUPT;
                    }
                }
            }
            VideoMode::VBlank => {
                if self.mode_step >= 456 {
                    self.mode_step = 0;
                    self.line += 1;

                    if self.line > 153 {
                        self.flip_screen_buffer();
                        self.mode = VideoMode::OamRead;
                        self.line = 0;
                        self.lcd_status &= !LcdStatus::ALL_MODE_FLAGS;
                        self.lcd_status |= LcdStatus::OAM;
                        stat_interrupt |= LcdStatus::OAM_INTERRUPT;
                    }
                }
            }
        };

        if self.line == self.lyc {
            self.lcd_status |= LcdStatus::LYC;
            stat_interrupt |= LcdStatus::LYC_INTERRUPT;
        } else {
            self.lcd_status &= !LcdStatus::LYC
        }

        // check if we need to trigger the STAT interrupt, this is only the case if any of the modes
        // or LYC=LY have been triggered & the corresponding bit in LCD status is set
        if !(stat_interrupt & self.lcd_status).is_empty() {
            interrupt_flags |= InterruptFlags::LCD_STAT;
            // self.lcd_status -= LcdStatus::ALL_INTERRUPT_FLAGS;
        }

        interrupt_flags
    }

    fn read_screen_buffer(&mut self) -> [u8; SCREEN_BUFFER_SIZE] {
        self.screen_buffer.set_position(0);
        let mut buffer = [0; SCREEN_BUFFER_SIZE];
        self.screen_buffer.read_exact(&mut buffer).unwrap();

        buffer
    }

    fn flip_screen_buffer(&mut self) {
        let buf = self.read_screen_buffer();
        let mut guard = self.external_screen_buffer.lock().unwrap();
        *guard = buf;

        drop(guard);
    }

    pub fn read_vram(&mut self) -> [u8; SCREEN_BUFFER_SIZE] {
        self.vram.set_position(0);
        let mut buffer = [0; SCREEN_BUFFER_SIZE];

        let mut tiles = vec![];
        for i in 0u32..=256 {
            tiles.push(self.get_tile(i as u8, false));
        }

        for i in 0u32..=128 {
            tiles.push(self.get_tile(i as u8, true));
        }

        for (i, tile) in tiles.iter().enumerate() {
            let pos_x = (i * 8) % SCREEN_WIDTH as usize;
            let pos_y = (i * 8 / SCREEN_WIDTH as usize) * 8;

            let corner_pos = pos_x + pos_y * SCREEN_WIDTH as usize;

            for (index, pixel) in tile.iter().enumerate() {
                let pos = corner_pos + (index % 8) + (index / 8 * SCREEN_WIDTH as usize);
                if pos >= buffer.len() {
                    continue;
                }
                buffer[pos] = self.index_to_color(*pixel, Palette::Obj0)
            }
        }

        buffer
    }

    pub fn read_byte(&mut self, pos: u16) -> u8 {
        if self.lcd_control.contains(LcdControl::LCD_ENABLE) && self.mode == VideoMode::VramRead {
            dbg!("illegal vram read");
            return 0xFF;
        }
        self.vram.set_position((pos - VRAM_START) as u64);
        self.vram.read_u8().unwrap()
    }

    pub fn write_byte(&mut self, pos: u16, byte: u8) {
        if self.lcd_control.contains(LcdControl::LCD_ENABLE) && self.mode == VideoMode::VramRead {
            dbg!("illegal vram write");
            return;
        }

        self.vram.set_position((pos - VRAM_START) as u64);
        self.vram.write_u8(byte).unwrap();
    }

    pub fn read_byte_oam(&mut self, pos: u16) -> u8 {
        if self.lcd_control.contains(LcdControl::LCD_ENABLE)
            && (self.mode == VideoMode::OamRead || self.mode == VideoMode::VramRead)
        {
            dbg!("illegal oam read");
            return 0xFF;
        }
        self.oam.set_position((pos - OAM_START) as u64);
        self.oam.read_u8().unwrap()
    }

    pub fn write_byte_oam(&mut self, pos: u16, byte: u8) {
        if self.lcd_control.contains(LcdControl::LCD_ENABLE)
            && (self.mode == VideoMode::OamRead || self.mode == VideoMode::VramRead)
        {
            dbg!("illegal oam write");
            return;
        }
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

        let mut objects: Vec<_> = data
            .chunks(4)
            .map(|object| {
                (
                    object[0] as i64 - 16,
                    object[1] as i64 - 8,
                    object[2],
                    ObjectAttributes::from_bits(object[3]).unwrap(),
                )
            })
            .filter(|(y_pos, _, _, _)| {
                if self.lcd_control.contains(LcdControl::OBJ_SIZE) {
                    y_pos + 16 > self.line as i64 && *y_pos <= self.line as i64
                } else {
                    y_pos + 8 > self.line as i64 && *y_pos <= self.line as i64
                }
            })
            .take(10)
            .collect();

        // Objects with the lowest x coord have the highest priority.
        // This means we should draw them last.
        objects.sort_by(|a, b| b.1.cmp(&a.1));

        objects
            .into_iter()
            .for_each(|(y_pos, x_pos, index, attributes)| {
                if self.lcd_control.contains(LcdControl::OBJ_SIZE) {
                    if y_pos + 8 > self.line as i64 && y_pos <= self.line as i64 {
                        let tile1 = self.get_tile(index & 0xFE, false);
                        self.draw_tile(tile1, x_pos, y_pos, Some(attributes));
                    } else if y_pos + 16 > self.line as i64 && y_pos + 8 <= self.line as i64 {
                        let tile2 = self.get_tile(index | 0x01, false);
                        self.draw_tile(tile2, x_pos, y_pos + 8, Some(attributes));
                    }
                } else if y_pos + 8 > self.line as i64 && y_pos <= self.line as i64 {
                    let tile = self.get_tile(index, false);
                    self.draw_tile(tile, x_pos, y_pos, Some(attributes));
                };
            });
    }

    /// This method draws the background tile map. The background is scrollable and will wrap
    /// around if the visible portion goes outside of the map.
    ///
    /// todo: implement wrapping
    fn draw_background(&mut self) {
        let tile_map_row = (self.line as i64 + self.scy as i64) / 8;
        let tile_map_row = if tile_map_row < 0 {
            (32 - tile_map_row) as u64
        } else {
            tile_map_row as u64
        };

        let tile_map_start = if self.lcd_control.contains(LcdControl::BG_TILE_MAP) {
            TILE_MAP_2_START
        } else {
            TILE_MAP_1_START
        };
        self.vram
            .set_position((tile_map_start - VRAM_START) as u64 + tile_map_row * 32);
        let mut tile_map = [0; 32];
        self.vram.read_exact(&mut tile_map).unwrap();

        tile_map.iter().enumerate().for_each(|(i, tile_index)| {
            let tile = self.get_tile(
                *tile_index,
                !self.lcd_control.contains(LcdControl::WINDOW_BG_ADDRES_MODE),
            );

            let anchor_x = ((i as i64 % 32) * 8) - self.scx as i64;
            let anchor_y = tile_map_row as i64 * 8 - self.scy as i64;

            self.draw_tile(tile, anchor_x, anchor_y, None);

            if self.scx > 255 - SCREEN_WIDTH as u8 {
                self.draw_tile(tile, anchor_x + 256, anchor_y, None);
            }
        });
    }

    /// The window is just tile map which can be drawn as a rectangle. It can be positioned
    /// but it will not wrap around. It has no transparency so the use is limited.
    fn draw_window(&mut self) {
        let tile_map_row = (self.line as i64 - self.window_y as i64) / 8;
        if tile_map_row < 0 {
            return;
        }
        self.vram
            .set_position((TILE_MAP_2_START - VRAM_START) as u64 + tile_map_row as u64 * 32);
        let mut tile_map = [0; 32];
        self.vram.read_exact(&mut tile_map).unwrap();

        tile_map.iter().enumerate().for_each(|(i, tile_index)| {
            let tile = self.get_tile(
                *tile_index,
                !self.lcd_control.contains(LcdControl::WINDOW_BG_ADDRES_MODE),
            );
            let anchor_x = ((i as i64 % 32) * 8) + self.window_x as i64 - 7;
            let anchor_y = tile_map_row * 8 + self.window_y as i64;

            self.draw_tile(tile, anchor_x, anchor_y, None);
        });
    }

    fn draw_tiles(&mut self) {
        for x in 0u64..=384 {
            let tile = self.get_tile(x as u8, false);
            let anchor_x = (x % (SCREEN_WIDTH as u64 / 8)) * 8;
            let anchor_y = (x / (SCREEN_WIDTH as u64 / 8)) * 8;
            self.draw_tile(tile, anchor_x as i64, anchor_y as i64, None);
        }
    }

    /// This method draws a tile on the screen buffer. A tile is 8x8 pixels in size, and each pixel
    /// can have 4 values. The values have different meanings when drawn as the BG/window or when
    /// drawn as an object. Objects will use the value 0 for transparent pixels, whereas the BG and
    /// window have no transparency.
    ///
    /// todo: implement transparency
    fn draw_tile(
        &mut self,
        tile: Tile,
        anchor_x: i64,
        anchor_y: i64,
        object_attributes: Option<ObjectAttributes>,
    ) {
        if anchor_x < -8
            || anchor_y < -8
            || anchor_x > SCREEN_WIDTH as i64
            || anchor_y > SCREEN_HEIGHT as i64
        {
            return;
        }

        let x_flip = if let Some(attributes) = object_attributes {
            attributes.contains(ObjectAttributes::X_FLIP)
        } else {
            false
        };

        let mut tile_line_index = self.line as i64 - anchor_y;

        if let Some(attributes) = object_attributes {
            if attributes.contains(ObjectAttributes::Y_FLIP) {
                tile_line_index = 8 - tile_line_index;
            }
        }

        if !(0..8).contains(&tile_line_index) {
            //panic!("should not happen");
            return;
        }

        let tile_line_index = tile_line_index as usize;

        let line_clip_start = 0; //anchor_x.min(0).unsigned_abs() as usize;
        let line_clip_end = (anchor_x + 8 - SCREEN_WIDTH as i64).max(0) as usize;
        let start_pos = tile_line_index * 8 + line_clip_start;
        let end_pos = start_pos + 8 - line_clip_end - line_clip_start;

        self.screen_buffer
            .set_position(self.line as u64 * SCREEN_WIDTH as u64 + anchor_x.max(0) as u64);

        let palette = if let Some(attributes) = object_attributes {
            if attributes.contains(ObjectAttributes::PALETTE) {
                Palette::Obj1
            } else {
                Palette::Obj0
            }
        } else {
            Palette::Bg
        };

        // note: after mapping the original color index and the mapped color are put together in a tuple
        let row_scaled: Vec<(u8, u8)> = if x_flip {
            tile[start_pos..end_pos]
                .iter()
                .rev()
                .skip(anchor_x.min(0).unsigned_abs() as usize)
                .map(|x| (*x, self.index_to_color(*x, palette)))
                .collect()
        } else {
            tile[start_pos..end_pos]
                .iter()
                .skip(anchor_x.min(0).unsigned_abs() as usize)
                .map(|x| (*x, self.index_to_color(*x, palette)))
                .collect()
        };

        // objects are transparent
        if object_attributes.is_none() {
            let _ = self
                .screen_buffer
                .write(&row_scaled.iter().map(|x| x.1).collect::<Vec<u8>>())
                .unwrap();
        } else {
            row_scaled.iter().for_each(|x| {
                if x.0 == 0 {
                    self.screen_buffer
                        .set_position(self.screen_buffer.position() + 1);
                } else {
                    self.screen_buffer.write_u8(x.1).unwrap();
                }
            })
        }
    }

    fn get_tile(&mut self, number: u8, signed: bool) -> Tile {
        let pos = if signed {
            ((0x9000 + (helpers::u8_to_i8(number) as i64 * 16)) - VRAM_START as i64) as u64
        } else {
            0x8000 + (number as u64 * 16) - VRAM_START as u64
        };

        self.vram.set_position(pos);
        let mut data = [0; 16];
        self.vram.read_exact(&mut data).unwrap();

        let mut tile: Tile = [0; 64];

        for (chunk_index, chunk) in data.chunks(2).enumerate() {
            let mut pixels: [u8; 8] = [0; 8];
            for (i, pixel) in pixels.iter_mut().enumerate() {
                // get the nth bit from both bytes
                let least = (chunk[0] >> (7 - i)) & 0x1;
                let most = (chunk[1] >> (7 - i)) & 0x1;

                // combine, first bit is the least significant bit and the second the most significant
                *pixel = least + (most << 1);
            }

            for (i, pixel) in pixels.iter().enumerate() {
                tile[chunk_index * 8 + i] = *pixel
            }
        }

        tile
    }

    fn index_to_color(&self, index: u8, palette: Palette) -> u8 {
        let mask = 0b00000011;

        let palette_data = match palette {
            Palette::Obj0 => self.obj_1_palette,
            Palette::Obj1 => self.obj_1_palette,
            Palette::Bg => self.bg_palette,
        };

        // bit 0 and 1 of a palette maps the color for index 0
        // bit 2 and 3 maps the color for index 1, etc.
        let color = (palette_data & (mask << (index * 2))) >> (index * 2);

        match color {
            0 => 240,
            1 => 100,
            2 => 50,
            3 => 0,
            _ => panic!("Unknown color"),
        }
    }
}

type Tile = [u8; 8 * 8];

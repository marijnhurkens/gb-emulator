
#[derive(Debug)]
pub struct Cartridge {
    pub header: CartridgeHeader,
    pub data: Vec<u8>,
}

#[derive(Debug)]
pub struct CartridgeHeader {
    pub title: String,
    pub cartridge_type: u8,
    pub licensee_code: u8,
    pub rom_size: u8,
    pub ram_size: u8,
}

impl Cartridge {
    pub fn load_rom(data: Vec<u8>) -> Self {
        verify_header(&data).unwrap();

        let header = load_header(&data);

        Cartridge { header, data }
    }
}

fn load_header(data: &[u8]) -> CartridgeHeader {
    let title = &data[0x0134..=0x0143];
    let cartridge_type = data[0x0147];
    let rom_size = data[0x0148];
    let ram_size = data[0x0149];
    let licensee_code = data[0x014B];

    CartridgeHeader {
        title: String::from_utf8(title.to_vec()).unwrap(),
        cartridge_type,
        rom_size,
        ram_size,
        licensee_code,
    }
}

fn verify_header(data: &[u8]) -> Result<(), &str> {
    let checksum = data[0x014D];
    println!("Verifying header checksum..");
    let mut x: u8 = 0;

    for item in data.iter().take(0x014C + 1).skip(0x0134) {
        x = x.overflowing_sub(*item).0.overflowing_sub(1).0;
    }

    if x == checksum {
        Ok(())
    } else {
        Err("Header checksum not valid.")
    }
}

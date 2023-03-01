pub fn signed_add(a: u16, b: u8) -> u16 {
    let b_signed: i8 = unsafe { std::mem::transmute(b) };
    if b_signed >= 0 {
        a + b_signed.unsigned_abs() as u16
    } else {
        a - b_signed.unsigned_abs() as u16
    }
}

pub fn u8_to_i8(a: u8) -> i8 {
    unsafe { std::mem::transmute(a) }
}

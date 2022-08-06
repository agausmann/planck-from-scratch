use usbd_hid::descriptor::gen_hid_descriptor;

use usbd_hid::descriptor::generator_prelude::*;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xe7) = {
            #[packed_bits 232] #[item_settings data,variable,absolute] keys=input;
        }
    }
)]
pub struct NkroKeyboardReport {
    keys: [u8; 29],
}

impl NkroKeyboardReport {
    pub const fn new() -> Self {
        Self { keys: [0; 29] }
    }

    pub fn press(&mut self, index: u8) {
        self.keys[index as usize / 8] |= 1 << (index % 8);
    }

    pub fn release(&mut self, index: u8) {
        self.keys[index as usize / 8] &= !(1 << (index % 8));
    }

    pub fn clear_all_but_mods(&mut self) {
        // excluding the last byte (codes 0xe0..=0xe7)
        self.keys[..28].fill(0);
    }
}

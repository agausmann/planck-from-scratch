#![no_std]
#![no_main]
#![feature(abi_avr_interrupt, asm_experimental_arch)]

mod nkro;

use core::mem::MaybeUninit;

use atmega_hal::{
    pins,
    port::{
        mode::{Input, Output, PullUp},
        Pin,
    },
    Peripherals,
};
use atmega_usbd::UsbBus;
use avr_device::{asm::sleep, entry, interrupt};
use avr_std_stub as _;
use nkro::NkroKeyboardReport;
use polybius::keycode::{qmk::*, Keycode, LayerAction};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{descriptor::SerializedDescriptor, hid_class::HIDClass};

const LAYER_LOWER: u8 = 1;
const LAYER_RAISE: u8 = 2;

const MO_LOWR: Keycode = MO(LAYER_LOWER);
const MO_RAIS: Keycode = MO(LAYER_RAISE);

#[rustfmt::skip]
static LAYERS: [[[Keycode; 12]; 4]; 3] = [
    // 0: Default/Base
    [
        [KC_TAB , KC_Q   , KC_W   , KC_E   , KC_R   , KC_T   , KC_Y   , KC_U   , KC_I   , KC_O   , KC_P   , KC_BSPC],
        [KC_CLCK, KC_A   , KC_S   , KC_D   , KC_F   , KC_G   , KC_H   , KC_J   , KC_K   , KC_L   , KC_SCLN, KC_QUOT],
        [KC_LSFT, KC_Z   , KC_X   , KC_C   , KC_V   , KC_B   , KC_N   , KC_M   , KC_COMM, KC_DOT , KC_SLSH, KC_RSFT],
        [KC_LCTL, KC_LGUI, KC_LALT, XXXXXXX, MO_LOWR, KC_ENT , KC_SPC , MO_RAIS, XXXXXXX, KC_RALT, KC_RGUI, KC_RCTL],
    ],
    // 1: Lower
    [
        [KC_ESC , KC_F1  , KC_F2  , KC_F3  , KC_F4  , _______, KC_HOME, KC_PGDN, KC_PGUP, KC_END , KC_INS , _______],
        [_______, KC_F5  , KC_F6  , KC_F7  , KC_F8  , _______, KC_LEFT, KC_DOWN, KC_UP  , KC_RGHT, KC_DEL , _______],
        [_______, KC_F9  , KC_F10 , KC_F11 , KC_F12 , _______, _______, KC_PAUS, KC_PSCR, KC_SLCK, _______, _______],
        [_______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______],
    ],
    // 2: Raise
    [
        [KC_GRV , KC_1   , KC_2   , KC_3   , KC_4   , KC_5   , KC_6   , KC_7   , KC_8   , KC_9   , KC_0   , _______],
        [_______, _______, _______, _______, _______, _______, _______, KC_MINS, KC_EQL , KC_LBRC, KC_RBRC, KC_BSLS],
        [_______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______],
        [_______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______],
    ],
];

struct UsbContext {
    device: UsbDevice<'static, UsbBus>,
    hid: HIDClass<'static, UsbBus>,
}

impl UsbContext {
    fn poll(&mut self, state: &mut UsbState) {
        self.device.poll(&mut [&mut self.hid]);
        if !state.sent && self.hid.push_input(&state.report).is_ok() {
            state.sent = true;
        }
    }
}

#[derive(Clone, Copy)]
struct UsbState {
    report: NkroKeyboardReport,
    sent: bool,
}

impl UsbState {
    const fn new() -> Self {
        Self {
            report: NkroKeyboardReport::new(),
            sent: true,
        }
    }
}

// Resources sent to the USB interrupt contexts.
static mut USB_CTX: MaybeUninit<UsbContext> = MaybeUninit::uninit();

// State that is shared with USB interrupts (e.g. reports).
static mut USB_STATE: UsbState = UsbState::new();

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = pins!(dp);

    // Disable JTAG functionality to gain control of pins PF4-PF7.
    // This procedure has tight timing requirements (4 cycles between writes)
    // which can't be guaranteed by the codegen/linker with the safe code:
    // dp.JTAG.mcucr.modify(|_, w| w.jtd().set_bit());
    // dp.JTAG.mcucr.modify(|_, w| w.jtd().set_bit());
    #[cfg(target_arch = "avr")]
    unsafe {
        core::arch::asm!(
            "in {tmp}, 0x35",
            "ori {tmp}, 0x80",
            "out 0x35, {tmp}",
            "out 0x35, {tmp}",
            tmp = out(reg_upper) _,
        );
    };

    let pll = dp.PLL;

    // Configure PLL -
    // Planck has 16MHz external crystal
    pll.pllcsr.write(|w| w.pindiv().set_bit());
    pll.pllfrq
        .write(|w| w.pdiv().mhz96().plltm().factor_15().pllusb().set_bit());

    pll.pllcsr.modify(|_, w| w.plle().set_bit());
    while pll.pllcsr.read().plock().bit_is_clear() {}

    let mut rows: [Pin<Output>; 4] = [
        pins.pd0.into_output_high().downgrade(),
        pins.pd5.into_output_high().downgrade(),
        pins.pb5.into_output_high().downgrade(),
        pins.pb6.into_output_high().downgrade(),
    ];
    let columns: [Pin<Input<PullUp>>; 12] = [
        pins.pf1.into_pull_up_input().downgrade(),
        pins.pf0.into_pull_up_input().downgrade(),
        pins.pb0.into_pull_up_input().downgrade(),
        pins.pc7.into_pull_up_input().downgrade(),
        pins.pf4.into_pull_up_input().downgrade(),
        pins.pf5.into_pull_up_input().downgrade(),
        pins.pf6.into_pull_up_input().downgrade(),
        pins.pf7.into_pull_up_input().downgrade(),
        pins.pd4.into_pull_up_input().downgrade(),
        pins.pd6.into_pull_up_input().downgrade(),
        pins.pb4.into_pull_up_input().downgrade(),
        pins.pd7.into_pull_up_input().downgrade(),
    ];
    let mut layer_mask = 1u8;
    let mut pressed_keys = [0u16; 4];

    let bus = {
        static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit();
        unsafe { USB_BUS.write(UsbBus::new(dp.USB_DEVICE)) }
    };

    let hid = HIDClass::new(bus, NkroKeyboardReport::desc(), 1);
    let usb_device = UsbDeviceBuilder::new(bus, UsbVidPid(0x03a8, 0xae01))
        .manufacturer("OLKB")
        .product("Planck")
        .device_release(0x0002)
        .build();

    unsafe {
        USB_CTX.write(UsbContext {
            device: usb_device,
            hid,
        });
    }

    unsafe { interrupt::enable() };
    loop {
        sleep();
        let mut state = interrupt::free(|_cs| unsafe { USB_STATE });

        if state.sent {
            let mut changed = false;
            for (i, row) in rows.iter_mut().enumerate() {
                row.set_low();
                for (j, col) in columns.iter().enumerate() {
                    let prev_pressed = (pressed_keys[i] & (1 << j)) != 0;
                    let pressed = col.is_low();

                    if prev_pressed != pressed {
                        let keycode = LAYERS
                            .iter()
                            .enumerate()
                            .rev()
                            .filter(|(k, _layer)| (layer_mask & (1 << k)) != 0)
                            .map(|(_k, layer)| layer[i][j])
                            .find(|kc| *kc != KC_TRNS)
                            .unwrap_or(KC_NO);
                        match keycode {
                            Keycode::Hid(hid_keycode) => {
                                if pressed {
                                    state.report.press(hid_keycode as u8);
                                } else {
                                    state.report.release(hid_keycode as u8);
                                }
                            }
                            Keycode::Layer(layer_keycode) => {
                                match layer_keycode.action() {
                                    LayerAction::Momentary => {
                                        if pressed {
                                            layer_mask |= 1 << layer_keycode.layer();
                                        } else {
                                            layer_mask &= !(1 << layer_keycode.layer());
                                        }
                                        state.report.clear_all_but_mods();
                                    }
                                    LayerAction::Toggle => {
                                        if pressed {
                                            layer_mask ^= 1 << layer_keycode.layer();
                                            state.report.clear_all_but_mods();
                                        }
                                    }
                                    LayerAction::Oneshot => {} //TODO
                                    LayerAction::To => {}      //TODO
                                }
                            }
                            _ => {}
                        }
                        changed = true;
                    }

                    if pressed {
                        pressed_keys[i] |= 1 << j;
                    } else {
                        pressed_keys[i] &= !(1 << j);
                    }
                }
                row.set_high();
            }
            if changed {
                state.sent = false;
                interrupt::free(|_cs| unsafe {
                    USB_STATE = state;
                })
            }
        }
    }
}

#[interrupt(atmega32u4)]
fn USB_GEN() {
    let ctx = unsafe { USB_CTX.assume_init_mut() };
    let state = unsafe { &mut USB_STATE };
    ctx.poll(state);
}

#[interrupt(atmega32u4)]
fn USB_COM() {
    let ctx = unsafe { USB_CTX.assume_init_mut() };
    let state = unsafe { &mut USB_STATE };
    ctx.poll(state);
}

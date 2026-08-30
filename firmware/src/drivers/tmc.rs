//! TMC2240 (bench) / TMC5160 (HIL) stepper driver over SPI2 — SPECS §7.2,
//! ARCHITECTURE §3.4. Blocking SPI, owned by the director task on core 1.
//!
//! The register subset used here (GCONF, GSTAT, IOIN, IHOLD_IRUN,
//! TPOWERDOWN, TPWMTHRS, GLOBALSCALER, CHOPCONF, DRV_STATUS) is identical on
//! the 2240 and 5160, so this driver serves both chips unchanged.
//!
//! SPI protocol: 40-bit datagrams (8-bit address + 32-bit data), MSB first,
//! mode 3 (CPOL=1, CPHA=1), CS asserted per datagram (esp-hal blocking SPI
//! auto-CS per transfer does exactly this). Confirmed against a real
//! TMC2240 on the bench (IOIN version byte 0x40).
//!
//! **Read quirk** (cost a long bring-up session — see decision log #19): a
//! register read's data is *not* returned in the same datagram. The chip
//! buffers it and returns it in the *following* datagram. `read_reg` sends
//! the request, then a follow-up transfer, and returns the follow-up's data.
//!
//! ENN is active-LOW on the chip (low = enabled). The firmware holds ENN
//! high (disabled/freewheel) until the registers are configured, and drives
//! it high again on any safe state. Note SPECS §7.2 says "EN low, freewheel"
//! — that contradicts the TMC ENN polarity; this driver follows the chip.

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{GPIO12, GPIO13, GPIO14, GPIO21, GPIO23, GPIO32, SPI2};
use esp_hal::spi::master::{Config, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::Blocking;
use log::info;

// Register addresses (TMC2240/5160 family).
mod reg {
    pub const GCONF: u8 = 0x00;
    pub const GSTAT: u8 = 0x01;
    pub const IOIN: u8 = 0x04;
    pub const GLOBALSCALER: u8 = 0x0B;
    pub const IHOLD_IRUN: u8 = 0x10;
    pub const TPOWERDOWN: u8 = 0x11;
    pub const TPWMTHRS: u8 = 0x13;
    pub const CHOPCONF: u8 = 0x6C;
    pub const DRV_STATUS: u8 = 0x6F;
}

/// CHOPCONF: SpreadCycle (chm=0) — required; StealthChop cannot sustain the
/// 21 kHz+ step rate of 24 fps pulldown. toff=3, hstrt=5, hend=3, TBL=2.
const CHOPCONF_SPREADCYCLE: u32 = 0x0001_01D3;

/// IHOLD=8, IRUN=16, IHOLDDELAY=6. With GLOBALSCALER=128 and Rsense=0.1 Ω
/// this is ≈0.4 A hold / ≈0.8 A run — conservative for a 1 A NEMA17.
const IHOLD_IRUN_VALUE: u32 = 0x0006_1008;

/// GLOBALSCALER: half-scale current (bench start value).
const GLOBALSCALER_VALUE: u32 = 128;

/// TPOWERDOWN: hold current 100 ms after motion stops.
const TPOWERDOWN_VALUE: u32 = 10;

/// DRV_STATUS hard-fault bits: OT(26) | OTPW(27). Deliberately excludes
/// S2GA(28)/S2GB(29)/OLA(30)/OLB(31): on the bench these false-trip at
/// standstill, at the reduced IHOLD current, and at creep speeds (TMC app
/// notes; confirmed on the bench with no wiring fault). They are reported
/// as log-only advisories instead. Revisit for HIL with final motor wiring.
pub const HARD_FAULT_MASK: u32 = 0x0C00_0000;

/// Advisory bits: S2GB(29) S2GA(28) OLB(31) OLA(30) — see HARD_FAULT_MASK.
pub const ADVISORY_MASK: u32 = 0xF000_0000;

/// IOIN version field (bits 31:24): 0x40 = TMC2240, 0x30 = TMC5160.
const IOIN_VERSION_MASK: u32 = 0xFF00_0000;

pub struct Tmc {
    spi: Spi<'static, Blocking>,
    en: Output<'static>,
    dir: Output<'static>,
    /// Level on DIR that corresponds to film-forward (motor wiring may
    /// differ — flip this constant if the bench motor runs backwards).
    pub forward_level: Level,
    pub enabled: bool,
}

impl Tmc {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        spi2: SPI2<'static>,
        cs: GPIO21<'static>,
        mosi: GPIO23<'static>,
        sck: GPIO12<'static>,
        miso: GPIO13<'static>,
        en: GPIO14<'static>,
        dir: GPIO32<'static>,
    ) -> Self {
        let config = Config::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_3);
        let spi = Spi::new(spi2, config)
            .expect("spi2 init")
            .with_sck(sck)
            .with_mosi(mosi)
            .with_miso(miso)
            .with_cs(cs);

        // ENN high = disabled: the fail-safe power-on state.
        let en = Output::new(en, Level::High, OutputConfig::default());
        let dir = Output::new(dir, Level::High, OutputConfig::default());

        Self {
            spi,
            en,
            dir,
            forward_level: Level::High,
            enabled: false,
        }
    }

    /// Configure the chip and enable the outputs. Returns the IOIN register
    /// for the boot self-test.
    pub fn init(&mut self) -> u32 {
        self.en.set_high(); // hold disabled while configuring

        self.write_reg(reg::GCONF, 0x40); // pdn_disable: keep µstep position
        self.write_reg(reg::CHOPCONF, CHOPCONF_SPREADCYCLE);
        self.write_reg(reg::IHOLD_IRUN, IHOLD_IRUN_VALUE);
        self.write_reg(reg::TPOWERDOWN, TPOWERDOWN_VALUE);
        self.write_reg(reg::TPWMTHRS, 0); // SpreadCycle at all velocities
        self.write_reg(reg::GLOBALSCALER, GLOBALSCALER_VALUE);

        let ioin = self.read_reg(reg::IOIN);
        let version = (ioin & IOIN_VERSION_MASK) >> 24;
        info!(
            "tmc: IOIN={ioin:#010x} (version {version:#04x}){}",
            if version == 0x40 || version == 0x30 { " (comms ok)" } else { " (NO COMMS?)" }
        );

        self.en.set_low(); // enable
        self.enabled = true;
        ioin
    }

    /// Film direction for the next/current job. `true` = forward.
    pub fn set_dir(&mut self, forward: bool) {
        self.dir.set_level(if forward { self.forward_level } else { !self.forward_level });
    }

    /// Raw IOIN read (SelfTest).
    pub fn read_ioin(&mut self) -> u32 {
        self.read_reg(reg::IOIN)
    }

    /// Raw DRV_STATUS read (SelfTest).
    pub fn read_drv_status(&mut self) -> u32 {
        self.read_reg(reg::DRV_STATUS)
    }

    /// Clear GSTAT latched errors.
    pub fn clear_gstat(&mut self) {
        let g = self.read_reg(reg::GSTAT);
        self.write_reg(reg::GSTAT, g);
    }

    fn write_reg(&mut self, addr: u8, data: u32) {
        let mut buf = [
            addr | 0x80,
            (data >> 24) as u8,
            (data >> 16) as u8,
            (data >> 8) as u8,
            data as u8,
        ];
        self.spi.transfer(&mut buf).ok();
    }

    /// TMC SPI read quirk (see module docs): send the request, then a
    /// follow-up transfer to actually fetch the data.
    fn read_reg(&mut self, addr: u8) -> u32 {
        let mut buf = [addr & 0x7F, 0, 0, 0, 0];
        self.spi.transfer(&mut buf).ok();
        let mut buf2 = [addr & 0x7F, 0, 0, 0, 0];
        self.spi.transfer(&mut buf2).ok();
        u32::from_be_bytes([buf2[1], buf2[2], buf2[3], buf2[4]])
    }
}

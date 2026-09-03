//! Interlock matrix (SPECS §11).
//!
//! Implemented twice in the firmware: as ISR-level immediate actions (door, deadman)
//! and as supervisor-level latching/reporting. This module is the authoritative
//! truth table, unit-tested against SPECS §11.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum ErrorCode {
    DoorOpen,
    Jam,
    Driver(u16),
    CriticalBattery,
    RollEnd,
    Watchdog,
    Brownout,
    RmtBusy,
}

impl ErrorCode {
    /// Numeric fault code for the field-atomic `Status` (ARCHITECTURE §5.2).
    /// Payload-carrying variants are tagged with the top bit.
    pub const fn code(self) -> u32 {
        match self {
            ErrorCode::DoorOpen => 1,
            ErrorCode::Jam => 2,
            ErrorCode::Driver(c) => 0x8000_0000 | c as u32,
            ErrorCode::CriticalBattery => 3,
            ErrorCode::RollEnd => 4,
            ErrorCode::Watchdog => 5,
            ErrorCode::Brownout => 6,
            ErrorCode::RmtBusy => 7,
        }
    }

    /// Inverse of [`Self::code`] — decodes the field-atomic `Status.fault`
    /// value the supervisor polls (ARCHITECTURE §5.2: faults are now
    /// poll-delta-detected, not channel-delivered).
    pub const fn from_code(code: u32) -> Option<Self> {
        match code {
            1 => Some(ErrorCode::DoorOpen),
            2 => Some(ErrorCode::Jam),
            3 => Some(ErrorCode::CriticalBattery),
            4 => Some(ErrorCode::RollEnd),
            5 => Some(ErrorCode::Watchdog),
            6 => Some(ErrorCode::Brownout),
            7 => Some(ErrorCode::RmtBusy),
            c if c & 0x8000_0000 != 0 => Some(ErrorCode::Driver((c & 0x7FFF) as u16)),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Condition {
    DoorOpen,
    DoorClosed,
    Jam,
    DriverFault(u16),
    LowBattery,
    CriticalBattery,
    FilmEnd,
    Watchdog,
    Brownout,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Response {
    /// De-energize everything immediately (ISR path).
    pub stop_now: bool,
    /// Finish the current frame, then park.
    pub stop_at_frame_end: bool,
    /// Persist counters + settings before going quiet.
    pub persist: bool,
    /// Latched until user acknowledgement.
    pub latch: Option<ErrorCode>,
    /// transient display line; errors override the state line (SPECS §9.4).
    pub warn: Option<&'static str>,
}

pub fn evaluate(c: Condition) -> Response {
    match c {
        Condition::DoorOpen => Response {
            stop_now: true,
            persist: true,
            latch: Some(ErrorCode::DoorOpen),
            warn: Some("DOOR OPEN"),
            ..Default::default()
        },
        Condition::DoorClosed => Response::default(),
        Condition::Jam => Response {
            stop_now: true,
            latch: Some(ErrorCode::Jam),
            warn: Some("ERROR JAM"),
            ..Default::default()
        },
        Condition::DriverFault(code) => Response {
            stop_now: true,
            latch: Some(ErrorCode::Driver(code)),
            warn: Some("ERROR DRV"),
            ..Default::default()
        },
        Condition::LowBattery => Response {
            warn: Some("LOW BATTERY"),
            ..Default::default()
        },
        Condition::CriticalBattery => Response {
            stop_at_frame_end: true,
            latch: Some(ErrorCode::CriticalBattery),
            warn: Some("BAT CRITICAL"),
            ..Default::default()
        },
        Condition::FilmEnd => Response {
            stop_at_frame_end: true,
            latch: Some(ErrorCode::RollEnd),
            warn: Some("ROLL END"),
            ..Default::default()
        },
        Condition::Watchdog => Response {
            stop_now: true,
            latch: Some(ErrorCode::Watchdog),
            ..Default::default()
        },
        Condition::Brownout => Response {
            stop_now: true,
            latch: Some(ErrorCode::Brownout),
            ..Default::default()
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn door_open_stops_immediately_and_persists() {
        let r = evaluate(Condition::DoorOpen);
        assert!(r.stop_now);
        assert!(!r.stop_at_frame_end);
        assert!(r.persist);
        assert_eq!(r.latch, Some(ErrorCode::DoorOpen));
    }

    #[test]
    fn low_battery_only_warns() {
        let r = evaluate(Condition::LowBattery);
        assert!(!r.stop_now && !r.stop_at_frame_end);
        assert_eq!(r.latch, None);
        assert!(r.warn.is_some());
    }

    #[test]
    fn critical_battery_and_film_end_finish_the_frame() {
        let r = evaluate(Condition::CriticalBattery);
        assert!(!r.stop_now && r.stop_at_frame_end);
        let r = evaluate(Condition::FilmEnd);
        assert!(r.stop_at_frame_end);
        assert_eq!(r.latch, Some(ErrorCode::RollEnd));
    }

    #[test]
    fn driver_fault_carries_code() {
        let r = evaluate(Condition::DriverFault(0x42));
        assert!(r.stop_now);
        assert_eq!(r.latch, Some(ErrorCode::Driver(0x42)));
    }

    #[test]
    fn door_closed_is_a_no_op() {
        assert_eq!(evaluate(Condition::DoorClosed), Response::default());
    }

    #[test]
    fn code_roundtrip() {
        for code in [
            ErrorCode::DoorOpen,
            ErrorCode::Jam,
            ErrorCode::CriticalBattery,
            ErrorCode::RollEnd,
            ErrorCode::Watchdog,
            ErrorCode::Brownout,
            ErrorCode::RmtBusy,
            ErrorCode::Driver(0x1234),
        ] {
            assert_eq!(ErrorCode::from_code(code.code()), Some(code));
        }
        assert_eq!(ErrorCode::from_code(0), None);
        assert_eq!(ErrorCode::from_code(0x8000_0000), Some(ErrorCode::Driver(0)));
    }
}

//! Flash persistence codec (ARCHITECTURE §12, decision #13) — pure logic.
//!
//! Ping-pong 4 KB sectors: a versioned record with CRC32, newest sequence
//! number wins, writes alternate sectors so a torn write during power loss
//! leaves the *previous* record intact in the other sector. Fully host-
//! testable: the firmware storage task reads/writes these through the
//! flash driver and never interprets the bytes itself.

use crate::settings::{Settings, SETTINGS_BYTES};

pub const SECTOR_SIZE: usize = 4096;
/// Payload = settings + both track counters (exposed frames).
pub const PAYLOAD_LEN: usize = SETTINGS_BYTES + 8;

const MAGIC: u32 = 0x3553_4D43; // "SCM5"
const VERSION: u16 = 1;
const HEADER_LEN: usize = 16;

/// Everything the camera persists: settings + track-A/B exposed counters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Payload {
    pub settings: Settings,
    pub exposed_a: u32,
    pub exposed_b: u32,
}

impl Default for Payload {
    fn default() -> Self {
        Self {
            settings: Settings::default(),
            exposed_a: 0,
            exposed_b: 0,
        }
    }
}

impl Payload {
    pub fn to_bytes(&self) -> [u8; PAYLOAD_LEN] {
        let mut b = [0u8; PAYLOAD_LEN];
        b[..SETTINGS_BYTES].copy_from_slice(&self.settings.to_bytes());
        b[SETTINGS_BYTES..SETTINGS_BYTES + 4].copy_from_slice(&self.exposed_a.to_le_bytes());
        b[SETTINGS_BYTES + 4..].copy_from_slice(&self.exposed_b.to_le_bytes());
        b
    }

    pub fn from_bytes(b: &[u8; PAYLOAD_LEN]) -> Option<Self> {
        let settings = Settings::from_bytes(b[..SETTINGS_BYTES].try_into().ok()?)?;
        Some(Self {
            settings,
            exposed_a: u32::from_le_bytes(
                b[SETTINGS_BYTES..SETTINGS_BYTES + 4].try_into().ok()?,
            ),
            exposed_b: u32::from_le_bytes(
                b[SETTINGS_BYTES + 4..].try_into().ok()?,
            ),
        })
    }
}

/// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320). Bitwise is plenty fast
/// for a ~56-byte record at boot/idle boundaries.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Encode a record into a sector. The sector is first filled with 0xFF
/// (flash erased state) so the flash layer can always write it wholesale.
pub fn encode(sector: &mut [u8; SECTOR_SIZE], seq: u32, payload: &Payload) {
    sector.fill(0xFF);
    let bytes = payload.to_bytes();

    let mut head = [0u8; HEADER_LEN];
    head[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    head[4..6].copy_from_slice(&VERSION.to_le_bytes());
    head[6..10].copy_from_slice(&seq.to_le_bytes());
    head[10..12].copy_from_slice(&(bytes.len() as u16).to_le_bytes());
    // CRC covers header (with crc field zeroed) + payload.
    let mut crc_input = [0u8; HEADER_LEN + PAYLOAD_LEN];
    crc_input[..HEADER_LEN].copy_from_slice(&head);
    crc_input[HEADER_LEN..].copy_from_slice(&bytes);
    let crc = crc32(&crc_input);
    head[12..16].copy_from_slice(&crc.to_le_bytes());

    sector[..HEADER_LEN].copy_from_slice(&head);
    sector[HEADER_LEN..HEADER_LEN + PAYLOAD_LEN].copy_from_slice(&bytes);
}

/// Decode a sector. `None` for an erased, torn, or corrupt record.
pub fn decode(sector: &[u8; SECTOR_SIZE]) -> Option<(u32, Payload)> {
    let magic = u32::from_le_bytes(sector[0..4].try_into().ok()?);
    if magic != MAGIC {
        return None;
    }
    let version = u16::from_le_bytes(sector[4..6].try_into().ok()?);
    if version != VERSION {
        return None;
    }
    let seq = u32::from_le_bytes(sector[6..10].try_into().ok()?);
    let len = u16::from_le_bytes(sector[10..12].try_into().ok()?) as usize;
    if len != PAYLOAD_LEN {
        return None;
    }
    let stored_crc = u32::from_le_bytes(sector[12..16].try_into().ok()?);
    let payload: [u8; PAYLOAD_LEN] = sector[HEADER_LEN..HEADER_LEN + PAYLOAD_LEN]
        .try_into()
        .ok()?;
    let mut crc_input = [0u8; HEADER_LEN + PAYLOAD_LEN];
    // Recompute over header with the CRC field zeroed (encode did the same).
    crc_input[..12].copy_from_slice(&sector[..12]);
    crc_input[HEADER_LEN..].copy_from_slice(&payload);
    if crc32(&crc_input) != stored_crc {
        return None;
    }
    Some((seq, Payload::from_bytes(&payload)?))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    A,
    B,
}

impl Slot {
    pub const fn other(self) -> Self {
        match self {
            Slot::A => Slot::B,
            Slot::B => Slot::A,
        }
    }
}

/// Pick the live record across both sectors: highest valid sequence number
/// wins; `None` if neither sector holds a valid record (fresh flash or
/// total corruption → defaults).
pub fn choose(a: &[u8; SECTOR_SIZE], b: &[u8; SECTOR_SIZE]) -> Option<(Slot, u32, Payload)> {
    match (decode(a), decode(b)) {
        (Some((sa, pa)), Some((sb, pb))) => {
            if sa >= sb {
                Some((Slot::A, sa, pa))
            } else {
                Some((Slot::B, sb, pb))
            }
        }
        (Some((sa, pa)), None) => Some((Slot::A, sa, pa)),
        (None, Some((sb, pb))) => Some((Slot::B, sb, pb)),
        (None, None) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_known_vector() {
        // "123456789" → 0xCBF43926 (IEEE CRC-32 check value).
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn round_trip_survives_and_picks_newest() {
        let mut a = [0xFFu8; SECTOR_SIZE];
        let mut b = [0xFFu8; SECTOR_SIZE];

        assert_eq!(choose(&a, &b), None); // erased flash → no record

        let mut p1 = Payload::default();
        p1.exposed_a = 42;
        encode(&mut a, 1, &p1);
        assert_eq!(choose(&a, &b), Some((Slot::A, 1, p1)));

        let mut p2 = p1;
        p2.exposed_a = 43;
        encode(&mut b, 2, &p2);
        // Newest seq wins regardless of slot.
        assert_eq!(choose(&a, &b), Some((Slot::B, 2, p2)));

        // Torn write in B (corrupt a record byte): A is still readable.
        b[20] ^= 0x55;
        assert_eq!(choose(&a, &b), Some((Slot::A, 1, p1)));
    }

    #[test]
    fn torn_write_is_rejected() {
        let mut s = [0xFFu8; SECTOR_SIZE];
        encode(&mut s, 7, &Payload::default());
        // A flip inside the payload must be caught by CRC.
        s[HEADER_LEN] ^= 0x01;
        assert!(decode(&s).is_none());
        // And a flip inside the header too.
        let mut s = [0xFFu8; SECTOR_SIZE];
        encode(&mut s, 7, &Payload::default());
        s[6] ^= 0xFF; // seq field
        assert!(decode(&s).is_none());
    }

    #[test]
    fn payload_round_trip() {
        let mut p = Payload::default();
        p.settings.fps = 21.0;
        p.exposed_b = 7;
        assert_eq!(Payload::from_bytes(&p.to_bytes()), Some(p));
    }
}

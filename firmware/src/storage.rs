//! Settings/counter persistence (ARCHITECTURE §12): idle-gated flash writes only
//! (run stop, door events, roll end, settings change), ping-pong 4 KB sectors,
//! versioned `logic::settings::Settings` records + CRC32.

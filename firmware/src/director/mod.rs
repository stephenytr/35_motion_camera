//! Motion director — the only task on core 1 (ARCHITECTURE §6).
//!
//! TODO: translate supervisor commands into jobs (logic::frame_fsm params),
//! own TMC5160 over SPI2 (init, currents, 250 ms status polls),
//! run the boost ramp (logic::ramp), rebuild RMT step tables (logic::profile),
//! program integer-µs phase params at frame boundaries,
//! brownout Recover job, self-test.

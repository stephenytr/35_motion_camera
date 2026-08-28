//! Supervisor task (core 0): applies the interlock matrix (logic::interlock),
//! latches faults, feeds the RTC watchdog, translates UI commands into
//! director queue entries (ARCHITECTURE §5.1–5.3).

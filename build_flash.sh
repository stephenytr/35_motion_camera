#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/firmware"
cargo build --release --features debug-prints
cd ..
espflash flash --monitor target/xtensa-esp32s3-none-elf/release/firmware

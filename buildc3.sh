#!/usr/bin/env bash
# =============================================================================
#  buildc3.sh — build and flash Baram Warp 4 for ESP32-C3
# =============================================================================
#
#  Usage:
#    ./buildc3.sh          # release build (riscv32imc)
#    ./buildc3.sh flash    # build + flash via espflash (needs the board)
#    ./buildc3.sh monitor  # flash and open the serial monitor
#    ./buildc3.sh viewer   # open the PC viewer for the USB frame stream
#    ./buildc3.sh clean    # cargo clean
#
#  Requirements:
#    * rustup target riscv32imc-unknown-none-elf  (added automatically)
#    * espflash                                     (installed automatically)
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="riscv32imc-unknown-none-elf"
BIN="target/$TARGET/release/examples/esp32c3"
FEATURES="esp32c3,bdf"

log()  { printf "\033[1;34m[buildc3]\033[0m %s\n" "$*"; }
die()  { printf "\033[1;31m[err ]\033[0m %s\n" "$*" >&2; exit 1; }

ensure_target() {
    if ! rustup target list --installed | grep -q "^${TARGET}$"; then
        log "Installing Rust target ${TARGET} ..."
        rustup target add "${TARGET}" || die "Failed to add ${TARGET}"
    fi
}

ensure_espflash() {
    if ! command -v espflash >/dev/null 2>&1; then
        log "espflash not found; installing ..."
        cargo install espflash --locked || die "Failed to install espflash"
    fi
}

build() {
    ensure_target
    log "Building ESP32-C3 firmware (${TARGET}, features: ${FEATURES}) ..."
    cargo build --release -p baram-warp4 --example esp32c3 \
        --target "${TARGET}" --no-default-features --features "${FEATURES}"
    test -f "$BIN" || die "Build did not produce $BIN"
    log "  -> $BIN ($(stat -f %z "$BIN" 2>/dev/null || stat -c %s "$BIN") bytes)"
}

case "${1:-build}" in
    build)
        build
        ;;
    flash)
        build
        ensure_espflash
        log "Flashing ..."
        espflash flash --before usb-reset "$BIN"
        ;;
    monitor)
        build
        ensure_espflash
        log "Flashing with monitor ..."
        espflash flash --before usb-reset --monitor "$BIN"
        ;;
    viewer)
        exec ./tools/run_viewer.sh
        ;;
    clean)
        cargo clean
        ;;
    help|-h|--help)
        sed -n '2,/^# =\+/p' "$0" | sed 's/^# \?//'
        ;;
    *)
        die "Unknown command: $1 (use build|flash|monitor|clean|help)"
        ;;
esac

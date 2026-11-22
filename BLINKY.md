# Blinky bootloader fork

This is blinky's own fork of the Adafruit nRF52 bootloader, maintained on its own. The working branch is **`blinky-local-patches`**; `master` is only a reference copy of upstream, and upstream changes are cherry-picked when they're useful. Bugs are fixed here and committed here; nothing waits on upstream.

## What it adds to upstream 0.8.0 / v0.10.0

- `DEFAULT_TO_OTA_DFU`: a sealed device with no valid app recovers over BLE, while physical-user entry (double-tap, button) still gives the USB drive.
- Dual-transport DFU (USB MSC + BLE together), with SoftDevice-aware flash writes and settings saves.
- Stuck-transfer recovery and BLE pause during a USB drive transfer.
- An app-handshake watchdog in `GPREGRET`: after 3 boots without the firmware clearing it, the bootloader stays in DFU.
- `tools/verify_bootloader.py`: every DFU path must start USB (the 2026-05-15 hat incident).

History and rationale: blinky-firmware `bootloader/README.md` and `docs/BOOTLOADER_PRODUCTION_AUDIT_2026_05_15.md`.

## Board

`xiao_nrf52840_ble_sense`. One image runs on the XIAO nRF52840 **Sense** (legacy fleet) and **Sense Plus** (new devices): every pin the board definition uses is identical (blinky-firmware `docs/BOOTLOADER_WORKFLOW.md` §5a). Every build must keep working on both.

## CI and releases

`.github/workflows/blinky-bootloader.yml` builds on every push/PR to `blinky-local-patches`:
- release and debug variants (debug = RTT logging, bootloader at 0xED000; RTT blocks when its buffer is full, so use it only in SWD debugging sessions);
- arm-none-eabi-gcc 9-2019-q4, the fleet toolchain;
- `tools/verify_bootloader.py` must pass;
- artifacts: ELF, linker map, hexes (with and without S140 7.3.0), update UF2, DFU zip, SHA256SUMS;
- reproducible: `SOURCE_DATE_EPOCH` is the commit time and the DFU zip is repacked, so the same commit always builds the same bytes.

A `bl-<version>` tag publishes a GitHub Release from the release variant (plus a debug tarball). blinky-firmware pins the source as its `bootloader/src` submodule.

## Changing it

PR into `blinky-local-patches`; CI green. A candidate then goes through the hardware gate on the test box before it's tagged, and reaches the fleet only through blinky-firmware's `scripts/deploy-bootloader.sh`. The full flow: blinky-firmware `docs/BOOTLOADER_WORKFLOW.md`.

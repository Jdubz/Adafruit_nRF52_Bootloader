/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * -# Receive start data packet.
 * -# Based on start packet, prepare NVM area to store received data.
 * -# Receive data packet.
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "nrfx.h"
#include "nrf_clock.h"
#include "nrfx_power.h"
#include "nrfx_pwm.h"

#include "nordic_common.h"
#include "sdk_common.h"
#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"

#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "nrf.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "nrf_error.h"

#include "boards.h"

#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "pstorage.h"
#include "nrfx_nvmc.h"

#ifdef NRF_USBD
#include "uf2/uf2.h"
#include "nrf_usbd.h"
#include "tusb.h"

void usb_init(bool cdc_only);
void usb_teardown(void);

// tinyusb function that handles power event (detected, ready, removed)
// We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled.
extern void tusb_hal_nrf_power_event(uint32_t event);

#else

#define usb_init(x)       led_state(STATE_USB_MOUNTED) // mark nrf52832 as mounted
#define usb_teardown()

#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/*
 * Blinking patterns:
 * - DFU Serial     : LED Status blink
 * - DFU OTA        : LED Status & Conn blink at the same time
 * - DFU Flashing   : LED Status blink 2x fast
 * - Factory Reset  : LED Status blink 2x fast
 * - Fatal Error    : LED Status & Conn blink one after another
 */

/* Magic that written to NRF_POWER->GPREGRET by application when it wish to go into DFU
 * - DFU_MAGIC_OTA_APPJUM        : used by BLEDfu service, SD is already inited
 * - DFU_MAGIC_OTA_RESET         : entered by soft reset, SD is not inited yet
 * - DFU_MAGIC_SERIAL_ONLY_RESET : with CDC interface only
 * - DFU_MAGIC_UF2_RESET         : with CDC and MSC interfaces
 * - DFU_MAGIC_SKIP              : skip DFU entirely including double reset delay,
 *                                 Can be used with systemoff or quick reset to app
 *
 * Note: for DFU_MAGIC_OTA_APPJUM Softdevice must not initialized.
 * since it is already in application. In all other case of OTA SD must be initialized
 */
#define DFU_MAGIC_OTA_APPJUM            BOOTLOADER_DFU_START  // 0xB1
#define DFU_MAGIC_OTA_RESET             0xA8
#define DFU_MAGIC_SERIAL_ONLY_RESET     0x4e
#define DFU_MAGIC_UF2_RESET             0x57
#define DFU_MAGIC_SKIP                  0x6d
#define DFU_MAGIC_QSPI_APPLY           0xCC  // Apply staged firmware from QSPI flash

#define DFU_DBL_RESET_MAGIC             0x5A1AD5      // SALADS
#define DFU_DBL_RESET_APP               0x4ee5677e
#define DFU_DBL_RESET_DELAY             500
#define DFU_DBL_RESET_MEM               0x20007F7C

// RAM-based bootloader entry: firmware writes these to DFU_DBL_RESET_MEM
// instead of GPREGRET. RAM at 0x20007F7C survives system reset (proven by
// double-reset detection above). Unlike GPREGRET, RAM is not affected by
// USB hub port power-cycling during device reset.
#define DFU_RAM_MAGIC_UF2              0xBEEF0057
#define DFU_RAM_MAGIC_BLE              0xBEEF00A8
#define DFU_RAM_MAGIC_QSPI             0xBEEF00CC

// Boot-attempt counter (OPEN_ISSUES §3.1 / [[project-bl-no-app-crash-fallback]]).
//
// Encoded into the chip's GPREGRET hardware register (8 bits, persists
// across NVIC_SystemReset, cleared only by power-on reset). This is the
// SAME storage class as the existing BL magic values (0x57=UF2, 0xA8=OTA,
// 0x4E=skipCRC, 0xB1=enterApp); we reserve the upper nibble 0x1_ for the
// counter so it cannot collide with those magics or with the firmware's
// SafeBootWatchdog::enterBootloaderWithMagic() writes.
//
// Encoding:
//   GPREGRET == 0x1N (N=0..F)  → counter, current count = N
//   GPREGRET == 0x00 / other   → no counter active, treat as count = 0
//
// We previously tried a RAM word at 0x20007F78 (one below DFU_DBL_RESET_MEM).
// That ADDRESS falls inside the firmware's .bss section — the firmware's
// Reset_Handler zeros BSS on every boot, BEFORE static init and main(),
// silently erasing the BL's increment. The mechanism could never fire.
// GPREGRET is not RAM, so the C runtime cannot touch it.
//
// Increment: BL writes 0x1(N+1) just before ``bootloader_app_start()``.
// Clear: firmware clears the upper-nibble pattern in
//   1) SafeBootWatchdog::begin() — early boot, catches "we are about to
//      run the app, the BL's worry is no longer relevant"
//   2) SafeBootWatchdog::markStable() at 60 s — defense in depth
// Trigger: check_dfu_mode reads the counter; if N ≥ BOOT_ATTEMPT_THRESHOLD,
// forces dfu_start = 1 so the BL enters DFU instead of jumping to an app
// that is crash-looping before reaching SafeBootWatchdog::begin().
#define BOOT_ATTEMPT_GPREGRET_PATTERN   0x10
#define BOOT_ATTEMPT_GPREGRET_MASK      0xF0
#define BOOT_ATTEMPT_COUNT_MASK         0x0F
#define BOOT_ATTEMPT_THRESHOLD          3

#define BOOTLOADER_VERSION_REGISTER     NRF_TIMER2->CC[0]
#define DFU_SERIAL_STARTUP_INTERVAL     1000

// Allow for using reset button essentially to swap between application and bootloader.
// This is controlled by a flag in the app and is the behavior of CPX and all Arcade boards when using MakeCode.
// DFU_DBL_RESET magic is used to determined which mode is entered
#define APP_ASKS_FOR_SINGLE_TAP_RESET() (*((uint32_t*)(DFU_BANK_0_REGION_START + 0x200)) == 0x87eeb07c)

// These value must be the same with one in dfu_transport_ble.c
#define BLEGAP_EVENT_LENGTH             6
// Bumped 23 → 247 to lift the legacy DFU 20-byte-per-packet ceiling.
// At MTU=247, ATT data payload is 244 bytes per write, which makes a
// ~510 KB BLE-DFU transfer ~12x faster (≈30 s vs ≈5.5 min legacy).
// MUST match the same define in dfu_transport_ble.c (the local define
// there shadows this one).
// OPEN_ISSUES §3.3 / [[feedback-enclosed-devices-no-physical]].
#define BLEGATT_ATT_MTU_MAX             247
enum { BLE_CONN_CFG_HIGH_BANDWIDTH = 1 };

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
uint32_t* dbl_reset_mem = ((uint32_t*)  DFU_DBL_RESET_MEM );

// true if ble, false if serial
bool _ota_dfu = false;
bool _ota_connected = false;
bool _sd_inited = false;

bool is_ota(void)
{
  return _ota_dfu;
}

static void check_dfu_mode(void);
static uint32_t ble_stack_init(void);

// The SoftDevice must only be initialized if a chip reset has occurred.
// Soft reset (jump ) from application must not reinitialize the SoftDevice.
static void mbr_init_sd(void)
{
  PRINTF("SD_MBR_COMMAND_INIT_SD\r\n");
  sd_mbr_command_t com = { .command = SD_MBR_COMMAND_INIT_SD };
  sd_mbr_command(&com);
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
int main(void)
{
  // Populate Boot Address and MBR Param into MBR if not already
  // MBR_BOOTLOADER_ADDR/MBR_PARAM_PAGE_ADDR are used if available, else UICR registers are used
  // Note: skip it for now since this will prevent us to change the size of bootloader in the future
  // bootloader_mbr_addrs_populate();

  // Save bootloader version to pre-defined register, retrieved by application
  // TODO move to CF2
  BOOTLOADER_VERSION_REGISTER = (MK_BOOTLOADER_VERSION);

  board_init();
  bootloader_init();

  PRINTF("Bootloader Start\r\n");

  led_state(STATE_BOOTLOADER_STARTED);

  // When updating SoftDevice, bootloader will reset before swapping SD
  if (bootloader_dfu_sd_in_progress())
  {
    led_state(STATE_WRITING_STARTED);

    bootloader_dfu_sd_update_continue();
    bootloader_dfu_sd_update_finalize();

    led_state(STATE_WRITING_FINISHED);
  }

  // Check all inputs and enter DFU if needed
  // Return when DFU process is complete (or not entered at all)
  check_dfu_mode();

  // Reset peripherals
  board_teardown();

  /* Jump to application if valid
   * "Master Boot Record and SoftDevice initializaton procedure"
   * - SD_MBR_COMMAND_INIT_SD (if not already)
   * - sd_softdevice_disable()
   * - sd_softdevice_vector_table_base_set(APP_ADDR)
   * - jump to App reset
   */

  if (bootloader_app_is_valid() && !bootloader_dfu_sd_in_progress())
  {
    PRINTF("App is valid\r\n");
    if ( is_sd_existed() )
    {
      // MBR forward IRQ to SD (if not already)
      if ( !_sd_inited ) mbr_init_sd();

      // Make sure SD is disabled
      sd_softdevice_disable();
    }

    // clear in case we kept DFU_DBL_RESET_APP there
    (*dbl_reset_mem) = 0;

    // App-handshake watchdog increment (OPEN_ISSUES §3.1). Bump the
    // GPREGRET-backed boot-attempt counter immediately before handing
    // control to the app — by definition, this is the moment the BL
    // gives up control. The firmware is responsible for clearing the
    // counter once it reaches a known-good runtime state
    // (SafeBootWatchdog::begin() at boot, markStable() at 60 s). If
    // the firmware crashes before begin() runs, the next BL boot
    // reads the incremented value; after BOOT_ATTEMPT_THRESHOLD
    // consecutive bumps with no clear, check_dfu_mode forces DFU.
    //
    // GPREGRET state at this point: existing magic processing above
    // either matched (and cleared GPREGRET to 0 via the dfu_start /
    // dfu_skip branch), or didn't match. So GPREGRET here is either
    // 0 (clean — fresh chip, cold boot, post-DFU, or post-markStable),
    // or 0x1N (counter from a previous crashy boot that didn't reach
    // SafeBootWatchdog::begin to clear).
    {
      uint8_t const cur = NRF_POWER->GPREGRET;
      uint8_t prev = ((cur & BOOT_ATTEMPT_GPREGRET_MASK) == BOOT_ATTEMPT_GPREGRET_PATTERN)
                     ? (uint8_t)(cur & BOOT_ATTEMPT_COUNT_MASK)
                     : 0;
      uint8_t next = (prev < BOOT_ATTEMPT_COUNT_MASK) ? (uint8_t)(prev + 1) : prev;
      NRF_POWER->GPREGRET = (uint8_t)(BOOT_ATTEMPT_GPREGRET_PATTERN | next);
      __DSB();
      __ISB();
    }

    // start application
    bootloader_app_start();
  }

#ifdef DEFAULT_TO_OTA_DFU
  if (!bootloader_app_is_valid()) {
    NRF_POWER->GPREGRET = DFU_MAGIC_OTA_RESET;
  }
#endif

  NVIC_SystemReset();
}

/*------------- QSPI Staged OTA Apply -------------*/
/* Firmware is staged in external QSPI flash (P25Q16H, 2 MB) by the running
 * application. When the app sets GPREGRET=0xCC and resets, the bootloader
 * reads the staged firmware from QSPI, validates CRC, and copies to internal
 * flash. If validation fails, the old application boots normally.
 *
 * QSPI layout:
 *   0x000000: 8-byte header {uint32 size, uint16 crc16, uint16 magic=0xB10C}
 *   0x001000: Firmware binary (4KB-aligned)
 *
 * Safety: GPREGRET is cleared BEFORE any QSPI operations. If this code
 * crashes, the device resets with GPREGRET=0 and boots the old app.
 * Internal flash is only erased after the QSPI data passes CRC validation.
 */
#include "nrfx_qspi.h"
#include "crc16.h"

#define QSPI_STAGING_HEADER_ADDR    0x000000
#define QSPI_STAGING_FIRMWARE_ADDR  0x001000
#define QSPI_STAGING_MAGIC          0xB10C

typedef struct __attribute__((packed)) {
  uint32_t size;
  uint16_t crc16;
  uint16_t magic;
} qspi_staging_header_t;

static bool qspi_apply_staged_firmware(void)
{
  // XIAO nRF52840 Sense QSPI pins (physical P0.xx, NOT Arduino logical pins)
  nrfx_qspi_config_t qcfg = {
    .xip_offset = 0,
    .pins = {
      .sck_pin = NRF_GPIO_PIN_MAP(0, 21),
      .csn_pin = NRF_GPIO_PIN_MAP(0, 25),
      .io0_pin = NRF_GPIO_PIN_MAP(0, 20),
      .io1_pin = NRF_GPIO_PIN_MAP(0, 24),
      .io2_pin = NRF_GPIO_PIN_MAP(0, 22),
      .io3_pin = NRF_GPIO_PIN_MAP(0, 23),
    },
    .prot_if = {
      .readoc   = NRF_QSPI_READOC_READ4O,
      .writeoc  = NRF_QSPI_WRITEOC_PP4O,
      .addrmode = NRF_QSPI_ADDRMODE_24BIT,
    },
    .phy_if = {
      .sck_delay = 10,
      .dpmen     = false,
      .spi_mode  = NRF_QSPI_MODE_0,
      .sck_freq  = NRF_QSPI_FREQ_32MDIV16,  // 2 MHz (conservative)
    },
    .irq_priority = 7,
  };

  // Init QSPI peripheral (polling mode — no IRQ handler)
  if (nrfx_qspi_init(&qcfg, NULL, NULL) != NRFX_SUCCESS) {
    PRINTF("QSPI init failed\r\n");
    return false;
  }

  // Read staging header
  qspi_staging_header_t hdr __attribute__((aligned(4)));
  if (nrfx_qspi_read(&hdr, sizeof(hdr), QSPI_STAGING_HEADER_ADDR) != NRFX_SUCCESS) {
    PRINTF("QSPI header read failed\r\n");
    nrfx_qspi_uninit();
    return false;
  }

  // Validate header
  // Validate header. Hardcoded upper bound (820 KB = 0xCD000) as safety net
  // in case DFU_IMAGE_MAX_SIZE_FULL is corrupted by bad SoftDevice info struct.
  if (hdr.magic != QSPI_STAGING_MAGIC || hdr.size == 0 ||
      hdr.size > DFU_IMAGE_MAX_SIZE_FULL || hdr.size > 0xCD000) {
    PRINTF("QSPI header invalid (magic=0x%04X size=%lu)\r\n", hdr.magic, hdr.size);
    nrfx_qspi_uninit();
    return false;
  }

  PRINTF("QSPI staged: %lu bytes, CRC=0x%04X\r\n", hdr.size, hdr.crc16);

  // Read firmware from QSPI and compute CRC16 incrementally
  static uint8_t chunk_buf[4096] __attribute__((aligned(4)));
  uint16_t crc = 0xFFFF;
  uint32_t remaining = hdr.size;
  uint32_t qspi_offset = QSPI_STAGING_FIRMWARE_ADDR;

  while (remaining > 0) {
    uint32_t chunk_len = (remaining > sizeof(chunk_buf)) ? sizeof(chunk_buf) : remaining;
    // Round up to 4-byte alignment for QSPI read
    uint32_t read_len = (chunk_len + 3) & ~3;

    if (nrfx_qspi_read(chunk_buf, read_len, qspi_offset) != NRFX_SUCCESS) {
      PRINTF("QSPI read failed at 0x%08lX\r\n", qspi_offset);
      nrfx_qspi_uninit();
      return false;
    }

    crc = crc16_compute(chunk_buf, chunk_len, &crc);
    qspi_offset += chunk_len;
    remaining -= chunk_len;
  }

  // Validate CRC
  if (crc != hdr.crc16) {
    PRINTF("QSPI CRC mismatch: computed=0x%04X expected=0x%04X\r\n", crc, hdr.crc16);
    nrfx_qspi_uninit();
    return false;
  }

  PRINTF("QSPI CRC OK — applying to internal flash\r\n");
  led_state(STATE_WRITING_STARTED);

  // === POINT OF NO RETURN: erase internal app flash and copy from QSPI ===
  remaining = hdr.size;
  qspi_offset = QSPI_STAGING_FIRMWARE_ADDR;
  uint32_t flash_addr = CODE_REGION_1_START;

  while (remaining > 0) {
    uint32_t chunk_len = (remaining > sizeof(chunk_buf)) ? sizeof(chunk_buf) : remaining;
    uint32_t read_len = (chunk_len + 3) & ~3;

    if (nrfx_qspi_read(chunk_buf, read_len, qspi_offset) != NRFX_SUCCESS) {
      PRINTF("QSPI re-read failed at 0x%08lX\r\n", qspi_offset);
      break;  // Can't recover — flash is partially written. SafeBootWatchdog will handle.
    }

    // flash_nrf5x_write handles page erase + write with caching
    flash_nrf5x_write(flash_addr, chunk_buf, chunk_len, true);

    flash_addr += chunk_len;
    qspi_offset += chunk_len;
    remaining -= chunk_len;
  }

  // Flush any remaining cached page
  flash_nrf5x_flush(true);

  // Update bootloader settings so bootloader_app_is_valid() returns true
  bootloader_settings_t settings;
  memset(&settings, 0, sizeof(settings));
  settings.bank_0 = BANK_VALID_APP;
  settings.bank_0_crc = hdr.crc16;
  settings.bank_0_size = hdr.size;
  settings.bank_1 = BANK_INVALID_APP;

  nrfx_nvmc_page_erase(BOOTLOADER_SETTINGS_ADDRESS);
  nrfx_nvmc_words_write(BOOTLOADER_SETTINGS_ADDRESS,
                        (uint32_t const *)&settings,
                        sizeof(settings) / sizeof(uint32_t));

  // Clear staging header by erasing the entire header sector (4 KB).
  // This sets all bits to 0xFF, so magic becomes 0xFFFF (invalid).
  // A sector erase is more reliable than a partial page write — the
  // nrfx_qspi_write of zeros was observed to fail silently on some boots,
  // leaving magic=0xB10C and causing harmless but wasteful re-application.
  nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, QSPI_STAGING_HEADER_ADDR);

  nrfx_qspi_uninit();

  PRINTF("QSPI OTA applied: %lu bytes to 0x%08lX\r\n", hdr.size, (uint32_t)CODE_REGION_1_START);
  led_state(STATE_WRITING_FINISHED);

  return true;
}

static void check_dfu_mode(void)
{
  uint32_t const gpregret = NRF_POWER->GPREGRET;

  // QSPI staged OTA: apply pre-validated firmware from external QSPI flash.
  // Check RAM magic first (reliable through USB hub resets), then GPREGRET.
  bool qspi_requested = false;
  if (*dbl_reset_mem == DFU_RAM_MAGIC_QSPI) {
    (*dbl_reset_mem) = 0;  // Clear immediately (one-shot)
    qspi_requested = true;
    PRINTF("QSPI OTA apply requested (RAM magic)\r\n");
  } else if (gpregret == DFU_MAGIC_QSPI_APPLY) {
    NRF_POWER->GPREGRET = 0;
    qspi_requested = true;
    PRINTF("QSPI OTA apply requested (GPREGRET)\r\n");
  }
  if (qspi_requested) {
    if (qspi_apply_staged_firmware()) {
      return;
    }
    PRINTF("QSPI apply failed — falling through to normal boot\r\n");
  }

  // RAM-based bootloader entry: check DFU_DBL_RESET_MEM for firmware-written
  // magic values. This bypasses GPREGRET entirely — RAM at 0x20007F7C survives
  // system reset (same address used by double-reset detection). Firmware writes
  // these values before NVIC_SystemReset() as a reliable alternative to GPREGRET
  // which can be cleared by USB hub port power-cycling during reset.
  uint32_t const ram_magic = *dbl_reset_mem;
  bool ram_uf2 = false;
  bool ram_ota = false;

  if (ram_magic == DFU_RAM_MAGIC_UF2) {
    (*dbl_reset_mem) = 0;  // Clear immediately (one-shot)
    ram_uf2 = true;
    PRINTF("RAM magic: UF2 mode requested\r\n");
  } else if (ram_magic == DFU_RAM_MAGIC_BLE) {
    (*dbl_reset_mem) = 0;
    ram_ota = true;
    PRINTF("RAM magic: BLE DFU mode requested\r\n");
  }

  // SD is already Initialized in case of BOOTLOADER_DFU_OTA_MAGIC
  _sd_inited = (gpregret == DFU_MAGIC_OTA_APPJUM);

  // Start Bootloader in BLE OTA mode
  _ota_dfu = ram_ota || (gpregret == DFU_MAGIC_OTA_APPJUM) || (gpregret == DFU_MAGIC_OTA_RESET);

  // Serial only mode
  bool const serial_only_dfu = (gpregret == DFU_MAGIC_SERIAL_ONLY_RESET);
  bool const uf2_dfu         = ram_uf2 || (gpregret == DFU_MAGIC_UF2_RESET);
  bool const dfu_skip        = (gpregret == DFU_MAGIC_SKIP);

  bool const reason_reset_pin = (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk) ? true : false;

  // start either serial, uf2 or ble
  bool dfu_start = _ota_dfu || serial_only_dfu || uf2_dfu ||
                    (((*dbl_reset_mem) == DFU_DBL_RESET_MAGIC) && reason_reset_pin);

  // Clear GPREGRET if it is our values
  if (dfu_start || dfu_skip) NRF_POWER->GPREGRET = 0;

  // skip dfu entirely
  if (dfu_skip) return;

  /*------------- Determine DFU mode (Serial, OTA, FRESET or normal) -------------*/
  // DFU button pressed
  dfu_start = dfu_start || button_pressed(BUTTON_DFU);

  // DFU + FRESET are pressed --> OTA
  _ota_dfu = _ota_dfu  || ( button_pressed(BUTTON_DFU) && button_pressed(BUTTON_FRESET) ) ;

  bool const valid_app = bootloader_app_is_valid();
  bool const just_start_app = valid_app && !dfu_start && (*dbl_reset_mem) == DFU_DBL_RESET_APP;

  if (!just_start_app && APP_ASKS_FOR_SINGLE_TAP_RESET()) dfu_start = 1;

  // App-handshake watchdog (OPEN_ISSUES §3.1). The
  // RebootFrequencyCounter in firmware catches crashes that survive
  // long enough to run configStorage.begin, but a crashy-but-valid
  // app that HardFaults pre-BLE-init (or pre-static-init) never
  // reaches that counter — the BL would jump to the same broken app
  // forever. The counter lives in GPREGRET (encoded as 0x1N) so it
  // survives NVIC_SystemReset and is NEVER touched by the firmware's
  // C startup .bss-zero (the previous RAM-based attempt at
  // 0x20007F78 was silently zeroed every boot, defeating the
  // mechanism). BL increments at app-jump time (block below at
  // bootloader_app_start); firmware clears in SafeBootWatchdog::begin()
  // (early — catches "we reached app, BL worry over") and
  // markStable() at 60 s (defense in depth). If we see THRESHOLD
  // consecutive attempts without a clear, force DFU.
  //
  // Note: GPREGRET may have been cleared to 0 just above (line
  // ~530) if existing magic matched (dfu_start || dfu_skip). In
  // that case our pattern check returns false and we don't force
  // DFU. That's correct: a magic value already triggered DFU,
  // so the §3.1 mechanism doesn't need to.
  {
    uint8_t const gpregret_now = NRF_POWER->GPREGRET;
    bool const counter_active =
        (gpregret_now & BOOT_ATTEMPT_GPREGRET_MASK) == BOOT_ATTEMPT_GPREGRET_PATTERN;
    uint8_t const attempts = counter_active ? (gpregret_now & BOOT_ATTEMPT_COUNT_MASK) : 0;
    if (counter_active && attempts >= BOOT_ATTEMPT_THRESHOLD) {
      PRINTF("Boot-attempt counter at %u (>= %u) — forcing DFU\r\n",
             attempts, BOOT_ATTEMPT_THRESHOLD);
      dfu_start = 1;
      // Pick DFU LED/transport hint based on VBUSDETECT — UF2 if a host
      // is plugged in (faster bench recovery), BLE-DFU otherwise (sealed
      // sculpture autonomous recovery). Mirrors the firmware's §3.2
      // logic so the BL-forced and firmware-initiated recovery paths
      // pick the same transport. Dual-transport is initialised in the
      // existing block below regardless, so this only affects the LED
      // hint and the eventual bootloader_dfu_start(_ota_dfu, ...) arg.
      bool const usb_present =
          (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
      if (!usb_present) {
        _ota_dfu = true;
      }
      // Clear GPREGRET so a successful DFU + reboot doesn't immediately
      // re-trigger the force-DFU path (counter would otherwise still
      // read >= threshold). The post-DFU reboot must start from a
      // clean GPREGRET state; firmware's begin()/markStable would
      // eventually clear it too, but only if the new firmware actually
      // boots — clearing here is safer.
      NRF_POWER->GPREGRET = 0;
      __DSB();
      __ISB();
    }
  }

#ifdef DEFAULT_TO_OTA_DFU
  /* Force BLE OTA DFU only for "silent" triggers: invalid app, or
   * app-initiated DFU without an explicit UF2/serial magic. Do NOT
   * coerce physical-user DFU entry (double-tap reset, DFU button,
   * APP_ASKS_FOR_SINGLE_TAP_RESET) — those have always meant
   * "give me USB UF2". Breaking them would remove the wired-recovery
   * escape hatch this bootloader's reset button is designed to provide.
   *
   * Use case split:
   *   sealed sculpture device, mid-DFU corruption → !valid_app → BLE DFU
   *   accessible device, user double-taps reset    → UF2 mode (preserved)
   */
  bool const physical_user_dfu =
      (((*dbl_reset_mem) == DFU_DBL_RESET_MAGIC) && reason_reset_pin) ||
      button_pressed(BUTTON_DFU) ||
      APP_ASKS_FOR_SINGLE_TAP_RESET();
  if ((dfu_start || !valid_app) && !serial_only_dfu && !uf2_dfu && !physical_user_dfu) {
    _ota_dfu = 1;
  }
#endif

  // App mode: Double Reset detection or DFU startup for nrf52832
  if ( ! (just_start_app || dfu_start || !valid_app) )
  {
#ifdef NRF52832_XXAA
    /* Even DFU is not active, we still force an 1000 ms dfu serial mode when startup
     * to support auto programming from Arduino IDE
     *
     * Note: Double Reset WONT work with nrf52832 since all its SRAM got cleared with GPIO reset.
     */
    bootloader_dfu_start(false, DFU_SERIAL_STARTUP_INTERVAL, false);
#else
    // Note: RESETREAS is not clear by bootloader, it should be cleared by application upon init()
    if (reason_reset_pin)
    {
      // Register our first reset for double reset detection
      (*dbl_reset_mem) = DFU_DBL_RESET_MAGIC;

      // if RST is pressed during this delay (double reset)--> if will enter dfu
      NRFX_DELAY_MS(DFU_DBL_RESET_DELAY);
    }
#endif
  }

  if (APP_ASKS_FOR_SINGLE_TAP_RESET())
  {
    (*dbl_reset_mem) = DFU_DBL_RESET_APP;
  }
  else
  {
    (*dbl_reset_mem) = 0;
  }

  // Enter DFU mode accordingly to input
  if ( dfu_start || !valid_app )
  {
    /* Always initialize BOTH transports — USB (MSC for UF2 + CDC) and BLE
     * — so a device in DFU mode is reachable by either path. One bootloader
     * binary for sealed and unsealed devices; no operational ambiguity
     * about "is this device visible right now?". The `_ota_dfu` flag
     * still influences the LED pattern (visual hint about how DFU was
     * entered) and the `ota` argument to `bootloader_dfu_start` (which
     * historically picked the transport — now both are started always).
     *
     * Cost: extra ~5-10 KB RAM for the USB stack alongside SoftDevice
     * (we have 250 KB+ headroom on nRF52840), and a couple hundred ms
     * extra init time at every DFU entry. Worth it.
     */
    led_state(_ota_dfu ? STATE_BLE_DISCONNECTED : STATE_USB_UNMOUNTED);
    if (!_sd_inited ) mbr_init_sd();
    _sd_inited = true;
    ble_stack_init();
    usb_init(serial_only_dfu);

    // Initiate an update of the firmware.
    if (APP_ASKS_FOR_SINGLE_TAP_RESET() || uf2_dfu || serial_only_dfu)
    {
      // If neither USB enumeration nor BLE connection happens within 3s
      // (eg. running on battery, fleet server not nearby), we restart
      // into the app. Both transports' activity cancels the timeout
      // (see dfu_transport_ble.c + msc_uf2.c first-write hooks).
       bootloader_dfu_start(_ota_dfu, 3000, true);
    }
    else
    {
      // No timeout: sealed-device autonomous recovery or user-action DFU.
       bootloader_dfu_start(_ota_dfu, 0, false);
    }

    // Teardown both transports.
    sd_softdevice_disable();
    usb_teardown();
  }
}


// Initializes the SotdDevice by following SD specs section
// "Master Boot Record and SoftDevice initializaton procedure"
static uint32_t ble_stack_init(void)
{
  // Forward vector table to bootloader address so that we can handle BLE events
  sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);

  // Enable Softdevice, Use Internal OSC to compatible with all boards
  nrf_clock_lf_cfg_t clock_cfg =
  {
      .source       = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv      = 16,
      .rc_temp_ctiv = 2,
      .accuracy     = NRF_CLOCK_LF_ACCURACY_250_PPM
  };

  sd_softdevice_enable(&clock_cfg, app_error_fault_handler);
  sd_nvic_EnableIRQ(SD_EVT_IRQn);

  /*------------- Configure BLE params  -------------*/
  extern uint32_t  __data_start__[]; // defined in linker
  uint32_t ram_start = (uint32_t) __data_start__;

  ble_cfg_t blecfg;

  // Configure the maximum number of connections.
  varclr(&blecfg);
  blecfg.gap_cfg.role_count_cfg.adv_set_count = 1;
  blecfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
  blecfg.gap_cfg.role_count_cfg.central_role_count = 0;
  blecfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
  sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &blecfg, ram_start);

  // NRF_DFU_BLE_REQUIRES_BONDS
  varclr(&blecfg);
  blecfg.gatts_cfg.service_changed.service_changed = 1;
  sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &blecfg, ram_start);

  // ATT MTU
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLEGATT_ATT_MTU_MAX;
  sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start);

  // Event Length + HVN queue + WRITE CMD queue setting affecting bandwidth
  varclr(&blecfg);
  blecfg.conn_cfg.conn_cfg_tag = BLE_CONN_CFG_HIGH_BANDWIDTH;
  blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = 1;
  blecfg.conn_cfg.params.gap_conn_cfg.event_length = BLEGAP_EVENT_LENGTH;
  sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start);

  // Enable BLE stack.
  // Note: Interrupt state (enabled, forwarding) is not work properly if not enable ble
  sd_ble_enable(&ram_start);

#if 0
  ble_opt_t  opt;
  varclr(&opt);
  opt.common_opt.conn_evt_ext.enable = 1; // enable Data Length Extension
  sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
#endif

  return NRF_SUCCESS;
}


//--------------------------------------------------------------------+
// Error Handler
//--------------------------------------------------------------------+
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */
  if ( (*ARM_CM_DHCSR) & 1UL ) __asm("BKPT #0\n"); /* Only halt mcu if debugger is attached */
  NVIC_SystemReset();
}

void assert_nrf_callback (uint16_t line_num, uint8_t const * p_file_name)
{
  app_error_fault_handler(0xDEADBEEF, 0, 0);
}

/*------------------------------------------------------------------*/
/* SoftDevice Event handler
 *------------------------------------------------------------------*/

// Process BLE event from SD
uint32_t proc_ble(void)
{
  __ALIGN(4) uint8_t ev_buf[ BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX) ];
  uint16_t ev_len = BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX);

  // Init header
  ble_evt_t* evt = (ble_evt_t*) ev_buf;
  evt->header.evt_id = BLE_EVT_INVALID;

  // Get BLE Event
  uint32_t err = sd_ble_evt_get(ev_buf, &ev_len);

  // Handle valid event, ignore error
  if( NRF_SUCCESS == err)
  {
    switch (evt->header.evt_id)
    {
      case BLE_GAP_EVT_CONNECTED:
        _ota_connected = true;
        led_state(STATE_BLE_CONNECTED);
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        _ota_connected = false;
        led_state(STATE_BLE_DISCONNECTED);
      break;

      default: break;
    }

    // from dfu_transport_ble
    extern void ble_evt_dispatch(ble_evt_t * p_ble_evt);
    ble_evt_dispatch(evt);
  }

  return err;
}

// process SOC event from SD
uint32_t proc_soc(void)
{
  uint32_t soc_evt = 0;
  uint32_t err = sd_evt_get(&soc_evt);

  if (NRF_SUCCESS == err)
  {
    pstorage_sys_event_handler(soc_evt);

    // Wakes flash_nrf5x_flush() when an SD-aware page erase/write completes.
    // Needed for dual-transport DFU (SD enabled) — the USB-MSC UF2 path
    // also routes through sd_flash_* in that mode.
    extern void flash_nrf5x_sd_event(uint32_t soc_evt);
    flash_nrf5x_sd_event(soc_evt);

#ifdef NRF_USBD
    /*------------- usb power event handler -------------*/
    int32_t usbevt = (soc_evt == NRF_EVT_POWER_USB_DETECTED   ) ? NRFX_POWER_USB_EVT_DETECTED:
                     (soc_evt == NRF_EVT_POWER_USB_POWER_READY) ? NRFX_POWER_USB_EVT_READY   :
                     (soc_evt == NRF_EVT_POWER_USB_REMOVED    ) ? NRFX_POWER_USB_EVT_REMOVED : -1;

    if ( usbevt >= 0) tusb_hal_nrf_power_event((uint32_t) usbevt);
#endif
  }

  return err;
}

void proc_sd_task(void* evt_data, uint16_t evt_size)
{
  (void) evt_data;
  (void) evt_size;

  // process BLE and SOC until there is no more events
  while( (NRF_ERROR_NOT_FOUND != proc_ble()) || (NRF_ERROR_NOT_FOUND != proc_soc()) )
  {

  }
}

void SD_EVT_IRQHandler(void)
{
  // Use App Scheduler to defer handling code in non-isr context
  app_sched_event_put(NULL, 0, proc_sd_task);
}


//--------------------------------------------------------------------+
// RTT printf retarget for Debug
//--------------------------------------------------------------------+
#ifdef CFG_DEBUG

#include "SEGGER_RTT.h"

__attribute__ ((used))
int _write (int fhdl, const void *buf, size_t count)
{
  (void) fhdl;
  SEGGER_RTT_Write(0, (char*) buf, (int) count);
  return count;
}

__attribute__ ((used))
int _read (int fhdl, char *buf, size_t count)
{
  (void) fhdl;
  return SEGGER_RTT_Read(0, buf, count);
}


#endif

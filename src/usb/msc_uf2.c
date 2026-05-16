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

#include "tusb.h"
#include "uf2/uf2.h"

#if CFG_TUD_MSC

#include "bootloader.h"
#include "boards.h"
#include "dfu_transport.h"

// If the host stops writing for this long while the BL hasn't seen
// `numWritten >= numBlocks` (i.e. a UF2 block went missing due to USB-level
// corruption — early-return in write_block leaves writtenMask permanently
// holed), declare the transfer stuck and reset. Without this the device
// sits in DFU forever and the operator must hardware-reset to retry.
//
// 8s is conservative — successful drops complete in ~5s, and a 3-4s idle
// after the last write is normal host behavior (FAT/dir update bookkeeping).
#define MSC_STUCK_TIMEOUT_MS  8000

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* UF2
 *------------------------------------------------------------------*/
static WriteState _wr_state = { 0 };

void read_block(uint32_t block_no, uint8_t *data);
int  write_block(uint32_t block_no, uint8_t *data, WriteState *state);

//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;

  const char vid[] = "Adafruit";
  const char pid[] = "nRF UF2";
  const char rev[] = "1.0";

  memcpy(vendor_id  , vid, strlen(vid));
  memcpy(product_id , pid, strlen(pid));
  memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  void const* response = NULL;
  uint16_t resplen = 0;

  // most scsi handled is input
  bool in_xfer = true;

  switch (scsi_cmd[0])
  {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      // Host is about to read/write etc ... better not to disconnect disk
      resplen = 0;
    break;

    default:
      // Set Sense = Invalid Command Operation
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

      // negative means error -> tinyusb could stall and/or response with failed status
      resplen = -1;
    break;
  }

  // return resplen must not larger than bufsize
  if ( resplen > bufsize ) resplen = bufsize;

  if ( response && (resplen > 0) )
  {
    if(in_xfer)
    {
      memcpy(buffer, response, resplen);
    }else
    {
      // SCSI output
    }
  }

  return resplen;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb (uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) lun;
  memset(buffer, 0, bufsize);

  // since we return block size each, offset should always be zero
  TU_ASSERT(offset == 0, -1);

  uint32_t count = 0;

  while ( count < bufsize )
  {
    read_block(lba, buffer);

    lba++;
    buffer += 512;
    count  += 512;
  }

  return count;
}

// Last WRITE10 timestamp (board_millis) — used by msc_uf2_check_stuck()
// to detect when the host has gone idle but the transfer never completed.
static volatile uint32_t _last_write10_ms = 0;

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb (uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  (void) lun;

  _last_write10_ms = board_millis();

  // Pause BLE advertising on the FIRST WRITE10 of the BL session, BEFORE
  // we process any UF2 blocks. BLE adv events contend with sd_flash_*
  // operations and can also corrupt USB MSC packets in flight, causing
  // is_uf2_block() magic checks to fail on the first WRITE10's blocks.
  // The previous version of this hook paused in write10_complete_cb
  // (status phase) — too late for the current transaction. This must
  // run BEFORE write_block() examines buffer contents.
  // Resumed in write10_complete_cb's abort path, or via system reset on
  // successful completion.
  static bool first_write_seen = false;
  if (!first_write_seen)
  {
    first_write_seen = true;
    dfu_transport_ble_advertising_pause();
  }

  uint32_t count = 0;
  while ( count < bufsize )
  {
    // Consider non-uf2 block write as successful
    // only break if write_block is busy with flashing (return 0)
    if ( 0 == write_block(lba, buffer, &_wr_state) ) break;

    lba++;
    buffer += 512;
    count  += 512;
  }

  return count;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
void tud_msc_write10_complete_cb(uint8_t lun)
{
  static bool first_write = true;

  // abort the DFU, uf2 block failed integrity check
  if ( _wr_state.aborted )
  {
    // aborted and reset
    PRINTF("Aborted\r\n");

    // BLE adv was paused at first_write below; resume so a BLE OTA fallback
    // is still reachable while we sit in DFU post-abort. The system reset
    // triggered by bootloader_dfu_update_process(DFU_RESET) will clear adv
    // state anyway, but resuming first keeps the GAP role consistent in the
    // brief window before reset.
    dfu_transport_ble_advertising_resume();

    dfu_update_status_t update_status;
    memset(&update_status, 0, sizeof(dfu_update_status_t ));
    update_status.status_code = DFU_RESET;

    bootloader_dfu_update_process(update_status);

    led_state(STATE_WRITING_FINISHED);
  }
  else if ( _wr_state.numBlocks )
  {
    // Start LED writing pattern with first write
    // (BLE pause was moved to tud_msc_write10_cb so it covers the first
    // WRITE10's data phase, not just everything after it.)
    if (first_write)
    {
      first_write = false;
      led_state(STATE_WRITING_STARTED);
    }

    // All block of uf2 file is complete --> complete DFU process
    if (_wr_state.numWritten >= _wr_state.numBlocks)
    {
      dfu_update_status_t update_status;
      memset(&update_status, 0, sizeof(dfu_update_status_t ));

      if ( _wr_state.update_bootloader )
      {
        // update bootloader always end with reset
        update_status.status_code = DFU_RESET;

        // Location of current stored new bootloader
        uint32_t * new_bootloader = (uint32_t *) BOOTLOADER_ADDR_NEW_RECIEVED;

        PRINT_HEX(new_bootloader);

        // skip if there is no bootloader change
        if ( memcmp(new_bootloader, (uint8_t*) BOOTLOADER_ADDR_START, DFU_BL_IMAGE_MAX_SIZE) )
        {
          PRINTF("Coyping new bootloader\r\n");

          sd_mbr_command_t command =
          {
            .command = SD_MBR_COMMAND_COPY_BL,
            .params.copy_bl.bl_src = new_bootloader,
            .params.copy_bl.bl_len = DFU_BL_IMAGE_MAX_SIZE/4 // size in words
          };

          // on success, COPY_BL won't return but run the new bootloader right away.
          sd_mbr_command(&command);
        }

        PRINTF("bootloader update complete\r\n");
      }else
      {
        // update App
        update_status.status_code = DFU_UPDATE_APP_COMPLETE;

        PRINTF("Application update complete\r\n");
      }

      bootloader_dfu_update_process(update_status);

      led_state(STATE_WRITING_FINISHED);
    }
  }
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;

  *block_count = CFG_UF2_NUM_BLOCKS;
  *block_size  = 512;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;

  if ( load_eject )
  {
    if (start)
    {
      // load disk storage
    }else
    {
      // unload disk storage
    }
  }

  return true;
}

// Detect a stuck UF2 transfer and recover by triggering a system reset.
//
// Symptom this addresses: the host's `cp` of a UF2 file completes its
// WRITE10 sequence, but the BL's numWritten counter never reaches numBlocks
// because at least one UF2 block was rejected by is_uf2_block() (USB-level
// corruption flipping a byte in the magic / family-id flag). The early
// `return -1` in write_block() never sets writtenMask for that block, so
// numWritten stays one (or more) below numBlocks forever. Without
// intervention the device sits in DFU mode permanently.
//
// Detection: numBlocks > 0 (transfer was started), numWritten < numBlocks
// (transfer not complete), and the last WRITE10 was MSC_STUCK_TIMEOUT_MS
// ago (host has gone idle, no more data is coming).
//
// Action: system reset via bootloader_dfu_update_process(DFU_RESET). The
// next boot starts a fresh BL session — `_wr_state` is in BSS so it's
// zeroed, advertising restarts, the host's re-drop will work cleanly.
//
// Called from wait_for_events() in bootloader.c on every loop iteration.
void msc_uf2_check_stuck(void)
{
  // Two stuck modes this recovers from:
  //   (a) numBlocks > 0 but numWritten < numBlocks — some UF2 block was lost
  //       to USB-level corruption (is_uf2_block magic check failed, write_block
  //       returned -1 without setting writtenMask for that block).
  //   (b) numBlocks == 0 but writes did land — host wrote FAT/dir metadata
  //       but no valid UF2 block was ever delivered to the family-id-switch
  //       in write_block (the entire UF2 data stream was lost / never sent).
  //
  // Both require: the host stopped writing >= MSC_STUCK_TIMEOUT_MS ago.
  // If no writes happened at all (_last_write10_ms == 0), there's nothing
  // to recover from — leave the BL idle waiting for a host.
  if (_last_write10_ms == 0) return;
  if (_wr_state.numBlocks > 0 && _wr_state.numWritten >= _wr_state.numBlocks) return;

  uint32_t const now = board_millis();
  if ((uint32_t)(now - _last_write10_ms) < MSC_STUCK_TIMEOUT_MS) return;

  // Reset via the existing DFU_RESET path. Tears down both transports and
  // tries to boot the (still-valid) app, allowing the host to retry the
  // UF2 drop after re-enumeration.
  dfu_update_status_t update_status;
  memset(&update_status, 0, sizeof(dfu_update_status_t));
  update_status.status_code = DFU_RESET;
  bootloader_dfu_update_process(update_status);
}

#endif

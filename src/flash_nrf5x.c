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

#include <string.h>
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "flash_nrf5x.h"
#include "boards.h"

#define FLASH_PAGE_SIZE           4096
#define FLASH_CACHE_INVALID_ADDR  0xffffffff

static uint32_t _fl_addr = FLASH_CACHE_INVALID_ADDR;
static uint8_t _fl_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

// Set true while a sd_flash_* operation is outstanding. Cleared from
// flash_nrf5x_sd_event() when the corresponding NRF_EVT_FLASH_OPERATION_*
// event arrives.
static volatile bool _sd_flash_pending = false;
static volatile uint32_t _sd_flash_result = NRF_SUCCESS;

// proc_ble / proc_soc live in main.c. We drain SoC events inline while
// waiting so the SoftDevice scheduler keeps making progress — otherwise
// blocking here would deadlock (the completion event would never be
// delivered because we'd be hogging the only thread that can pump it).
extern uint32_t proc_ble (void);
extern uint32_t proc_soc (void);

static inline bool _sd_is_running(void)
{
  uint8_t enabled = 0;
  sd_softdevice_is_enabled(&enabled);
  return enabled != 0;
}

void flash_nrf5x_sd_event (uint32_t soc_evt)
{
  if ( !_sd_flash_pending ) return;

  if ( soc_evt == NRF_EVT_FLASH_OPERATION_SUCCESS )
  {
    _sd_flash_result  = NRF_SUCCESS;
    _sd_flash_pending = false;
  }
  else if ( soc_evt == NRF_EVT_FLASH_OPERATION_ERROR )
  {
    _sd_flash_result  = NRF_ERROR_INTERNAL;
    _sd_flash_pending = false;
  }
}

// Block until _sd_flash_pending is cleared by flash_nrf5x_sd_event(),
// draining BLE and SOC events along the way so pstorage / BLE-DFU don't
// starve while we wait. Returns the latched result.
static uint32_t _sd_flash_wait_done(void)
{
  while ( _sd_flash_pending )
  {
    // Pump every pending event. proc_soc() calls flash_nrf5x_sd_event()
    // via the hook added in main.c; proc_ble() keeps BLE DFU alive.
    while ( (NRF_ERROR_NOT_FOUND != proc_ble()) || (NRF_ERROR_NOT_FOUND != proc_soc()) )
    {
    }

    if ( _sd_flash_pending )
    {
      // No event yet — sleep until next interrupt fires. WFI is safe here
      // even though we're called from a TinyUSB task: USB interrupts will
      // wake us so MSC handshakes stay responsive.
      sd_app_evt_wait();
    }
  }

  return _sd_flash_result;
}

static bool _sd_flash_erase_page(uint32_t addr)
{
  uint32_t err;

  _sd_flash_result  = NRF_SUCCESS;
  _sd_flash_pending = true;

  do
  {
    err = sd_flash_page_erase(addr / FLASH_PAGE_SIZE);
    if ( err == NRF_ERROR_BUSY ) sd_app_evt_wait();
  } while ( err == NRF_ERROR_BUSY );

  if ( err != NRF_SUCCESS )
  {
    _sd_flash_pending = false;
    PRINTF("sd_flash_page_erase(0x%08lX) failed: 0x%lX\r\n", addr, err);
    return false;
  }

  return _sd_flash_wait_done() == NRF_SUCCESS;
}

static bool _sd_flash_write_page(uint32_t addr, uint32_t const *src)
{
  uint32_t err;

  _sd_flash_result  = NRF_SUCCESS;
  _sd_flash_pending = true;

  do
  {
    err = sd_flash_write((uint32_t *) addr, src, FLASH_PAGE_SIZE / 4);
    if ( err == NRF_ERROR_BUSY ) sd_app_evt_wait();
  } while ( err == NRF_ERROR_BUSY );

  if ( err != NRF_SUCCESS )
  {
    _sd_flash_pending = false;
    PRINTF("sd_flash_write(0x%08lX) failed: 0x%lX\r\n", addr, err);
    return false;
  }

  return _sd_flash_wait_done() == NRF_SUCCESS;
}

void flash_nrf5x_flush (bool need_erase)
{
  if ( _fl_addr == FLASH_CACHE_INVALID_ADDR ) return;

  // skip the write if contents matches
  if ( memcmp(_fl_buf, (void *) _fl_addr, FLASH_PAGE_SIZE) != 0 )
  {
    // When the SoftDevice is enabled (e.g. dual-transport DFU mode where
    // both USB MSC and BLE DFU are alive simultaneously), direct NVMC
    // register access silently no-ops — the SD owns NVMC. Route through
    // the SD flash API in that case. This is the same path pstorage_raw
    // takes for the BLE OTA DFU write side; we use it for the USB UF2
    // write side so ghostfat works when SD is up.
    if ( _sd_is_running() )
    {
      if ( need_erase )
      {
        PRINTF("(SD) Erase 0x%08lX and ", _fl_addr);
        if ( !_sd_flash_erase_page(_fl_addr) )
        {
          // Erase failed; abort this flush. The bootloader will eventually
          // give up after the MSC retries time out, which is the right
          // behavior — better than silently leaving stale flash.
          _fl_addr = FLASH_CACHE_INVALID_ADDR;
          return;
        }
      }

      PRINTF("(SD) Write 0x%08lX\r\n", _fl_addr);
      (void) _sd_flash_write_page(_fl_addr, (uint32_t *) _fl_buf);
    }
    else
    {
      // - nRF52832 dfu via uart can miss incoming byte when erasing because cpu is blocked for > 2ms.
      // Since dfu_prepare_func_app_erase() already erase the page for us, we can skip it here.
      // - nRF52840 dfu serial/uf2 are USB-based which are DMA and should have no problems.
      //
      // Note: MSC uf2 does not erase page in advance like dfu serial
      if ( need_erase )
      {
        PRINTF("Erase and ");
        nrfx_nvmc_page_erase(_fl_addr);
      }

      PRINTF("Write 0x%08lX\r\n", _fl_addr);
      nrfx_nvmc_words_write(_fl_addr, (uint32_t *) _fl_buf, FLASH_PAGE_SIZE / 4);
    }
  }

  _fl_addr = FLASH_CACHE_INVALID_ADDR;
}

void flash_nrf5x_write (uint32_t dst, void const *src, int len, bool need_erase)
{
  uint32_t newAddr = dst & ~(FLASH_PAGE_SIZE - 1);

  if ( newAddr != _fl_addr )
  {
    flash_nrf5x_flush(need_erase);
    _fl_addr = newAddr;
    memcpy(_fl_buf, (void *) newAddr, FLASH_PAGE_SIZE);
  }
  memcpy(_fl_buf + (dst & (FLASH_PAGE_SIZE - 1)), src, len);
}

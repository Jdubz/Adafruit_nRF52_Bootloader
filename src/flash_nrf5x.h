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

#ifndef FLASH_NRF5X_H_
#define FLASH_NRF5X_H_

#include <stdint.h>
#include <stdbool.h>

#include "nrfx_nvmc.h"

#ifdef __cplusplus
 extern "C" {
#endif

void flash_nrf5x_write (uint32_t dst, void const *src, int len, bool need_erase);
void flash_nrf5x_flush (bool need_erase);

// Hook invoked from main.c proc_soc() for each SoC event. Lets the flash
// layer pick up NRF_EVT_FLASH_OPERATION_{SUCCESS,ERROR} when SD-aware
// writes are in flight. Safe to call with any event id.
void flash_nrf5x_sd_event (uint32_t soc_evt);

#ifdef __cplusplus
 }
#endif

#endif /* FLASH_NRF5X_H_ */

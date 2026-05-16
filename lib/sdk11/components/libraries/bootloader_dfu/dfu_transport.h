/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup nrf_dfu_transport DFU transport API.
 * @{     
 *  
 * @brief DFU transport module interface.
 */
 
#ifndef DFU_TRANSPORT_H__
#define DFU_TRANSPORT_H__

#include <stdint.h>

/**@brief Function for starting the update of Device Firmware.
 *
 * @retval NRF_SUCCESS Operation success.   
 */
uint32_t dfu_transport_serial_update_start(void);

/**@brief Function for closing the transport layer.
 *
 * @retval NRF_SUCCESS Operation success.    
 */
uint32_t dfu_transport_serial_close(void);


uint32_t dfu_transport_ble_update_start(void);
uint32_t dfu_transport_ble_close();

/**@brief Pause / resume BLE advertising during USB MSC activity.
 *
 * On dual-transport DFU mode (BLE + USB MSC simultaneously) the SoftDevice
 * shares a flash op scheduler between the BLE radio and pstorage/sd_flash_*.
 * Dense BLE advertising starves the flash op queue, stretching
 * flash_nrf5x_flush() wait times from milliseconds to tens of seconds.
 *
 * The USB MSC path calls pause() on first WRITE10 and resume() on abort.
 * Success path lets the system reset clear advertising state. Both are
 * no-ops when BLE isn't advertising (no BLE init / central connected).
 */
void dfu_transport_ble_advertising_pause(void);
void dfu_transport_ble_advertising_resume(void);

#endif // DFU_TRANSPORT_H__

/**@} */

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
 
/** @file
 *
 * @defgroup memory_pool_internal Memory Pool Internal
 * @{
 * @ingroup memory_pool
 *
 * @brief Memory pool internal definitions
 */
 
#ifndef MEM_POOL_INTERNAL_H__
#define MEM_POOL_INTERNAL_H__

#define TX_BUF_SIZE       4u    /**< TX buffer size in bytes. */
// Bumped 32 → 256 to accommodate the ATT_MTU=247 bump in main.c +
// dfu_transport_ble.c (OPEN_ISSUES §3.3). At MTU=247 the per-packet
// data payload reaches 244 bytes; the old 32-byte buffer would
// overflow on every full-size DFU data write. 256 is the next
// power-of-two ≥244 with comfortable padding. RAM cost: 8 buffers ×
// (256 - 32) = 1.8 KB extra in the BL — well within the available
// headroom from the linker's 0x20008000 RAM start vs. 256 KB total.
#define RX_BUF_SIZE       256u  /**< RX buffer size in bytes. */

#define RX_BUF_QUEUE_SIZE 8u    /**< RX buffer element size. */

#endif // MEM_POOL_INTERNAL_H__
 
/** @} */

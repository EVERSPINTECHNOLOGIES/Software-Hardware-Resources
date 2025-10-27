/**********************************************************************************************************************
 * File Name    : everspin.c
 * Release      : 1.2 (DFIM autorun + LEDs + fixes)
 * Description  : Everspin MRAM Single-SPI driver/tests for Renesas RA (manual CS), FreeRTOS timing.
 *********************************************************************************************************************/

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bsp_api.h"
#include "common_data.h"
#include "common_utils.h"
#include "everspin.h"
#include "r_spi.h"
#include "r_ioport.h"

/* Forward declarations for internal helpers defined later */
static fsp_err_t read_chunked(spi_ctrl_t *p_api_ctrl, bsp_io_port_pin_t cs,
                              uint32_t addr, uint8_t *rx, uint32_t len, uint32_t chunk_len);
static fsp_err_t chunked_write_verified(spi_ctrl_t *p_api_ctrl, bsp_io_port_pin_t cs,
                                        uint32_t addr, const uint8_t *tx, uint32_t len);

#ifndef BUFFER_LINE_LENGTH
#define BUFFER_LINE_LENGTH (256u)
#endif
static char evr_print_buf[BUFFER_LINE_LENGTH];

static inline void evr_puts(const char *s)
{
#if EVR_DEBUG
    print_to_console((char *) s);
#else
    (void) s;
#endif
}

static inline void evr_printf(const char *fmt, ...)
{
#if EVR_DEBUG
    va_list args; va_start(args, fmt);
    vsnprintf(evr_print_buf, sizeof(evr_print_buf), fmt, args);
    va_end(args);
    print_to_console(evr_print_buf);
#else
    (void) fmt;
#endif
}

static volatile bool     g_spi_transfer_complete = false;

void spi_callback(spi_callback_args_t *p_args)
{
    if (p_args && (p_args->event == SPI_EVENT_TRANSFER_COMPLETE))
    {
        g_spi_transfer_complete = true;
    }
}

static bool wait_xfer_complete(uint32_t timeout_ms)
{
    uint32_t remaining = timeout_ms;
    while (!g_spi_transfer_complete && remaining--)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return g_spi_transfer_complete;
}

/* --------------------------------------------------------------------------------------------------------------------
 * Manufacturing helpers: PASS/FAIL LEDs and DFIM marker
 * ------------------------------------------------------------------------------------------------------------------ */
static void evr_led_pass(bool on)
{
#if defined(EVR_LED_PASS_PIN)
    R_IOPORT_PinWrite(&g_ioport_ctrl, EVR_LED_PASS_PIN, on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
#else
    (void) on;
#endif
}

static void evr_led_fail(bool on)
{
#if defined(EVR_LED_FAIL_PIN)
    R_IOPORT_PinWrite(&g_ioport_ctrl, EVR_LED_FAIL_PIN, on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
#else
    (void) on;
#endif
}

static void evr_led_blink_fail(uint32_t times)
{
    for (uint32_t i = 0; i < times; i++)
    {
        evr_led_fail(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        evr_led_fail(false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static bool evr_marker_present(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs)
{
    const uint32_t addr = EVR_ADDR_DFIM_MARKER;
    const size_t   len  = sizeof(EVR_DFIM_MARKER_STR) - 1;
    uint8_t rx[32] = {0};
    if (len > sizeof(rx)) return false;
    if (FSP_SUCCESS != read_chunked(p_api_ctrl, cs, addr, rx, (uint32_t)len, (uint32_t)len)) return false;
    return (0 == memcmp(rx, EVR_DFIM_MARKER_STR, len));
}

static fsp_err_t evr_marker_write(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs)
{
    const uint32_t addr = EVR_ADDR_DFIM_MARKER;
    const size_t   len  = sizeof(EVR_DFIM_MARKER_STR) - 1;
    uint8_t tx[32] = {0};
    memcpy(tx, EVR_DFIM_MARKER_STR, len);
    return chunked_write_verified(p_api_ctrl, cs, addr, tx, (uint32_t)len);
}

/* --------------------------------------------------------------------------------------------------------------------
 * Test 1: JEDEC ID
 * ------------------------------------------------------------------------------------------------------------------ */
fsp_err_t everspin_spi_jedec(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, uint8_t out_id[3])
{
    if (!out_id) return FSP_ERR_INVALID_POINTER;

    fsp_err_t err;
#if EVR_DEBUG
    print_to_console("\r\n[SPI] JEDEC ID Test...\r\n");
#endif
    memset(out_id, 0, 3);

    uint8_t tx = EVERSPIN_READ_ID_SPI;
    uint8_t rx[4] = {0};

    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->writeRead(p_api_ctrl, &tx, rx, sizeof(rx), SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);
        return err;
    }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS))
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);
        return FSP_ERR_TIMEOUT;
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    out_id[0] = rx[1];
    out_id[1] = rx[2];
    out_id[2] = rx[3];

    return ((out_id[0] | out_id[1] | out_id[2]) != 0) ? FSP_SUCCESS : FSP_ERR_ABORTED;
}

/* --------------------------------------------------------------------------------------------------------------------
 * Test 2: 4-byte Basic Read/Write
 * ------------------------------------------------------------------------------------------------------------------ */
fsp_err_t everspin_spi_read_write_test(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs)
{
    fsp_err_t err;
#if EVR_DEBUG
    print_to_console("\r\n[SPI] Basic Read/Write Test...\r\n");
#endif

    const uint32_t addr = EVR_ADDR_BASIC;
    uint8_t wren = MRAM_WRITE_ENABLE_CMD;
    uint8_t cmd_addr[4];
    uint8_t wr[4] = {0xAA, 0x55, 0x33, 0xCC};
    uint8_t rd[4] = {0};

    /* WREN */
    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, &wren, 1, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    /* WRITE header */
    cmd_addr[0] = MRAM_WRITE_CMD;
    cmd_addr[1] = (uint8_t)(addr >> 16);
    cmd_addr[2] = (uint8_t)(addr >> 8);
    cmd_addr[3] = (uint8_t)(addr);

    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, cmd_addr, 4, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }

    /* WRITE payload */
    g_spi_transfer_complete = false;
    err = g_spi1.p_api->write(p_api_ctrl, wr, sizeof(wr), SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    /* READ header */
    cmd_addr[0] = MRAM_READ_CMD;

    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, cmd_addr, 4, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }

    /* READ payload */
    g_spi_transfer_complete = false;
    err = g_spi1.p_api->read(p_api_ctrl, rd, sizeof(rd), SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    return (0 == memcmp(wr, rd, sizeof(wr))) ? FSP_SUCCESS : FSP_ERR_ABORTED;
}

/* Pattern seed for tests */
static const uint8_t * sp_source = (const uint8_t *)
    "1234567891234567896789123456789678912345678967891"
    "12345678912345678967891234567896789123456789678912"
    "123456789123456789678912345678967891234567896789"
    "123456789123456789678912345678967891234567896789123456789678912"
    "12345678912345678967891234567896789123456789678912345678967891234567"
    "1234567891234567896789123456789678912345678967891234567896789123456789"
    "1234567891234567896789123456789678912345678967891234567896789123456"
    "123456789123456789678912345678967891234567896789123456789678912345678"
    "123456789123456789678912345678967891234567896789123456789678912345678";

/* --------------------------------------------------------------------------------------------------------------------
 * Internal helpers for chunked IO
 * ------------------------------------------------------------------------------------------------------------------ */
static fsp_err_t read_chunked(spi_ctrl_t *p_api_ctrl, bsp_io_port_pin_t cs,
                              uint32_t addr, uint8_t *rx, uint32_t len, uint32_t chunk_len)
{
    fsp_err_t err;
    uint8_t hdr[4] = { MRAM_READ_CMD, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)(addr) };

    /* Send header */
    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, hdr, sizeof(hdr), SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }

    /* Read in fixed-size chunks, CS held low */
    uint32_t pos = 0;
    while (pos < len)
    {
        uint32_t chunk = len - pos;
        if (chunk > chunk_len) chunk = chunk_len;

        g_spi_transfer_complete = false;
        err = g_spi1.p_api->read(p_api_ctrl, &rx[pos], chunk, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }

        pos += chunk;
    }

    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);
    return FSP_SUCCESS;
}

static fsp_err_t chunked_write_verified(spi_ctrl_t *p_api_ctrl, bsp_io_port_pin_t cs,
                                        uint32_t addr, const uint8_t *tx, uint32_t len)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t pos = 0;

    while (pos < len)
    {
        uint32_t chunk = len - pos;
        if (chunk > PERF_WRITE_CHUNK) chunk = PERF_WRITE_CHUNK;

        /* WREN */
        uint8_t wren = MRAM_WRITE_ENABLE_CMD;
        g_spi_transfer_complete = false;
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
        err = g_spi1.p_api->write(p_api_ctrl, &wren, 1, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

        /* WRITE header */
        uint8_t hdr[4] = { MRAM_WRITE_CMD,
                           (uint8_t)((addr + pos) >> 16),
                           (uint8_t)((addr + pos) >> 8),
                           (uint8_t)(addr + pos) };
        g_spi_transfer_complete = false;
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
        err = g_spi1.p_api->write(p_api_ctrl, hdr, sizeof(hdr), SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }

        /* WRITE payload */
        g_spi_transfer_complete = false;
        err = g_spi1.p_api->write(p_api_ctrl, &tx[pos], chunk, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

        pos += chunk;
    }

    /* Quick probe read to confirm first bytes landed correctly */
    uint8_t probe[PERF_PROBE_READ_LEN] = {0};
    err = read_chunked(p_api_ctrl, cs, addr, probe, sizeof(probe), sizeof(probe));
    if (FSP_SUCCESS != err) return err;

    if (0 != memcmp(probe, tx, sizeof(probe))) { return FSP_ERR_ABORTED; }
    return FSP_SUCCESS;
}

/* --------------------------------------------------------------------------------------------------------------------
 * Test 3: 64-byte Enhanced Pattern (16B chunked)
 * ------------------------------------------------------------------------------------------------------------------ */
fsp_err_t everspin_spi_pattern_test(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs)
{
    const uint32_t test_address = EVR_ADDR_PATTERN;
    const uint32_t test_size    = 64;
    fsp_err_t err = FSP_SUCCESS;

    uint8_t *wr = (uint8_t *) pvPortMalloc(test_size);
    uint8_t *rd = (uint8_t *) pvPortMalloc(test_size);
    if (!wr || !rd) { if (wr) vPortFree(wr); if (rd) vPortFree(rd); return FSP_ERR_OUT_OF_MEMORY; }

    size_t seed_len = strlen((const char *) sp_source);
    for (uint32_t i = 0; i < test_size; i++) wr[i] = (uint8_t) ((i + sp_source[i % seed_len]) & 0xFF);
    memset(rd, 0, test_size);

    /* Write in 16B chunks */
    uint32_t pos = 0;
    while (pos < test_size)
    {
        uint32_t chunk = test_size - pos;
        if (chunk > 16u) chunk = 16u;

        /* WREN */
        uint8_t wren = MRAM_WRITE_ENABLE_CMD;
        g_spi_transfer_complete = false;
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
        err = g_spi1.p_api->write(p_api_ctrl, &wren, 1, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); goto done; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); err = FSP_ERR_TIMEOUT; goto done; }
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

        /* WRITE header */
        uint8_t hdr[4] = { MRAM_WRITE_CMD,
                           (uint8_t)((test_address + pos) >> 16),
                           (uint8_t)((test_address + pos) >> 8),
                           (uint8_t)(test_address + pos) };
        g_spi_transfer_complete = false;
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
        err = g_spi1.p_api->write(p_api_ctrl, hdr, sizeof(hdr), SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); goto done; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); err = FSP_ERR_TIMEOUT; goto done; }

        /* WRITE payload */
        g_spi_transfer_complete = false;
        err = g_spi1.p_api->write(p_api_ctrl, &wr[pos], chunk, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); goto done; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); err = FSP_ERR_TIMEOUT; goto done; }
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

        pos += chunk;
    }

    /* Read back in 16B chunks */
    pos = 0;
    while (pos < test_size)
    {
        uint32_t chunk = test_size - pos;
        if (chunk > 16u) chunk = 16u;

        uint8_t hdr[4] = { MRAM_READ_CMD,
                           (uint8_t)((test_address + pos) >> 16),
                           (uint8_t)((test_address + pos) >> 8),
                           (uint8_t)(test_address + pos) };
        g_spi_transfer_complete = false;
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
        err = g_spi1.p_api->write(p_api_ctrl, hdr, sizeof(hdr), SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); goto done; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); err = FSP_ERR_TIMEOUT; goto done; }

        g_spi_transfer_complete = false;
        err = g_spi1.p_api->read(p_api_ctrl, &rd[pos], chunk, SPI_BIT_WIDTH_8_BITS);
        if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); goto done; }
        if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); err = FSP_ERR_TIMEOUT; goto done; }
        R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

        pos += chunk;
    }

    err = (0 == memcmp(wr, rd, test_size)) ? FSP_SUCCESS : FSP_ERR_ABORTED;

done:
    if (wr) vPortFree(wr);
    if (rd) vPortFree(rd);
    return err;
}

/* --------------------------------------------------------------------------------------------------------------------
 * Test 4: Performance (chunked write, adaptive read)
 * ------------------------------------------------------------------------------------------------------------------ */
fsp_err_t everspin_spi_performance_test(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs,
                                        uint32_t block_size_kb, uint32_t *p_write_time_us, uint32_t *p_read_time_us)
{
    if (!p_write_time_us || !p_read_time_us) return FSP_ERR_INVALID_POINTER;

    const uint32_t len = block_size_kb * 1024u;
    const uint32_t addr = EVR_ADDR_PERF;
    const uint32_t us_per_tick = (1000000UL / configTICK_RATE_HZ);

    fsp_err_t err = FSP_SUCCESS;
    uint8_t *wr = (uint8_t *) pvPortMalloc(len);
    uint8_t *rd = (uint8_t *) pvPortMalloc(len);
    if (!wr || !rd) { if (wr) vPortFree(wr); if (rd) vPortFree(rd); return FSP_ERR_OUT_OF_MEMORY; }

    /* Generate pattern */
    size_t seed_len = strlen((const char *) sp_source);
    for (uint32_t i = 0; i < len; i++) wr[i] = (uint8_t)((i + sp_source[i % seed_len]) & 0xFF);
    memset(rd, 0, len);

    /* WRITE (chunked) */
    TickType_t t0 = xTaskGetTickCount();
    err = chunked_write_verified(p_api_ctrl, cs, addr, wr, len);
    if (FSP_SUCCESS != err) { goto cleanup; }
    TickType_t t1 = xTaskGetTickCount();
    *p_write_time_us = (uint32_t) ((t1 - t0) * us_per_tick);

    /* READ (adaptive) */
    uint32_t read_time_us = 0;
    err = read_chunked(p_api_ctrl, cs, addr, rd, len, PERF_READ_CHUNK_PRIMARY);
    if (FSP_SUCCESS != err) err = read_chunked(p_api_ctrl, cs, addr, rd, len, PERF_READ_CHUNK_FALLBACK1);
    if (FSP_SUCCESS != err) err = read_chunked(p_api_ctrl, cs, addr, rd, len, PERF_READ_CHUNK_FALLBACK2);
    if (FSP_SUCCESS != err) { goto cleanup; }

    /* Verify */
    if (0 != memcmp(wr, rd, len)) { err = FSP_ERR_ABORTED; goto cleanup; }
    *p_read_time_us = (uint32_t) ((xTaskGetTickCount() - t1) * us_per_tick);

cleanup:
    if (wr) vPortFree(wr);
    if (rd) vPortFree(rd);
    return err;
}

/* --------------------------------------------------------------------------------------------------------------------
 * DFIM helpers: Status Register read/write and Write Disable
 * ------------------------------------------------------------------------------------------------------------------ */
fsp_err_t everspin_dfim_read_status(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, uint8_t *out_sr)
{
    if (!out_sr) return FSP_ERR_INVALID_POINTER;
    fsp_err_t err;
    uint8_t cmd = MRAM_READ_STATUS_CMD;
    uint8_t rx[2] = {0};

    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->writeRead(p_api_ctrl, &cmd, rx, sizeof(rx), SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    *out_sr = rx[1];
    return FSP_SUCCESS;
}

fsp_err_t everspin_dfim_write_disable(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs)
{
    fsp_err_t err;
    uint8_t cmd = MRAM_WRITE_DISABLE_CMD;
    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, &cmd, 1, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);
    return FSP_SUCCESS;
}

fsp_err_t everspin_dfim_write_status(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, uint8_t value)
{
    fsp_err_t err;
    /* Per datasheet, usually requires WREN prior to WRSR. */
    uint8_t wren = MRAM_WRITE_ENABLE_CMD;
    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, &wren, 1, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    /* Write SR */
    uint8_t tx[2] = { MRAM_WRITE_STATUS_CMD, value };
    g_spi_transfer_complete = false;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_LOW);
    err = g_spi1.p_api->write(p_api_ctrl, tx, sizeof(tx), SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return err; }
    if (!wait_xfer_complete(EVR_XFER_TIMEOUT_MS)) { R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH); return FSP_ERR_TIMEOUT; }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs, BSP_IO_LEVEL_HIGH);

    return FSP_SUCCESS;
}

/* --------------------------------------------------------------------------------------------------------------------
 * Autorun DFIM detection (EST3000 Rev 2.1) with verbose section references
 * ------------------------------------------------------------------------------------------------------------------ */
fsp_err_t everspin_autorun_dfim_check(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, int verbose)
{
    /* Reference: EST3000 "Device Initialization, Power Cycle, System Recovery" Rev 2.1
       Sections 4–7 describe reset, SR/NVCR flows; Section 12 indicates device READY.
       This check is NON-INTRUSIVE: it reads SR and looks for a user-space marker only. */
    if (verbose) {
        print_to_console("\r\n[Autorun] EST3000 Rev 2.1 DFIM Check (Sections 4–7)\r\n");
        print_to_console("  • Sec 4: JESD Reset → 1S mode (invoked by board init)\r\n");
        print_to_console("  • Sec 5: SR verify via 0x05 (read), 0x01 (write) [check only]\r\n");
        print_to_console("  • Sec 6: NVCR 0xB1/0xB5 [not executed in check mode]\r\n");
        print_to_console("  • Sec 7: DFIM Exit VCR 0x1E→0x00 [not executed in check mode]\r\n");
    }

    /* 0) Marker check in array (non-intrusive) */
    bool marker = evr_marker_present(p_api_ctrl, cs);
    if (verbose) {
        print_to_console(marker ? "  • Marker: PRESENT (DFIM previously completed)\r\n"
                                : "  • Marker: ABSENT (no prior DFIM record)\r\n");
    }

    /* 1) Read Status Register (0x05) */
    uint8_t sr = 0;
    fsp_err_t err = everspin_dfim_read_status(p_api_ctrl, cs, &sr);
    if (FSP_SUCCESS != err) {
        print_to_console("  ✖ SR read failed (0x05).\r\n");
        evr_led_blink_fail(3);
        return err;
    }
    if (verbose) {
        char line[80];
        snprintf(line, sizeof(line), "  • SR (0x05) = 0x%02X  [Sec 5]\r\n", sr);
        print_to_console(line);
    }

    /* Consider BP bits [7:2] clear ⇒ array unprotected; WEL(bit1) is transient. */
    bool bp_clear = ((sr & 0xFC) == 0x00);

    bool already_done = marker && bp_clear;
    if (already_done) {
        print_to_console("RESULT: DFIM already completed — device READY. [Sec 12]\r\n");
        evr_led_pass(true);
        evr_led_fail(false);
        return FSP_SUCCESS;
    } else {
        print_to_console("RESULT: DFIM initialization REQUIRED. [Sec 4–7]\r\n");
        evr_led_pass(false);
        evr_led_blink_fail(2);
        return FSP_SUCCESS;
    }
}

fsp_err_t everspin_dfim_check_again(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, int verbose)
{
    return everspin_autorun_dfim_check(p_api_ctrl, cs, verbose);
}

/**********************************************************************************************************************
 * End of file everspin.c - Release 1.2
 **********************************************************************************************************************/
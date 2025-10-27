/**********************************************************************************************************************
 * File Name    : everspin.h
 * Release      : 1.2 (DFIM autorun + LEDs)
 * Description  : Public API and configuration for Everspin MRAM Single-SPI tests (Renesas RA / FreeRTOS).
 *********************************************************************************************************************/

#ifndef EVERSPIN_H
#define EVERSPIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Includes                                                                  */
/* -------------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "bsp_api.h"
#include "r_spi.h"

/* -------------------------------------------------------------------------- */
/*  Versioning                                                                */
/* -------------------------------------------------------------------------- */
#define EVR_VERSION_STR             "1.2"

/* -------------------------------------------------------------------------- */
/*  Build-time options                                                        */
/* -------------------------------------------------------------------------- */
#ifndef EVR_DEBUG
#define EVR_DEBUG                   (0)     /* 0 = quiet, 1 = verbose diagnostics */
#endif

/* Manual-chip-select GPIO (set to your board pin) */
#ifndef EVERSPIN_CS_PIN
#define EVERSPIN_CS_PIN             (BSP_IO_PORT_04_PIN_13) /* P413 by default */
#endif

/* -------------------------------------------------------------------------- */
/*  MRAM opcodes (per Everspin Serial MRAM datasheet)                         */
/* -------------------------------------------------------------------------- */
#define MRAM_WRITE_ENABLE_CMD       (0x06u)
#define MRAM_WRITE_DISABLE_CMD      (0x04u)
#define MRAM_WRITE_STATUS_CMD       (0x01u)
#define MRAM_READ_STATUS_CMD        (0x05u)
#define MRAM_WRITE_CMD              (0x02u)
#define MRAM_READ_CMD               (0x03u)
#define EVERSPIN_READ_ID_SPI        (0x9Fu)

/* -------------------------------------------------------------------------- */
/*  Test address map                                                          */
/* -------------------------------------------------------------------------- */
#ifndef EVR_ADDR_BASIC
#define EVR_ADDR_BASIC              (0x00001000u)
#endif
#ifndef EVR_ADDR_PATTERN
#define EVR_ADDR_PATTERN            (0x00002000u)
#endif
#ifndef EVR_ADDR_PERF
#define EVR_ADDR_PERF               (0x00004000u)
#endif

/* Performance/read/write tunables */
#ifndef PERF_WRITE_CHUNK
#define PERF_WRITE_CHUNK            (64u)
#endif
#ifndef PERF_READ_CHUNK_PRIMARY
#define PERF_READ_CHUNK_PRIMARY     (256u)
#endif
#ifndef PERF_READ_CHUNK_FALLBACK1
#define PERF_READ_CHUNK_FALLBACK1   (128u)
#endif
#ifndef PERF_READ_CHUNK_FALLBACK2
#define PERF_READ_CHUNK_FALLBACK2   (64u)
#endif
#ifndef PERF_PROBE_READ_LEN
#define PERF_PROBE_READ_LEN         (16u)
#endif
#ifndef EVR_XFER_TIMEOUT_MS
#define EVR_XFER_TIMEOUT_MS         (1000u)
#endif

/* -------------------------------------------------------------------------- */
/*  Manufacturing / Autorun options                                           */
/* -------------------------------------------------------------------------- */
#ifndef EVR_MFG_AUTORUN
#define EVR_MFG_AUTORUN              (1)     /* 1=autorun DFIM check at boot (non-intrusive) */
#endif

#ifndef EVR_VERBOSE_AUTORUN
#define EVR_VERBOSE_AUTORUN          (1)     /* 1=print detailed step logs with section refs */
#endif

/* PASS/FAIL LED pins — set to match your board (or ignore) */
#ifndef EVR_LED_PASS_PIN
#define EVR_LED_PASS_PIN             (BSP_IO_PORT_01_PIN_00)  /* TODO: map to your board */
#endif
#ifndef EVR_LED_FAIL_PIN
#define EVR_LED_FAIL_PIN             (BSP_IO_PORT_01_PIN_01)  /* TODO: map to your board */
#endif

/* DFIM-complete non-intrusive marker (user array space) */
#ifndef EVR_ADDR_DFIM_MARKER
#define EVR_ADDR_DFIM_MARKER         (0x00007F00u) /* 256 bytes below 0x8000 for safety */
#endif
#ifndef EVR_DFIM_MARKER_STR
#define EVR_DFIM_MARKER_STR          "DFIM-EST3000-REV2.1"
#endif

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */
fsp_err_t everspin_spi_jedec(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, uint8_t out_id[3]);
fsp_err_t everspin_spi_read_write_test(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs);
fsp_err_t everspin_spi_pattern_test(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs);
fsp_err_t everspin_spi_performance_test(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs,
                                        uint32_t block_size_kb, uint32_t *p_write_time_us, uint32_t *p_read_time_us);

/* DFIM helpers (safe) */
fsp_err_t everspin_dfim_read_status(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, uint8_t *out_sr);
fsp_err_t everspin_dfim_write_disable(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs);
fsp_err_t everspin_dfim_write_status(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, uint8_t value);

/* Autorun DFIM check (non-intrusive, for MFG) */
fsp_err_t everspin_autorun_dfim_check(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, int verbose);
fsp_err_t everspin_dfim_check_again(spi_ctrl_t * const p_api_ctrl, bsp_io_port_pin_t cs, int verbose);

#ifdef __cplusplus
}
#endif

#endif /* EVERSPIN_H */
/**********************************************************************************************************************
 * End of file everspin.h - Release 1.2
 **********************************************************************************************************************/
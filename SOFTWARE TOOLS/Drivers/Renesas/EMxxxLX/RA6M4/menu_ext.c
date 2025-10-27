/**********************************************************************************************************************
 * File Name    : menu_ext.c
 * Release      : 1.2 (Integrated + DFIM + Autorun)
 * Description  : Menu using ONLY everspin.c functions with Help/README added
 *********************************************************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "everspin.h"
#include "menu_ext.h"
#include "common_utils.h"
#include "FreeRTOS.h"
#include "task.h"

void print_to_console(const char *s);
extern int console_try_getchar(void);
extern const spi_instance_t g_spi1;

/* README snippet omitted for brevity in this clean package */

static void print_result(bool passed)
{
    if (passed) {
        print_to_console("*** PASS ***\r\n");
    } else {
        print_to_console("*** FAIL ***\r\n");
    }
}

static char get_user_input(void)
{
    print_to_console("Enter choice (1-9, q): ");
    int c;
    while ((c = console_try_getchar()) < 0) { vTaskDelay(pdMS_TO_TICKS(10)); }
    char echo[5];
    snprintf(echo, sizeof(echo), "%c\r\n", (char)c);
    print_to_console(echo);
    return (char)c;
}

void ext_display_menu(void)
{
#if EVR_MFG_AUTORUN
    print_to_console("\r\n[Autorun] Running DFIM status check per EST3000 Rev 2.1...\r\n");
    (void) everspin_autorun_dfim_check(&g_spi1_ctrl, EVERSPIN_CS_PIN, EVR_VERBOSE_AUTORUN);
    print_to_console("[Autorun] Done.\r\n\r\n");
#endif

    print_to_console("=== EVERSPIN MRAM TESTS (Integrated + DFIM) ===\r\n");
    print_to_console("Release: 1.2\r\n");
    print_to_console("==============================================\r\n");
    print_to_console("1. JEDEC ID Test\r\n");
    print_to_console("2. Basic Read/Write Test\r\n");
    print_to_console("3. Pattern Test (64 bytes)\r\n");
    print_to_console("4. Performance Test\r\n");
    print_to_console("5. Help / README\r\n");
    print_to_console("6. DFIM: Read Status Register (RDSR)\r\n");
    print_to_console("7. DFIM: Write Disable (WRDI)\r\n");
    print_to_console("8. DFIM: Write Status Register (WRSR) [CAUTION]\r\n");
    print_to_console("9. DFIM: Check/Announce (Auto-Run logic)\r\n");
    print_to_console("q. Quit to Main Menu\r\n");
    print_to_console("==============================================\r\n");

    bool menu_active = true;
    while (menu_active)
    {
        char input = get_user_input();

        switch (input)
        {
            case '1': {
                uint8_t id[3] = {0};
                fsp_err_t err = everspin_spi_jedec(&g_spi1_ctrl, EVERSPIN_CS_PIN, id);
                bool passed = (FSP_SUCCESS == err);
                if (passed) {
                    char msg[60];
                    snprintf(msg, sizeof(msg), "JEDEC ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);
                    print_to_console(msg);
                }
                print_result(passed);
                break;
            }
            case '2': {
                fsp_err_t err = everspin_spi_read_write_test(&g_spi1_ctrl, EVERSPIN_CS_PIN);
                print_result(FSP_SUCCESS == err);
                break;
            }
            case '3': {
                fsp_err_t err = everspin_spi_pattern_test(&g_spi1_ctrl, EVERSPIN_CS_PIN);
                print_result(FSP_SUCCESS == err);
                break;
            }
            case '4': {
                uint32_t sizes[] = {1, 2, 4, 8};
                bool all_passed = true;
                for (int i = 0; i < 4; i++) {
                    uint32_t w=0, r=0;
                    fsp_err_t err = everspin_spi_performance_test(&g_spi1_ctrl, EVERSPIN_CS_PIN, sizes[i], &w, &r);
                    if (FSP_SUCCESS != err) all_passed = false;
                }
                print_result(all_passed);
                break;
            }
            case '5': {
                print_to_console("Help omitted in clean build.\r\n");
                break;
            }
            case '6': {
                uint8_t sr=0; fsp_err_t err = everspin_dfim_read_status(&g_spi1_ctrl, EVERSPIN_CS_PIN, &sr);
                if (FSP_SUCCESS == err) { char msg[40]; snprintf(msg, sizeof(msg), "SR=0x%02X\r\n", sr); print_to_console(msg); }
                print_result(FSP_SUCCESS == err);
                break;
            }
            case '7': {
                fsp_err_t err = everspin_dfim_write_disable(&g_spi1_ctrl, EVERSPIN_CS_PIN);
                print_result(FSP_SUCCESS == err);
                break;
            }
            case '8': {
                print_to_console("Enter 2 hex digits for new SR value (e.g., 00, 02): ");
                int c1, c2;
                while ((c1 = console_try_getchar()) < 0) { vTaskDelay(pdMS_TO_TICKS(10)); }
                while ((c2 = console_try_getchar()) < 0) { vTaskDelay(pdMS_TO_TICKS(10)); }
                char inbuf[5]; inbuf[0] = (char)c1; inbuf[1] = (char)c2; inbuf[2] = 0;
                print_to_console("\r\n");
                unsigned value = 0;
                bool ok = (sscanf(inbuf, "%2x", &value) == 1);
                if (!ok) { print_to_console("Invalid hex.\r\n"); print_result(false); break; }
                fsp_err_t err = everspin_dfim_write_status(&g_spi1_ctrl, EVERSPIN_CS_PIN, (uint8_t)value);
                print_result(FSP_SUCCESS == err);
                break;
            }
            case '9': { // DFIM: Check/Announce (Auto-Run logic)
                fsp_err_t err = everspin_dfim_check_again(&g_spi1_ctrl, EVERSPIN_CS_PIN, 1);
                print_result(FSP_SUCCESS == err);
                break;
            }
            case 'q':
            case 'Q':
                menu_active = false;
                break;
            default:
                print_to_console("Invalid choice. Please try again.\r\n");
                break;
        }
    }

    print_to_console("Returning to main menu.\r\n");
}

/**********************************************************************************************************************
 * End of file menu_ext.c (Release 1.2 Integrated + DFIM + Autorun)
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * File Name    : menu_ext.h
 * Version      : Rev.1.1
 * Description  : Extended menu header for diagnostics
 *********************************************************************************************************************/

#ifndef MENU_EXT_H_
#define MENU_EXT_H_

#ifdef __cplusplus
extern "C" {
#endif

void ext_display_menu(void);
void run_test_simple_spi1(void);
void run_test_spi1_jedec(void);
void run_test_spi1_jedec_manual_cs(void);

#ifdef __cplusplus
}
#endif

#endif /* MENU_EXT_H_ */

/**********************************************************************************************************************
 * End of file menu_ext.h (Rev.1.1)
 **********************************************************************************************************************/
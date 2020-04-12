#ifndef __BOARDS_H__
#define __BOARDS_H__

#ifdef BOARD_NUCLEO_L476RG
 #define 	STM32L4XX_FAMILY
 #include "nucleo-l476rg.h"
#endif
   
#ifdef BOARD_NUCLEO_L152RE
 #define 	STM32L1XX_FAMILY
 #include "nucleo-l152re.h"
#endif

#ifdef BOARD_NUCLEO_L053R8
 #define 	STM32L0XX_FAMILY
 #include "nucleo-l053r8.h"
#endif

#endif // __BOARDS_H__

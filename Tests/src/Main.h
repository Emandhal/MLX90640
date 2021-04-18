/*
 * Main.h
 *
 * Created: 09/04/2020 15:57:04
 *  Author: Fabien
 */


#ifndef MAIN_H_
#define MAIN_H_
//-----------------------------------------------------------------------------
#include <stdint.h>
#include "conf_board.h"
#include "Console.h"
#include "ErrorsDef.h"
#include "Interface/Console_V71Interface.h"
#include "StringTools.h"
//-----------------------------------------------------------------------------


#define EXT2_RST_IRQn        ( PIOA_IRQn )
#define EXT2_RST_ID          ( ID_PIOA )
#define EXT2_RST_PIO         ( PIOA )
#define EXT2_RST_PORT        ( IOPORT_PIOA )
#define EXT2_RST_MASK        ( PIO_PA6 )
#define EXT2_RST_PIO_En        EXT2_RST_PIO->PIO_PER  |= EXT2_RST_MASK
#define EXT2_RST_PIO_Dis       EXT2_RST_PIO->PIO_PDR  |= EXT2_RST_MASK
#define EXT2_RST_PullUp_En     EXT2_RST_PIO->PIO_PUER |= EXT2_RST_MASK
#define EXT2_RST_PullUp_Dis    EXT2_RST_PIO->PIO_PUDR |= EXT2_RST_MASK
#define EXT2_RST_Filter_En     EXT2_RST_PIO->PIO_IFER |= EXT2_RST_MASK
#define EXT2_RST_Filter_Dis    EXT2_RST_PIO->PIO_IFDR |= EXT2_RST_MASK
#define EXT2_RST_Out           EXT2_RST_PIO->PIO_OER  |= EXT2_RST_MASK
#define EXT2_RST_In            EXT2_RST_PIO->PIO_ODR  |= EXT2_RST_MASK
#define EXT2_RST_High          EXT2_RST_PIO->PIO_SODR |= EXT2_RST_MASK
#define EXT2_RST_Low           EXT2_RST_PIO->PIO_CODR |= EXT2_RST_MASK
#define EXT2_RST_Status      ( EXT2_RST_PIO->PIO_PDSR &  EXT2_RST_MASK )


#define EXT2_PWM_IRQn        ( PIOD_IRQn )
#define EXT2_PWM_ID          ( ID_PIOD )
#define EXT2_PWM_PIO         ( PIOD )
#define EXT2_PWM_PORT        ( IOPORT_PIOD )
#define EXT2_PWM_MASK        ( PIO_PD30 )
#define EXT2_PWM_PIO_En        EXT2_PWM_PIO->PIO_PER  |= EXT2_PWM_MASK
#define EXT2_PWM_PIO_Dis       EXT2_PWM_PIO->PIO_PDR  |= EXT2_PWM_MASK
#define EXT2_PWM_PullUp_En     EXT2_PWM_PIO->PIO_PUER |= EXT2_PWM_MASK
#define EXT2_PWM_PullUp_Dis    EXT2_PWM_PIO->PIO_PUDR |= EXT2_PWM_MASK
#define EXT2_PWM_Filter_En     EXT2_PWM_PIO->PIO_IFER |= EXT2_PWM_MASK
#define EXT2_PWM_Filter_Dis    EXT2_PWM_PIO->PIO_IFDR |= EXT2_PWM_MASK
#define EXT2_PWM_Out           EXT2_PWM_PIO->PIO_OER  |= EXT2_PWM_MASK
#define EXT2_PWM_In            EXT2_PWM_PIO->PIO_ODR  |= EXT2_PWM_MASK
#define EXT2_PWM_High          EXT2_PWM_PIO->PIO_SODR |= EXT2_PWM_MASK
#define EXT2_PWM_Low           EXT2_PWM_PIO->PIO_CODR |= EXT2_PWM_MASK
#define EXT2_PWM_Status      ( EXT2_PWM_PIO->PIO_PDSR &  EXT2_PWM_MASK )

//-----------------------------------------------------------------------------



extern volatile uint32_t msCount;              //! Milli-seconds count from start of the system



// System defines
#	define SYSTEM_TICK_MS  (1) // 1ms for system tick



//! Console commands
typedef enum
{
  NO_COMMAND = 0,
  DUMPEE,
  RECALLEE,
  SHOWEE,
  SAVEEE,
  SHOWPARAMS,
} eConsoleCommand;

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************

void ShowError(eERRORRESULT error);

//-----------------------------------------------------------------------------





#endif /* MAIN_H_ */
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
#include "Conf_Console.h"
#include "Console.h"
#include "ErrorsDef.h"
#include "Interface/Console_V71Interface.h"
#include "StringTools.h"
#include "TWIHS_V71.h"
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


#define EXT2_AN_IRQn        ( PIOD_IRQn )
#define EXT2_AN_ID          ( ID_PIOD )
#define EXT2_AN_PIO         ( PIOD )
#define EXT2_AN_PORT        ( IOPORT_PIOD )
#define EXT2_AN_MASK        ( PIO_PD30 )
#define EXT2_AN_PIO_En        EXT2_AN_PIO->PIO_PER  |= EXT2_AN_MASK
#define EXT2_AN_PIO_Dis       EXT2_AN_PIO->PIO_PDR  |= EXT2_AN_MASK
#define EXT2_AN_PullUp_En     EXT2_AN_PIO->PIO_PUER |= EXT2_AN_MASK
#define EXT2_AN_PullUp_Dis    EXT2_AN_PIO->PIO_PUDR |= EXT2_AN_MASK
#define EXT2_AN_Filter_En     EXT2_AN_PIO->PIO_IFER |= EXT2_AN_MASK
#define EXT2_AN_Filter_Dis    EXT2_AN_PIO->PIO_IFDR |= EXT2_AN_MASK
#define EXT2_AN_Out           EXT2_AN_PIO->PIO_OER  |= EXT2_AN_MASK
#define EXT2_AN_In            EXT2_AN_PIO->PIO_ODR  |= EXT2_AN_MASK
#define EXT2_AN_High          EXT2_AN_PIO->PIO_SODR |= EXT2_AN_MASK
#define EXT2_AN_Low           EXT2_AN_PIO->PIO_CODR |= EXT2_AN_MASK
#define EXT2_AN_Status      ( EXT2_AN_PIO->PIO_PDSR &  EXT2_AN_MASK )


#define EXT2_ADCm_IRQn        ( PIOC_IRQn )
#define EXT2_ADCm_ID          ( ID_PIOC )
#define EXT2_ADCm_PIO         ( PIOC )
#define EXT2_ADCm_PORT        ( IOPORT_PIOC )
#define EXT2_ADCm_MASK        ( PIO_PC13 )
#define EXT2_ADCm_PIO_En        EXT2_ADCm_PIO->PIO_PER  |= EXT2_ADCm_MASK
#define EXT2_ADCm_PIO_Dis       EXT2_ADCm_PIO->PIO_PDR  |= EXT2_ADCm_MASK
#define EXT2_ADCm_PullUp_En     EXT2_ADCm_PIO->PIO_PUER |= EXT2_ADCm_MASK
#define EXT2_ADCm_PullUp_Dis    EXT2_ADCm_PIO->PIO_PUDR |= EXT2_ADCm_MASK
#define EXT2_ADCm_Filter_En     EXT2_ADCm_PIO->PIO_IFER |= EXT2_ADCm_MASK
#define EXT2_ADCm_Filter_Dis    EXT2_ADCm_PIO->PIO_IFDR |= EXT2_ADCm_MASK
#define EXT2_ADCm_Out           EXT2_ADCm_PIO->PIO_OER  |= EXT2_ADCm_MASK
#define EXT2_ADCm_In            EXT2_ADCm_PIO->PIO_ODR  |= EXT2_ADCm_MASK
#define EXT2_ADCm_High          EXT2_ADCm_PIO->PIO_SODR |= EXT2_ADCm_MASK
#define EXT2_ADCm_Low           EXT2_ADCm_PIO->PIO_CODR |= EXT2_ADCm_MASK
#define EXT2_ADCm_Status      ( EXT2_ADCm_PIO->PIO_PDSR &  EXT2_ADCm_MASK )


#define EXT2_GPIO_IRQn        ( PIOA_IRQn )
#define EXT2_GPIO_ID          ( ID_PIOA )
#define EXT2_GPIO_PIO         ( PIOA )
#define EXT2_GPIO_PORT        ( IOPORT_PIOA )
#define EXT2_GPIO_MASK        ( PIO_PA24 )
#define EXT2_GPIO_PIO_En        EXT2_GPIO_PIO->PIO_PER  |= EXT2_GPIO_MASK
#define EXT2_GPIO_PIO_Dis       EXT2_GPIO_PIO->PIO_PDR  |= EXT2_GPIO_MASK
#define EXT2_GPIO_PullUp_En     EXT2_GPIO_PIO->PIO_PUER |= EXT2_GPIO_MASK
#define EXT2_GPIO_PullUp_Dis    EXT2_GPIO_PIO->PIO_PUDR |= EXT2_GPIO_MASK
#define EXT2_GPIO_Filter_En     EXT2_GPIO_PIO->PIO_IFER |= EXT2_GPIO_MASK
#define EXT2_GPIO_Filter_Dis    EXT2_GPIO_PIO->PIO_IFDR |= EXT2_GPIO_MASK
#define EXT2_GPIO_Out           EXT2_GPIO_PIO->PIO_OER  |= EXT2_GPIO_MASK
#define EXT2_GPIO_In            EXT2_GPIO_PIO->PIO_ODR  |= EXT2_GPIO_MASK
#define EXT2_GPIO_High          EXT2_GPIO_PIO->PIO_SODR |= EXT2_GPIO_MASK
#define EXT2_GPIO_Low           EXT2_GPIO_PIO->PIO_CODR |= EXT2_GPIO_MASK
#define EXT2_GPIO_Status      ( EXT2_GPIO_PIO->PIO_PDSR &  EXT2_GPIO_MASK )

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
  SHOWFRAME,
} eConsoleCommand;

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************

void ShowError(eERRORRESULT error);

//-----------------------------------------------------------------------------





#endif /* MAIN_H_ */
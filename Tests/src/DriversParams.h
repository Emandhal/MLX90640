/*******************************************************************************
  File name:    DriversParams.h
  Author:       FMA
  Version:      1.0
  Date (d/m/y): 29/04/2020
  Description:  Drivers parameters for the DEMO

  History :
*******************************************************************************/
#ifndef CANEXTFUNCTIONS_H_
#define CANEXTFUNCTIONS_H_
//=============================================================================

//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#include "stdio.h"
#include <stdarg.h>
#include "47x16.h"
#include "TCA9543A.h"
#include "MLX90640.h"
#include "ErrorsDef.h"
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------




// Structure of the 47L16 used in the demo
extern struct EERAM47x16 EERAM47L16_V71;
# define EERAM_47L16  &EERAM47L16_V71





//-----------------------------------------------------------------------------
// Structure of the TCA9543A used in the demo
extern struct TCA9543A TCA9543A_V71;
# define I2CMUX  &TCA9543A_V71
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Channel 0 I2C Configuration structure of I2CMUX (TCA9543A)
extern struct TCA9543A_I2C I2C0_MUX;
# define I2C0  &I2C0_MUX

// Channel 1 I2C Configuration structure of I2CMUX (TCA9543A)
extern struct TCA9543A_I2C I2C1_MUX;
# define I2C1  &I2C1_MUX
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
extern MLX90640_EEPROM IrCAM_EEPROM;
extern MLX90640_Parameters IrCAM_Params;
extern MLX90640_FrameData IrCAM_FrameData;


// Structure of the MLX90640 used in the demo
extern struct MLX90640 MLX90640_V71;
#define IrCAM  &MLX90640_V71


extern MLX90640_Config IrCAM_Config;
//-----------------------------------------------------------------------------



/*! @brief Detect I2C devices on a TCA9543A I2C channel
 *
 * This function detect devices on the I2C bus and show it on console
 * @param[in] *pI2C Is the I2C channel of the TCA9543A where to detect the I2C devices
 * @return Returns an #eERRORRESULT value enum
 */
//eERRORRESULT I2Cdetect(TCA9543A_I2C *pI2C);

//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------
#endif /* CANEXTFUNCTIONS_H_ */
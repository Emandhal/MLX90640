/*******************************************************************************
  File name:    DriversParams.c
  Author:       FMA
  Version:      1.0
  Date (d/m/y): 29/04/2020
  Description:  Drivers parameters for the DEMO

  History :
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "Main.h"
#include "DriversParams.h"
#include "I2C_Interface.h"
//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
#include "stdafx.h"
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// Configuration structure of the 47L16 with hard I2C on the V71
//=============================================================================
struct EERAM47x16 EERAM47L16_V71 =
{
  .UserDriverData  = NULL,
  //--- Interface driver call functions ---
  .I2C =
  {
    .InterfaceDevice = TWIHS0,
    .fnI2C_Init      = TWIHS_DMA_MasterInit_Gen,
    .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
  },
  .I2CclockSpeed   = BOARD_I2C_CLK_SPEED_HZ,
  //--- Time call function ---
  .fnGetCurrentms  = GetCurrentms_V71,
  //--- Interface clocks ---
  .AddrA2A1        = EERAM47x16_ADDR(1, 0),
};





//********************************************************************************************************************
//=============================================================================
// Configuration structure of the TCA9543A with I2C on the V71
//=============================================================================
struct TCA9543A TCA9543A_V71 =
{
  //--- Interface driver call functions ---
  .I2C =
  {
    .InterfaceDevice = TWIHS0,
    .fnI2C_Init      = TWIHS_DMA_MasterInit_Gen,
    .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
  },
  .I2CclockSpeed     = BOARD_I2C_CLK_SPEED_HZ,
  //--- Device address ---
  .AddrA1A0          = TCA9543A_ADDR(0, 0),
};



// @warning Each Channel and Device tuple should be unique. Only 2 possible tuple on TCA9543A devices
//=============================================================================
// Channel 0 I2C Configuration structure of I2CMUX (TCA9543A)
//=============================================================================
const struct I2C_Interface I2C0_MUX =
{
  //--- Device configuration ---
  .InterfaceDevice = I2CMUX, // Shall be a TCA9543A device
  .fnI2C_Init      = TCA9543A_I2CInit,
  .fnI2C_Transfer  = TCA9543A_I2CTransfer,
  //--- I2C configuration ---
  .Channel         = TCA9543A_SELECT_CHANNEL_0,
};

//=============================================================================
// Channel 1 I2C Configuration structure of I2CMUX (TCA9543A)
//=============================================================================
const struct I2C_Interface I2C1_MUX =
{
  //--- Device configuration ---
  .InterfaceDevice = I2CMUX, // Shall be a TCA9543A device
  .fnI2C_Init      = TCA9543A_I2CInit,
  .fnI2C_Transfer  = TCA9543A_I2CTransfer,
  //--- I2C configuration ---
  .Channel         = TCA9543A_SELECT_CHANNEL_1,
};





//********************************************************************************************************************

COMPILER_ALIGNED(8) MLX90640_EEPROM IrCAM_EEPROM;
MLX90640_Parameters IrCAM_Params;
COMPILER_ALIGNED(8) MLX90640_FrameData IrCAM_FrameData;



//=============================================================================
// Configuration structure of the MLX90640 with I2C on the V71
//=============================================================================
struct MLX90640 MLX90640_V71 =
{
  .UserDriverData    = NULL,
  //--- Interface driver params and call functions ---
  .I2Caddress        = MLX90640_CHIPADDRESS_DEFAULT,
  .I2C =
  {
    //--- Device configuration ---
    .InterfaceDevice = I2CMUX,                    // Connected on a TCA9543A device
    .fnI2C_Init      = TCA9543A_I2CInit,          // Functions to use to communicate through the TCA9543A device
    .fnI2C_Transfer  = TCA9543A_I2CTransfer,      // Functions to use to communicate through the TCA9543A device
    //--- I2C configuration ---
    .Channel         = TCA9543A_SELECT_CHANNEL_0, // Channel of the TCA9543A
  },
  .I2CclockSpeed     = BOARD_I2C_CLK_SPEED_HZ,
  //--- Time call function ---
  .fnGetCurrentms    = GetCurrentms_V71,
  //--- Device EEPROM ---
#if !defined(MLX90640_PRECALCULATE_PIXELS_COEFFS)
  .EEPROM            = &IrCAM_EEPROM,
#endif
  //--- Device parameters ---
  .Params            = &IrCAM_Params,
};



MLX90640_Config IrCAM_Config =
{
  //--- Subpage configuration ---
  .SubpageMode    = MLX90640_MEASURE_ALTERNATE_SUBPAGES,
  .RefreshRate    = MLX90640_IR_REFRESH_RATE_16Hz,
  .ReadingPattern = MLX90640_READING_CHESS_PATTERN_MODE,
  .ADCresolution  = MLX90640_ADC_RESOLUTION_18bits,
  //--- I2C configuration ---
  .I2C_FMpEnable            = false,
  .SetThresholdTo1V8        = false,
  .SetSDAdriverCurrentLimit = false,
};





//********************************************************************************************************************
//=============================================================================
// Get millisecond
//=============================================================================
uint32_t GetCurrentms_V71(void)
{
  return msCount;
}

//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
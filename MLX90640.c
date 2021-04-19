/*******************************************************************************
 * @file    MLX90640.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.1
 * @date    27/02/2021
 * @brief   MLX90640 driver
 *
 * 32x24 IR array
 * Follow datasheet 3901090640 Rev.12 (Dec 2019)
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "MLX90640.h"
//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#include <math.h>
#include <float.h>
#ifdef __cplusplus
#include <cstdint>
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------

// Choose one of the following for the fourth root
#define ftrtf(value)  ( sqrtf(sqrtf(value)) )  // Apply a successive square root for the fourth root
//#define ftrtf(value)  ( powf((value),0.25f) ) // Apply a x^(1/4) for the fourth root

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Detect the byte order of the MCU
//=============================================================================
static uint16_t MLX90640_DetectByteOrder(void)
{
  uint16_t x = 1 | (((uint16_t)2) << (16 - 8));
  uint8_t *cp = (uint8_t*)&x;
  if (*cp == 2) return MLX90640_BIG_ENDIAN; // Big-endian
  return MLX90640_LITTLE_ENDIAN;            // By default, work with little-endian
}



//**********************************************************************************************************************************************************
//=============================================================================
// MLX90640 device initialization
//=============================================================================
eERRORRESULT Init_MLX90640(MLX90640 *pComp, const MLX90640_Config *pConf)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (pConf == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Init == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Initialize internal config ---
  pComp->InternalConfig = MLX90640_DEV_NOT_PARAMETERIZED | MLX90640_DetectByteOrder();

  //--- Check and adjust I2C SCL speed ---
  if (pComp->I2C_ClockSpeed > MLX90640_I2CCLOCK_FMp_MAX) return ERR__I2C_FREQUENCY_ERROR;                // Desired I2C SCL frequency too high for the device
  uint32_t SCLfreq = pComp->I2C_ClockSpeed;
  if ((SCLfreq > MLX90640_I2CCLOCK_FM_MAX) && pConf->I2C_FMp_Enable) SCLfreq = MLX90640_I2CCLOCK_FM_MAX; // FM+ mode is wanted but the device is not yet configured for this so set the FM max frequency
  if (SCLfreq > MLX90640_I2CCLOCK_FM_MAX) return ERR__I2C_FREQUENCY_ERROR;                               // If the SCL frequency is too high and the device will not be configured for FM+ mode then return an error

  //--- Initialize the interface ---
  Error = pComp->fnI2C_Init(pComp->InterfaceDevice, SCLfreq);                 // Init the I2C with a safe SCL clock speed for configuration
  if (Error != ERR_OK) return Error;                                          // If there is an error while calling fnI2C_Init() then return the Error
  if (MLX90640_IsReady(pComp) == false) return ERR__NO_DEVICE_DETECTED;       // Check device presence

  //--- Configure the I2C on device side ---
  Error = MLX90640_ConfigureDeviceI2C(pComp, pConf->I2C_FMp_Enable, pConf->SetThresholdTo1V8, pConf->SetSDAdriverCurrentLimit);
  if (Error != ERR_OK) return Error;                                          // If there is an error while calling MLX90640_ConfigureI2C() then return the Error

  //--- Finally set the desired I2C SCL speed ---
  if ((SCLfreq != pComp->I2C_ClockSpeed) && pConf->I2C_FMp_Enable)            // The working SCL frequency differ from the configuration frequency then
  {
    Error = pComp->fnI2C_Init(pComp->InterfaceDevice, pComp->I2C_ClockSpeed); // Re-init the I2C with the desired SCL clock speed
    if (Error != ERR_OK) return Error;                                        // If there is an error while calling fnI2C_Init() then return the Error
  }

  //--- Configure the device ---
  return MLX90640_ConfigureDevice(pComp, pConf->SubpageMode, pConf->RefreshRate, pConf->ReadingPattern, pConf->ADCresolution);
}



//=============================================================================
// Is the MLX90640 device ready
//=============================================================================
bool MLX90640_IsReady(MLX90640 *pComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return false;
  if (pComp->fnI2C_Transfer == NULL) return false;
#endif
  uint8_t ChipAddrW = pComp->I2Caddress & MLX90640_I2C_WRITE;
  return (pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddrW, NULL, 0, true, true) == ERR_OK); // Send only the chip address and get the Ack flag
}



//=============================================================================
// Is a frame available on the MLX90640 device
//=============================================================================
bool MLX90640_IsFrameAvailable(MLX90640 *pComp)
{
  eERRORRESULT Error;
  MLX90640_Status RegStatus;
  Error = MLX90640_ReadRegister(pComp, RegMLX90640_Status, &RegStatus.Status);
  if (Error != ERR_OK) return false; // If there is an error while calling MLX90640_ReadRegister() then return no frame available
  if ((RegStatus.Status & MLX90640_NEW_DATA_AVAILABLE) > 0) return true;
  return false;
}



//=============================================================================
// Get actual device of the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_GetDeviceID(MLX90640 *pComp, eMLX90640_Devices* device, uint16_t* deviceId1, uint16_t* deviceId2, uint16_t* deviceId3)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (device == NULL)) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  uint16_t Value;
  Error = MLX90640_ReadRegister(pComp, EepMLX90640_DeviceOption, &Value);
  if (Error != ERR_OK) return Error;                                      // If there is an error while calling MLX90640_ReadRegister() then return the error
  Value &= MLX90640_FOV_Mask;
  switch (Value)
  {
    case 0b1000: *device = MLX90640BAB; break;
    case 0b1100: *device = MLX90640BAA; break;
    default: return ERR__UNKNOWN_DEVICE;
  }

  if ((deviceId1 != NULL) && (deviceId2 != NULL) && (deviceId3 != NULL))
  {
    uint16_t DeviceID[3];
    Error = MLX90640_ReadData(pComp, EepMLX90640_DeviceID, &DeviceID[0], sizeof(DeviceID) / sizeof(uint16_t)); // Read values of the DeviceID register
    if (Error != ERR_OK) return Error;                                    // If there is an error while calling MLX90640_ReadData() then return the error
    *deviceId1 = DeviceID[0];
    *deviceId2 = DeviceID[1];
    *deviceId3 = DeviceID[2];
  }
  return ERR_OK;
}





//**********************************************************************************************************************************************************
// Write 2-bytes address the MLX90640 (DO NOT USE DIRECTLY)
static eERRORRESULT __MLX90640_WriteAddress(MLX90640 *pComp, const uint8_t chipAddr, const uint16_t address)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t ChipAddrW = chipAddr & MLX90640_I2C_WRITE;

  //--- Create address ---
  uint8_t Address[sizeof(uint16_t)];
  for (int_fast8_t z = sizeof(uint16_t); --z >= 0;) Address[z] = (uint8_t)((address >> ((sizeof(uint16_t) - z - 1) * 8)) & 0xFF);
  //--- Send the address ---
  Error = pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddrW, &Address[0], sizeof(uint16_t), true, false); // Transfer the address
  if (Error == ERR__I2C_NACK) return ERR__NOT_READY;                                                            // If the device receive a NAK, then the device is not ready
  if (Error == ERR__I2C_NACK_DATA) return ERR__I2C_INVALID_ADDRESS;                                             // If the device receive a NAK while transferring data, then this is an invalid address
  return Error;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Read data from the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_ReadData(MLX90640 *pComp, const uint16_t address, uint16_t* data, size_t size)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t DataBuf[2];
  uint8_t ChipAddrW = (pComp->I2Caddress & MLX90640_CHIPADDRESS_MASK);
  uint8_t ChipAddrR = (ChipAddrW | MLX90640_I2C_READ);

  //--- Read data ---
  Error = __MLX90640_WriteAddress(pComp, ChipAddrW, address); // Start a write at address with the device
  if (Error == ERR__I2C_NACK) return ERR__NOT_READY;          // If the device receive a NAK, then the device is not ready
  if (Error == ERR_OK)                                        // If there is no error while writing address then
  {
    bool First = true;
    while (size > 0)
    {
      Error = pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddrR, &DataBuf[0], sizeof(uint16_t), First, size == 1); // Restart at first data read transfer, get the data and stop transfer at last data
      if (Error != ERR_OK) return Error;                                                                                 // If there is an error while calling fnI2C_Transfer() then return the Error
      if (MLX90640_IS_LITTLE_ENDIAN(pComp->InternalConfig)) *data = ((uint16_t)DataBuf[0] << 8) | (uint16_t)DataBuf[1];  // The device's communication is in big endian then MSB first & LSB last in little endian
      First = false;
      data++;
      size--;
    }
  }
  return Error;
}



//=============================================================================
// Write data to the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_WriteData(MLX90640 *pComp, const uint16_t address, const uint16_t* data, size_t size)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t DataBuf[2];
  uint16_t* pData = (uint16_t*)data;
  uint8_t ChipAddr = (pComp->I2Caddress & MLX90640_CHIPADDRESS_MASK);

  //--- Program Address ---
  Error = __MLX90640_WriteAddress(pComp, ChipAddr, address); // Start a write at address with the device
  if (Error == ERR__I2C_NACK) return ERR__NOT_READY;         // If the device receive a NAK, then the device is not ready
  if (Error == ERR_OK)                                       // If there is no error while writing address then
    while (size > 0)
    {
      if (MLX90640_IS_LITTLE_ENDIAN(pComp->InternalConfig))
      {
        DataBuf[0] = (uint8_t)((*pData >> 8) & 0xFF);          // The device's communication is in big endian then MSB first
        DataBuf[1] = (uint8_t)((*pData >> 0) & 0xFF);          // LSB last
      }
      else
      {
        DataBuf[0] = (uint8_t)((*pData >> 0) & 0xFF);          // The device's communication is in big endian too so don't change endianness, then MSB first
        DataBuf[1] = (uint8_t)((*pData >> 8) & 0xFF);          // LSB last
      }
      Error = pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddr, &DataBuf[0], sizeof(uint16_t), false, size == 1); // Continue the transfer by sending the data and stop transfer at last data (chip address will not be used)
      if (Error != ERR_OK) return Error;                                                                                // If there is an error while calling fnI2C_Transfer() then return the Error
      pData++;
      size--;
    }
  return Error;
}



//=============================================================================
// Dump the entire EEPROM data from the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_DumpEEPROM(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
#ifdef CHECK_NULL_PARAM
  if (eepromDump == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Limit EEPROM I2C clock ---
  if (pComp->I2C_ClockSpeed > MLX90640_I2CCLOCK_FM_MAX)
  {
    Error = pComp->fnI2C_Init(pComp->InterfaceDevice, MLX90640_I2CCLOCK_FM_MAX); // Init the I2C with a safe SCL clock speed for EEPROM operations
    if (Error != ERR_OK) return Error;                                           // If there is an error while calling fnI2C_Init() then return the Error
  }

  //--- Read EEPROM data ---
  Error = MLX90640_ReadData(pComp, EepMLX90640_StartAddress, &eepromDump->Words[0], sizeof(MLX90640_EEPROM) / sizeof(uint16_t));
  if (Error != ERR_OK) return Error;                                             // If there is an error while calling MLX90640_ReadData() then return the Error

  //--- Return to the original I2C clock ---
  if (pComp->I2C_ClockSpeed > MLX90640_I2CCLOCK_FM_MAX)
  {
    Error = pComp->fnI2C_Init(pComp->InterfaceDevice, pComp->I2C_ClockSpeed);    // Re-init the I2C with the desired SCL clock speed
  }
  return Error;
}



//=============================================================================
// Get the last frame data on the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_GetFrameData(MLX90640 *pComp, MLX90640_FrameData* frameData)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (frameData == NULL)) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Read RAM data ---
  Error = MLX90640_ReadData(pComp, RamMLX90640_StartAddress, &frameData->Words[0], (sizeof(MLX90640_FrameData) / sizeof(uint16_t)) - 1);
  if (Error != ERR_OK) return Error;                                            // If there is an error while calling MLX90640_ReadData() then return the Error

  //--- Read status register ---
  Error = MLX90640_ReadRegister(pComp, RegMLX90640_Status, &frameData->StatusReg.Status);
  if (Error != ERR_OK) return Error;                                            // If there is an error while calling MLX90640_ReadRegister() then return the Error

  //--- Clear status register ---
  Error = MLX90640_WriteRegister(pComp, RegMLX90640_Status, MLX90640_CLEAR_DEVICE_STATUS);
  if (Error != ERR_OK) return Error;                                            // If there is an error while calling MLX90640_WriteRegister() then return the Error

  //--- Check data ---
  uint8_t CurrSubPage = MLX90640_LAST_SUBPAGE_GET(frameData->StatusReg.Status); // Get current subpage, i.e. this frame data subpage
  for (int y = MLX90640_ROW_COUNT; --y >= 0;)
    if ((frameData->FrameYX[y][0] == 0x7FFF) && ((y & 0x1) == CurrSubPage)) return ERR__BAD_DATA;
  if (frameData->AuxWords[0x00] == 0x7FFF) return ERR__BAD_DATA;
  for (size_t z = 0x08; z < 0x21; ++z)
  {
    if (z == 0x13) continue;
    if (z == 0x17) continue;
    if (frameData->AuxWords[z] == 0x7FFF) return ERR__BAD_DATA;
  }
  for (size_t z = 0x28; z < 0x41; ++z)
  {
    if (z == 0x33) continue;
    if (z == 0x37) continue;
    if (frameData->AuxWords[z] == 0x7FFF) return ERR__BAD_DATA;
  }
  return ERR_OK;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Configure the I2C on the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_ConfigureDeviceI2C(MLX90640 *pComp, bool i2c_FMp_Enable, bool setThresholdTo1V8, bool setSDAdriverCurrentLimit)
{
  MLX90640_I2Cconfig Reg;
  Reg.I2Cconfig = MLX90640_FMp_ENABLE | MLX90640_THRESHOLD_VDD | MLX90640_SDA_CURRENT_LIMIT_ENABLE; // Default values
  if (i2c_FMp_Enable           == false) Reg.I2Cconfig |= MLX90640_FMp_DISABLE;
  if (setThresholdTo1V8                ) Reg.I2Cconfig |= MLX90640_THRESHOLD_1V8;
  if (setSDAdriverCurrentLimit == false) Reg.I2Cconfig |= MLX90640_SDA_CURRENT_LIMIT_DISABLE;
  return MLX90640_WriteRegister(pComp, RegMLX90640_I2Cconfig, Reg.I2Cconfig);
}



//=============================================================================
// Configure the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_ConfigureDevice(MLX90640 *pComp, eMLX90640_SubpageMode SubpageMode, eMLX90640_RefreshRate RefreshRate, eMLX90640_ReadingPattern ReadingPattern, eMLX90640_ADCresolution ADCresolution)
{
  MLX90640_Control1 Reg;
  Reg.Control1 = MLX90640_SUBPAGE_MODE_ENABLE | MLX90640_DATA_HOLD_DISABLE | MLX90640_SUBPAGE_REPEAT_DISABLE // Default values
               | MLX90640_IR_REFRESH_RATE_SET(RefreshRate)                                                   // Set refresh rate
               | MLX90640_ADC_RESOLUTION_SET(ADCresolution)                                                  // Set ADC resolution
               | MLX90640_READING_INTERLEAVED;                                                               // Default value
  switch (SubpageMode)
  {
    case MLX90640_MEASURE_ONLY_SUBPAGE0:
      Reg.Control1 |= MLX90640_SUBPAGE_REPEAT_ENABLE | MLX90640_SELECT_SUBPAGE_SET(MLX90640_SUBPAGE_0_IS_SELECTED); // Set subpage mode, enable subpage repeat and select subpage 0
      break;
    case MLX90640_MEASURE_ONLY_SUBPAGE1:
      Reg.Control1 |= MLX90640_SUBPAGE_REPEAT_ENABLE | MLX90640_SELECT_SUBPAGE_SET(MLX90640_SUBPAGE_1_IS_SELECTED); // Set subpage mode, enable subpage repeat and select subpage 1
      break;
    case MLX90640_MEASURE_ALTERNATE_SUBPAGES:
      Reg.Control1 |= MLX90640_SUBPAGE_REPEAT_DISABLE;
      break;
    default: break;
  }
  if (ReadingPattern == MLX90640_READING_CHESS_PATTERN) Reg.Control1 |= MLX90640_READING_CHESS_PATTERN; // If ask for reading chess pattern then set reading pattern
  pComp->InternalConfig &= ~(MLX90640_READING_PATTERN_Mask | MLX90640_ADC_RESOLUTION_Mask);             // Clear the config in the internal config
  pComp->InternalConfig |= MLX90640_READING_PATTERN_SET(ReadingPattern) | MLX90640_ADC_RESOLUTION_SET(ADCresolution); // Set the new configuration in the internal config
  return MLX90640_WriteRegister(pComp, RegMLX90640_Control1, Reg.Control1);
}



//=============================================================================
// Change the I2C address of the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_ChangeI2Caddress(MLX90640 *pComp, uint8_t newAddress)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->fnGetCurrentms == NULL) return ERR__PARAMETER_ERROR;
#endif
  if ((newAddress < 0x01) || (newAddress > 0x7F)) return ERR__I2C_INVALID_ADDRESS;
  eERRORRESULT Error;
  MLX90640_I2Caddress RegI2Caddr;

  //--- Check previous value ---
  Error = MLX90640_ReadRegister(pComp, EepMLX90640_I2Caddress, &RegI2Caddr.I2Caddress);
  if (Error != ERR_OK) return Error;                     // If there is an error while calling MLX90640_ReadRegister() then return the Error
  //--- Initialize register ---
  Error = MLX90640_WriteRegister(pComp, EepMLX90640_I2Caddress, 0x0000);
  if (Error != ERR_OK) return Error;                     // If there is an error while calling MLX90640_WriteRegister() then return the Error
  //--- Sleep 10ms ---
  uint32_t Timeout = pComp->fnGetCurrentms() + 10 + 1;   // Wait at least 10 + 1ms because GetCurrentms can be 1 cycle before the new ms
  while (pComp->fnGetCurrentms() < Timeout);             // Wait until timeout
  //--- Check value ---
  uint16_t Value;
  Error = MLX90640_ReadRegister(pComp, EepMLX90640_I2Caddress, &Value);
  if (Error != ERR_OK) return Error;                     // If there is an error while calling MLX90640_ReadRegister() then return the Error
  if (Value != 0x0000) return ERR__DATA_NOT_INITIALIZED; // The value of the register should be 0x0000
  //--- Set new address ---
  RegI2Caddr.I2Caddress = (RegI2Caddr.I2Caddress & 0xFF00) | newAddress; // Set new address value
  Error = MLX90640_WriteRegister(pComp, EepMLX90640_I2Caddress, RegI2Caddr.I2Caddress);
  if (Error != ERR_OK) return Error;                     // If there is an error while calling MLX90640_WriteRegister() then return the Error
  //--- Sleep 10ms ---
  Timeout = pComp->fnGetCurrentms() + 10 + 1;            // Wait at least 10 + 1ms because GetCurrentms can be 1 cycle before the new ms
  while (pComp->fnGetCurrentms() < Timeout);             // Wait until timeout
  //--- Check value ---
  Error = MLX90640_ReadRegister(pComp, EepMLX90640_I2Caddress, &Value);
  if (Error != ERR_OK) return Error;                     // If there is an error while calling MLX90640_ReadRegister() then return the Error
  if (Value != RegI2Caddr.I2Caddress) return ERR__DATA_NOT_INITIALIZED; // The value of the register should be the same as the one written
  return ERR_OK;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Calculate and store Vdd parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractVDDParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  int16_t Kv_Vdd;
  int16_t Vdd_25;
  uint8_t ResCtrlCalib;

  //--- Get values from EEPROM ---
  ResCtrlCalib = MLX90640_RES_CTRL_CAL_GET(eepromDump->K_Scale.Kscale);
  Kv_Vdd = MLX90640_Kv_Vdd_GET(eepromDump->Vdd);
  Vdd_25 = MLX90640_Vdd_25_GET(eepromDump->Vdd);

  //--- Calculate and store values ---
  pComp->Params->Resolution = ResCtrlCalib;
  pComp->Params->Kv_Vdd = (Kv_Vdd * 32);                // See §11.1.1
  pComp->Params->Vdd_25 = ((Vdd_25 - 256) * 32) - 8192; // See §11.1.1
}


//=============================================================================
// Calculate and store Ambient Temperature coefficients parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractPTATParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  int16_t Kv_PTAT = 0;
  int16_t Kt_PTAT = 0;
  int16_t PTAT_25 = 0;
  uint8_t AlphaPTAT = 0u;

  //--- Get values from EEPROM ---
  Kv_PTAT = MLX90640_Kv_PTAT_GET(eepromDump->K_PTAT.KPTAT);
  Kt_PTAT = MLX90640_Kt_PTAT_GET(eepromDump->K_PTAT.KPTAT);
  PTAT_25 = eepromDump->PTAT_25;
  AlphaPTAT = MLX90640_ALPHA_PTAT_GET(eepromDump->ScaleOCC.ScaleOCC);

  //--- Calculate and store values ---
  pComp->Params->KvPTAT    = (float)Kv_PTAT / 4096.0f;         // See §11.1.2
  pComp->Params->KtPTAT    = (float)Kt_PTAT /    8.0f;         // See §11.1.2
  pComp->Params->PTAT_25   = PTAT_25;                          // See §11.1.2
  pComp->Params->AlphaPTAT = ((float)AlphaPTAT / 4.0f) + 8.0f; // See §11.1.2
}


//=============================================================================
// Calculate and store Pixels Offset parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractPixelsOffsetParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  uint16_t OffsetCPsp0;
  int16_t OffsetCPdelta;

  //--- Get values from EEPROM ---
  OffsetCPsp0   = MLX90640_OFFSET_CP_SUBPAGE_GET(eepromDump->Offset_CP.OffsetCPsubpage);
  OffsetCPdelta = MLX90640_OFFSET_CP_P1P0_DELTA_GET(eepromDump->Offset_CP.OffsetCPsubpage);

  //--- Calculate and store Offset CP subpages ---
  pComp->Params->CPsubpageOffset[0] = OffsetCPsp0;                 // See §11.1.13
  pComp->Params->CPsubpageOffset[1] = OffsetCPsp0 + OffsetCPdelta; // See §11.1.13

#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
  uint16_t OccScaleRem;
  uint16_t OccScaleCol;
  uint16_t OccScaleRow;
  int16_t OffsetAverage;
  uint16_t* OffsetRow;
  uint16_t* OffsetCol;
  int16_t OccRow[MLX90640_ROW_COUNT];
  int16_t OccCol[MLX90640_COL_COUNT];
  size_t Offset;
  int16_t OffsetPixel;

  OccScaleRem = MLX90640_SCALE_OCC_REM_GET(eepromDump->ScaleOCC.ScaleOCC);
  OccScaleCol = MLX90640_SCALE_OCC_COL_GET(eepromDump->ScaleOCC.ScaleOCC);
  OccScaleRow = MLX90640_SCALE_OCC_ROW_GET(eepromDump->ScaleOCC.ScaleOCC);
  OffsetAverage = eepromDump->PixOsAvg;
  OffsetRow = &eepromDump->Occ.Row.Row[0];
  OffsetCol = &eepromDump->Occ.Column.Column[0];

  //--- Calculate Row values ---
  Offset = 0;
  for (size_t z = 0; z < (MLX90640_ROW_COUNT / 4); ++z)
  {
    Offset = (z << 2);
    OccRow[Offset + 0] = MLX90640_EXTRACT_NIBBLE_0(OffsetRow[z]);
    OccRow[Offset + 1] = MLX90640_EXTRACT_NIBBLE_1(OffsetRow[z]);
    OccRow[Offset + 2] = MLX90640_EXTRACT_NIBBLE_2(OffsetRow[z]);
    OccRow[Offset + 3] = MLX90640_EXTRACT_NIBBLE_3(OffsetRow[z]);
  }

  //--- Calculate Column values ---
  Offset = 0;
  for (size_t z = 0; z < (MLX90640_COL_COUNT / 4); ++z)
  {
    Offset = (z << 2);
    OccCol[Offset + 0] = MLX90640_EXTRACT_NIBBLE_0(OffsetCol[z]);
    OccCol[Offset + 1] = MLX90640_EXTRACT_NIBBLE_1(OffsetCol[z]);
    OccCol[Offset + 2] = MLX90640_EXTRACT_NIBBLE_2(OffsetCol[z]);
    OccCol[Offset + 3] = MLX90640_EXTRACT_NIBBLE_3(OffsetCol[z]);
  }

  //--- Calculate and store Pixels values ---
  for(int y = 0; y < MLX90640_ROW_COUNT; ++y)
  {
    //--- Calculate and store pixel value ---
    for(int x = 0; x < MLX90640_COL_COUNT; ++x)
    {
      OffsetPixel = MLX90640_OFFSET_PIXEL_GET(eepromDump->PixelYXData[y][x].Pixel);
      pComp->Params->OffsetsYX[y][x] = OffsetAverage + (OccRow[y] << OccScaleRow) + (OccCol[x] << OccScaleCol) + (OffsetPixel << OccScaleRem); // *** Pixel offset restoring. See §11.1.3
    }
  }
#endif
}


//=============================================================================
// Calculate and store ILchess parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractILchessParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  int16_t ILchessC1;
  int16_t ILchessC2;
  int16_t ILchessC3;
  uint8_t CalibrationMode;

  //--- Get values from EEPROM ---
  ILchessC1 = MLX90640_IL_CHESS_C1_GET(eepromDump->IL_Chess.ILchess);
  ILchessC2 = MLX90640_IL_CHESS_C2_GET(eepromDump->IL_Chess.ILchess);
  ILchessC3 = MLX90640_IL_CHESS_C3_GET(eepromDump->IL_Chess.ILchess);
  CalibrationMode = (uint8_t)MLX90640_CALIBRATION_MODE_GET(eepromDump->DeviceOptions);

  //--- Calculate and store values ---
  pComp->Params->ILchess[MLX90640_C1] = (float)ILchessC1 / 16.0f; // See §11.1.3.1
  pComp->Params->ILchess[MLX90640_C2] = (float)ILchessC2 /  2.0f; // See §11.1.3.1
  pComp->Params->ILchess[MLX90640_C3] = (float)ILchessC3 /  8.0f; // See §11.1.3.1
  pComp->Params->CalibrationPattern = CalibrationMode;
}


//=============================================================================
// Calculate and store Pixels Sensitivity parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractPixelsSensitivityParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  uint16_t AlphaScale;
  uint16_t AlphaCPsp0;
  int16_t AlphaCPratio;

  //--- Get values from EEPROM ---
  AlphaScale   = MLX90640_ALPHA_SCALE_GET(eepromDump->ScaleACC.ScaleACC);
  AlphaCPsp0   = MLX90640_ALPHA_CP_SUBPAGE0_GET(eepromDump->Alpha_CP.AlphaCPsubpage);
  AlphaCPratio = MLX90640_ALPHA_CP_P1P0_RATIO_GET(eepromDump->Alpha_CP.AlphaCPsubpage);

  //--- Calculate and store Alpha CP subpages ---
  pComp->Params->CPsubpageAlpha[0] = (float)AlphaCPsp0/(float)((uint64_t)1 << (AlphaScale + 27));                      // See §11.1.12
  pComp->Params->CPsubpageAlpha[1] = pComp->Params->CPsubpageAlpha[0] * (1.0 + ((float)AlphaCPratio/(float)(1 << 7))); // See §11.1.12

#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
  uint16_t AccScaleRem;
  uint16_t AccScaleCol;
  uint16_t AccScaleRow;
  int16_t AlphaRef;
  uint16_t* AlphaRow;
  uint16_t* AlphaCol;
  int16_t AccRow[MLX90640_ROW_COUNT];
  int16_t AccCol[MLX90640_COL_COUNT];
  size_t Offset;
  int16_t AlphaPixel;

  //--- Get values from EEPROM ---
  AccScaleRem  = MLX90640_SCALE_ACC_REM_GET(eepromDump->ScaleACC.ScaleACC);
  AccScaleCol  = MLX90640_SCALE_ACC_COL_GET(eepromDump->ScaleACC.ScaleACC);
  AccScaleRow  = MLX90640_SCALE_ACC_ROW_GET(eepromDump->ScaleACC.ScaleACC);
  AlphaRef     = eepromDump->PixSensitivityAvg;
  AlphaRow     = &eepromDump->Acc.Row.Row[0];
  AlphaCol     = &eepromDump->Acc.Column.Column[0];

  //--- Calculate Row values ---
  Offset = 0;
  for (size_t z = 0; z < (MLX90640_ROW_COUNT / 4); ++z)
  {
    Offset = (z << 2);
    AccRow[Offset + 0] = MLX90640_EXTRACT_NIBBLE_0(AlphaRow[z]);
    AccRow[Offset + 1] = MLX90640_EXTRACT_NIBBLE_1(AlphaRow[z]);
    AccRow[Offset + 2] = MLX90640_EXTRACT_NIBBLE_2(AlphaRow[z]);
    AccRow[Offset + 3] = MLX90640_EXTRACT_NIBBLE_3(AlphaRow[z]);
  }

  //--- Calculate Column values ---
  Offset = 0;
  for (size_t z = 0; z < (MLX90640_COL_COUNT / 4); ++z)
  {
    Offset = (z << 2);
    AccCol[Offset + 0] = MLX90640_EXTRACT_NIBBLE_0(AlphaCol[z]);
    AccCol[Offset + 1] = MLX90640_EXTRACT_NIBBLE_1(AlphaCol[z]);
    AccCol[Offset + 2] = MLX90640_EXTRACT_NIBBLE_2(AlphaCol[z]);
    AccCol[Offset + 3] = MLX90640_EXTRACT_NIBBLE_3(AlphaCol[z]);
  }

  //--- Calculate and store Pixels values ---
  AlphaScale += 30;
  for(int y = 0; y < MLX90640_ROW_COUNT; ++y)
  {
    //--- Calculate and store pixel value ---
    for(int x = 0; x < MLX90640_COL_COUNT; ++x)
    {
      AlphaPixel = MLX90640_ALPHA_PIXEL_GET(eepromDump->PixelYXData[y][x].Pixel);
      pComp->Params->AlphasYX[y][x]  = (float)((int32_t)AlphaRef + ((int32_t)AccRow[y] << AccScaleRow) + ((int32_t)AccCol[x] << AccScaleCol) + ((int32_t)AlphaPixel << AccScaleRem));
      pComp->Params->AlphasYX[y][x] /= (float)((uint64_t)1 << AlphaScale); // *** Pixel alpha restoring. See §11.1.4
    }
  }
#endif
}


//=============================================================================
// Calculate and store Kta Coefficients Pixels parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractKtaCoeffPixelsParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  int16_t KtaCP;
  uint16_t KtaScale1;

  KtaScale1 = MLX90640_Kta_SCALE1_GET(eepromDump->K_Scale.Kscale); // Get Kta Scale 1 value
  KtaCP = MLX90640_Kta_CP_GET(eepromDump->K_CP);                   // Get KtaCP value

  //--- Calculate and store KtaCP value ---
  KtaScale1 += 8;
  pComp->Params->KtaCP = (float)KtaCP / (float)(1 << KtaScale1);   // See §11.1.15

#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
  int16_t KtaAvgTable[4];
  uint16_t KtaScale2;
  int16_t KtaPixel;
  size_t Select;

  //--- Get values from EEPROM ---
  KtaAvgTable[0] = MLX90640_KtaAvg_ROW_EVEN_COL_EVEN_GET(eepromDump->KtaAvg_ColumnEven); // Get Kta Row Even Column Even value
  KtaAvgTable[1] = MLX90640_KtaAvg_ROW_ODD_COL_EVEN_GET(eepromDump->KtaAvg_ColumnEven);  // Get Kta Row Odd Column Even value
  KtaAvgTable[2] = MLX90640_KtaAvg_ROW_EVEN_COL_ODD_GET(eepromDump->KtaAvg_ColumnOdd);   // Get Kta Row Even Column Odd value
  KtaAvgTable[3] = MLX90640_KtaAvg_ROW_ODD_COL_ODD_GET(eepromDump->KtaAvg_ColumnOdd);    // Get Kta Row Odd Column Odd value
  KtaScale2 = MLX90640_Kta_SCALE2_GET(eepromDump->K_Scale.Kscale);                       // Get Kta Scale 2 value

  //--- Calculate and store Kta Pixels values ---
  for(int y = 0; y < MLX90640_ROW_COUNT; ++y)
  {
    //--- Calculate and store pixel value ---
    for(int x = 0; x < MLX90640_COL_COUNT; ++x)
    {
      Select = ((y & 0x1) << 1) + (x & 0x1); // Select the good Kta Average value in KtaAvgTable
      KtaPixel = MLX90640_Kta_GET(eepromDump->PixelYXData[y][x].Pixel);
      pComp->Params->KtaCoeffYX[y][x]  = (float)((int32_t)KtaAvgTable[Select] + ((int32_t)KtaPixel << KtaScale2));
      pComp->Params->KtaCoeffYX[y][x] /= (float)((uint32_t)1 << KtaScale1); // *** Pixel Kta restoring. See §11.1.6
    }
  }
#endif
}


//=============================================================================
// Calculate and store Kv Coefficients Pixels parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractKvCoeffPixelsParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  int16_t KvCP;
  uint16_t KvScale;

  KvScale = MLX90640_Kv_SCALE_GET(eepromDump->K_Scale.Kscale); // Get Kv Scale value
  KvCP    = MLX90640_Kv_CP_GET(eepromDump->K_CP);              // Get KvCP value

  //--- Calculate and store KvCP value ---
  pComp->Params->KvCP = (float)KvCP / (float)(1 << KvScale);   // See §11.1.14

#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
  int16_t KvAvgTable[4];
  size_t Select;

  //--- Get values from EEPROM ---
  KvAvgTable[0] = MLX90640_KvAVG_ROW_EVEN_COL_EVEN_GET(eepromDump->Kv_Avg.KvAvg); // Get Kv Row Even Column Even value
  KvAvgTable[1] = MLX90640_KvAVG_ROW_ODD_COL_EVEN_GET(eepromDump->Kv_Avg.KvAvg);  // Get Kv Row Odd Column Even value
  KvAvgTable[2] = MLX90640_KvAVG_ROW_EVEN_COL_ODD_GET(eepromDump->Kv_Avg.KvAvg);  // Get Kv Row Even Column Odd value
  KvAvgTable[3] = MLX90640_KvAVG_ROW_ODD_COL_ODD_GET(eepromDump->Kv_Avg.KvAvg);   // Get Kv Row Odd Column Odd value

  //--- Calculate and store Kv Pixels values ---
  for(int y = 0; y < MLX90640_ROW_COUNT; ++y)
  {
    //--- Calculate and store pixels values ---
    for(int x = 0; x < MLX90640_COL_COUNT; ++x)
    {
      Select = ((y & 0x1) << 1) + (x & 0x1); // Select the good Kv Average value in KvAvgTable
      pComp->Params->KvCoeffYX[y][x] = ((float)KvAvgTable[Select] / (float)(1 << KvScale)); // *** Pixel Kv restoring. See §11.1.5
    }
  }
#endif
}


//=============================================================================
// Calculate and store KsTo parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractKsToParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  uint16_t Step;
  uint16_t CT3;
  uint16_t CT4;
  uint16_t KsToScale;
  int16_t KsToRange1;
  int16_t KsToRange2;
  int16_t KsToRange3;
  int16_t KsToRange4;

  //--- Get values from EEPROM ---
  Step = MLX90640_TEMP_STEP_GET(eepromDump->CT_Temp.CTtemp);               // Get Step value
  CT3  = MLX90640_CT3_GET(eepromDump->CT_Temp.CTtemp);                     // Get CT3 value
  CT4  = MLX90640_CT4_GET(eepromDump->CT_Temp.CTtemp);                     // Get CT4 value
  KsToScale  = MLX90640_KsTo_SCALE_OFFSET_GET(eepromDump->CT_Temp.CTtemp); // Get KsTo Scale Offset value
  KsToRange1 = MLX90640_KsTo_CT_RANGE_1_GET(eepromDump->KsTo_Range1_2);    // Get KsTo range 1 value
  KsToRange2 = MLX90640_KsTo_CT_RANGE_2_GET(eepromDump->KsTo_Range1_2);    // Get KsTo range 2 value
  KsToRange3 = MLX90640_KsTo_CT_RANGE_3_GET(eepromDump->KsTo_Range3_4);    // Get KsTo range 3 value
  KsToRange4 = MLX90640_KsTo_CT_RANGE_4_GET(eepromDump->KsTo_Range3_4);    // Get KsTo range 4 value

  //--- Calculate and store CT values ---
  Step *= 10;
  pComp->Params->CT[MLX90640_CT1] = -40;                                                       // Fixed Corner Temperature 1. See §11.1.9
  pComp->Params->CT[MLX90640_CT2] =   0;                                                       // Fixed Corner Temperature 2. See §11.1.9
  pComp->Params->CT[MLX90640_CT3] = (int16_t)(CT3 * Step);                                     // Calculate Corner Temperature 3. See §11.1.9
  pComp->Params->CT[MLX90640_CT4] = (int16_t)((CT4 * Step) + pComp->Params->CT[MLX90640_CT3]); // Calculate Corner Temperature 4. See §11.1.9

  //--- Calculate and store KsTo values ---
  KsToScale += 8;
  pComp->Params->KsTo[MLX90640_KsTo1] = (float)KsToRange1 / (float)(1 << KsToScale); // Calculate KsTo coefficient 1. See §11.1.10
  pComp->Params->KsTo[MLX90640_KsTo2] = (float)KsToRange2 / (float)(1 << KsToScale); // Calculate KsTo coefficient 2. See §11.1.10
  pComp->Params->KsTo[MLX90640_KsTo3] = (float)KsToRange3 / (float)(1 << KsToScale); // Calculate KsTo coefficient 3. See §11.1.10
  pComp->Params->KsTo[MLX90640_KsTo4] = (float)KsToRange4 / (float)(1 << KsToScale); // Calculate KsTo coefficient 4. See §11.1.10
}


//=============================================================================
// Calculate and store Miscellaneous parameters on the MLX90640 device
//=============================================================================
static void __MLX90640_ExtractMiscellaneousParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  int16_t Gain;
  int16_t TGC;
  int16_t KsTa;

  //--- Get values from EEPROM ---
  Gain = eepromDump->Gain;
  TGC  = MLX90640_TGC_GET(eepromDump->TGC_KsTa);
  KsTa = MLX90640_KsTa_GET(eepromDump->TGC_KsTa);

  //--- Calculate and store values ---
  pComp->Params->Gain = Gain;                           // See §11.1.7
  pComp->Params->TGC  = (float)TGC / (float)(1 << 5);   // See §11.1.16
  pComp->Params->KsTa = (float)KsTa / (float)(1 << 13); // See §11.1.8
}


//=============================================================================
// Extract defective pixels from the MLX90640 device
//=============================================================================
static eERRORRESULT __MLX90640_ExtractDefectivePixels(MLX90640 *pComp, MLX90640_EEPROM *eepromDump)
{
  size_t Current = 0;
  for (size_t z = 0; z < MLX90640_MAX_DEFECT_PIXELS; ++z) pComp->Params->DefectivePixels[z].Type = MLX90640_NOT_DETECTIVE;

  //--- Extract defective pixels ---
  for(int y = 0; y < MLX90640_ROW_COUNT; ++y)
  {
    //--- Get pixel value ---
    for(int x = 0; x < MLX90640_COL_COUNT; ++x)
    {
      //--- Check pixel ---
      if (MLX90640_PIXEL_IS_BROKEN(eepromDump->PixelYXData[y][x].Pixel))
      {
        if (Current >= MLX90640_MAX_DEFECT_PIXELS) return ERR__TOO_MANY_BAD;
        pComp->Params->DefectivePixels[Current].Type = MLX90640_BROKEN; // *** Set pixel. See note 1 page 21
        pComp->Params->DefectivePixels[Current].X    = x;
        pComp->Params->DefectivePixels[Current].Y    = y;
        Current++;
      }
      else if (MLX90640_PIXEL_IS_OUTLIER(eepromDump->PixelYXData[y][x].Pixel))
      {
        if (Current >= MLX90640_MAX_DEFECT_PIXELS) return ERR__TOO_MANY_BAD;
        pComp->Params->DefectivePixels[Current].Type = MLX90640_OUTLIER; // *** Set pixel. See §9
        pComp->Params->DefectivePixels[Current].X    = x;
        pComp->Params->DefectivePixels[Current].Y    = y;
        Current++;
      }
    }
  }

  //--- Check if defective pixels are neighbors ---
  for (size_t zCur = 0; zCur < Current; ++zCur)
  {
    for (size_t zTest = 0; zTest < Current; ++zTest)
    {
      if (pComp->Params->DefectivePixels[zCur].X > 0)
        if ((pComp->Params->DefectivePixels[zCur].X - 1) == pComp->Params->DefectivePixels[zTest].X) return ERR__TWO_BAD_SIDE_BY_SIDE;
      if (pComp->Params->DefectivePixels[zCur].X < (MLX90640_COL_COUNT - 1))
        if ((pComp->Params->DefectivePixels[zCur].X + 1) == pComp->Params->DefectivePixels[zTest].X) return ERR__TWO_BAD_SIDE_BY_SIDE;

      if (pComp->Params->DefectivePixels[zCur].Y > 0)
        if ((pComp->Params->DefectivePixels[zCur].Y - 1) == pComp->Params->DefectivePixels[zTest].Y) return ERR__TWO_BAD_SIDE_BY_SIDE;
      if (pComp->Params->DefectivePixels[zCur].Y < (MLX90640_ROW_COUNT - 1))
        if ((pComp->Params->DefectivePixels[zCur].Y + 1) == pComp->Params->DefectivePixels[zTest].Y) return ERR__TWO_BAD_SIDE_BY_SIDE;
    }
  }
  return ERR_OK;
}





//=============================================================================
// Extract parameters on the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_ExtractDeviceParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump, bool dumpEEPROM)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->Params == NULL) return ERR__PARAMETER_ERROR;
  if (eepromDump == NULL) return ERR__NULL_BUFFER; // No pointer to EEPROM struct so impossible to dump the EEPROM
#endif
  eERRORRESULT Error;

  //--- Dump EEPROM ---
  if (dumpEEPROM)
  {
    Error = MLX90640_DumpEEPROM(pComp, eepromDump);  // Dump EEPROM if asked
    if (Error != ERR_OK) return Error;               // If there is an error while calling MLX90640_DumpEEPROM() then return the Error
  }

  //--- Check EEPROM ---
  uint16_t Value = eepromDump->Words[EepMLX90640_DeviceOption - EepMLX90640_StartAddress] & 0x000B;
  if (Value != 0x0009) return ERR__UNKNOWN_ELEMENT;

  //=== Extract parameters ===
  pComp->InternalConfig &= MLX90640_DEV_NOT_PARAMETERIZED_SET;
  __MLX90640_ExtractVDDParameters(pComp, eepromDump);               // Extract Vdd parameters from EEPROM dump or directly on device
  __MLX90640_ExtractPTATParameters(pComp, eepromDump);              // Extract PTAT parameters from EEPROM dump or directly on device
  __MLX90640_ExtractPixelsOffsetParameters(pComp, eepromDump);      // Extract offsets of each pixels parameters from EEPROM dump or directly on device
  __MLX90640_ExtractILchessParameters(pComp, eepromDump);           // Extract IL chess parameters from EEPROM dump or directly on device
  __MLX90640_ExtractPixelsSensitivityParameters(pComp, eepromDump); // Extract Pixels sensitivity parameters from EEPROM dump or directly on device
  __MLX90640_ExtractKtaCoeffPixelsParameters(pComp, eepromDump);    // Extract Kta coefficients of each pixels parameters from EEPROM dump or directly on device
  __MLX90640_ExtractKvCoeffPixelsParameters(pComp, eepromDump);     // Extract Kv coefficients of each pixels parameters from EEPROM dump or directly on device
  __MLX90640_ExtractKsToParameters(pComp, eepromDump);              // Extract KsTo parameters from EEPROM dump or directly on device
  __MLX90640_ExtractMiscellaneousParameters(pComp, eepromDump);     // Extract miscellaneous parameters from EEPROM dump or directly on device
  Error = __MLX90640_ExtractDefectivePixels(pComp, eepromDump);     // Extract defective pixels from EEPROM dump or directly on device
  if (Error != ERR_OK) return Error;                                // If there is an error while calling __MLX90640_ExtractDefectivePixels() then return the Error

  pComp->InternalConfig |= MLX90640_DEV_PARAMETERIZED;              // Device is parameterized
  return ERR_OK;
}





//**********************************************************************************************************************************************************
#if !defined(MLX90640_PRECALCULATE_PIXELS_COEFFS)
//=============================================================================
// Calculate and return a Pixel Offset on the MLX90640 device
//=============================================================================
static float __MLX90640_Extract1PixelOffsetParameter(MLX90640 *pComp, size_t y, size_t x)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->EEPROM == NULL) return ERR__PARAMETER_ERROR;
#endif
  MLX90640_EEPROM* pEEPROM = pComp->EEPROM;
  uint16_t OccScaleRem;
  uint16_t OccScaleCol;
  uint16_t OccScaleRow;
  int16_t OffsetAverage;
  int16_t OccRow;
  int16_t OccCol;
  int16_t OffsetPixel;

  //--- Get values from EEPROM ---
  OccScaleRem   = MLX90640_SCALE_OCC_REM_GET(pEEPROM->ScaleOCC.ScaleOCC);
  OccScaleCol   = MLX90640_SCALE_OCC_COL_GET(pEEPROM->ScaleOCC.ScaleOCC);
  OccScaleRow   = MLX90640_SCALE_OCC_ROW_GET(pEEPROM->ScaleOCC.ScaleOCC);
  OffsetAverage = pEEPROM->PixOsAvg;

  //--- Calculate Row & Column values ---
  OccRow = MLX90640_DATA_EXTRACT_TO_INT16(pEEPROM->Occ.Row.Row[y >> 2], y % 4, 4);
  OccCol = MLX90640_DATA_EXTRACT_TO_INT16(pEEPROM->Occ.Column.Column[x >> 2], x % 4, 4);

  //--- Calculate and return Pixel value ---
  OffsetPixel = MLX90640_OFFSET_PIXEL_GET(pEEPROM->PixelYXData[y][x].Pixel);
  return (float)(OffsetAverage + (OccRow << OccScaleRow) + (OccCol << OccScaleCol) + (OffsetPixel << OccScaleRem)); // *** Pixel offset restoring. See §11.1.3
}


//=============================================================================
// Calculate and return a Pixel Sensitivity on the MLX90640 device
//=============================================================================
static float __MLX90640_Extract1PixelSensitivityParameter(MLX90640 *pComp, size_t y, size_t x)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->EEPROM == NULL) return ERR__PARAMETER_ERROR;
#endif
  MLX90640_EEPROM* pEEPROM = pComp->EEPROM;
  uint16_t AccScaleRem;
  uint16_t AccScaleCol;
  uint16_t AccScaleRow;
  uint16_t AlphaScale;
  int16_t AlphaRef;
  int16_t AccRow;
  int16_t AccCol;
  int16_t AlphaPixel;

  //--- Get values from EEPROM ---
  AccScaleRem  = MLX90640_SCALE_ACC_REM_GET(pEEPROM->ScaleACC.ScaleACC);
  AccScaleCol  = MLX90640_SCALE_ACC_COL_GET(pEEPROM->ScaleACC.ScaleACC);
  AccScaleRow  = MLX90640_SCALE_ACC_ROW_GET(pEEPROM->ScaleACC.ScaleACC);
  AlphaScale   = MLX90640_ALPHA_SCALE_GET(pEEPROM->ScaleACC.ScaleACC);
  AlphaRef     = pEEPROM->PixSensitivityAvg;

  //--- Calculate Row & Column values ---
  AccRow = MLX90640_DATA_EXTRACT_TO_INT16(pEEPROM->Acc.Row.Row[y >> 2], y % 4, 4);
  AccCol = MLX90640_DATA_EXTRACT_TO_INT16(pEEPROM->Acc.Column.Column[x >> 2], x % 4, 4);

  //--- Calculate and return Pixel value ---
  AlphaScale += 30;
  AlphaPixel = MLX90640_ALPHA_PIXEL_GET(pEEPROM->PixelYXData[y][x].Pixel);
  float Alpha = (float)((int32_t)AlphaRef + ((int32_t)AccRow << AccScaleRow) + ((int32_t)AccCol << AccScaleCol) + ((int32_t)AlphaPixel << AccScaleRem));
  return Alpha / (float)((uint64_t)1 << AlphaScale); // *** Pixel alpha restoring. See §11.1.4
}


//=============================================================================
// Calculate and return Kta Coefficient of a Pixel on the MLX90640 device
//=============================================================================
static float __MLX90640_ExtractKtaCoeff1PixelParameter(MLX90640 *pComp, size_t y, size_t x)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->EEPROM == NULL) return ERR__PARAMETER_ERROR;
#endif
  MLX90640_EEPROM* pEEPROM = pComp->EEPROM;
  int16_t KtaAvgTable[4];
  uint16_t KtaScale1;
  uint16_t KtaScale2;
  int16_t KtaPixel;
  size_t Select;

  //--- Get values from EEPROM ---
  KtaAvgTable[0] = MLX90640_KtaAvg_ROW_EVEN_COL_EVEN_GET(pEEPROM->KtaAvg_ColumnEven); // Get Kta Row Even Column Even value
  KtaAvgTable[1] = MLX90640_KtaAvg_ROW_ODD_COL_EVEN_GET(pEEPROM->KtaAvg_ColumnEven);  // Get Kta Row Odd Column Even value
  KtaAvgTable[2] = MLX90640_KtaAvg_ROW_EVEN_COL_ODD_GET(pEEPROM->KtaAvg_ColumnOdd);   // Get Kta Row Even Column Odd value
  KtaAvgTable[3] = MLX90640_KtaAvg_ROW_ODD_COL_ODD_GET(pEEPROM->KtaAvg_ColumnOdd);    // Get Kta Row Odd Column Odd value
  KtaScale1 = MLX90640_Kta_SCALE1_GET(pEEPROM->K_Scale.Kscale);                       // Get Kta Scale 1 value
  KtaScale2 = MLX90640_Kta_SCALE2_GET(pEEPROM->K_Scale.Kscale);                       // Get Kta Scale 2 value

  //--- Calculate and return Kta Pixel value ---
  KtaScale1 += 8;
  Select = ((y & 0x1) << 1) + (x & 0x1); // Select the good Kta Average value in KtaAvgTable
  KtaPixel = MLX90640_Kta_GET(pEEPROM->PixelYXData[y][x].Pixel);
  float Kta = (float)((int32_t)KtaAvgTable[Select] + ((int32_t)KtaPixel << KtaScale2));
  return Kta / (float)((uint32_t)1 << KtaScale1); // *** Pixel Kta restoring. See §11.1.6
}


//=============================================================================
// Calculate and return Kv Coefficients of a Pixel on the MLX90640 device
//=============================================================================
static float __MLX90640_ExtractKvCoeff1PixelParameter(MLX90640 *pComp, size_t y, size_t x)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->EEPROM == NULL) return ERR__PARAMETER_ERROR;
#endif
  MLX90640_EEPROM* pEEPROM = pComp->EEPROM;
  int16_t KvAvgTable[4];
  uint16_t KvScale;
  size_t Select;

  //--- Get values from EEPROM ---
  KvAvgTable[0] = MLX90640_KvAVG_ROW_EVEN_COL_EVEN_GET(pEEPROM->Kv_Avg.KvAvg); // Get Kv Row Even Column Even value
  KvAvgTable[1] = MLX90640_KvAVG_ROW_ODD_COL_EVEN_GET(pEEPROM->Kv_Avg.KvAvg);  // Get Kv Row Odd Column Even value
  KvAvgTable[2] = MLX90640_KvAVG_ROW_EVEN_COL_ODD_GET(pEEPROM->Kv_Avg.KvAvg);  // Get Kv Row Even Column Odd value
  KvAvgTable[3] = MLX90640_KvAVG_ROW_ODD_COL_ODD_GET(pEEPROM->Kv_Avg.KvAvg);   // Get Kv Row Odd Column Odd value
  KvScale = MLX90640_Kv_SCALE_GET(pEEPROM->K_Scale.Kscale);                    // Get Kv Scale value

  //--- Calculate and store Kv Pixels values ---
  Select = ((y & 0x1) << 1) + (x & 0x1); // Select the good Kv Average value in KvAvgTable
  return ((float)KvAvgTable[Select] / (float)(1 << KvScale)); // *** Pixel Kv restoring. See §11.1.5
}
#endif





//**********************************************************************************************************************************************************
//=============================================================================
// Get the Vdd voltage an Ambient Temperature of the frame on the MLX90640 device
//=============================================================================
static eERRORRESULT __MLX90640_GetVddAndTa(MLX90640 *pComp, MLX90640_FrameData* frameData, float* deltaVdd3V3Frame, float* deltaTaFrame)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (frameData == NULL) || (deltaVdd3V3Frame == NULL) || (deltaTaFrame == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->Params == NULL) return ERR__PARAMETER_ERROR;
#endif
  MLX90640_Parameters* pParams = pComp->Params;

  //--- Calculate Vdd ---
  uint16_t ResRAM = MLX90640_ADC_RESOLUTION_GET(pComp->InternalConfig);
  float ResCorrection = ((float)(1 << pParams->Resolution)) / ((float)(1 << ResRAM));                                   // See §11.2.2.1
  *deltaVdd3V3Frame = ((ResCorrection * (float)frameData->VddPix) - (float)pParams->Vdd_25) / ((float)pParams->Kv_Vdd); // See §11.2.2.2, the 3.3V will not be add here because all calculus that use this works on the delta Vdd3.3

  //--- Calculate Ta ---
  float VtaPTAT = (float)frameData->VtaPTAT;
  float VPTATart = (VtaPTAT / (VtaPTAT * pParams->AlphaPTAT + (float)frameData->VbeTa)) * (float)((uint32_t)1u << 18);
  *deltaTaFrame = (((VPTATart / (1.0f + pParams->KvPTAT * *deltaVdd3V3Frame)) - (float)pParams->PTAT_25) / pParams->KtPTAT); // See §11.2.2.3, the 25° will not be add here because all calculus that use this works on the delta Ta25
  return ERR_OK;
}


#if (MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT > 1)
//=============================================================================
// Apply a moving average filter on the PixGain_Cp_SPx of the frame
//=============================================================================
static void __MLX90640_MovingAverageFilter(float *pixGainCPSP0, float *pixGainCPSP1, MLX90640_FrameTo *result)
{
  result->PixGainCPSP0_Filter[result->Filter0_Index] = *pixGainCPSP0; *pixGainCPSP0 = 0.0f;
  result->PixGainCPSP1_Filter[result->Filter1_Index] = *pixGainCPSP1; *pixGainCPSP1 = 0.0f;
  for (int z = MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT; --z >= 0;)
  {
    *pixGainCPSP0 += result->PixGainCPSP0_Filter[z];
    *pixGainCPSP1 += result->PixGainCPSP1_Filter[z];
  }
  *pixGainCPSP0 /= MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT;
  *pixGainCPSP1 /= MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT;
  result->Filter0_Index++; if (result->Filter0_Index >= MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT) result->Filter0_Index = 0;
  result->Filter1_Index++; if (result->Filter1_Index >= MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT) result->Filter1_Index = 0;
}
#else
#  define __MLX90640_MovingAverageFilter(a,b,c)
#endif



//=============================================================================
// Calculate Object Temperature of the frame on the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_CalculateTo(MLX90640 *pComp, MLX90640_FrameData* frameData, float emissivity, float tr, MLX90640_FrameTo *result)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (frameData == NULL) || (result == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->Params == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  MLX90640_Parameters* pParams = pComp->Params;

  //--- Set auxiliary data ---
  float Vdd, Ta;
  Error = __MLX90640_GetVddAndTa(pComp, frameData, &Vdd, &Ta);
  if (Error != ERR_OK) return Error;                           // If there is an error while calling __MLX90640_GetVddAndTa() then return the Error
  result->Vdd = Vdd + MLX90640_VddV0;                          // See §11.2.2.3, add here the 3.3V
  result->Ta  = Ta + MLX90640_TaV0;                            // See §11.2.2.3, add here the +25°

  //--- Miscellaneous data ---
  uint8_t CurrSubPage = MLX90640_LAST_SUBPAGE_GET(frameData->StatusReg.Status);     // Get current subpage, i.e. this frame data subpage
  uint8_t CurrReadingPattern = MLX90640_READING_PATTERN_GET(pComp->InternalConfig); // Get current reading pattern, i.e. the reading pattern used for this subpage

  //--- Calculate Gain ---
  float Kgain = (float)pParams->Gain / (float)frameData->Gain; // See §11.2.2.4

  //--- Compensating the Gain of CP pixel ---
  float PixOS_CP_SP[2];
  float KtaKvCPComp = (1.0f + (pParams->KtaCP * Ta)) * (1.0f + (pParams->KvCP * Vdd));
  float PixGainCPSP0 = (float)frameData->CPsubpage0 * Kgain;                        // See §11.2.2.6.1
  float PixGainCPSP1 = (float)frameData->CPsubpage1 * Kgain;                        // See §11.2.2.6.1
  __MLX90640_MovingAverageFilter(&PixGainCPSP0, &PixGainCPSP1, result);             // Apply a moving average filter, only if MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT > 1
  PixOS_CP_SP[0] = PixGainCPSP0 - (pParams->CPsubpageOffset[0] * KtaKvCPComp);      // See §11.2.2.6.2
  if (CurrReadingPattern == pParams->CalibrationPattern)                            // If the current pattern is the same as the one used at calibration
       PixOS_CP_SP[1] = PixGainCPSP1 - (pParams->CPsubpageOffset[1] * KtaKvCPComp); // See §11.2.2.6.2
  else PixOS_CP_SP[1] = PixGainCPSP1 - ((pParams->CPsubpageOffset[1] + pParams->ILchess[MLX90640_C1]) * KtaKvCPComp); // See §11.2.2.6.2

  //--- Calculating To for basic temperature range (0°C..CT3°C) ---
  float TaK4 = powf((Ta + 273.15f + MLX90640_TaV0), 4.0f); // See §11.2.2.9
  float TrK4 = powf((tr + 273.15f), 4.0f);                 // See §11.2.2.9
  float InvEmissivity = 1.0f / emissivity;                 // Done once to avoid multiple divisions
  float Ta_r = TrK4 - ((TrK4 - TaK4) * InvEmissivity);     // See §11.2.2.9

  //--- Calculations for extended temperature ranges ---
  float AlphaCorrRange[4];
  AlphaCorrRange[MLX90640_KsTo1] = 1.0f / (1.0f + pParams->KsTo[MLX90640_KsTo1] * (0.0f - (-40.0f)));                                                                   // See §11.2.2.9.1.2
  AlphaCorrRange[MLX90640_KsTo2] = 1.0f;                                                                                                                                // See §11.2.2.9.1.2
  AlphaCorrRange[MLX90640_KsTo3] = (1.0f + (pParams->KsTo[MLX90640_KsTo3] * pParams->CT[MLX90640_CT3]));                                                                // See §11.2.2.9.1.2
  AlphaCorrRange[MLX90640_KsTo4] = AlphaCorrRange[MLX90640_KsTo3] * (1.0f + (pParams->KsTo[MLX90640_KsTo4] * (pParams->CT[MLX90640_CT4] - pParams->CT[MLX90640_CT3]))); // See §11.2.2.9.1.2

  //--- Calculate To of all subpage's pixels ---
  result->MinToSubpage[CurrSubPage] =  FLT_MAX;
  result->MaxToSubpage[CurrSubPage] = -FLT_MIN;
  for (int_fast16_t zPix = MLX90640_TOTAL_PIXELS_COUNT; --zPix >= 0;)
  {
    uint8_t Pattern = 0;
    uint8_t ILpattern = ((zPix >> 5) & 0x1);
    if (CurrReadingPattern == MLX90640_INTERLEAVED_MODE)
         Pattern =  ILpattern;                 // '1' if pixel's row is odd else '0'                                   // See §11.2.2.7
    else Pattern = (ILpattern ^ (zPix & 0x1)); // '1' if pixel is odd in even row or pixel is even in odd row else '0' // See §11.2.2.7

    if (Pattern == CurrSubPage)                // Pixel pattern in the current subpage? Then calculate its To
    {
      //--- Get parameters of the pixel ---
#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
      float Offset   = (float)pParams->Offsets[zPix];
      float Alpha    = pParams->Alphas[zPix];
      float KtaCoeff = pParams->KtaCoeff[zPix];
      float KvCoeff  = pParams->KvCoeff[zPix];
#else
      size_t y = zPix / MLX90640_COL_COUNT;
      size_t x = zPix - (y * MLX90640_ROW_COUNT);
      float Offset   = __MLX90640_Extract1PixelOffsetParameter(pComp, y, x);
      float Alpha    = __MLX90640_Extract1PixelSensitivityParameter(pComp, y, x);
      float KtaCoeff = __MLX90640_ExtractKtaCoeff1PixelParameter(pComp, y, x);
      float KvCoeff  = __MLX90640_ExtractKvCoeff1PixelParameter(pComp, y, x);
#endif
      //--- Correct Ir data of the pixel ---
      float IrPixel  = (float)frameData->Frame[zPix] * Kgain;
      IrPixel -= Offset * (1.0f + (KtaCoeff * Ta)) * (1.0f + (KvCoeff * Vdd));
      if (CurrReadingPattern != pParams->CalibrationPattern) // If the current pattern is not the same as the one used at calibration
      {
        int8_t ConvPattern = ((zPix & 0x2) - 1) * (zPix & 0x1) * (1 - (int8_t)(ILpattern << 1)); // Alternate '0', '-1', '0', '1', ... from the first pixel. This return the value at the current pixel
        IrPixel += (pParams->ILchess[MLX90640_C3] * (float)((int8_t)(ILpattern << 1) - 1)) - (pParams->ILchess[MLX90640_C2] * (float)ConvPattern);
      }
      IrPixel *= InvEmissivity;
      IrPixel -= pParams->TGC * PixOS_CP_SP[CurrSubPage];

      //--- Normalizing to sensitivity ---
      float AlphaComp = Alpha - ((pParams->TGC * pParams->CPsubpageAlpha[CurrSubPage]) * (1.0f + (pParams->KsTa * Ta))); // See §11.2.2.8

      //--- Calculating To for basic temperature range (0°C..CT3°C) ---
      float SxPix = powf(AlphaComp, 3.0f) * (IrPixel + (AlphaComp * Ta_r)); // See §11.2.2.9
      SxPix = ftrtf(SxPix) * pParams->KsTo[MLX90640_KsTo2];                 // See §11.2.2.9
      float ToPix = (IrPixel / (AlphaComp * (1.0f - (pParams->KsTo[MLX90640_KsTo2] * 273.15f)) + SxPix)) + Ta_r;
      ToPix = ftrtf(ToPix) - 273.15f;

      //--- Calculations for extended temperature ranges ---
      size_t Range = 3;
      if      (ToPix < pParams->CT[MLX90640_CT2]) Range = 0;
      else if (ToPix < pParams->CT[MLX90640_CT3]) Range = 1;
      else if (ToPix < pParams->CT[MLX90640_CT4]) Range = 2;
      ToPix = (IrPixel / (AlphaComp * AlphaCorrRange[Range] * (1.0f + (pParams->KsTo[Range] * (ToPix - pParams->CT[Range]))))) + Ta_r;
      result->Pixel[zPix] = ftrtf(ToPix) - 273.15f;                         // See §11.2.2.9.1.3
      if (result->Pixel[zPix] < result->MinToSubpage[CurrSubPage]) result->MinToSubpage[CurrSubPage] = result->Pixel[zPix]; // Set the min of the subpage
      if (result->Pixel[zPix] > result->MaxToSubpage[CurrSubPage]) result->MaxToSubpage[CurrSubPage] = result->Pixel[zPix]; // Set the max of the subpage
    }
  }
  return ERR_OK;
}



//=============================================================================
// Correct bad pixels in a frame on the MLX90640 device
//=============================================================================
eERRORRESULT MLX90640_CorrectBadPixels(MLX90640 *pComp, MLX90640_FrameTo *result)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (result == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->Params == NULL) return ERR__PARAMETER_ERROR;
#endif
  MLX90640_Parameters* pParams = pComp->Params;

  for (size_t z = 0; z < MLX90640_MAX_DEFECT_PIXELS; ++z)
    if (pParams->DefectivePixels[z].Type != MLX90640_NOT_DETECTIVE)
    {
      float CorrectedValue = 0.0f;
      uint_fast16_t ValCount = 0;
      size_t x = pParams->DefectivePixels[z].X;
      size_t y = pParams->DefectivePixels[z].Y;
      if (pParams->DefectivePixels[z].X >                     0 ) { CorrectedValue += result->PixelYX[y][x-1] ; ++ValCount; }
      if (pParams->DefectivePixels[z].X < (MLX90640_COL_COUNT-1)) { CorrectedValue += result->PixelYX[y][x+1] ; ++ValCount; }
      if (pParams->DefectivePixels[z].Y >                     0 ) { CorrectedValue += result->PixelYX[y-1][x] ; ++ValCount; }
      if (pParams->DefectivePixels[z].Y < (MLX90640_ROW_COUNT-1)) { CorrectedValue += result->PixelYX[y+1][x] ; ++ValCount; }
      result->PixelYX[y][x] = CorrectedValue / (float)ValCount;
    }
  return ERR_OK;
}





//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------
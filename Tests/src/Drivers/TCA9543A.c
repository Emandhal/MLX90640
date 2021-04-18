/*******************************************************************************
 * @file    TCA9543A.c
 * @author  FMA
 * @version 1.0.0
 * @date    27/02/2020
 * @brief   TCA9543A driver
 *
 * Low Voltage 2-Channel I2C Bus Switch With Interrupt Logic And Reset
 * Follow datasheet SCPS206B Rev B (Mar 2014)
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "TCA9543A.h"
//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
#include <cstdint>
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// TCA9543A initialization
//=============================================================================
eERRORRESULT Init_TCA9543A(TCA9543A *pComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Init == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Transfer == NULL) return false;
#endif
  eERRORRESULT Error;

  //--- Initialize the interface ---
  if (pComp->I2C_ClockSpeed > TCA9543A_I2C_CLOCK_MAX) return ERR__FREQUENCY_ERROR;
  Error = pComp->fnI2C_Init(pComp->InterfaceDevice, pComp->I2C_ClockSpeed);
  if (Error != ERR_OK) return Error;                                                                      // If there is an error while calling fnI2C_Init() then return the Error
  pComp->InternalConfig = (TTCA9543ADriverInternal)(TCA9543A_CHANNELx_SET(TCA9543A_NO_CHANNEL_SELECTED)); // By default no channels are selected

  //--- Detect the presence of the device ---
  uint8_t ChipAddrR = (TCA9543A_CHIPADDRESS_BASE | pComp->AddrA1A0) | TCA9543A_I2C_READ;
  Error = pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddrR, NULL, 0, true, true); // Send only the chip address and get the Ack flag
  if (Error == ERR__I2C_NACK) return ERR__NO_DEVICE_DETECTED;                            // If a NACK is received, then the device is not detected
  return Error;
}





//**********************************************************************************************************************************************************
// Read data from control register of the TCA9543A
static eERRORRESULT __TCA9543A_ReadControlRegister(TCA9543A *pComp, uint8_t* data)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t ChipAddr = (((TCA9543A_CHIPADDRESS_BASE | pComp->AddrA1A0) & TCA9543A_CHIPADDRESS_MASK) | TCA9543A_I2C_READ);

  //--- Write control register ---
  return pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddr, data, 1, true, true); // Continue the transfer by sending the data and stop transfer (chip address will not be used)
}



// Write data to control register of the TCA9543A
static eERRORRESULT __TCA9543A_WriteControlRegister(TCA9543A *pComp, const uint8_t data)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
  if (pComp->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t ChipAddr = ((TCA9543A_CHIPADDRESS_BASE | pComp->AddrA1A0) & TCA9543A_CHIPADDRESS_MASK) & TCA9543A_I2C_WRITE;

  //--- Write control register ---
  uint8_t* pData = (uint8_t*)&data;
  return pComp->fnI2C_Transfer(pComp->InterfaceDevice, ChipAddr, pData, 1, true, true); // Continue the transfer by sending the data and stop transfer (chip address will not be used)
}





//**********************************************************************************************************************************************************
//=============================================================================
// Select the output channel of the TCA9543A device
//=============================================================================
eERRORRESULT TCA9543A_SelectChannel(TCA9543A *pComp, eTCA9543A_ChannelSelect channelSelect)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (channelSelect >= TCA9543A_CHANNEL_COUNT) return ERR__UNKNOWN_CHANNEL;

  //--- Checking the need to change the channel ---
  if (channelSelect == TCA9543A_CHANNELx_GET(pComp->InternalConfig)) return ERR_OK;

  //--- Change the channel ---
  eERRORRESULT Error;
  TCA9543A_Control_Register Reg;
  Reg.Control = TCA9543A_CHANNELx_SET(channelSelect);               // Set the channel to the control register
  Error = __TCA9543A_WriteControlRegister(pComp, Reg.Control);      // Write control register
  if (Error == ERR_OK)
  {
    pComp->InternalConfig = (TTCA9543ADriverInternal)(Reg.Control); // Set the new current channel in the internal config variable
  }
  return Error;
}



//=============================================================================
// Get the status of the TCA9543A device
//=============================================================================
eERRORRESULT TCA9543A_GetDeviceStatus(TCA9543A *pComp, eTCA9543A_ChannelSelect *currentChannel, uint8_t *interruptChannel)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  TCA9543A_Control_Register Reg;
  Error = __TCA9543A_ReadControlRegister(pComp, &Reg.Control); // Read control register
  if (Error == ERR_OK)
  {
    eTCA9543A_ChannelSelect Channel = TCA9543A_CHANNELx_GET(Reg.Control);
    if (currentChannel   != NULL) *currentChannel = Channel;                                   // Save the current channel in the out pointer if not null
    pComp->InternalConfig = (TTCA9543ADriverInternal)(TCA9543A_CHANNELx_SET(Channel));         // Set the new current channel in the internal config variable
    if (interruptChannel != NULL) *interruptChannel = (Reg.Control & TCA9543A_INTERRUPT_MASK); // Save the current interrupt in the out pointer if not null
  }
  return Error;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Perform the I2C Init of the specified I2C channel of the TCA9543A device
//=============================================================================
eERRORRESULT TCA9543A_I2CInit(void *pIntDev, const uint32_t sclFreq)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  struct TCA9543A_I2C* pI2C = (struct TCA9543A_I2C*)pIntDev; // Transform the pIntDev into a TCA9543A_I2C port
  TCA9543A* pDevice = pI2C->Device;                          // Get the TCA9543A device of this I2C port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pDevice->fnI2C_Init == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (sclFreq > TCA9543A_I2C_CLOCK_MAX) return ERR__FREQUENCY_ERROR; // Check the SCL frequency. If it is too high for the TCA9543A, return an error
  return pDevice->fnI2C_Init(pDevice->InterfaceDevice, sclFreq);     // Perform the I2C Init
}



//=============================================================================
// Perform the I2C Transfer to the specified I2C channel of the TCA9543A device
//=============================================================================
eERRORRESULT TCA9543A_I2CTranfert(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  struct TCA9543A_I2C* pI2C = (struct TCA9543A_I2C*)pIntDev; // Transform the pIntDev into a TCA9543A_I2C port
  TCA9543A* pDevice = pI2C->Device;                          // Get the TCA9543A device of this I2C port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pDevice->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error = TCA9543A_SelectChannel(pDevice, pI2C->Channel); // Select the channel if not already selected
  if (Error != ERR_OK) return Error;                                   // If there is an error while calling TCA9543A_SelectChannel() then return the Error
  return pDevice->fnI2C_Transfer(pDevice->InterfaceDevice, deviceAddress, data, byteCount, start, stop); // Perform the I2C Transfer
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
//-----------------------------------------------------------------------------
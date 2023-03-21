/*!*****************************************************************************
 * @file    TCA9543A.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    27/02/2020
 * @brief   TCA9543A driver
 * @details Low Voltage 2-Channel I2C Bus Switch With Interrupt Logic And Reset
 * Follow datasheet SCPS206B Rev B (Mar 2014)
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "TCA9543A.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
#include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef USE_DYNAMIC_INTERFACE
#  define GET_I2C_INTERFACE(dev)  dev->I2C
#else
#  define GET_I2C_INTERFACE(dev)  &dev->I2C
#endif

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// TCA9543A initialization
//=============================================================================
eERRORRESULT Init_TCA9543A(TCA9543A *pComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE(pComp);
#if defined(CHECK_NULL_PARAM) && defined(USE_DYNAMIC_INTERFACE)
  if (pI2C->fnI2C_Init == NULL) return ERR__PARAMETER_ERROR;
  if (pI2C->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Initialize the interface ---
  if (pComp->I2CclockSpeed > TCA9543A_I2C_CLOCK_MAX) return ERR__FREQUENCY_ERROR;
  Error = pI2C->fnI2C_Init(pI2C, pComp->I2CclockSpeed);
  if (Error != ERR_OK) return Error;                                                                      // If there is an error while calling fnI2C_Init() then return the Error
  pComp->InternalConfig = (TTCA9543ADriverInternal)(TCA9543A_CHANNELx_SET(TCA9543A_NO_CHANNEL_SELECTED)); // By default no channels are selected

  //--- Detect the presence of the device ---
  I2CInterface_Packet PacketDesc = I2C_INTERFACE_NO_DATA_DESC((TCA9543A_CHIPADDRESS_BASE | pComp->AddrA1A0) | I2C_READ_ORMASK);
  Error = pI2C->fnI2C_Transfer(pI2C, &PacketDesc);            // Send only the chip address and get the Ack flag
  if (Error == ERR__I2C_NACK) return ERR__NO_DEVICE_DETECTED; // If a NACK is received, then the device is not detected
  return Error;
}





//**********************************************************************************************************************************************************
//=============================================================================
// [STATIC] Read data from control register of the TCA9543A
//=============================================================================
static eERRORRESULT __TCA9543A_ReadControlRegister(TCA9543A *pComp, uint8_t* data)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE(pComp);
#if defined(CHECK_NULL_PARAM) && defined(USE_DYNAMIC_INTERFACE)
  if (pI2C->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  I2CInterface_Packet PacketDesc =
  {
    I2C_MEMBER(Config.Value) I2C_NO_POLLING | I2C_ENDIAN_TRANSFORM_SET(I2C_NO_ENDIAN_CHANGE) | I2C_TRANSFER_TYPE_SET(I2C_SIMPLE_TRANSFER),
    I2C_MEMBER(ChipAddr    ) (((TCA9543A_CHIPADDRESS_BASE | pComp->AddrA1A0) & TCA9543A_CHIPADDRESS_MASK) | I2C_READ_ORMASK),
    I2C_MEMBER(Start       ) true,
    I2C_MEMBER(pBuffer     ) data,
    I2C_MEMBER(BufferSize  ) 1,
    I2C_MEMBER(Stop        ) true,
  };
  //--- Write control register ---
  return pI2C->fnI2C_Transfer(pI2C, &PacketDesc); // Continue the transfer by sending the data and stop transfer (chip address will not be used)
}



//=============================================================================
// [STATIC] Write data to control register of the TCA9543A
//=============================================================================
static eERRORRESULT __TCA9543A_WriteControlRegister(TCA9543A *pComp, const uint8_t data)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE(pComp);
#if defined(CHECK_NULL_PARAM) && defined(USE_DYNAMIC_INTERFACE)
  if (pI2C->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t* pData = (uint8_t*)&data;
  I2CInterface_Packet PacketDesc =
  {
    I2C_MEMBER(Config.Value) I2C_NO_POLLING | I2C_ENDIAN_TRANSFORM_SET(I2C_NO_ENDIAN_CHANGE) | I2C_TRANSFER_TYPE_SET(I2C_SIMPLE_TRANSFER),
    I2C_MEMBER(ChipAddr    ) (((TCA9543A_CHIPADDRESS_BASE | pComp->AddrA1A0) & TCA9543A_CHIPADDRESS_MASK) & I2C_WRITE_ANDMASK),
    I2C_MEMBER(Start       ) true,
    I2C_MEMBER(pBuffer     ) pData,
    I2C_MEMBER(BufferSize  ) 1,
    I2C_MEMBER(Stop        ) true,
  };
  //--- Write control register ---
  return pI2C->fnI2C_Transfer(pI2C, &PacketDesc); // Continue the transfer by sending the data and stop transfer (chip address will not be used)
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
eERRORRESULT TCA9543A_I2CInit(I2C_Interface *pIntDev, const uint32_t sclFreq)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  TCA9543A* pDevice = (TCA9543A*)(pIntDev->InterfaceDevice);         // Get the TCA9543A device of this I2C port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE(pDevice);
#if defined(CHECK_NULL_PARAM) && defined(USE_DYNAMIC_INTERFACE)
  if (pI2C->fnI2C_Init == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (sclFreq > TCA9543A_I2C_CLOCK_MAX) return ERR__FREQUENCY_ERROR; // Check the SCL frequency. If it is too high for the TCA9543A, return an error
  return pI2C->fnI2C_Init(pI2C, sclFreq);                            // Perform the I2C Init
}



//=============================================================================
// Perform the I2C Transfer to the specified I2C channel of the TCA9543A device
//=============================================================================
eERRORRESULT TCA9543A_I2CTransfer(I2C_Interface *pIntDev, I2CInterface_Packet* const pPacketDesc)
{
#ifdef CHECK_NULL_PARAM
  if ((pIntDev == NULL) || (pPacketDesc == NULL)) return ERR__PARAMETER_ERROR;
#endif
  TCA9543A* pDevice = (TCA9543A*)(pIntDev->InterfaceDevice);              // Get the TCA9543A device of this I2C port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE(pDevice);
#if defined(CHECK_NULL_PARAM) && defined(USE_DYNAMIC_INTERFACE)
  if (pI2C->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error = TCA9543A_SelectChannel(pDevice, pIntDev->Channel); // Select the channel if not already selected
  if (Error != ERR_OK) return Error;                                      // If there is an error while calling TCA9543A_SelectChannel() then return the Error
  return pI2C->fnI2C_Transfer(pI2C, pPacketDesc);                         // Perform the I2C Transfer
}

//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
/*******************************************************************************
 * @file    TCA9543A.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    27/02/2020
 * @brief   TCA9543A driver
 *
 * Low Voltage 2-Channel I2C Bus Switch With Interrupt Logic And Reset
 * Follow datasheet SCPS206B Rev B (Mar 2014)
 ******************************************************************************/
 /* @page License
 *
 * Copyright (c) 2020 Fabien MAILLY
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS,
 * IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *****************************************************************************/
#ifndef TCA9543A_H_INC
#define TCA9543A_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#include "ErrorsDef.h"
#include "I2C_Interface.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#  define __TCA9543A_PACKED__
#  define TCA9543A_PACKITEM    __pragma(pack(push, 1))
#  define TCA9543A_UNPACKITEM  __pragma(pack(pop))
#else
#  define __TCA9543A_PACKED__  __attribute__((packed))
#  define TCA9543A_PACKITEM
#  define TCA9543A_UNPACKITEM
#endif
//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define TCA9543A_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------



// Limits definitions
#define TCA9543A_I2C_CLOCK_MAX  ( 400000u ) //!< Max I2C clock frequency



// Device definitions
#define TCA9543A_CHIPADDRESS_BASE  ( 0xE0u ) //!< TCA9543A chip base address
#define TCA9543A_CHIPADDRESS_MASK  ( 0xF0u ) //!< TCA9543A chip base address



/*! @brief Generate the TCA9543A chip configurable address following the state of A0 and A1
 * You shall set '1' (when corresponding pin is connected to +V) or '0' (when corresponding pin is connected to Ground) on each parameter
 */
#define TCA9543A_ADDR(A1, A0)  ( (uint8_t)((((A1) & 0x01) << 2) | (((A0) & 0x01) << 1)) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCA9543A Specific Controller Registers
//********************************************************************************************************************

//! Status Register
TCA9543A_PACKITEM
typedef union __TCA9543A_PACKED__ TCA9543A_Control_Register
{
  uint8_t Control;
  struct
  {
    uint8_t B0  : 1; //!< 0   - Channel 0: '1' = Enable ; '0' = Disable
    uint8_t B1  : 1; //!< 1   - Channel 1: '1' = Enable ; '0' = Disable
    uint8_t     : 2; //!< 2-3
    uint8_t INT0: 1; //!< 4   - Interrupt on channel 0: '1' = An interrupt occur on pin INT0 ; '0' = No interrupt on pin INT0
    uint8_t INT1: 1; //!< 5   - Interrupt on channel 1: '1' = An interrupt occur on pin INT1 ; '0' = No interrupt on pin INT1
    uint8_t     : 2; //!< 6-7
  } Bits;
} TCA9543A_Control_Register;
TCA9543A_UNPACKITEM;
TCA9543A_CONTROL_ITEM_SIZE(TCA9543A_Control_Register, 1);

#define TCA9543A_CHANNEL0_ENABLE   (0x1u << 0) //!< Enable Channel 0
#define TCA9543A_CHANNEL0_DISABLE  (0x0u << 0) //!< Disable Channel 0
#define TCA9543A_CHANNEL1_ENABLE   (0x1u << 1) //!< Enable Channel 1
#define TCA9543A_CHANNEL1_DISABLE  (0x0u << 1) //!< Disable Channel 1

//! Channel select
typedef enum
{
  TCA9543A_NO_CHANNEL_SELECTED = 0b00, //!< No write protect
  TCA9543A_SELECT_CHANNEL_0    = 0b01, //!< Select only channel 0
  TCA9543A_SELECT_CHANNEL_1    = 0b10, //!< Select only channel 1
  TCA9543A_SELECT_All_CHANNELS = 0b11, //!< Select both channel 0 and 1. Care should be taken not to exceed the maximum bus capacitance (not recommanded if components have same address on both channels)
  TCA9543A_CHANNEL_COUNT,              // Keep last
} eTCA9543A_ChannelSelect;

#define TCA9543A_CHANNELx_Pos          0
#define TCA9543A_CHANNELx_Mask         (0x3u << TCA9543A_CHANNELx_Pos)
#define TCA9543A_CHANNELx_SET(value)   (((uint8_t)(value) << TCA9543A_CHANNELx_Pos) & TCA9543A_CHANNELx_Mask)                   //!< Set Channels bits
#define TCA9543A_CHANNELx_GET(value)   ((eTCA9543A_ChannelSelect)(((value) & TCA9543A_CHANNELx_Mask) >> TCA9543A_CHANNELx_Pos)) //!< Get Channels bits
#define TCA9543A_INTERRUPT_ON_INT0     (0x1u << 4) //!< Interrupt on channel 0
#define TCA9543A_NO_INTERRUPT_ON_INT0  (0x0u << 4) //!< No interrupt on channel 0
#define TCA9543A_INTERRUPT_ON_INT1     (0x1u << 5) //!< Interrupt on channel 1
#define TCA9543A_NO_INTERRUPT_ON_INT1  (0x0u << 5) //!< No interrupt on channel 1
#define TCA9543A_INTERRUPT_MASK        (TCA9543A_INTERRUPT_ON_INT0 | TCA9543A_INTERRUPT_ON_INT1) //!< This is the interrupt channels mask

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCA9543A Driver API
//********************************************************************************************************************
typedef struct TCA9543A TCA9543A;        //! Typedef of TCA9543A device object structure
typedef uint8_t TTCA9543ADriverInternal; //! Alias for Driver Internal data flags


//! TCA9543A device object structure
struct TCA9543A
{
  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  I2C_Interface* I2C;                     //!< This is the I2C_Interface descriptor pointer that will be used to communicate with the device
#else
  I2C_Interface I2C;                      //!< This is the I2C_Interface descriptor that will be used to communicate with the device
#endif
  uint32_t I2CclockSpeed;                 //!< Clock frequency of the I2C interface in Hertz

  //--- Device address ---
  uint8_t AddrA1A0;                       //!< Device configurable address A1 and A0. You can use the macro TCA9543A_ADDR() to help filling this parameter. Only these 2 lower bits are used: .....10. where 1 is A1, 0 is A0, and '.' are fixed by device

  //--- Last channel set ---
  TTCA9543ADriverInternal InternalConfig; //!< DO NOT USE OR CHANGE THIS VALUE, IT'S THE INTERNAL DRIVER CONFIGURATION
};

//********************************************************************************************************************





/*! @brief TCA9543A initialization
 *
 * This function initializes the TCA9543A driver and call the initialization of the interface driver (I2C).
 * Next it checks parameters and configures the TCA9543A
 * @param[in] *pComp Is the pointed structure of the device to be initialized
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_TCA9543A(TCA9543A *pComp);

//-----------------------------------------------------------------------------



/*! @brief Select the output channel of the TCA9543A device
 *
 * This function select the new channel output. It can be no channel, channel 0, channel 1, or both
 * The function check the need to change the channel. If the new channel is already selected then the function does nothing
 * @warning When select both channel, care should be taken not to exceed the maximum bus capacitance. This mode is not recommanded if components have same address on both channels
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] channelSelect Is the new channel to select
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCA9543A_SelectChannel(TCA9543A *pComp, eTCA9543A_ChannelSelect channelSelect);


/*! @brief Get the device status of the TCA9543A device
 *
 * This function reads the control register and return its status
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *currentChannel Is where the current channel will be stored
 * @param[out] *interruptChannel Is where the current interrupt will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCA9543A_GetDeviceStatus(TCA9543A *pComp, eTCA9543A_ChannelSelect *currentChannel, uint8_t *interruptChannel);


/*! @brief Get the current channel of the TCA9543A device
 *
 * This function reads the control register and return the current channel
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *currentChannel Is where the current channel will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCA9543A_GetCurrentChannel(TCA9543A *pComp, eTCA9543A_ChannelSelect *currentChannel)
{
  return TCA9543A_GetDeviceStatus(pComp, currentChannel, NULL);
}


/*! @brief Get the current interrupt of the TCA9543A device
 *
 * This function reads the control register and return the interrupt status of INT0 and INT1
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *interruptChannel Is where the current interrupt will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCA9543A_GetCurrentInterrupt(TCA9543A *pComp, uint8_t *interruptChannel)
{
  return TCA9543A_GetDeviceStatus(pComp, NULL, interruptChannel);
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCA9543A I2C Driver API
//********************************************************************************************************************

/*! @brief Perform the I2C Initialization of the specified I2C channel of the TCA9543A device
 *
 * This is the function that shall be called each time a I2C reinitialization or a SCL frequency change needs to be done
 * Before calling the TCA9543A.fnI2C_Init() function, there is a check of the SCL frequency
 * @warning Calling this function to change the I2C SCL speed will change the I2C speed for all devices connected on all channels of the TCA9543A device
 * @param[in] *pIntDev Is the I2C interface container structure used for the communication. It will convert the pIntDev->InterfaceDevice into a TCA9543A structure to use its I2C interface
 * @param[in] sclFreq Is the SCL frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCA9543A_I2CInit(I2C_Interface *pIntDev, const uint32_t sclFreq);


/*! @brief Perform the I2C Transfer to the specified I2C channel of the TCA9543A device
 *
 * This function will be called by another device that needs to communicate with a device connected on one of a TCA9543A I2C channel
 * This function will change the current channel, if necessary, before sending the I2C Transfer
 * Can be a read of data or a transmit of data. It also indicate if it needs a start and/or a stop
 * @warning A TCA9543A_I2CInit() must be called before using this function for the first time
 * @param[in] *pIntDev Is the I2C interface container structure used for the communication. It will convert the pIntDev->InterfaceDevice into a TCA9543A structure to use its I2C interface
 * @param[in] *pPacketConf Is the packet description to transfer through I2C
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCA9543A_I2CTransfer(I2C_Interface *pIntDev, I2CInterface_Packet* const pPacketDesc);

//********************************************************************************************************************





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* TCA9543A_H_INC */
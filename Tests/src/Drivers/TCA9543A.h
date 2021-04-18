/*******************************************************************************
 * @file    TCA9543A.h
 * @author  FMA
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
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------

#ifndef __PACKED__
# ifndef __cplusplus
#   define __PACKED__  __attribute__((packed))
# else
#   define __PACKED__
# endif
#endif

#ifndef PACKITEM
# ifndef __cplusplus
#   define PACKITEM
# else
#   define PACKITEM  __pragma(pack(push, 1))
# endif
#endif

#ifndef UNPACKITEM
# ifndef __cplusplus
#   define UNPACKITEM
# else
#   define UNPACKITEM  __pragma(pack(pop))
# endif
#endif
//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#ifndef ControlItemSize
#  define ControlItemSize(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }
#endif

//-----------------------------------------------------------------------------



// Limits definitions
#define TCA9543A_I2C_CLOCK_MAX  ( 400000u ) //!< Max I2C clock frequency



// Device definitions
#define TCA9543A_I2C_READ   ( 0x01u ) //!< Standard I2C LSB bit to set
#define TCA9543A_I2C_WRITE  ( 0xFEu ) //!< Standard I2C bit mask which clear the LSB

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
PACKITEM
typedef union __PACKED__ TCA9543A_Control_Register
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
UNPACKITEM;
ControlItemSize(TCA9543A_Control_Register, 1);

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



/*! @brief Interface function for driver initialization of the TCA9543A
 *
 * This function will be called at driver initialization to configure the interface driver
 * @param[in] *pIntDev Is the TCA9543A.InterfaceDevice of the device that call the interface initialization
 * @param[in] sclFreq Is the SCL frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
typedef eERRORRESULT (*TCA9543A_I2CInit_Func)(void *pIntDev, const uint32_t sclFreq);


/*! @brief Interface function for I2C transfer of the AT24MAC402
 *
 * This function will be called when the driver needs to transfer data over the I2C communication with the device
 * Can be a read of data or a transmit of data. It also indicate if it needs a start and/or a stop
 * @warning A I2CInit_Func() must be called before using this function
 * @param[in] *pIntDev Is the AT24MAC402.InterfaceDevice of the device that call the I2C transfer
 * @param[in] deviceAddress Is the device address on the bus (8-bits only). The LSB bit indicate if it is a I2C Read (bit at '1') or a I2C Write (bit at '0')
 * @param[in,out] *data Is a pointer to memory data to write in case of I2C Write, or where the data received will be stored in case of I2C Read (can be NULL if no data transfer other than chip address)
 * @param[in] byteCount Is the byte count to write over the I2C bus or the count of byte to read over the bus
 * @param[in] start Indicate if the transfer needs a start (in case of a new transfer) or restart (if the previous transfer have not been stopped)
 * @param[in] stop Indicate if the transfer needs a stop after the last byte sent
 * @return Returns an #eERRORRESULT value enum
 */
typedef eERRORRESULT (*TCA9543A_I2CTranfer_Func)(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop);



//! TCA9543A device object structure
struct TCA9543A
{
  //--- Interface clocks ---
  uint32_t I2C_ClockSpeed;                 //!< Clock frequency of the I2C interface in Hertz

  //--- Interface driver call functions ---
  void *InterfaceDevice;                   //!< This is the pointer that will be in the first parameter of all interface call functions
  TCA9543A_I2CInit_Func fnI2C_Init;        //!< This function will be called at driver initialization to configure the interface driver
  TCA9543A_I2CTranfer_Func fnI2C_Transfer; //!< This function will be called when the driver needs to transfer data over the I2C communication with the device

  //--- Device address ---
  uint8_t AddrA1A0;                        //!< Device configurable address A1 and A0. You can use the macro TCA9543A_ADDR() to help filling this parameter. Only these 2 lower bits are used: .....10. where 1 is A1, 0 is A0, and '.' are fixed by device

  //--- Last channel set ---
  TTCA9543ADriverInternal InternalConfig;  //!< DO NOT USE OR CHANGE THIS VALUE, IT'S THE INTERNAL DRIVER CONFIGURATION
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
typedef struct TCA9543A_I2C TCA9543A_I2C; //! SC16IS7XX UART object structure


/*! @brief SC16IS7XX UART object structure
 * @warning Each Channel and Device tuple should be unique. Only 1 possible tuple on SC16IS7X0 and 2 possible tuple on SC16IS7X2 devices
 */
struct TCA9543A_I2C
{
  //--- Device configuration ---
  void *UserDriverData;            //!< Optional, can be used to store driver data or NULL
  TCA9543A *Device;                //!< TCA9543A device where this I2C comes from

  //--- I2C configuration ---
  eTCA9543A_ChannelSelect Channel; //!< I2C channel of the TCA9543A
};

//********************************************************************************************************************





/*! @brief Perform the I2C Init of the specified I2C channel of the TCA9543A device
 *
 * This is the function that shall be called each time a I2C reinitialization or a SCL frequency change needs to be done
 * Before calling the TCA9543A.fnI2C_Init() function, there is a check of the SCL frequency
 * @warning Calling this function to change the I2C SCL speed will change the I2C speed for all devices connected on all channels of the TCA9543A device
 * @param[in] *pComp Is the pointed structure of the device that call the interface initialization
 * @param[in] *pIntDev Is the TCA9543A.InterfaceDevice of the device that call the interface initialization
 * @param[in] sclFreq Is the SCL frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCA9543A_I2CInit(void *pIntDev, const uint32_t sclFreq);


/*! @brief Perform the I2C Transfer to the specified I2C channel of the TCA9543A device
 *
 * This function will be called by another device that needs to communicate with a device connected on one of a TCA9543A I2C channel
 * This function will change the current channel, if necessary, before sending the I2C Transfer
 * Can be a read of data or a transmit of data. It also indicate if it needs a start and/or a stop
 * @warning A TCA9543A_I2CInit() must be called before using this function for the first time
 * @param[in] *pIntDev This will be transformed into a TCA9543A_I2C device and use associated TCA9543A.fnI2C_Transfer to call the I2C transfer
 * @param[in] deviceAddress Is the device address on the bus (8-bits only). The LSB bit indicate if it is a I2C Read (bit at '1') or a I2C Write (bit at '0')
 * @param[in,out] *data Is a pointer to memory data to write in case of I2C Write, or where the data received will be stored in case of I2C Read (can be NULL if no data transfer other than chip address)
 * @param[in] byteCount Is the byte count to write over the I2C bus or the count of byte to read over the bus
 * @param[in] start Indicate if the transfer needs a start (in case of a new transfer) or restart (if the previous transfer have not been stopped)
 * @param[in] stop Indicate if the transfer needs a stop after the last byte sent
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCA9543A_I2CTranfert(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop);

//********************************************************************************************************************





//-----------------------------------------------------------------------------
#undef __PACKED__
#undef PACKITEM
#undef UNPACKITEM
#undef ControlItemSize
//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------
#endif /* TCA9543A_H_INC */
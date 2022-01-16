/*******************************************************************************
    File name:    TWIHS_V71.h
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 18/04/2021
    Description:  TWIHS driver for Atmel MCUs
                  This interface implements a synchronous use of the I2C and
                  an asynchronous use of I2C by using a DMA
    History :
*******************************************************************************/
#ifndef TWIHS_V71_H_INC
#define TWIHS_V71_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
//#include "twihs.h"  // For Twihs struct of the MCU
#include "sysclk.h" // For system clocks of the MCU
//#include "core_cm7.h
//-----------------------------------------------------------------------------
#include "ErrorsDef.h"
#include "I2C_Interface.h"
#ifdef __cplusplus
extern "C" {
#endif
//-----------------------------------------------------------------------------



// Limits definitions
#define TWIHS_I2CCLOCK_FM_MIN        (  384000u )               //!< Min I2C clock frequency in FM mode (Fast-Mode)
#define TWIHS_I2CCLOCK_FM_MAX        (  400000u )               //!< Max I2C clock frequency in FM mode (Fast-Mode)
#define TWIHS_I2CCLOCK_HSM_MAX       ( 3400000u )               //!< Max I2C clock frequency in HSM mode (High-Speed Mode)
#define TWIHS_MASTER_MODE_CLOCK_MAX  ( TWIHS_I2CCLOCK_FM_MAX )  //!< Max I2C clock frequency in master mode
#define TWIHS_SLAVE_MODE_CLOCK_MAX   ( TWIHS_I2CCLOCK_HSM_MAX ) //!< Max I2C clock frequency in slave mode


// Definitions
#define TWIHS_CLKDIV_EXPONENT  ( 2 )   //! See formulas of TWIHS Clock Waveform Generator Register
#define TWIHS_CLKDIV_ARGUMENT  ( 3 )   //! See formulas of TWIHS Clock Waveform Generator Register
#define TWIHS_CL_CK_DIV_MAX    ( 255 ) //! See formulas of TWIHS Clock Waveform Generator Register
#define TWIHS_CK_DIV_MAX       ( 7 )   //! See formulas of TWIHS Clock Waveform Generator Register
#define TWIHS_CLK_DIVIDER      ( 2 )   //! See formulas of TWIHS Clock Waveform Generator Register

#define TWIHS_INVALID_PERIPHERAL  ( 0xFFFFFFFF ) //! Invalid peripheral value
#define TWIHS_ALL_INTERRUPTS      ( 0x003D0FF7 ) //! Select all interrupts
#define TWIHS_FAULT_STATUS        ( TWIHS_SR_NACK | TWIHS_SR_ARBLST | TWIHS_SR_OVRE | TWIHS_SR_UNRE ) //! Fault status of the TWIHS for the interrupt handler

#define XDMAC_TWIHS_PERID_Base  ( 14 ) //! Base of the TWIHS HW Interface Number (XDMAC_CC.PERID)

#define TWIHS_TIMEOUT  ( 30000 ) //! Time-out value (number of attempts)

//-----------------------------------------------------------------------------

#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
#  define TWIHS_COUNT  3
#else
#  if SAMG55
#    define TWIHS_COUNT  8
#  else
#    define TWIHS_COUNT  1
#  endif
#endif

//-----------------------------------------------------------------------------

//=== I2C0 pins ===
#define SCL_I2C0_FLAGS       ( IOPORT_MODE_MUX_A )
#define SCL_I2C0_GPIO        ( PIO_PA4_IDX )
#define SCL_I2C0_IRQn        ( PIOA_IRQn )
#define SCL_I2C0_ID          ( ID_PIOA )
#define SCL_I2C0_PIO         ( PIOA )
#define SCL_I2C0_PORT        ( IOPORT_PIOA )
#define SCL_I2C0_MASK        ( PIO_PA4 )
#define SCL_I2C0_PIO_En        SCL_I2C0_PIO->PIO_PER  |= SCL_I2C0_MASK
#define SCL_I2C0_Out           SCL_I2C0_PIO->PIO_OER  |= SCL_I2C0_MASK
#define SCL_I2C0_In            SCL_I2C0_PIO->PIO_ODR  |= SCL_I2C0_MASK
#define SCL_I2C0_High          SCL_I2C0_PIO->PIO_SODR |= SCL_I2C0_MASK
#define SCL_I2C0_Low           SCL_I2C0_PIO->PIO_CODR |= SCL_I2C0_MASK

#define SDA_I2C0_FLAGS       ( IOPORT_MODE_MUX_A )
#define SDA_I2C0_GPIO        ( PIO_PA3_IDX )
#define SDA_I2C0_IRQn        ( PIOA_IRQn )
#define SDA_I2C0_ID          ( ID_PIOA )
#define SDA_I2C0_PIO         ( PIOA )
#define SDA_I2C0_PORT        ( IOPORT_PIOA )
#define SDA_I2C0_MASK        ( PIO_PA3 )
#define SDA_I2C0_PIO_En        SDA_I2C0_PIO->PIO_PER  |= SDA_I2C0_MASK
#define SDA_I2C0_Out           SDA_I2C0_PIO->PIO_OER  |= SDA_I2C0_MASK
#define SDA_I2C0_In            SDA_I2C0_PIO->PIO_ODR  |= SDA_I2C0_MASK
#define SDA_I2C0_High          SDA_I2C0_PIO->PIO_SODR |= SDA_I2C0_MASK
#define SDA_I2C0_Low           SDA_I2C0_PIO->PIO_CODR |= SDA_I2C0_MASK
#define SDA_I2C0_Status      ( SDA_I2C0_PIO->PIO_PDSR &  SDA_I2C0_MASK )

//=== I2C1 pins ===
#define SCL_I2C1_FLAGS       ( IOPORT_MODE_MUX_A )
#define SCL_I2C1_GPIO        ( PIO_PB5_IDX )
#define SCL_I2C1_IRQn        ( PIOB_IRQn )
#define SCL_I2C1_ID          ( ID_PIOB )
#define SCL_I2C1_PIO         ( PIOB )
#define SCL_I2C1_PORT        ( IOPORT_PIOB )
#define SCL_I2C1_MASK        ( PIO_PB5 )
#define SCL_I2C1_PIO_En        SCL_I2C1_PIO->PIO_PER  |= SCL_I2C1_MASK
#define SCL_I2C1_Out           SCL_I2C1_PIO->PIO_OER  |= SCL_I2C1_MASK
#define SCL_I2C1_In            SCL_I2C1_PIO->PIO_ODR  |= SCL_I2C1_MASK
#define SCL_I2C1_High          SCL_I2C1_PIO->PIO_SODR |= SCL_I2C1_MASK
#define SCL_I2C1_Low           SCL_I2C1_PIO->PIO_CODR |= SCL_I2C1_MASK

#define SDA_I2C1_FLAGS       ( IOPORT_MODE_MUX_A )
#define SDA_I2C1_GPIO        ( PIO_PB4_IDX )
#define SDA_I2C1_IRQn        ( PIOB_IRQn )
#define SDA_I2C1_ID          ( ID_PIOB )
#define SDA_I2C1_PIO         ( PIOB )
#define SDA_I2C1_PORT        ( IOPORT_PIOB )
#define SDA_I2C1_MASK        ( PIO_PB4 )
#define SDA_I2C1_PIO_En        SDA_I2C1_PIO->PIO_PER  |= SDA_I2C1_MASK
#define SDA_I2C1_Out           SDA_I2C1_PIO->PIO_OER  |= SDA_I2C1_MASK
#define SDA_I2C1_In            SDA_I2C1_PIO->PIO_ODR  |= SDA_I2C1_MASK
#define SDA_I2C1_High          SDA_I2C1_PIO->PIO_SODR |= SDA_I2C1_MASK
#define SDA_I2C1_Low           SDA_I2C1_PIO->PIO_CODR |= SDA_I2C1_MASK
#define SDA_I2C1_Status      ( SDA_I2C1_PIO->PIO_PDSR &  SDA_I2C1_MASK )

//=== I2C2 pins ===
#define SCL_I2C2_FLAGS       ( IOPORT_MODE_MUX_C )
#define SCL_I2C2_GPIO        ( PIO_PD28_IDX )
#define SCL_I2C2_IRQn        ( PIOD_IRQn )
#define SCL_I2C2_ID          ( ID_PIOD )
#define SCL_I2C2_PIO         ( PIOD )
#define SCL_I2C2_PORT        ( IOPORT_PIOD )
#define SCL_I2C2_MASK        ( PIO_PD28 )
#define SCL_I2C2_PIO_En        SCL_I2C2_PIO->PIO_PER  |= SCL_I2C2_MASK
#define SCL_I2C2_Out           SCL_I2C2_PIO->PIO_OER  |= SCL_I2C2_MASK
#define SCL_I2C2_In            SCL_I2C2_PIO->PIO_ODR  |= SCL_I2C2_MASK
#define SCL_I2C2_High          SCL_I2C2_PIO->PIO_SODR |= SCL_I2C2_MASK
#define SCL_I2C2_Low           SCL_I2C2_PIO->PIO_CODR |= SCL_I2C2_MASK

#define SDA_I2C2_FLAGS       ( IOPORT_MODE_MUX_C )
#define SDA_I2C2_GPIO        ( PIO_PD27_IDX )
#define SDA_I2C2_IRQn        ( PIOD_IRQn )
#define SDA_I2C2_ID          ( ID_PIOD )
#define SDA_I2C2_PIO         ( PIOD )
#define SDA_I2C2_PORT        ( IOPORT_PIOD )
#define SDA_I2C2_MASK        ( PIO_PD27 )
#define SDA_I2C2_PIO_En        SDA_I2C2_PIO->PIO_PER  |= SDA_I2C2_MASK
#define SDA_I2C2_Out           SDA_I2C2_PIO->PIO_OER  |= SDA_I2C2_MASK
#define SDA_I2C2_In            SDA_I2C2_PIO->PIO_ODR  |= SDA_I2C2_MASK
#define SDA_I2C2_High          SDA_I2C2_PIO->PIO_SODR |= SDA_I2C2_MASK
#define SDA_I2C2_Low           SDA_I2C2_PIO->PIO_CODR |= SDA_I2C2_MASK
#define SDA_I2C2_Status      ( SDA_I2C2_PIO->PIO_PDSR &  SDA_I2C2_MASK )

//-----------------------------------------------------------------------------

//! TWIHS transfer status enumerator
typedef enum
{
  TWIHS_STATUS_UNINITIALIZED, //!< TWIHS uninitialized
  TWIHS_STATUS_READY,         //!< TWIHS transfer is ready
  TWIHS_STATUS_IN_PROGRESS,   //!< TWIHS transfer in progress
  TWIHS_STATUS_COMPLETE,      //!< TWIHS transfer complete
  TWIHS_STATUS_FAULT,         //!< TWIHS transfer in fault
} eTWIHS_TransferStatus;

//! TWIHS & XDMAC transfer structure
typedef struct TWIHS_TransferStruct
{
  //--- Buffer Data ---
  uint8_t* Data;                //!< Data buffer
  size_t Size;                  //!< Size of the buffer
  size_t RemainingBytes;        //!< Byte remaining to process

  //--- Transfer Status ---
  bool DeviceWrite;             //!< Current transfer is a write?
  eTWIHS_TransferStatus Status; //!< Transfer status
  eERRORRESULT Error;           //!< Error status
  I2C_Conf Config;              //!< Transfer configuration
  uint8_t TransactionCounter;   //!< The current transaction counter
} TWIHS_TransferStruct;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TWIHS driver API
//********************************************************************************************************************

/*! @brief Atmel TWIHS master initialization
 *
 * It initialize the I2C and set its clock speed
 * @param[in] *pTWIHS Is the TWIHS peripheral to initialize
 * @param[in] sclFreq Is the desired frequency of the SCL pin of the TWIHS
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_MasterInit(Twihs* pTWIHS, const uint32_t sclFreq);
eERRORRESULT TWIHS_MasterInit_Gen(I2C_Interface* pIntDev, const uint32_t sclFreq);


/*! @brief Get peripheral ID of the Atmel TWIHS
 *
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @return The TWIHS peripheral ID. Returns TWIHS_INVALID_PERIPHERAL if not found
 */
uint32_t TWIHS_GetPeripheralID(Twihs* pTWIHS);


/*! @brief Get peripheral number of the Atmel TWIHS
 *
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @return The TWIHS peripheral number. Returns TWIHS_INVALID_PERIPHERAL if not found
 */
uint32_t TWIHS_GetPeripheralNumber(Twihs* pTWIHS);

//-----------------------------------------------------------------------------



/*! @brief Enable interrupts of the Atmel TWIHS
 *
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @param[in] sourcesInterrupts Source interrupts to enable (can be OR'ed)
 * @param[in] enableNVIC Set to 'true' to enable the peripheral in the NVIC. Setting 'false' does not disable the peripheral in the NVIC
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_InterruptEnable(Twihs* pTWIHS, uint32_t sourcesInterrupts, bool enableNVIC);


/*! @brief Disable interrupts of the Atmel TWIHS
 *
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @param[in] sourcesInterrupts Source interrupts to disable (can be OR'ed)
 * @param[in] disableNVIC Set to 'true' to disable the peripheral in the NVIC. Setting 'false' does not enable the peripheral in the NVIC
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_InterruptDisable(Twihs* pTWIHS, uint32_t sourcesInterrupts, bool disableNVIC);


/*! @brief Get interrupt status of the Atmel TWIHS
 *
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @return Returns the actual interrupt status of the peripheral
 */
inline uint32_t TWIHS_GetInterruptStatus(Twihs* pTWIHS)
{
  return pTWIHS->TWIHS_SR;
}

//-----------------------------------------------------------------------------



/*! @brief Reset the Atmel TWIHS
 * @param[in] *pTWIHS Is the TWIHS peripheral to reset
 */
void TWIHS_Reset(Twihs* pTWIHS);


/*! @brief Recover an I2C bus of the Atmel TWIHS
 *
 * When a the CPU is reset when transfering data, the stop is not sent. This function will search where the current data blocked and recover the bus
 * @param[in] *pTWIHS Is the TWIHS peripheral to recover
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_BusRecovery(Twihs* pTWIHS);

//-----------------------------------------------------------------------------



/*! @brief Set the I2C SCL clock in Hertz of the Atmel TWIHS
 *
 * Calculate the timings parameters and set them to the TWIHS peripheral
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @param[in] desiredClockHz Is the desired SCL clock speed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_SetI2CclockHz(Twihs* pTWIHS, uint32_t desiredClockHz);

//-----------------------------------------------------------------------------



/*! @brief Hardware I2C data transfer communication for the Atmel TWIHS
 *
 * This function takes care of the transfer of the specified packet following the I2C_Interface specification
 * @param[in] *pTWIHS Is the TWIHS peripheral to use
 * @param[in] *pPacketDesc(deviceAddress,*data,byteCount,start,stop) Is the packet configuration to transfer
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_Transfer(Twihs* pTWIHS, I2CInterface_Packet* const pPacketDesc);
eERRORRESULT TWIHS_Transfer_Gen(I2C_Interface *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop);

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TWIHS with DMA driver API
//********************************************************************************************************************

/*! @brief Atmel TWIHS master with DMA initialization
 *
 * It initialize the I2C and set its clock speed. It will prepare a DMA channel for transfer
 * @param[in] *pIntDev Is the TWIHS peripheral to initialize
 * @param[in] sclFreq Is the desired frequency of the SCL pin of the TWIHS
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_DMA_MasterInit(Twihs *pIntDev, const uint32_t sclFreq);
eERRORRESULT TWIHS_DMA_MasterInit_Gen(I2C_Interface *pIntDev, const uint32_t sclFreq);


/*! @brief Hardware I2C data transfer with DMA communication for the Atmel TWIHS
 *
 * This function takes care of the transfer of the specified packet following the I2C_Interface specification
 * It configures a DMA channel for the transfer. If no DMA transfer is specified, the function will redirect to the TWIHS_Transfer() function
 * @param[in] *pIntDev Is the TWIHS peripheral to use
 * @param[in] *pPacketDesc Is the packet configuration to transfer
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TWIHS_PacketTransfer(Twihs *pIntDev, I2CInterface_Packet* const pPacketDesc);
eERRORRESULT TWIHS_PacketTransfer_Gen(I2C_Interface *pIntDev, I2CInterface_Packet* const pPacketDesc);

//********************************************************************************************************************





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* TWIHS_V71_H_INC */
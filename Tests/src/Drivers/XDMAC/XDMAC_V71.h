/*******************************************************************************
    File name:    XDMAC_V71.h
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 18/04/2021
    Description:  XDMAC driver for Atmel MCUs
                  This interface implements a driver the the XDMAC of the SAM V71
    History :
*******************************************************************************/
#ifndef XDMAC_V71_H_INC
#define XDMAC_V71_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
//#include "xdmac.h"  // For XDMAC struct of the MCU
#include "sysclk.h" // For system clocks of the MCU
//#include "core_cm7.h
//-----------------------------------------------------------------------------
#include "ErrorsDef.h"
#ifdef __cplusplus
extern "C" {
#endif
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// XDMAC Specific Controller Registers
//********************************************************************************************************************

//! Peripheral Hardware Requests list for the DMA channel hardware interface number
typedef enum
{
  XDMAC_HW_INTERFACE_HSMCI_TXRX =  0, //!< Hardware Interface Number (XDMAC_CC.PERID) 0: HighSpeed Multimedia Card Interface (HSMCI) in Transmit/Receive
  XDMAC_HW_INTERFACE_SPI0_TX    =  1, //!< Hardware Interface Number (XDMAC_CC.PERID) 1: Serial Peripheral Interface 0 (SPI0) in Transmit
  XDMAC_HW_INTERFACE_SPI0_RX    =  2, //!< Hardware Interface Number (XDMAC_CC.PERID) 2: Serial Peripheral Interface 0 (SPI0) in Receive
  XDMAC_HW_INTERFACE_SPI1_TX    =  3, //!< Hardware Interface Number (XDMAC_CC.PERID) 3: Serial Peripheral Interface 1 (SPI1) in Transmit
  XDMAC_HW_INTERFACE_SPI1_RX    =  4, //!< Hardware Interface Number (XDMAC_CC.PERID) 4: Serial Peripheral Interface 1 (SPI1) in Receive
  XDMAC_HW_INTERFACE_QSPI_TX    =  5, //!< Hardware Interface Number (XDMAC_CC.PERID) 5: Quad Serial Peripheral Interface (QSPI) in Transmit
  XDMAC_HW_INTERFACE_QSPI_RX    =  6, //!< Hardware Interface Number (XDMAC_CC.PERID) 6: Quad Serial Peripheral Interface (QSPI) in Receive
  XDMAC_HW_INTERFACE_USART0_TX  =  7, //!< Hardware Interface Number (XDMAC_CC.PERID) 7: Universal Synchronous Asynchronous Receiver Transceiver 0 (USART0) in Transmit
  XDMAC_HW_INTERFACE_USART0_RX  =  8, //!< Hardware Interface Number (XDMAC_CC.PERID) 8: Universal Synchronous Asynchronous Receiver Transceiver 0 (USART0) in Receive
  XDMAC_HW_INTERFACE_USART1_TX  =  9, //!< Hardware Interface Number (XDMAC_CC.PERID) 9: Universal Synchronous Asynchronous Receiver Transceiver 1 (USART1) in Transmit
  XDMAC_HW_INTERFACE_USART1_RX  = 10, //!< Hardware Interface Number (XDMAC_CC.PERID) 10: Universal Synchronous Asynchronous Receiver Transceiver 1 (USART1) in Receive
  XDMAC_HW_INTERFACE_USART2_TX  = 11, //!< Hardware Interface Number (XDMAC_CC.PERID) 11: Universal Synchronous Asynchronous Receiver Transceiver 2 (USART2) in Transmit
  XDMAC_HW_INTERFACE_USART2_RX  = 12, //!< Hardware Interface Number (XDMAC_CC.PERID) 12: Universal Synchronous Asynchronous Receiver Transceiver 2 (USART2) in Receive
  XDMAC_HW_INTERFACE_PWM0_TX    = 13, //!< Hardware Interface Number (XDMAC_CC.PERID) 13: Pulse Width Modulation Controller 0 (PWM0) in Transmit
  XDMAC_HW_INTERFACE_TWIHS0_TX  = 14, //!< Hardware Interface Number (XDMAC_CC.PERID) 14: Two Wire Interface High-Speed 0 (TWIHS0) in Transmit
  XDMAC_HW_INTERFACE_TWIHS0_RX  = 15, //!< Hardware Interface Number (XDMAC_CC.PERID) 15: Two Wire Interface High-Speed 0 (TWIHS0) in Receive
  XDMAC_HW_INTERFACE_TWIHS1_TX  = 16, //!< Hardware Interface Number (XDMAC_CC.PERID) 16: Two Wire Interface High-Speed 1 (TWIHS1) in Transmit
  XDMAC_HW_INTERFACE_TWIHS1_RX  = 17, //!< Hardware Interface Number (XDMAC_CC.PERID) 17: Two Wire Interface High-Speed 1 (TWIHS1) in Receive
  XDMAC_HW_INTERFACE_TWIHS2_TX  = 18, //!< Hardware Interface Number (XDMAC_CC.PERID) 18: Two Wire Interface High-Speed 2 (TWIHS2) in Transmit
  XDMAC_HW_INTERFACE_TWIHS2_RX  = 19, //!< Hardware Interface Number (XDMAC_CC.PERID) 19: Two Wire Interface High-Speed 2 (TWIHS2) in Receive
  XDMAC_HW_INTERFACE_UART0_TX   = 20, //!< Hardware Interface Number (XDMAC_CC.PERID) 20: Universal Asynchronous Receiver Transceiver 0 (USART0) in Transmit
  XDMAC_HW_INTERFACE_UART0_RX   = 21, //!< Hardware Interface Number (XDMAC_CC.PERID) 21: Universal Asynchronous Receiver Transceiver 0 (USART0) in Receive
  XDMAC_HW_INTERFACE_UART1_TX   = 22, //!< Hardware Interface Number (XDMAC_CC.PERID) 22: Universal Asynchronous Receiver Transceiver 1 (USART1) in Transmit
  XDMAC_HW_INTERFACE_UART1_RX   = 23, //!< Hardware Interface Number (XDMAC_CC.PERID) 23: Universal Asynchronous Receiver Transceiver 1 (USART1) in Receive
  XDMAC_HW_INTERFACE_UART2_TX   = 24, //!< Hardware Interface Number (XDMAC_CC.PERID) 24: Universal Asynchronous Receiver Transceiver 2 (USART2) in Transmit
  XDMAC_HW_INTERFACE_UART2_RX   = 25, //!< Hardware Interface Number (XDMAC_CC.PERID) 25: Universal Asynchronous Receiver Transceiver 2 (USART2) in Receive
  XDMAC_HW_INTERFACE_UART3_TX   = 26, //!< Hardware Interface Number (XDMAC_CC.PERID) 26: Universal Asynchronous Receiver Transceiver 3 (USART3) in Transmit
  XDMAC_HW_INTERFACE_UART3_RX   = 27, //!< Hardware Interface Number (XDMAC_CC.PERID) 27: Universal Asynchronous Receiver Transceiver 3 (USART3) in Receive
  XDMAC_HW_INTERFACE_UART4_TX   = 28, //!< Hardware Interface Number (XDMAC_CC.PERID) 28: Universal Asynchronous Receiver Transceiver 4 (USART4) in Transmit
  XDMAC_HW_INTERFACE_UART4_RX   = 29, //!< Hardware Interface Number (XDMAC_CC.PERID) 29: Universal Asynchronous Receiver Transceiver 4 (USART4) in Receive
  XDMAC_HW_INTERFACE_DACC0_TX   = 30, //!< Hardware Interface Number (XDMAC_CC.PERID) 30: Digital-to-Analog Converter Controller 0 (DACC0) in Transmit
  XDMAC_HW_INTERFACE_DACC1_TX   = 31, //!< Hardware Interface Number (XDMAC_CC.PERID) 31: Digital-to-Analog Converter Controller 1 (DACC1) in Transmit
  XDMAC_HW_INTERFACE_SSC_TX     = 32, //!< Hardware Interface Number (XDMAC_CC.PERID) 32: Synchronous Serial Controller in Transmit
  XDMAC_HW_INTERFACE_SSC_RX     = 33, //!< Hardware Interface Number (XDMAC_CC.PERID) 33: Synchronous Serial Controller in Receive
  XDMAC_HW_INTERFACE_PIOA_RX    = 34, //!< Hardware Interface Number (XDMAC_CC.PERID) 34: Parallel Input/Output Controller A (PIOA) in Receive
  XDMAC_HW_INTERFACE_AFEC0_RX   = 35, //!< Hardware Interface Number (XDMAC_CC.PERID) 35: Analog Front-End Controller 0 (AFEC0) in Receive
  XDMAC_HW_INTERFACE_AFEC1_RX   = 36, //!< Hardware Interface Number (XDMAC_CC.PERID) 36: Analog Front-End Controller 1 (AFEC0) in Receive
  XDMAC_HW_INTERFACE_AES_TX     = 37, //!< Hardware Interface Number (XDMAC_CC.PERID) 37: Advanced Encryption Standard in Transmit
  XDMAC_HW_INTERFACE_AES_RX     = 38, //!< Hardware Interface Number (XDMAC_CC.PERID) 38: Advanced Encryption Standard in Receive
  XDMAC_HW_INTERFACE_PWM1_TX    = 39, //!< Hardware Interface Number (XDMAC_CC.PERID) 39: Pulse Width Modulation Controller 1 (PWM1) in Transmit
  XDMAC_HW_INTERFACE_TC0_RX     = 40, //!< Hardware Interface Number (XDMAC_CC.PERID) 40: Time Counter 0 in Receive
  XDMAC_HW_INTERFACE_TC3_RX     = 41, //!< Hardware Interface Number (XDMAC_CC.PERID) 41: Time Counter 3 in Receive
  XDMAC_HW_INTERFACE_TC6_RX     = 42, //!< Hardware Interface Number (XDMAC_CC.PERID) 42: Time Counter 6 in Receive
  XDMAC_HW_INTERFACE_TC9_RX     = 43, //!< Hardware Interface Number (XDMAC_CC.PERID) 43: Time Counter 9 in Receive
  XDMAC_HW_INTERFACE_I2SC0_TXL  = 44, //!< Hardware Interface Number (XDMAC_CC.PERID) 44: Inter-IC Sound Controller 0 (I2CS0) in Transmit Left
  XDMAC_HW_INTERFACE_I2SC0_RXL  = 45, //!< Hardware Interface Number (XDMAC_CC.PERID) 45: Inter-IC Sound Controller 0 (I2CS0) in Receive Left
  XDMAC_HW_INTERFACE_I2SC1_TXL  = 46, //!< Hardware Interface Number (XDMAC_CC.PERID) 46: Inter-IC Sound Controller 1 (I2CS1) in Transmit Left
  XDMAC_HW_INTERFACE_I2SC1_RXL  = 47, //!< Hardware Interface Number (XDMAC_CC.PERID) 47: Inter-IC Sound Controller 1 (I2CS1) in Receive Left
  XDMAC_HW_INTERFACE_I2SC0_TXR  = 48, //!< Hardware Interface Number (XDMAC_CC.PERID) 48: Inter-IC Sound Controller 0 (I2CS0) in Transmit Right
  XDMAC_HW_INTERFACE_I2SC0_RXR  = 49, //!< Hardware Interface Number (XDMAC_CC.PERID) 49: Inter-IC Sound Controller 0 (I2CS0) in Receive Right
  XDMAC_HW_INTERFACE_I2SC1_TXR  = 50, //!< Hardware Interface Number (XDMAC_CC.PERID) 50: Inter-IC Sound Controller 1 (I2CS1) in Transmit Right
  XDMAC_HW_INTERFACE_I2SC1_RXR  = 51, //!< Hardware Interface Number (XDMAC_CC.PERID) 51: Inter-IC Sound Controller 1 (I2CS1) in Receive Right
  XDMAC_HW_INTERFACE_ID_COUNT,        // KEEP LAST! Count of Hardware Interface Numbers
} eXDMAC_PERID;

//-----------------------------------------------------------------------------

typedef enum
{
  XDMAC_TXRX = 0x0, //!< Transmit and Receive
  XDMAC_TX   = 0x1, //!< Transmit
  XDMAC_RX   = 0x2, //!< Receive
} eXdmacTxRx;

//! Peripheral to ID element structure
typedef struct
{
  const uintptr_t HWinterface;    //!< Hardware interface address
  const eXdmacTxRx Direction;     //!< Direction of the transfer (Tx or Rx)
  const eXDMAC_PERID InterfaceID; //!< Peripheral ID (PERID) of the peripheral for the XDMAC
} XDMAC_PeriphConvert;

//-----------------------------------------------------------------------------

/*! @brief Linked List Descriptor View 0 structure
 * Structure for storing parameters for DMA view0 that can be performed by the DMA Master transfer
 */
typedef struct
{
  uint32_t MBR_NDA; //!< Next Descriptor Address number
  uint32_t MBR_UBC; //!< Microblock Control Member
  uint32_t MBR_DA;  //!< Destination Address Member
} XDMAC_LinkedListDescView0;

/*! @brief Linked List Descriptor View 1 structure
 * Structure for storing parameters for DMA view1 that can be performed by the DMA Master transfer
 */
typedef struct
{
  uint32_t MBR_NDA; //!< Next Descriptor Address number
  uint32_t MBR_UBC; //!< Microblock Control Member
  uint32_t MBR_SA;  //!< Source Address Member
  uint32_t MBR_DA;  //!< Destination Address Member
} XDMAC_LinkedListDescView1;

/*! @brief Linked List Descriptor View 2 structure
 * Structure for storing parameters for DMA view2 that can be performed by the DMA Master transfer
 */
typedef struct
{
  uint32_t MBR_NDA; //!< Next Descriptor Address number
  uint32_t MBR_UBC; //!< Microblock Control Member
  uint32_t MBR_SA;  //!< Source Address Member
  uint32_t MBR_DA;  //!< Destination Address Member
  uint32_t MBR_CFG; //!< Configuration Register
} XDMAC_LinkedListDescView2;

/*! @brief Linked List Descriptor View 3 structure
 * Structure for storing parameters for DMA view3 that can be performed by the DMA Master transfer
 */
typedef struct
{
  uint32_t MBR_NDA; //!< Next Descriptor Address number
  uint32_t MBR_UBC; //!< Microblock Control Member
  uint32_t MBR_SA;  //!< Source Address Member
  uint32_t MBR_DA;  //!< Destination Address Member
  uint32_t MBR_CFG; //!< Configuration Register
  uint32_t MBR_BC;  //!< Block Control Member
  uint32_t MBR_DS;  //!< Data Stride Member
  uint32_t MBR_SUS; //!< Source Microblock Stride Member
  uint32_t MBR_DUS; //!< Destination Microblock Stride Member
} XDMAC_LinkedListDescView3;

#define XDMAC_UBC_UBLEN_Pos         0
#define XDMAC_UBC_UBLEN_Mask        (0xFFFFFFu << XDMAC_UBC_UBLEN_Pos)
#define XDMAC_UBC_UBLEN_SET(value)  (((uint32_t)(value) << XDMAC_UBC_UBLEN_Pos) & XDMAC_UBC_UBLEN_Mask) //!< Set Microblock Length
#define XDMAC_UBC_NDE               (0x1u << 24) //!< Next Descriptor Enable
#define XDMAC_UBC_NDE_FETCH_DIS     (0x0u << 24) //!< Next Descriptor fetch is disabled
#define XDMAC_UBC_NDE_FETCH_EN      (0x1u << 24) //!< Next Descriptor fetch is enabled
#define XDMAC_UBC_NSEN              (0x1u << 25) //!< Next Descriptor Source Update
#define XDMAC_UBC_NSEN_UNCHANGED    (0x0u << 25) //!< Next Descriptor Source parameters remain unchanged
#define XDMAC_UBC_NSEN_UPDATED      (0x1u << 25) //!< Next Descriptor Source parameters are updated when the descriptor is retrieved
#define XDMAC_UBC_NDEN              (0x1u << 26) //!< Next Descriptor Destination Update
#define XDMAC_UBC_NDEN_UNCHANGED    (0x0u << 26) //!< Next Descriptor Destination parameters remain unchanged
#define XDMAC_UBC_NDEN_UPDATED      (0x1u << 26) //!< Next Descriptor Destination parameters are updated when the descriptor is retrieved

//! Next Descriptor View enumerator
typedef enum
{
  XDMAC_NEXT_DESCRIPTOR_VIEW0 = 0b00, //!< Next Descriptor View 0
  XDMAC_NEXT_DESCRIPTOR_VIEW1 = 0b01, //!< Next Descriptor View 1
  XDMAC_NEXT_DESCRIPTOR_VIEW2 = 0b10, //!< Next Descriptor View 2
  XDMAC_NEXT_DESCRIPTOR_VIEW3 = 0b11, //!< Next Descriptor View 3
} eXDMAC_NextDescriptorView;

#define XDMAC_UBC_NVIEW_Pos         27
#define XDMAC_UBC_NVIEW_Mask        (0x3u << XDMAC_UBC_NVIEW_Pos)
#define XDMAC_UBC_NVIEW_SET(value)  (((uint32_t)(value) << XDMAC_UBC_NVIEW_Pos) & XDMAC_UBC_NVIEW_Mask) //!< Set Next Descriptor View

//-----------------------------------------------------------------------------

//! Channels Interrupt Events, can be OR'ed.
typedef enum
{
  XDMAC_INT_NO_EVENT                 = 0x00, //!< No interrupt events
  XDMAC_INT_END_OF_BLOCK_EVENT       = 0x01, //!< End of block interrupt
  XDMAC_INT_END_OF_LINKED_LIST_EVENT = 0x02, //!< End of Linked List Interrupt
  XDMAC_INT_END_OF_DISABLE_EVENT     = 0x04, //!< End of Disable Interrupt
  XDMAC_INT_END_OF_FLUSH_EVENT       = 0x08, //!< End of Flush Interrupt
  XDMAC_INT_READ_BUS_ERROR_EVENT     = 0x10, //!< Read Bus Error Interrupt
  XDMAC_INT_WRITE_BUS_ERROR_EVENT    = 0x20, //!< Write Bus Error Interrupt
  XDMAC_INT_REQUEST_OVERFLOW_EVENT   = 0x40, //!< Request Overflow Error Interrupt
  XDMAC_INT_ENABLE_ALL_EVENTS        = 0x7F, //!< Enable all events
} eXDMACchan_InterruptEvents;

typedef eXDMACchan_InterruptEvents setXDMACchan_InterruptEvents; //! Set of Interrupt Events (can be OR'ed)

//-----------------------------------------------------------------------------

typedef uintptr_t HandleXDMAC; //! DMA handle

//-----------------------------------------------------------------------------

#define XDMAC_INVALID_HANDLE      0
#define XDMAC_INVALID_PERIPHERAL  ( 0xFFFFFFFF ) //! Invalid peripheral value

/*! @brief Definition for DMA callback function for interrupt
 * @param[in] dmaChannel Is the handle of the DMA channel that generate this interrupt callback
 * @param[in] context Is a pointer to the context of the DMA callback
 * @param[in] interrupts Is the set of current interrupts events of this DMA channel
 * @return context Is a context pointer for the callback
 */
typedef void (*DMA_IntCallBack_Func)(HandleXDMAC dmaChannel, uintptr_t context, setXDMACchan_InterruptEvents interrupts);


//! XDMAC channel informations structure
typedef struct
{
  Xdmac* pXDMAC;                      //!< Pointer to the XDMA controller
  XdmacChid* pChannel;                //!< Pointer to the DMA channel registers
  bool InUse;                         //!< Indicate if the DMA channel in use or not
  DMA_IntCallBack_Func fnDMACallback; //!< DMA callback function for interrupt
  uintptr_t DMAInterruptContext;      //!< DMA context pointer for the interrupt callback function
} XDMACchannelHandle;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// XDMAC driver API
//********************************************************************************************************************

//! XDMA config register for channel
typedef struct
{
  //--- DMA configuration ---
  uint32_t  MBR_UBC; //!< Microblock Control Member
  uint32_t  MBR_SA;  //!< Source Address Member
  uint32_t  MBR_DA;  //!< Destination Address Member
  uint32_t  MBR_CFG; //!< Configuration Register
  uint32_t  MBR_BC;  //!< Block Control Member
  uint32_t  MBR_DS;  //!< Data Stride Member
  uint32_t  MBR_SUS; //!< Source Microblock Stride Member
  uint32_t  MBR_DUS; //!< Destination Microblock Stride Member
  //--- Next descriptor description ---
  uintptr_t MBR_NDA; //!< Next Descriptor Address
  uint32_t  MBR_NDC; //!< Next Descriptor Control
  uint8_t   NDAIF;   //!< Next Descriptor Interface
  //--- Interrupts ---
  setXDMACchan_InterruptEvents Interrupts; //!< Interrupts of the channel
} XDMAC_ChannelConfig;

//********************************************************************************************************************





/*! @brief Atmel XDMAC initialization
 * @param[in] *pXDMAC Is the XDMAC peripheral to initialize
 * @param[in] enableNVIC Set the NVIC status. 'true' to enable the NVIC, 'false to disable the NVIC
 * @param[in] nvicPriority Set the NVIC priority. Only 8 levels of priority
 */
eERRORRESULT XDMAC_Init(Xdmac* pXDMAC, bool enableNVIC, uint32_t nvicPriority);


/*! @brief Get the XDMAC peripheral identification
 * @param[in] *pXDMAC Is a XDMAC peripheral to use
 * @return The peripheral identification on a SAM V71
 */
uint32_t XDMAC_GetPeripheralID(Xdmac* pXDMAC);

//********************************************************************************************************************



/*! @brief Open a DMA channel
 * It also stores informations and configures the corresponding interruption
 * @param[in] *pXDMAC Is a XDMAC peripheral to use
 * @param[in] interruptCallback Is the corresponding interrupt source of the SPI
 * @param[in] dmaInterruptContext Is the function to call when a DMA IRQ occurs (can be NULL)
 * @return The opened DMA channel handle. If no channel available, the function return XDMAC_INVALID_HANDLE (0)
 */
HandleXDMAC XDMAC_OpenChannel(Xdmac* pXDMAC, DMA_IntCallBack_Func interruptCallback, uintptr_t dmaInterruptContext);


/*! @brief Close a DMA channel
 * @param[in] handleDMA Is the DMA channel handle to close
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_CloseChannel(HandleXDMAC handleDMA);

//********************************************************************************************************************



/*! @brief Configure DMA for a transfer
 * @param[in] handleDMA Is the DMA channel handle to configure
 * @param[in] *pConfig Is the DMA configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ConfigureTransfer(HandleXDMAC handleDMA, const XDMAC_ChannelConfig* pConfig);

//********************************************************************************************************************



/*! @brief Enable channel interrupts of the Atmel XDMAC
 * This function enables automatically the general channel interruption
 * If no source interrupt are enabled, the function will disable interrupts for the channel
 * @param[in] handleDMA Is the DMA channel handle to configure
 * @param[in] sourcesInterrupts Is the source interrupt to enable (can be OR'ed)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_InterruptEnable(HandleXDMAC handleDMA, setXDMACchan_InterruptEvents sourcesInterrupts);


/*! @brief Disable channel interrupts of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to configure
 * @param[in] sourcesInterrupts Is the source interrupt to disable (can be OR'ed)
 * @param[in] disableGeneralInterrupt Specify if the general interruption of the channel should be disabled
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_InterruptDisable(HandleXDMAC handleDMA, setXDMACchan_InterruptEvents sourcesInterrupts, bool disableGeneralInterrupt);


/*! @brief Get a channel interrupt status of the Atmel XDMAC
 *
 * @param[in] handleDMA Is the DMA channel handle to use
 * @param[out] *interruptsFlags Is the return value of interrupt events. Flags are OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_GetInterruptStatus(HandleXDMAC handleDMA, setXDMACchan_InterruptEvents* interruptsFlags);

//********************************************************************************************************************



/*! @brief Reset the Atmel XDMAC
 * @param[in] handleDMA Is the DMA handle to reset
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_Reset(Xdmac* pXDMAC);

//********************************************************************************************************************



/*! @brief Get XDMAC global type of the Atmel XDMAC
 * @param[in] *pXDMAC Is a XDMAC peripheral to use
 * @return The type register's value
 */
inline uint32_t XDMAC_GetType(Xdmac* pXDMAC)
{
  return pXDMAC->XDMAC_GTYPE;
}


/*! @brief Get XDMAC global configuration of the Atmel XDMAC
 * @param[in] *pXDMAC Is a XDMAC peripheral to use
 * @return The config register's value
 */
inline uint32_t XDMAC_GetConfig(Xdmac* pXDMAC)
{
  return pXDMAC->XDMAC_GCFG;
}


/*! @brief Get XDMAC global weighted arbiter configuration of the Atmel XDMAC
 * @param[in] *pXDMAC Is a XDMAC peripheral to use
 * @return The arbiter register's value
 */
inline uint32_t XDMAC_GetArbiter(Xdmac* pXDMAC)
{
  return pXDMAC->XDMAC_GWAC;
}

//********************************************************************************************************************



/*! @brief Enables the relevant channel of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelEnable(HandleXDMAC handleDMA);


/*! @brief Disables the relevant channel of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelDisable(HandleXDMAC handleDMA);


/*! @brief Suspend the relevant channel's read of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelReadSuspend(HandleXDMAC handleDMA);


/*! @brief Suspend the relevant channel's write of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelWriteSuspend(HandleXDMAC handleDMA);


/*! @brief Suspend the relevant channel's read & write of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelReadWriteSuspend(HandleXDMAC handleDMA);


/*! @brief Resume the relevant channel's read & write of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelReadWriteResume(HandleXDMAC handleDMA);


/*! @brief Set software transfer request on the relevant channel of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelSoftwareRequest(HandleXDMAC handleDMA);


/*! @brief Get software transfer status of the relevant channel of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @param[out] *requestIsPending indicate if a request is pending or not on the given handle
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_GetSoftwareRequestStatus(HandleXDMAC handleDMA, bool* requestIsPending);


/*! @brief Set software flush request on the relevant channel of the Atmel XDMAC
 * @param[in] handleDMA Is the DMA channel handle to use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT XDMAC_ChannelSoftwareFlushRequest(HandleXDMAC handleDMA);

//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* XDMAC_V71_H_INC */
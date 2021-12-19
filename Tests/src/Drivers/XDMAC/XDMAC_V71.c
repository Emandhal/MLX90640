/*******************************************************************************
    File name:    XDMAC_V71.c
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 18/04/2021
    Description:  XDMAC driver for Atmel MCUs
                  This interface implements a driver the the XDMAC of the SAM V71

    History :
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "XDMAC_V71.h"
//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifndef __cplusplus
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------

/*//! Conversion table to get the HardWare Interface ID
const XDMAC_PeriphConvert __XDMAC_PERID_Table[XDMAC_HW_INTERFACE_ID_COUNT] =
{
  { .HWinterface = (uintptr_t)HSMCI , .Direction = XDMAC_TXRX, .InterfaceID = XDMAC_HW_INTERFACE_HSMCI_TXRX, },
  { .HWinterface = (uintptr_t)SPI0  , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_SPI0_TX   , },
  { .HWinterface = (uintptr_t)SPI0  , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_SPI0_RX   , },
  { .HWinterface = (uintptr_t)SPI1  , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_SPI1_TX   , },
  { .HWinterface = (uintptr_t)SPI1  , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_SPI1_RX   , },
  { .HWinterface = (uintptr_t)QSPI  , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_QSPI_TX   , },
  { .HWinterface = (uintptr_t)QSPI  , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_QSPI_RX   , },
  { .HWinterface = (uintptr_t)USART0, .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_USART0_TX , },
  { .HWinterface = (uintptr_t)USART0, .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_USART0_RX , },
  { .HWinterface = (uintptr_t)USART1, .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_USART1_TX , },
  { .HWinterface = (uintptr_t)USART1, .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_USART1_RX , },
  { .HWinterface = (uintptr_t)USART2, .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_USART2_TX , },
  { .HWinterface = (uintptr_t)USART2, .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_USART2_RX , },
  { .HWinterface = (uintptr_t)PWM0  , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_PWM0_TX   , },
  { .HWinterface = (uintptr_t)TWIHS0, .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_TWIHS0_TX , },
  { .HWinterface = (uintptr_t)TWIHS0, .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TWIHS0_RX , },
  { .HWinterface = (uintptr_t)TWIHS1, .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_TWIHS1_TX , },
  { .HWinterface = (uintptr_t)TWIHS1, .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TWIHS1_RX , },
  { .HWinterface = (uintptr_t)TWIHS2, .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_TWIHS2_TX , },
  { .HWinterface = (uintptr_t)TWIHS2, .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TWIHS2_RX , },
  { .HWinterface = (uintptr_t)UART0 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_UART0_TX  , },
  { .HWinterface = (uintptr_t)UART0 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_UART0_RX  , },
  { .HWinterface = (uintptr_t)UART1 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_UART1_TX  , },
  { .HWinterface = (uintptr_t)UART1 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_UART1_RX  , },
  { .HWinterface = (uintptr_t)UART2 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_UART2_TX  , },
  { .HWinterface = (uintptr_t)UART2 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_UART2_RX  , },
  { .HWinterface = (uintptr_t)UART3 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_UART3_TX  , },
  { .HWinterface = (uintptr_t)UART3 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_UART3_RX  , },
  { .HWinterface = (uintptr_t)UART4 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_UART4_TX  , },
  { .HWinterface = (uintptr_t)UART4 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_UART4_RX  , },
  { .HWinterface = (uintptr_t)DACC0 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_DACC0_TX  , },
  { .HWinterface = (uintptr_t)DACC1 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_DACC1_TX  , },
  { .HWinterface = (uintptr_t)SSC   , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_SSC_TX    , },
  { .HWinterface = (uintptr_t)SSC   , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_SSC_RX    , },
  { .HWinterface = (uintptr_t)PIOA  , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_PIOA_RX   , },
  { .HWinterface = (uintptr_t)AFEC0 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_AFEC0_RX  , },
  { .HWinterface = (uintptr_t)AFEC1 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_AFEC1_RX  , },
  { .HWinterface = (uintptr_t)AES   , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_AES_TX    , },
  { .HWinterface = (uintptr_t)AES   , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_AES_RX    , },
  { .HWinterface = (uintptr_t)PWM1  , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_PWM1_TX   , },
  { .HWinterface = (uintptr_t)TC0   , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TC0_RX    , },
  { .HWinterface = (uintptr_t)TC3   , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TC3_RX    , },
  { .HWinterface = (uintptr_t)TC6   , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TC6_RX    , },
  { .HWinterface = (uintptr_t)TC9   , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_TC9_RX    , },
  { .HWinterface = (uintptr_t)I2SC0 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC0_TXL , },
  { .HWinterface = (uintptr_t)I2SC0 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC0_RXL , },
  { .HWinterface = (uintptr_t)I2SC1 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC1_TXL , },
  { .HWinterface = (uintptr_t)I2SC1 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC1_RXL , },
  { .HWinterface = (uintptr_t)I2SC0 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC0_TXR , },
  { .HWinterface = (uintptr_t)I2SC0 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC0_RXR , },
  { .HWinterface = (uintptr_t)I2SC1 , .Direction = XDMAC_TX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC1_TXR , },
  { .HWinterface = (uintptr_t)I2SC1 , .Direction = XDMAC_RX  , .InterfaceID = XDMAC_HW_INTERFACE_I2SC1_RXR , },
};*/

//-----------------------------------------------------------------------------

//! List of the DMA channels handles
XDMACchannelHandle ChannelHandles[XDMACCHID_NUMBER];

//------------------------------------------------------------------------------





//********************************************************************************************************************
// XDMAC driver
//********************************************************************************************************************
//=============================================================================
// Atmel XDMAC initialization
//=============================================================================
eERRORRESULT XDMAC_Init(Xdmac* pXDMAC, bool enableNVIC, uint32_t nvicPriority)
{
#ifdef CHECK_NULL_PARAM
  if (pXDMAC == NULL) return ERR__PARAMETER_ERROR;
#endif

  //--- Enable peripheral clock ---
  uint32_t PeriphID = XDMAC_GetPeripheralID(pXDMAC);
  if (PeriphID == XDMAC_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;
  sysclk_enable_peripheral_clock(PeriphID);

  //--- Enable NVIC IRQs ? ---
  NVIC_ClearPendingIRQ(PeriphID);
  NVIC_SetPriority(PeriphID, nvicPriority);
  if (enableNVIC)
       NVIC_EnableIRQ(PeriphID);
  else NVIC_DisableIRQ(PeriphID); // Disable NVIC IRQs

  //--- Reset XDMAC peripheral ---
  XDMAC_Reset(pXDMAC);            // Reset the XDMAC peripheral
  return ERR_OK;
}


//=============================================================================
// Get the XDMAC peripheral identification
//=============================================================================
uint32_t XDMAC_GetPeripheralID(Xdmac* pXDMAC)
{
  if (pXDMAC == XDMAC) return ID_XDMAC;
  return XDMAC_INVALID_PERIPHERAL;
}


//=============================================================================
// Get the XDMAC channel by handle
//=============================================================================
static size_t __XDMAC_GetChannelPerHandle(HandleXDMAC handleDMA)
{
  for (size_t z = 0; z < XDMACCHID_NUMBER; ++z)
  {
    if ((uintptr_t)&ChannelHandles[z] == handleDMA)
    {
      return z; // Get the channel index
    }
  }    
  return (size_t)-1;
}

//------------------------------------------------------------------------------





//==============================================================================
// Open a DMA channel
//==============================================================================
HandleXDMAC XDMAC_OpenChannel(Xdmac* pXDMAC, DMA_IntCallBack_Func interruptCallback, uintptr_t dmaInterruptContext)
{
  for (size_t z = 0; z < XDMACCHID_NUMBER; ++z)
    if (ChannelHandles[z].InUse == false)
    {
      ChannelHandles[z].pXDMAC              = pXDMAC;
      ChannelHandles[z].pChannel            = &pXDMAC->XDMAC_CHID[z];
      ChannelHandles[z].InUse               = true;
      ChannelHandles[z].fnDMACallback       = interruptCallback;
      ChannelHandles[z].DMAInterruptContext = dmaInterruptContext;
      return (HandleXDMAC)&ChannelHandles[z]; // Construct the handle
    }
  return XDMAC_INVALID_HANDLE;
}


//==============================================================================
// Close a DMA channel
//==============================================================================
eERRORRESULT XDMAC_CloseChannel(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;

  ChannelHandles[Channel].pXDMAC              = NULL;
  ChannelHandles[Channel].pChannel            = NULL;
  ChannelHandles[Channel].InUse               = false;
  ChannelHandles[Channel].fnDMACallback       = NULL;
  ChannelHandles[Channel].DMAInterruptContext = 0;
  return ERR_OK;
}





//**********************************************************************************************************************************************************
//==============================================================================
// Configure DMA for a transfer
//==============================================================================
eERRORRESULT XDMAC_ConfigureTransfer(HandleXDMAC handleDMA, const XDMAC_ChannelConfig* pConfig)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  eERRORRESULT Error;
  setXDMACchan_InterruptEvents InterruptsFlags;
  Error = XDMAC_GetInterruptStatus(handleDMA, &InterruptsFlags); // Clear Interrupts status, and do nothing with the flags
  if (Error != ERR_OK) return Error;

  //--- Configure the DMA channel ---
  ChannelHandles[Channel].pChannel->XDMAC_CSA  = pConfig->MBR_SA;                    // Source Address Register
  ChannelHandles[Channel].pChannel->XDMAC_CDA  = pConfig->MBR_DA;                    // Destination Address Register
  ChannelHandles[Channel].pChannel->XDMAC_CUBC = XDMAC_CUBC_UBLEN(pConfig->MBR_UBC); // Microblock Control Register
  ChannelHandles[Channel].pChannel->XDMAC_CBC  = XDMAC_CBC_BLEN(pConfig->MBR_BC);    // Block Control Register
  ChannelHandles[Channel].pChannel->XDMAC_CC   = pConfig->MBR_CFG;                   // Configuration Register
  ChannelHandles[Channel].pChannel->XDMAC_CDS_MSP = pConfig->MBR_DS;                 // Data Stride Memory Set Pattern Register
  ChannelHandles[Channel].pChannel->XDMAC_CSUS = XDMAC_CSUS_SUBS(pConfig->MBR_SUS);  // Source Microblock Stride Register
  ChannelHandles[Channel].pChannel->XDMAC_CDUS = XDMAC_CDUS_DUBS(pConfig->MBR_DUS);  // Destination Microblock Stride Register

  //--- Configure next descriptor ---
  if ((pConfig->MBR_NDA & XDMAC_CNDA_NDA_Msk) != pConfig->MBR_NDA) return ERR__ADDRESS_ALIGNMENT;   // NDA should be align4
  if (pConfig->NDAIF > 1) return ERR__CONFIGURATION;                                                // Only 2 interfaces
  ChannelHandles[Channel].pChannel->XDMAC_CNDA = XDMAC_CNDA_NDA(pConfig->MBR_NDA) | pConfig->NDAIF; // Next Descriptor Address Register
  ChannelHandles[Channel].pChannel->XDMAC_CNDC = pConfig->MBR_NDC;                                  // Next Descriptor Control Register

  //--- Set interrupts ---
  return XDMAC_InterruptEnable(handleDMA, pConfig->Interrupts);
}





//**********************************************************************************************************************************************************
//=============================================================================
// Enable channel interrupts of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_InterruptEnable(HandleXDMAC handleDMA, setXDMACchan_InterruptEvents sourcesInterrupts)
{
  if ((uint32_t)sourcesInterrupts == 0u) return XDMAC_InterruptDisable(handleDMA, sourcesInterrupts, false);
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;

  //--- Enable the specified interrupts ---
  ChannelHandles[Channel].pXDMAC->XDMAC_GIE   = (XDMAC_GIE_IE0 << Channel);
  ChannelHandles[Channel].pChannel->XDMAC_CIE = sourcesInterrupts;
  return ERR_OK;
}


//=============================================================================
// Disable channel interrupts of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_InterruptDisable(HandleXDMAC handleDMA, setXDMACchan_InterruptEvents sourcesInterrupts, bool disableGeneralInterrupt)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;

  //--- Disable the specified interrupts ---
  if (disableGeneralInterrupt)
  {
    ChannelHandles[Channel].pXDMAC->XDMAC_GID = (XDMAC_GID_ID0 << Channel);
  }
  ChannelHandles[Channel].pChannel->XDMAC_CID = sourcesInterrupts;
  setXDMACchan_InterruptEvents InterruptsFlags;
  return XDMAC_GetInterruptStatus(handleDMA, &InterruptsFlags); // Clear Interrupts status, and do nothing with the flags
}


//=============================================================================
// Get a channel interrupt status of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_GetInterruptStatus(HandleXDMAC handleDMA, setXDMACchan_InterruptEvents* interruptsFlags)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;

  //--- Get the interrupts flags ---
  *interruptsFlags = (setXDMACchan_InterruptEvents)ChannelHandles[Channel].pChannel->XDMAC_CIS;
  return ERR_OK;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Reset the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_Reset(Xdmac* pXDMAC)
{
  //--- Disable channels interrupts ---
  pXDMAC->XDMAC_GID = ~0ul;

  //--- Disable all channels ---
  pXDMAC->XDMAC_GD = ~0ul;

  //--- Initialize DMA channel table ---
  for (size_t z = 0; z < XDMACCHID_NUMBER; ++z)
  {
    ChannelHandles[z].pChannel            = &pXDMAC->XDMAC_CHID[z];
    ChannelHandles[z].InUse               = false;
    ChannelHandles[z].fnDMACallback       = NULL;
    ChannelHandles[z].DMAInterruptContext = 0;
  }
  return ERR_OK;
}





//**********************************************************************************************************************************************************
//=============================================================================
// XDMAC interrupt handler
//=============================================================================
void XDMAC_Handler(void)
{
  uint32_t GeneralInts = XDMAC->XDMAC_GIS;

  //--- Check all channels ---
  size_t Channel = 0;
  while (GeneralInts > 0)
  {
    if ((GeneralInts & 0x1) > 0)
    {
      uint32_t ChannelInt = XDMAC->XDMAC_CHID[Channel].XDMAC_CIS; // Get channel interrupts
      if (ChannelHandles[Channel].fnDMACallback != NULL)
      {
        ChannelHandles[Channel].fnDMACallback((HandleXDMAC)&ChannelHandles[Channel], ChannelHandles[Channel].DMAInterruptContext, (setXDMACchan_InterruptEvents)ChannelInt);
      }
    }
    GeneralInts >>= 1;
    ++Channel;
  }
}





//**********************************************************************************************************************************************************
//=============================================================================
// Enables the relevant channel of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelEnable(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;

  // All the data transfers, block or non-block (data transfer < 512 bytes), to and from SDIO module takes place through XDMAC and no CPU is involved. We must be cautious when CACHE is enabled.
  //--- Update DCache before DMA transmit ---
  //SCB_CleanInvalidateDCache();
  ChannelHandles[Channel].pXDMAC->XDMAC_GE = (XDMAC_GE_EN0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Disables the relevant channel of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelDisable(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GD = (XDMAC_GD_DI0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Suspend the relevant channel's read of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelReadSuspend(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GRS |= (XDMAC_GRS_RS0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Suspend the relevant channel's write of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelWriteSuspend(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GWS |= (XDMAC_GWS_WS0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Suspend the relevant channel's read & write of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelReadWriteSuspend(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GRWS = (XDMAC_GRWS_RWS0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Resume the relevant channel's read & write of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelReadWriteResume(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GRWR = (XDMAC_GRWR_RWR0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Set software transfer request on the relevant channel of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelSoftwareRequest(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GSWR = (XDMAC_GSWR_SWREQ0 << Channel);
  return ERR_OK;
}



//=============================================================================
// Get software transfer status of the relevant channel of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_GetSoftwareRequestStatus(HandleXDMAC handleDMA, bool* requestIsPending)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  *requestIsPending = ((ChannelHandles[Channel].pXDMAC->XDMAC_GSWS & (XDMAC_GSWS_SWRS0 << Channel)) > 0);
  return ERR_OK;
}



//=============================================================================
// Set software flush request on the relevant channel of the Atmel XDMAC
//=============================================================================
eERRORRESULT XDMAC_ChannelSoftwareFlushRequest(HandleXDMAC handleDMA)
{
  size_t Channel = __XDMAC_GetChannelPerHandle(handleDMA);
  if (Channel == (size_t)-1) return ERR__INVALID_HANDLE;
  ChannelHandles[Channel].pXDMAC->XDMAC_GSWF = (XDMAC_GSWF_SWF0 << Channel);
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
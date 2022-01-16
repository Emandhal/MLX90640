/*******************************************************************************
    File name:    TWIHS_V71.c
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 18/04/2021
    Description:  TWIHS driver for Atmel MCUs
                  This interface implements a synchronous use of the I2C and
                  an asynchronous use of I2C by using a DMA

    History :
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "TWIHS_V71.h"
#include "XDMAC_V71.h"
//-----------------------------------------------------------------------------
#ifndef __cplusplus
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------

static HandleXDMAC __HasReservedDMAchannel[TWIHS_COUNT] =
{
  XDMAC_INVALID_HANDLE,
#if (TWIHS_COUNT > 2)
  XDMAC_INVALID_HANDLE,
  XDMAC_INVALID_HANDLE,
#endif
#if (TWIHS_COUNT > 3)
  XDMAC_INVALID_HANDLE,
  XDMAC_INVALID_HANDLE,
  XDMAC_INVALID_HANDLE,
  XDMAC_INVALID_HANDLE,
  XDMAC_INVALID_HANDLE,
#endif
};

static volatile TWIHS_TransferStruct __TWIHStransferList[TWIHS_COUNT] =
{
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI0/TWIHS0
#if (TWIHS_COUNT > 2)
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI1/TWIHS1
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI2/TWIHS2
#endif
#if (TWIHS_COUNT > 3)
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI3
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI4
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI5
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI6
  { .Data = NULL, .RemainingBytes = 0, .Status = TWIHS_STATUS_UNINITIALIZED, .Error = ERR_OK, .TransactionCounter = 0, }, // TWI7
#endif
};

COMPILER_WORD_ALIGNED static XDMAC_ChannelConfig __XDMAC_I2Cconfig; // XDMAC I2C channel configuration

//-----------------------------------------------------------------------------


#define TWI_IRQ_level 0


//=============================================================================
// Atmel TWIHS master initialization
//=============================================================================
eERRORRESULT TWIHS_MasterInit(Twihs* pTWIHS, const uint32_t sclFreq)
{
#ifdef CHECK_NULL_PARAM
  if (pTWIHS == NULL) return ERR__PARAMETER_ERROR;
#endif

  //--- Enable peripheral clock ---
  uint32_t PeriphID = TWIHS_GetPeripheralID(pTWIHS);
  if (PeriphID == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;
  sysclk_enable_peripheral_clock(PeriphID);

  //--- Disable TWIHS interrupts ---
  TWIHS_InterruptDisable(pTWIHS, ~0ul, false);

  //--- Reset TWIHS peripheral ---
  pTWIHS->TWIHS_CR = TWIHS_CR_MSDIS | TWIHS_CR_SVDIS | TWIHS_CR_STOP; // Disable master and slave mode
  TWIHS_Reset(pTWIHS);                         // Reset the TWIHS peripheral

  //--- Enable TWIHS peripheral ---
  pTWIHS->TWIHS_CR = TWIHS_CR_MSEN;

  //--- Set I2C SCL clock frequency ---
  return TWIHS_SetI2CclockHz(pTWIHS, sclFreq); // Set the SCL frequency
}

eERRORRESULT TWIHS_MasterInit_Gen(I2C_Interface* pIntDev, const uint32_t sclFreq)
{
  Twihs* pTWIHS = (Twihs*)pIntDev->InterfaceDevice;
  return TWIHS_MasterInit(pTWIHS, sclFreq);
}



//=============================================================================
// Get peripheral ID of the Atmel TWIHS
//=============================================================================
uint32_t TWIHS_GetPeripheralID(Twihs* pTWIHS)
{
#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
  if (pTWIHS == TWIHS0) {
    return ID_TWIHS0;
  } else if (pTWIHS == TWIHS1) {
    return ID_TWIHS1;
  } else if (pTWIHS == TWIHS2) {
    return ID_TWIHS2;
  } else {
    return TWIHS_INVALID_PERIPHERAL;
  }
#else
  if (pTWIHS == TWI0) {
    return ID_TWI0;
# if SAMG55
  } else if (pTWIHS == TWI1) {
    return ID_TWI1;
  } else if (pTWIHS == TWI2) {
    return ID_TWI2;
  } else if (pTWIHS == TWI3) {
    return ID_TWI3;
  } else if (pTWIHS == TWI4) {
    return ID_TWI4;
  } else if (pTWIHS == TWI5) {
    return ID_TWI5;
  } else if (pTWIHS == TWI6) {
    return ID_TWI6;
  } else if (pTWIHS == TWI7) {
    return ID_TWI7;
# endif
  } else {
    return TWIHS_INVALID_PERIPHERAL;
  }
#endif
  return TWIHS_INVALID_PERIPHERAL;
}



//=============================================================================
// Get peripheral number of the Atmel TWIHS
//=============================================================================
uint32_t TWIHS_GetPeripheralNumber(Twihs* pTWIHS)
{
#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
  if (pTWIHS == TWIHS0) {
    return 0;
  } else if (pTWIHS == TWIHS1) {
    return 1;
  } else if (pTWIHS == TWIHS2) {
    return 2;
  } else {
    return TWIHS_INVALID_PERIPHERAL;
  }
#else
  if (pTWIHS == TWI0) {
    return 0;
# if SAMG55
  } else if (pTWIHS == TWI1) {
    return 1;
  } else if (pTWIHS == TWI2) {
    return 2;
  } else if (pTWIHS == TWI3) {
    return 3;
  } else if (pTWIHS == TWI4) {
    return 4;
  } else if (pTWIHS == TWI5) {
    return 5;
  } else if (pTWIHS == TWI6) {
    return 6;
  } else if (pTWIHS == TWI7) {
    return 7;
# endif
  } else {
    return TWIHS_INVALID_PERIPHERAL;
  }
#endif
  return TWIHS_INVALID_PERIPHERAL;
}





//********************************************************************************************************************
//=============================================================================
// Enable interrupts of the Atmel TWIHS
//=============================================================================
eERRORRESULT TWIHS_InterruptEnable(Twihs* pTWIHS, uint32_t sourcesInterrupts, bool enableNVIC)
{
  //--- Enable NVIC IRQs ? ---
  if (enableNVIC)
  {
    uint32_t PeriphID = TWIHS_GetPeripheralID(pTWIHS);
    if (PeriphID == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;
    NVIC_EnableIRQ(PeriphID);
  }
  //--- Enable the specified interrupts ---
  pTWIHS->TWIHS_IER = sourcesInterrupts;
  return ERR_OK;
}


//=============================================================================
// Disable interrupts of the Atmel TWIHS
//=============================================================================
eERRORRESULT TWIHS_InterruptDisable(Twihs* pTWIHS, uint32_t sourcesInterrupts, bool disableNVIC)
{
  //--- Disable also NVIC IRQs ? ---
  if (disableNVIC)
  {
    uint32_t PeriphID = TWIHS_GetPeripheralID(pTWIHS);
    if (PeriphID == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;
    NVIC_DisableIRQ(PeriphID);
  }
  //--- Disable the specified interrupts ---
  pTWIHS->TWIHS_IDR = sourcesInterrupts;
  (void)TWIHS_GetInterruptStatus(pTWIHS); // Clear Interrupts status
  return ERR_OK;
}





//********************************************************************************************************************
//=============================================================================
// Reset the Atmel TWIHS
//=============================================================================
void TWIHS_Reset(Twihs* pTWIHS)
{
  pTWIHS->TWIHS_CR = TWIHS_CR_SWRST; // Set SWRST bit to reset TWIHS peripheral
  (void)(pTWIHS->TWIHS_RHR);         // Read dummy data in receive register (RHR)
}



//=============================================================================
// Recover an I2C bus of the Atmel TWIHS
//=============================================================================
eERRORRESULT TWIHS_BusRecovery(Twihs* pTWIHS)
{
  uint32_t PeriphID = TWIHS_GetPeripheralID(pTWIHS);
  if (PeriphID == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;

  switch (PeriphID)
  {
#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
    case ID_TWIHS0:
      if (SDA_I2C0_Status == 0) // A device is stuck in communication and wait for a missing clocks
      {
        SDA_I2C0_PIO_En;
        SDA_I2C0_High;   // SDA = 1
        SDA_I2C0_In;     // SDA in
        SCL_I2C0_PIO_En;
        SCL_I2C0_High;   // SCL = 1
        SCL_I2C0_Out;    // SCL out
        size_t z = 9;                 // Clock up to 9 cycles. Here we force I2C SCL to clock until a device stuck in communication respond
        while (SDA_I2C0_Status == 0) // Look for SDA high in each cycle while SCL is high and then break
        {
          delay_us(5);
          SCL_I2C0_Low;  // SCL = 0
          delay_us(5);
          SCL_I2C0_High; // SCL = 1
          if (--z == 0) break;
        }
        ioport_set_pin_mode(SDA_I2C0_GPIO, SDA_I2C0_FLAGS); // Restore SDA pin function
        ioport_disable_pin(SDA_I2C0_GPIO);                  // Restore SDA pin function
        ioport_set_pin_mode(SCL_I2C0_GPIO, SCL_I2C0_FLAGS); // Restore SCL pin function
        ioport_disable_pin(SCL_I2C0_GPIO);                  // Restore SCL pin function
      }        
      break;

    case ID_TWIHS1:
      if (SDA_I2C1_Status == 0) // A device is stuck in communication and wait for a missing clocks
      {
        SDA_I2C1_PIO_En;
        SDA_I2C1_High;   // SDA = 1
        SDA_I2C1_In;     // SDA in
        SCL_I2C1_PIO_En;
        SCL_I2C1_High;   // SCL = 1
        SCL_I2C1_Out;    // SCL out
        size_t z = 9;                 // Clock up to 9 cycles. Here we force I2C SCL to clock until a device stuck in communication respond
        while (SDA_I2C1_Status == 0) // Look for SDA high in each cycle while SCL is high and then break
        {
          delay_us(5);
          SCL_I2C1_Low;  // SCL = 0
          delay_us(5);
          SCL_I2C1_High; // SCL = 1
          if (--z == 0) break;
        }
        ioport_set_pin_mode(SDA_I2C1_GPIO, SDA_I2C1_FLAGS); // Restore SDA pin function
        ioport_disable_pin(SDA_I2C1_GPIO);                  // Restore SDA pin function
        ioport_set_pin_mode(SCL_I2C1_GPIO, SCL_I2C1_FLAGS); // Restore SCL pin function
        ioport_disable_pin(SCL_I2C1_GPIO);                  // Restore SCL pin function
      }
      break;

    case ID_TWIHS2:
      if (SDA_I2C2_Status == 0) // A device is stuck in communication and wait for a missing clocks
      {
        SDA_I2C2_PIO_En;
        SDA_I2C2_High;   // SDA = 1
        SDA_I2C2_In;     // SDA in
        SCL_I2C2_PIO_En;
        SCL_I2C2_High;   // SCL = 1
        SCL_I2C2_Out;    // SCL out
        size_t z = 9;                 // Clock up to 9 cycles. Here we force I2C SCL to clock until a device stuck in communication respond
        while (SDA_I2C2_Status == 0) // Look for SDA high in each cycle while SCL is high and then break
        {
          delay_us(5);
          SCL_I2C2_Low;  // SCL = 0
          delay_us(5);
          SCL_I2C2_High; // SCL = 1
          if (--z == 0) break;
        }
        ioport_set_pin_mode(SDA_I2C2_GPIO, SDA_I2C2_FLAGS); // Restore SDA pin function
        ioport_disable_pin(SDA_I2C2_GPIO);                  // Restore SDA pin function
        ioport_set_pin_mode(SCL_I2C2_GPIO, SCL_I2C2_FLAGS); // Restore SCL pin function
        ioport_disable_pin(SCL_I2C2_GPIO);                  // Restore SCL pin function
      }
      break;
#endif

    default: break;
  }
  return ERR_OK;
}





//********************************************************************************************************************
//=============================================================================
// Set the I2C SCL clock in Hertz of the Atmel TWIHS
//=============================================================================
eERRORRESULT TWIHS_SetI2CclockHz(Twihs* pTWIHS, uint32_t desiredClockHz)
{
  uint32_t CLHDIV;
  uint32_t CLDIV, CHDIV, CKDIV = 0;
  
  //--- Check parameters ---
  if (desiredClockHz > TWIHS_MASTER_MODE_CLOCK_MAX) return ERR__I2C_FREQUENCY_ERROR; // I2C SCL frequency in master mode is 400kHz
  uint32_t PeripheralClock = sysclk_get_peripheral_hz();

  //--- Calculate Clock Waveform dividers ---
  if (desiredClockHz > TWIHS_I2CCLOCK_FM_MIN)                            // In Fast-Mode ?
  {                                                                      // Low level time not less than 1.3us of I2C Fast Mode
    CLDIV = PeripheralClock / (TWIHS_I2CCLOCK_FM_MIN * TWIHS_CLKDIV_EXPONENT) - TWIHS_CLKDIV_ARGUMENT;
    CHDIV = PeripheralClock / ((desiredClockHz + (desiredClockHz - TWIHS_I2CCLOCK_FM_MIN)) * TWIHS_CLKDIV_EXPONENT) - TWIHS_CLKDIV_ARGUMENT;
    while ((CLDIV > TWIHS_CL_CK_DIV_MAX) && (CKDIV < TWIHS_CK_DIV_MAX))  // CLDIV must fit in 8 bits, CKDIV must fit in 3 bits
    {
      CKDIV++;                        // Increase clock divider
      CLDIV /= TWIHS_CLKDIV_EXPONENT; // Divide CLDIV value
    }
    while ((CHDIV > TWIHS_CL_CK_DIV_MAX) && (CKDIV < TWIHS_CK_DIV_MAX))  // CHDIV must fit in 8 bits, CKDIV must fit in 3 bits
    {
      CKDIV++;                        // Increase clock divider
      CHDIV /= TWIHS_CLKDIV_EXPONENT; // Divide CHDIV value
    }
    // Set clock waveform generator register
    pTWIHS->TWIHS_CWGR = TWIHS_CWGR_CLDIV(CLDIV) | TWIHS_CWGR_CHDIV(CHDIV) | TWIHS_CWGR_CKDIV(CKDIV);
  }
  else                                                                   // Else set identical CLDIV and CHDIV
  {
    CLHDIV = PeripheralClock / (desiredClockHz * TWIHS_CLKDIV_EXPONENT) - TWIHS_CLKDIV_ARGUMENT;
    while ((CLHDIV > TWIHS_CL_CK_DIV_MAX) && (CKDIV < TWIHS_CK_DIV_MAX)) // CLDIV must fit in 8 bits, CKDIV must fit in 3 bits
    {
      CKDIV++;                     // Increase clock divider
      CLHDIV /= TWIHS_CLK_DIVIDER; // Divide CLDIV and CHDIV value
    }
    // Set clock waveform generator register
    pTWIHS->TWIHS_CWGR = TWIHS_CWGR_CLDIV(CLHDIV) | TWIHS_CWGR_CHDIV(CLHDIV) | TWIHS_CWGR_CKDIV(CKDIV);
  }
  return ERR_OK;
}





//********************************************************************************************************************
//=============================================================================
// Hardware I2C data transfer communication for the Atmel TWIHS
//=============================================================================
eERRORRESULT TWIHS_Transfer(Twihs* pTWIHS, I2CInterface_Packet* const pPacketDesc)
{
  if (pTWIHS == NULL) return ERR__I2C_PARAMETER_ERROR;
  uint32_t PeriphNumber = TWIHS_GetPeripheralNumber(pTWIHS);

  //--- Check the state of the transfer ---
  switch (__TWIHStransferList[PeriphNumber].Status)
  {
    case TWIHS_STATUS_COMPLETE:      //** TWIHS transfer complete
    case TWIHS_STATUS_FAULT:         //** TWIHS transfer in fault
    case TWIHS_STATUS_IN_PROGRESS:   //** TWIHS transfer in progress
      return ERR__I2C_OTHER_BUSY;    // These status are related to another transfer

    case TWIHS_STATUS_UNINITIALIZED: //** TWIHS uninitialized
    case TWIHS_STATUS_READY:         //** TWIHS transfer is ready
    default:                         // Configure the transfer with the following lines
      break;
  }

  const bool DeviceWrite = ((pPacketDesc->ChipAddr & 0x01) == 0);
  size_t RemainingBytes = pPacketDesc->BufferSize;
  uint32_t Status;
  uint32_t Timeout = TWIHS_TIMEOUT;
  pTWIHS->TWIHS_MMR  = 0;
  pTWIHS->TWIHS_MMR  = TWIHS_MMR_DADR(pPacketDesc->ChipAddr >> 1) | (DeviceWrite ? 0 : TWIHS_MMR_MREAD);
  pTWIHS->TWIHS_IADR = 0; // Not used
  pTWIHS->TWIHS_CR   = TWIHS_CR_MSEN | TWIHS_CR_SVDIS;

  //--- Device polling ? ---
  if ((pPacketDesc->pBuffer == NULL) || (pPacketDesc->BufferSize <= 0)) // Device polling only
  { // Little hack because TWI of V71 does not support device polling without using SMBus
    pTWIHS->TWIHS_MMR &= ~TWIHS_MMR_MREAD;                     // The SMBus of this device does not support quick read command (no Stop will be sent)
    pTWIHS->TWIHS_CR |= TWIHS_CR_SMBEN + TWIHS_CR_PECDIS;      // Enable SMBus
    pTWIHS->TWIHS_CR |= TWIHS_CR_STOP;                         // Send a stop
    pTWIHS->TWIHS_CR |= TWIHS_CR_QUICK;                        // Start the polling with a quick command

    Timeout = TWIHS_TIMEOUT;
    while (true)                                               // Wait the polling to finish
    {
      Status = pTWIHS->TWIHS_SR;
      if ((Status & TWIHS_SR_NACK) > 0) return ERR__I2C_NACK;
      if (!Timeout--) return ERR__I2C_TIMEOUT;                 // Timeout ? return an error
      if ((Status & TWIHS_SR_TXCOMP) > 0) break;
    }
    return ERR_OK;
  }

  //--- Endianness configuration for data striding ---
  const eI2C_EndianTransform EndianTransform = I2C_ENDIAN_TRANSFORM_GET(pPacketDesc->Config.Value); // Only the endianness configuration is important for this driver
  const size_t BlockSize = (EndianTransform == I2C_NO_ENDIAN_CHANGE ? 1 : (size_t)EndianTransform); // Get block size. No endian change = 8-bits data
  if ((pPacketDesc->BufferSize % BlockSize) > 0) return ERR__DATA_MODULO;                           // Data block size shall be a multiple of data size
  size_t CurrentBlockPos = BlockSize;

  //--- Transfer data ---
  uint8_t* pBuffer = &pPacketDesc->pBuffer[BlockSize - 1];     // Adjust the start of data for endianness
  if (pPacketDesc->Start) pTWIHS->TWIHS_CR |= TWIHS_CR_START;  // Send a start if asked
  if (DeviceWrite) // Device write
  {
    while (true)
    {
      Status = pTWIHS->TWIHS_SR;
      if ((Status & TWIHS_SR_NACK) > 0) return ERR__I2C_NACK_DATA;
      if (!Timeout--) return ERR__I2C_TIMEOUT;                 // Timeout ? return an error
      if ((Status & TWIHS_SR_TXRDY) == 0) continue;
      Timeout = TWIHS_TIMEOUT;

      if (RemainingBytes == 0) break;                          // No data remaining to send, then break the loop
      pTWIHS->TWIHS_THR = *pBuffer;                            // Send next data byte
      RemainingBytes--;
      //--- Adjust buffer address with data striding ---
      --CurrentBlockPos;
      if (CurrentBlockPos == 0)
      {
        pBuffer += (2 * BlockSize) - 1;
        CurrentBlockPos = BlockSize;
      }
      else --pBuffer;
    }
    if (pPacketDesc->Stop) pTWIHS->TWIHS_CR |= TWIHS_CR_STOP;  // Send a stop if asked
  }
  else // Device read
  {
    while (RemainingBytes > 0)
    {
      if ((RemainingBytes == 1) && pPacketDesc->Stop)
        pTWIHS->TWIHS_CR |= TWIHS_CR_STOP;                     // Last byte ? Send a stop if asked

      Timeout = TWIHS_TIMEOUT;
      while (true)                                             // Wait the polling to finish
      {
        Status = pTWIHS->TWIHS_SR;
        if (!Timeout--) return ERR__I2C_TIMEOUT;               // Timeout ? return an error
        if ((Status & TWIHS_SR_RXRDY) > 0) break;
      }

      *pBuffer = pTWIHS->TWIHS_RHR;                            // Get next data byte
      RemainingBytes--;
      //--- Adjust buffer address with data striding ---
      --CurrentBlockPos;
      if (CurrentBlockPos == 0)
      {
        pBuffer += (2 * BlockSize) - 1;
        CurrentBlockPos = BlockSize;
      }
      else --pBuffer;
    }
  }
  if (pPacketDesc->Stop) while ((pTWIHS->TWIHS_SR & TWIHS_SR_TXCOMP) == 0); // Wait until both holding register and internal shifter are empty and STOP condition has been sent

  //--- Endianness result ---
  pPacketDesc->Config.Value &= ~I2C_ENDIAN_RESULT_Mask;
  pPacketDesc->Config.Value |= I2C_ENDIAN_RESULT_SET(EndianTransform); // Indicate that the endian transform have been processed
  return ERR_OK;
}

eERRORRESULT TWIHS_Transfer_Gen(I2C_Interface *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop)
{
  if (pIntDev == NULL) return ERR__I2C_PARAMETER_ERROR;
  Twihs* pTWIHS = (Twihs*)(pIntDev->InterfaceDevice);
  I2CInterface_Packet PacketDesc =
  {
    I2C_MEMBER(Config.Value) I2C_NO_POLLING | I2C_ENDIAN_TRANSFORM_SET(I2C_NO_ENDIAN_CHANGE) | I2C_TRANSFER_TYPE_SET(I2C_SIMPLE_TRANSFER),
    I2C_MEMBER(ChipAddr    ) deviceAddress,
    I2C_MEMBER(pBuffer     ) data,
    I2C_MEMBER(BufferSize  ) byteCount,
    I2C_MEMBER(Start       ) start,
    I2C_MEMBER(Stop        ) stop,
  };
  return TWIHS_Transfer(pTWIHS, &PacketDesc);
}





//********************************************************************************************************************
// TWIHS with DMA driver API
//********************************************************************************************************************
//=============================================================================
// TWIHS DMA interrupt handler
//=============================================================================
static void TWIHS_DMA_Handler(HandleXDMAC dmaChannel, uintptr_t context, setXDMACchan_InterruptEvents interrupts)
{
  Twihs* pTWIHS = (Twihs*)context;
  uint32_t PeriphNumber = TWIHS_GetPeripheralNumber(pTWIHS);

  //--- Get DMA Status ---
  if ((interrupts & XDMAC_CIS_BIS) > 0) // End of Block Interrupt Status Bit
  {
    //--- Disable DMA ---
    (void)XDMAC_InterruptDisable(dmaChannel, XDMAC_CIE_BIE, true); // Disable general DMA channel interrupt and more specifically the End of Block Interrupt
    (void)XDMAC_ChannelDisable(dmaChannel);

    //--- Prepare TWIHS manual finish ---
    if (__TWIHStransferList[PeriphNumber].DeviceWrite)
    {
      __TWIHStransferList[PeriphNumber].Data           = &__TWIHStransferList[PeriphNumber].Data[__TWIHStransferList[PeriphNumber].RemainingBytes - 1];
      __TWIHStransferList[PeriphNumber].RemainingBytes = 1; // I2C write with DMA needs the last byte to be processed manually
    }
    else
    {
      __TWIHStransferList[PeriphNumber].Data           = &__TWIHStransferList[PeriphNumber].Data[__TWIHStransferList[PeriphNumber].RemainingBytes - 2];
      __TWIHStransferList[PeriphNumber].RemainingBytes = 2; // I2C read with DMA needs the last 2-bytes to be processed manually
    }
    pTWIHS->TWIHS_IER = TWIHS_IER_RXRDY; // Enable Receive Holding Register Ready Interrupt Enable
    return;
  }

  //--- Generate fault status ---
  __TWIHStransferList[PeriphNumber].Status = TWIHS_STATUS_FAULT;
  __TWIHStransferList[PeriphNumber].Error  = ERR__DMA_ERROR;     // Generic DMA error
  if ((interrupts & XDMAC_CIE_ROIE) > 0) __TWIHStransferList[PeriphNumber].Error = ERR__DMA_OVERFLOW_ERROR;  // Request Overflow Error Interrupt Enable Bit
  if ((interrupts & XDMAC_CIE_WBIE) > 0) __TWIHStransferList[PeriphNumber].Error = ERR__DMA_WRITE_BUS_ERROR; // Write Bus Error Interrupt Enable Bit
  if ((interrupts & XDMAC_CIE_RBIE) > 0) __TWIHStransferList[PeriphNumber].Error = ERR__DMA_READ_BUS_ERROR;  // Read Bus Error Interrupt Enable Bit
}



//=============================================================================
// TWIHSx interrupts handlers
//=============================================================================
static void __TWIHS_Handler(Twihs* pTWIHS)
{
  //--- Get TWI status ---
  uint32_t PeriphNumber = TWIHS_GetPeripheralNumber(pTWIHS);
  uint32_t Status = pTWIHS->TWIHS_SR & pTWIHS->TWIHS_IMR;

  //--- Checks status ---
  if ((Status & TWIHS_FAULT_STATUS) > 0)
  {
#ifdef ASYNC_I2C_BY_INTERRUPT
    (void)XDMAC_InterruptDisable(__HasReservedDMAchannel[PeriphNumber], ~0ul, true); // Disable general DMA channel interrupt
    (void)XDMAC_ChannelDisable(__HasReservedDMAchannel[PeriphNumber]);
#endif
    __TWIHStransferList[PeriphNumber].Status = TWIHS_STATUS_FAULT;
    if ((Status & TWIHS_SR_NACK  ) > 0) __TWIHStransferList[PeriphNumber].Error  = ERR__I2C_NACK;
    if ((Status & TWIHS_SR_ARBLST) > 0) __TWIHStransferList[PeriphNumber].Error  = ERR__I2C_DEVICE_NOT_READY;
    if ((Status & TWIHS_SR_OVRE  ) > 0) __TWIHStransferList[PeriphNumber].Error  = ERR__I2C_OVERFLOW_ERROR;
    if ((Status & TWIHS_SR_UNRE  ) > 0) __TWIHStransferList[PeriphNumber].Error  = ERR__I2C_UNDERFLOW_ERROR;
    pTWIHS->TWIHS_IDR = ~0ul; // Clear all interrupts
    return;
  }

  size_t Size = __TWIHStransferList[PeriphNumber].Size;
  volatile size_t* RemainingBytes = &__TWIHStransferList[PeriphNumber].RemainingBytes;
  //--- Data Receive status ---
  if ((Status & TWIHS_SR_RXRDY) > 0)
  {
    __TWIHStransferList[PeriphNumber].Data[Size - *RemainingBytes] = pTWIHS->TWIHS_RHR; // Get next data byte
    *RemainingBytes -= 1;
    if (*RemainingBytes == 1) pTWIHS->TWIHS_CR |= TWIHS_CR_STOP; // Last byte? Send a stop
    if (*RemainingBytes == 0)
    {
      pTWIHS->TWIHS_IDR = TWIHS_IDR_RXRDY;  // Disable Rx ready flag
      pTWIHS->TWIHS_IER = TWIHS_IER_TXCOMP; // Enable Tx Complete flag
    }
    return;
  }

  //--- Data Transmit status ---
  if ((Status & TWIHS_SR_TXRDY) > 0)
  {
    pTWIHS->TWIHS_THR = __TWIHStransferList[PeriphNumber].Data[Size - *RemainingBytes]; // Send next data byte
    *RemainingBytes -= 1;
    if (*RemainingBytes == 0)
    {
      pTWIHS->TWIHS_IDR = TWIHS_IDR_TXRDY;  // Disable Tx ready flag
      pTWIHS->TWIHS_IER = TWIHS_IER_TXCOMP; // Enable Tx Complete flag
    }
    return;
  }

  //--- Transfer complete ---
  if ((Status & TWIHS_SR_TXCOMP) > 0)
  {
    __TWIHStransferList[PeriphNumber].Status = TWIHS_STATUS_COMPLETE;
    __TWIHStransferList[PeriphNumber].Error  = ERR_OK;
    pTWIHS->TWIHS_IDR = ~0ul;
  }
}
void TWIHS0_Handler(void) { __TWIHS_Handler(TWIHS0); }
void TWIHS1_Handler(void) { __TWIHS_Handler(TWIHS1); }
void TWIHS2_Handler(void) { __TWIHS_Handler(TWIHS2); }





//=============================================================================
// Hardware I2C data transfer with DMA communication for the Atmel TWIHS
//=============================================================================
static eERRORRESULT __TWIHS_DMA_Transfer(Twihs *pTWIHS, I2CInterface_Packet* const pPacketDesc)
{
  eERRORRESULT Error;
  uint32_t PeriphNumber = TWIHS_GetPeripheralNumber(pTWIHS);
  if (PeriphNumber == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;
  uint32_t PeriphID = TWIHS_GetPeripheralID(pTWIHS);
  if (PeriphID == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;

  //--- Check the state of the transfer ---
  switch (__TWIHStransferList[PeriphNumber].Status)
  {
    case TWIHS_STATUS_UNINITIALIZED: return ERR__I2C_CONFIG_ERROR;   //** TWIHS uninitialized
    case TWIHS_STATUS_IN_PROGRESS:   return ERR__I2C_BUSY;           //** TWIHS transfer in progress

    case TWIHS_STATUS_COMPLETE:                                      //** TWIHS transfer complete
#ifdef CONF_BOARD_ENABLE_CACHE
      SCB_InvalidateDCache_by_Addr((uint32_t*)__TWIHStransferList[PeriphNumber].Data, __TWIHStransferList[PeriphNumber].Size);
#endif
      pPacketDesc->Config.Value = __TWIHStransferList[PeriphNumber].Config.Value;
//      pPacketDesc->Config.Value |= I2C_ENDIAN_RESULT_SET(I2C_ENDIAN_TRANSFORM_GET(pPacketDesc->Config.Value)); // Indicate that the endian transform have been processed
      __TWIHStransferList[PeriphNumber].Status = TWIHS_STATUS_READY; // Set status as ready for the next transfer
      __TWIHStransferList[PeriphNumber].Config.Value = 0;            // Clear current configuration
      return ERR_OK;

    case TWIHS_STATUS_FAULT:                                         //** TWIHS transfer in fault
      __TWIHStransferList[PeriphNumber].Status = TWIHS_STATUS_READY; // Set ready for a new transfer
      __TWIHStransferList[PeriphNumber].Config.Value = 0;            // Clear current configuration
      return __TWIHStransferList[PeriphNumber].Error;                // Return the last transfer error

    case TWIHS_STATUS_READY:                                         //** TWIHS transfer is ready
    default:                                                         // Configure the transfer with the following lines
      break;
  }
  if ((pPacketDesc->pBuffer == NULL) || (pPacketDesc->BufferSize == 0)) return ERR_OK; // Nothing to send, return

  //--- Configure the transfer ---
  const bool DeviceWrite = ((pPacketDesc->ChipAddr & 0x01) == 0);
  pTWIHS->TWIHS_MMR |= TWIHS_MMR_DADR(pPacketDesc->ChipAddr >> 1) // Device address
                     | (DeviceWrite ? 0 : TWIHS_MMR_MREAD);       // Read or write?
 // Configure and enable interrupt of TWIHS
  NVIC_EnableIRQ(PeriphID);
  NVIC_SetPriority(PeriphID, TWI_IRQ_level);
  Error = TWIHS_InterruptDisable(pTWIHS, TWIHS_ALL_INTERRUPTS, false);
  if (Error != ERR_OK) return Error;

  //--- Configure status ---
  __TWIHStransferList[PeriphNumber].DeviceWrite    = DeviceWrite;
  __TWIHStransferList[PeriphNumber].Data           = pPacketDesc->pBuffer;
  __TWIHStransferList[PeriphNumber].Size           = pPacketDesc->BufferSize;
  __TWIHStransferList[PeriphNumber].RemainingBytes = pPacketDesc->BufferSize;
  __TWIHStransferList[PeriphNumber].Status         = TWIHS_STATUS_IN_PROGRESS;
  __TWIHStransferList[PeriphNumber].Error          = ERR_OK;
  __TWIHStransferList[PeriphNumber].TransactionCounter++;
  if (__TWIHStransferList[PeriphNumber].TransactionCounter > I2C_TRANSACTION_NUMBER_Mask) __TWIHStransferList[PeriphNumber].TransactionCounter = 1; // Value cannot be 0
  __TWIHStransferList[PeriphNumber].Config.Value   = (pPacketDesc->Config.Value & ~I2C_ENDIAN_RESULT_Mask)
                                                   | I2C_TRANSACTION_NUMBER_SET(__TWIHStransferList[PeriphNumber].TransactionCounter);
  pPacketDesc->Config.Value |= I2C_TRANSACTION_NUMBER_SET(__TWIHStransferList[PeriphNumber].TransactionCounter); // Set the transaction number for the driver

  //--- Configure the XDMAC ---
  if ((DeviceWrite && (pPacketDesc->BufferSize > 1)) || ((DeviceWrite == false) && (pPacketDesc->BufferSize > 2)))
  {
//    const eI2C_EndianTransform EndianTransform = I2C_ENDIAN_TRANSFORM_GET(pPacketDesc->Config.Value);
    //--- Prepare DMA channel ---
    if (DeviceWrite)
    {
      //--- Configure the DMA for a write ---
/*      switch (EndianTransform)
      {
        default:
        case I2C_NO_ENDIAN_CHANGE:*/
            __XDMAC_I2Cconfig.MBR_SA  = (uint32_t)pPacketDesc->pBuffer; // Source Address Member
            __XDMAC_I2Cconfig.MBR_DA  = (uint32_t)pTWIHS->TWIHS_THR;    // Destination Address Member
            __XDMAC_I2Cconfig.MBR_BC  = 0;                              // Block Control Member. Microblock count is 1 per blocks
            __XDMAC_I2Cconfig.MBR_UBC = (pPacketDesc->BufferSize - 1);  // Microblock Control Member (size). I2C write with DMA needs the last byte to be processed manually
            __XDMAC_I2Cconfig.MBR_CFG = XDMAC_CC_TYPE_PER_TRAN          // Synchronized mode (Peripheral to Memory or Memory to Peripheral Transfer)
                                      | XDMAC_CC_MBSIZE_SINGLE          // The memory burst size is set to one
                                      | XDMAC_CC_DSYNC_MEM2PER          // Memory transfer to Peripheral
//                                      | XDMAC_CC_SWREQ_HWR_CONNECTED    // Hardware request line is connected to the peripheral request line
                                      | XDMAC_CC_MEMSET_NORMAL_MODE     // Memset is not activated
                                      | XDMAC_CC_CSIZE_CHK_1            // Chunk Size 1 data transferred
                                      | XDMAC_CC_DWIDTH_BYTE            // The data size is set to 8 bits
                                      | XDMAC_CC_SIF_AHB_IF0            // The data is read through the system bus interface 0
                                      | XDMAC_CC_DIF_AHB_IF1            // The data is written through the system bus interface 1
                                      | XDMAC_CC_SAM_INCREMENTED_AM     // The Source Addressing mode is incremented (the increment size is set to the data size)
                                      | XDMAC_CC_DAM_FIXED_AM           // The Destination Address remains unchanged
                                      | XDMAC_CC_PERID(XDMAC_TWIHS_PERID_Base + 0 + (PeriphNumber * 2)); // Channel x Peripheral Identifier
            __XDMAC_I2Cconfig.MBR_DS  = 0;                              // Data Stride Member
            __XDMAC_I2Cconfig.MBR_SUS = 0;                              // Source Microblock Stride Member.
            __XDMAC_I2Cconfig.MBR_DUS = 0;                              // Destination Microblock Stride Member
/*            break;
        case I2C_SWITCH_ENDIAN_16BITS:
            __XDMAC_I2Cconfig.MBR_SA  = (uint32_t)(pPacketDesc->pBuffer + 1); // Source Address Member + 1
            __XDMAC_I2Cconfig.MBR_DA  = (uint32_t)pTWIHS->TWIHS_THR;          // Destination Address Member
            __XDMAC_I2Cconfig.MBR_BC  = (pPacketDesc->BufferSize - 1) / 2;    // Block Control Member. I2C write with DMA needs the last byte to be processed manually. Divide per 2 because there will be 2-bytes per microblock
            __XDMAC_I2Cconfig.MBR_UBC = 2;                                    // Microblock Control Member (size). 2 bytes per microblock for 16-bits data
            __XDMAC_I2Cconfig.MBR_CFG = XDMAC_CC_TYPE_PER_TRAN                // Synchronized mode (Peripheral to Memory or Memory to Peripheral Transfer)
                                      | XDMAC_CC_MBSIZE_SINGLE                // The memory burst size is set to one
                                      | XDMAC_CC_DSYNC_MEM2PER                // Memory transfer to Peripheral
//                                      | XDMAC_CC_SWREQ_HWR_CONNECTED          // Hardware request line is connected to the peripheral request line
                                      | XDMAC_CC_MEMSET_NORMAL_MODE           // Memset is not activated
                                      | XDMAC_CC_CSIZE_CHK_1                  // Chunk Size 1 data transferred
                                      | XDMAC_CC_DWIDTH_BYTE                  // The data size is set to 8 bits
                                      | XDMAC_CC_SIF_AHB_IF0                  // The data is read through the system bus interface 0
                                      | XDMAC_CC_DIF_AHB_IF1                  // The data is written through the system bus interface 1
                                      | XDMAC_CC_SAM_UBS_DS_AM                // The microblock stride is added at the microblock boundary, the data stride is added at the data boundary
                                      | XDMAC_CC_DAM_FIXED_AM                 // The Destination Address remains unchanged
                                      | XDMAC_CC_PERID(XDMAC_TWIHS_PERID_Base + 0 + (PeriphNumber * 2)); // Channel x Peripheral Identifier
            __XDMAC_I2Cconfig.MBR_DS  = XDMAC_CDS_MSP_DDS_MSP((uint16_t)-2);  // Data Stride Member. Save 2-bytes data backward to perform the endian swap
            __XDMAC_I2Cconfig.MBR_SUS = 2;                                    // Source Microblock Stride Member.
            __XDMAC_I2Cconfig.MBR_DUS = 0;                                    // Destination Microblock Stride Member
            break;
      }*/
    }
    else
    {
      //--- Configure the DMA for a read ---
/*      switch (EndianTransform)
      {
        default:
        case I2C_NO_ENDIAN_CHANGE:*/
            __XDMAC_I2Cconfig.MBR_SA  = (uint32_t)pTWIHS->TWIHS_RHR;    // Source Address Member
            __XDMAC_I2Cconfig.MBR_DA  = (uint32_t)pPacketDesc->pBuffer; // Destination Address Member
            __XDMAC_I2Cconfig.MBR_BC  = 0;                              // Block Control Member. Microblock count is 1 per blocks
            __XDMAC_I2Cconfig.MBR_UBC = (pPacketDesc->BufferSize - 2);  // Microblock Control Member (size). I2C read with DMA needs the last 2-bytes to be processed manually
            __XDMAC_I2Cconfig.MBR_CFG = XDMAC_CC_TYPE_PER_TRAN          // Synchronized mode (Peripheral to Memory or Memory to Peripheral Transfer)
                                      | XDMAC_CC_MBSIZE_SINGLE          // The memory burst size is set to one
                                      | XDMAC_CC_DSYNC_PER2MEM          // Peripheral to Memory transfer
//                                      | XDMAC_CC_SWREQ_HWR_CONNECTED    // Hardware request line is connected to the peripheral request line
                                      | XDMAC_CC_MEMSET_NORMAL_MODE     // Memset is not activated
                                      | XDMAC_CC_CSIZE_CHK_1            // Chunk Size 1 data transferred
                                      | XDMAC_CC_DWIDTH_BYTE            // The data size is set to 8 bits
                                      | XDMAC_CC_SIF_AHB_IF1            // The data is read through the system bus interface 1
                                      | XDMAC_CC_DIF_AHB_IF0            // The data is written through the system bus interface 0
                                      | XDMAC_CC_SAM_FIXED_AM           // The Source Address remains unchanged
                                      | XDMAC_CC_DAM_INCREMENTED_AM     // The Destination Addressing mode is incremented (the increment size is set to the data size)
                                      | XDMAC_CC_PERID(XDMAC_TWIHS_PERID_Base + 1 + (PeriphNumber * 2)); // Channel x Peripheral Identifier
            __XDMAC_I2Cconfig.MBR_DS  = 0;                              // Data Stride Member
            __XDMAC_I2Cconfig.MBR_SUS = 0;                              // Source Microblock Stride Member.
            __XDMAC_I2Cconfig.MBR_DUS = 0;                              // Destination Microblock Stride Member
/*            break;
        case I2C_SWITCH_ENDIAN_16BITS:
            __XDMAC_I2Cconfig.MBR_SA  = (uint32_t)pTWIHS->TWIHS_RHR;          // Source Address Member
            __XDMAC_I2Cconfig.MBR_DA  = (uint32_t)(pPacketDesc->pBuffer + 1); // Destination Address Member + 1
            __XDMAC_I2Cconfig.MBR_BC  = (pPacketDesc->BufferSize - 2) / 2;    // Block Control Member. I2C read with DMA needs the last 2-bytes to be processed manually. Divide per 2 because there will be 2-bytes per microblock
            __XDMAC_I2Cconfig.MBR_UBC = 2;                                    // Microblock Control Member (size). 2 bytes per microblock for 16-bits data
            __XDMAC_I2Cconfig.MBR_CFG = XDMAC_CC_TYPE_PER_TRAN                // Synchronized mode (Peripheral to Memory or Memory to Peripheral Transfer)
                                      | XDMAC_CC_MBSIZE_SINGLE                // The memory burst size is set to one
                                      | XDMAC_CC_DSYNC_MEM2PER                // Memory transfer to Peripheral
//                                      | XDMAC_CC_SWREQ_HWR_CONNECTED          // Hardware request line is connected to the peripheral request line
                                      | XDMAC_CC_MEMSET_NORMAL_MODE           // Memset is not activated
                                      | XDMAC_CC_CSIZE_CHK_1                  // Chunk Size 1 data transferred
                                      | XDMAC_CC_DWIDTH_BYTE                  // The data size is set to 8 bits
                                      | XDMAC_CC_SIF_AHB_IF0                  // The data is read through the system bus interface 0
                                      | XDMAC_CC_DIF_AHB_IF1                  // The data is written through the system bus interface 1
                                      | XDMAC_CC_SAM_FIXED_AM                 // The Source Address remains unchanged
                                      | XDMAC_CC_DAM_UBS_DS_AM                // The microblock stride is added at the microblock boundary, the data stride is added at the data boundary
                                      | XDMAC_CC_PERID(XDMAC_TWIHS_PERID_Base + 1 + (PeriphNumber * 2)); // Channel x Peripheral Identifier
            __XDMAC_I2Cconfig.MBR_DS  = XDMAC_CDS_MSP_SDS_MSP((uint16_t)-2);  // Data Stride Member. Save 2-bytes data backward to perform the endian swap
            __XDMAC_I2Cconfig.MBR_SUS = 0;                                    // Source Microblock Stride Member.
            __XDMAC_I2Cconfig.MBR_DUS = 2;                                    // Destination Microblock Stride Member
            break;
      }*/
    }
    __XDMAC_I2Cconfig.MBR_NDA = 0;                 // Next Descriptor Address
    __XDMAC_I2Cconfig.MBR_NDC = 0;                 // Next Descriptor Control
    __XDMAC_I2Cconfig.NDAIF   = 0;                 // Next Descriptor Interface
    //--- Set XDMA interrupts ---
    __XDMAC_I2Cconfig.Interrupts = XDMAC_CIE_BIE   // End of Block Interrupt Enable Bit
                                 | XDMAC_CIE_DIE   // End of Disable Interrupt Enable Bit
                                 | XDMAC_CIE_FIE   // End of Flush Interrupt Enable Bit
                                 | XDMAC_CIE_RBIE  // Read Bus Error Interrupt Enable Bit
                                 | XDMAC_CIE_WBIE  // Write Bus Error Interrupt Enable Bit
                                 | XDMAC_CIE_ROIE; // Request Overflow Error Interrupt Enable Bit
    //--- Configure and enable DMA channel ---
    Error = XDMAC_ConfigureTransfer(__HasReservedDMAchannel[PeriphNumber], &__XDMAC_I2Cconfig);
    if (Error != ERR_OK) return Error;
# ifdef CONF_BOARD_ENABLE_CACHE
    SCB_CleanDCache_by_Addr((uint32_t*)__TWIHStransferList[PeriphNumber].Data, __TWIHStransferList[PeriphNumber].Size);
# endif
    Error = XDMAC_ChannelEnable(__HasReservedDMAchannel[PeriphNumber]);
    if (Error != ERR_OK) return Error;
  }

  //--- Set I2C interrupts ---
  pTWIHS->TWIHS_IER = TWIHS_IER_NACK                                    // Not Acknowledge Interrupt Enable
                    | (DeviceWrite ? TWIHS_IER_TXRDY : TWIHS_IER_RXRDY) // Transmit/Receive Holding Register Ready Interrupt Enable
                    | TWIHS_IER_OVRE                                    // Overrun Error Interrupt Enable
                    | TWIHS_IER_ARBLST;                                 // Arbitration Lost Interrupt Enable

  //--- Send a START condition ---
  if (pPacketDesc->BufferSize == 1)
       pTWIHS->TWIHS_CR |= TWIHS_CR_START | TWIHS_CR_STOP;
  else pTWIHS->TWIHS_CR |= TWIHS_CR_START;
  return ERR__I2C_BUSY;
}





//********************************************************************************************************************
//=============================================================================
// Atmel TWIHS master with DMA initialization
//=============================================================================
eERRORRESULT TWIHS_DMA_MasterInit(Twihs *pTWIHS, const uint32_t sclFreq)
{
  if (pTWIHS == NULL) return ERR__I2C_PARAMETER_ERROR;

  //--- Configure DMA ---
  uint32_t PeriphNumber = TWIHS_GetPeripheralNumber(pTWIHS);
  if (PeriphNumber == TWIHS_INVALID_PERIPHERAL) return ERR__PERIPHERAL_NOT_VALID;
  if (__HasReservedDMAchannel[PeriphNumber] == XDMAC_INVALID_HANDLE)
  {
    __TWIHStransferList[PeriphNumber].Status = TWIHS_STATUS_READY;
    __HasReservedDMAchannel[PeriphNumber] = XDMAC_OpenChannel(XDMAC, TWIHS_DMA_Handler, (uintptr_t)pTWIHS); // Open a DMA channel for this TWIHS peripheral
  }
  if ((__TWIHStransferList[PeriphNumber].Status != TWIHS_STATUS_UNINITIALIZED)
   && (__TWIHStransferList[PeriphNumber].Status != TWIHS_STATUS_READY)) return ERR__I2C_OTHER_BUSY;

  //--- Configuration of the TWI interface ---
  return TWIHS_MasterInit(pTWIHS, sclFreq);
}

eERRORRESULT TWIHS_DMA_MasterInit_Gen(I2C_Interface *pIntDev, const uint32_t sclFreq)
{
  if (pIntDev == NULL) return ERR__I2C_PARAMETER_ERROR;
  Twihs* pTWIHS = (Twihs*)(pIntDev->InterfaceDevice);
  return TWIHS_DMA_MasterInit(pTWIHS, sclFreq);
}



//=============================================================================
// Hardware I2C data packet transfer communication for the Atmel TWIHS
//=============================================================================
eERRORRESULT TWIHS_PacketTransfer(Twihs *pTWIHS, I2CInterface_Packet* const pPacketDesc)
{
  if (pTWIHS == NULL) return ERR__I2C_PARAMETER_ERROR;
  uint32_t PeriphNumber = TWIHS_GetPeripheralNumber(pTWIHS);

  //--- Use polling? ---
  if ((pPacketDesc->Config.Value & I2C_USE_POLLING) == I2C_USE_POLLING)
  {
    if (__HasReservedDMAchannel[PeriphNumber] == XDMAC_INVALID_HANDLE) return ERR__DMA_NOT_CONFIGURED;
    const uint8_t CurrentTransaction = I2C_TRANSACTION_NUMBER_GET(__TWIHStransferList[PeriphNumber].Config.Value);
    const uint8_t NewTransaction     = I2C_TRANSACTION_NUMBER_GET(pPacketDesc->Config.Value);
    if (NewTransaction != CurrentTransaction) return ERR__I2C_OTHER_BUSY; // It is a transfer from an another driver

    if (I2C_IS_FIRST_TRANSFER(pPacketDesc->Config.Value))
    {
       if (pPacketDesc->BufferSize < 4)
       {
         pTWIHS->TWIHS_MMR = TWIHS_MMR_IADRSZ(pPacketDesc->BufferSize); // Prepare internal device address byte count
         pTWIHS->TWIHS_CR  = TWIHS_CR_MSEN | TWIHS_CR_SVDIS;            // Master enable and slave disable
         //--- Set the internal device address ---
         pTWIHS->TWIHS_IADR = 0;
         for (size_t z = 0; z < pPacketDesc->BufferSize; ++z)
         {
           pTWIHS->TWIHS_IADR <<= 8;
           pTWIHS->TWIHS_IADR |= pPacketDesc->pBuffer[z];
         }
         return ERR_OK;
       }
    }
    else return __TWIHS_DMA_Transfer(pTWIHS, pPacketDesc);
  }
  
  //--- Do a regular transfer ---
  return TWIHS_Transfer(pTWIHS, pPacketDesc);
}

eERRORRESULT TWIHS_PacketTransfer_Gen(I2C_Interface *pIntDev, I2CInterface_Packet* const pPacketDesc)
{
  Twihs* pTWIHS = (Twihs*)(pIntDev->InterfaceDevice);
  return TWIHS_PacketTransfer(pTWIHS, pPacketDesc);
}





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
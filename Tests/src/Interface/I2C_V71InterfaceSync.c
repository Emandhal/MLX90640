/*******************************************************************************
    File name:    I2C_V71InterfaceSync.c
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 15/04/2020
    Description:  I2C interface for driver
                  This interface implements the synchronous use of the I2C on a SAMV71
                  and is also specific with the SAMV71 Xplained Ultra board

    History :
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "I2C_V71InterfaceSync.h"
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


static bool I2Cconfigured = false;
#if defined(SOFT_I2C)
  static bool I2CstopSent   = true;
#endif

//-----------------------------------------------------------------------------



#if defined(SOFT_I2C)
//=============================================================================
// Soft I2C driver interface configuration for the ATSAMV71
//=============================================================================
eERRORRESULT SoftI2C_InterfaceInit_V71(void *pIntDev, const uint32_t sclFreq)
{
  if (pIntDev == NULL) return ERR__I2C_PARAMETER_ERROR;
  if (I2Cconfigured) return ERR_OK;

  //--- Pin configuration of the soft I2C ---
  SDA0_SOFT_PIO_En;
  SDA0_SOFT_High;   // SDA = 1
  SDA0_SOFT_In;     // SDA in

  SCL0_SOFT_PIO_En;
  SCL0_SOFT_High;   // SCL = 1
  SCL0_SOFT_Out;    // SCL out

  //--- Reset ---
  size_t z = 9;                 // Clock up to 9 cycles. Here we force I2C SCL to clock until a device stuck in communication respond
  while (SDA0_SOFT_Status == 0) // Look for SDA high in each cycle while SCL is high and then break
  {
		delay_us(1);
		SCL0_SOFT_Low;  // SCL = 0
		delay_us(1);
		SCL0_SOFT_High; // SCL = 1
    if (--z == 0) break;
  }

  I2Cconfigured = true;
  I2CstopSent   = true;
  return ERR_OK;
}



//=============================================================================
// Soft I2C - Start an I2C communication for the ATSAMV71
//=============================================================================
eERRORRESULT SoftI2C_Start_V71(void *pIntDev, uint8_t addrComp)
{
  if (I2CstopSent)
  {
    //--- I2C Start ---
    SDA0_SOFT_High; // SDA = 1
    SDA0_SOFT_Out;  // SDA out
    SCL0_SOFT_High; // SCL = 1
    SCL0_SOFT_Out;  // SCL out
    I2CstopSent = false;
  }
  else
  {
    //--- I2C Restart ---
    SDA0_SOFT_High; // SDA = 1
    SDA0_SOFT_Out;  // SDA out
    SCL0_SOFT_Low;  // SCL = 0
    SCL0_SOFT_Out;  // SCL out
    delay_us(1);
    SCL0_SOFT_High; // SCL = 1
  }
  delay_us(1);
  SDA0_SOFT_Low;  // SDA = 0
  delay_us(1);
  SCL0_SOFT_Low;  // SCL = 0
  delay_us(1);

  //--- Transmit component address ---
  return SoftI2C_TxByte_V71(pIntDev, addrComp);
}



//=============================================================================
// Soft I2C - Stop an I2C communication for the ATSAMV71
//=============================================================================
eERRORRESULT SoftI2C_Stop_V71(void *pIntDev)
{
  SDA0_SOFT_Low;  // SDA = 0
  SDA0_SOFT_Out;  // SDA out
  delay_us(1);
  SCL0_SOFT_High; // SCL = 1
  delay_us(1);
  SDA0_SOFT_High; // SDA = 1
  delay_us(1);
  SCL0_SOFT_In;   // SCL in
  SDA0_SOFT_In;   // SDA in
  I2CstopSent = true;
  return ERR_OK;
}



//=============================================================================
// Soft I2C - Transmit a byte through an I2C communication for the ATSAMV71
//=============================================================================
eERRORRESULT SoftI2C_TxByte_V71(void *pIntDev, const uint8_t dataByte)
{
  bool Ack = false;
  uint8_t ByteToSend = dataByte;
  SDA0_SOFT_Out;                 // SDA out
  for (int_fast8_t z = 8; --z >= 0;)
  {
    //--- Set SDA ---
    if ((ByteToSend & 0x80) > 0)
         SDA0_SOFT_High;         // SDA = 1
    else SDA0_SOFT_Low;          // SDA = 0
    ByteToSend <<= 1;
    delay_us(1);

    //--- SCL period ---
    SCL0_SOFT_High;              // SCL = 1
    delay_us(2);
    SCL0_SOFT_Low;               // SCL = 0
    delay_us(1);
  }

  //--- Check Ack ---
  SDA0_SOFT_In;                  // SDA in
  delay_us(1);
  SCL0_SOFT_High;                // SCL = 1
  delay_us(1);
  Ack = (SDA0_SOFT_Status == 0); // Ack if SDA = 0
  delay_us(1);
  SCL0_SOFT_Low;                 // SCL = 0
  delay_us(1);
  SDA0_SOFT_High;                // SDA = 1
  SDA0_SOFT_Out;                 // SDA out
  return Ack ? ERR_OK : ERR__I2C_NACK;
}



//=============================================================================
// Soft I2C - Receive a byte through an I2C communication for the ATSAMV71
//=============================================================================
eERRORRESULT SoftI2C_RxByte_V71(void *pIntDev, uint8_t *dataByte, bool ack)
{
  *dataByte = 0x00;
  SDA0_SOFT_In;               // SDA in
  for (int_fast8_t z = 8; --z >= 0;)
  {
    delay_us(1);
    SCL0_SOFT_High;           // SCL = 1
    delay_us(1);
    *dataByte <<= 1u;
    if (SDA0_SOFT_Status > 0) // Get bit value on SDA
      *dataByte |= 0x01;
    delay_us(1);
    SCL0_SOFT_Low;            // SCL = 0
    delay_us(1);
  }

  //--- Set ACK ---
  SDA0_SOFT_Out;              // SDA out
  if (ack)
       SDA0_SOFT_Low;         // SDA = 0
  else SDA0_SOFT_High;        // SDA = 1
  delay_us(1);
  SCL0_SOFT_High;             // SCL = 1
  delay_us(2);
  SCL0_SOFT_Low;              // SCL = 0
  delay_us(1);

  return ERR_OK;
}


//=============================================================================
// Software I2C - Transfer data through an I2C communication for the ATSAMV71
//=============================================================================
eERRORRESULT SoftI2C_Tranfert_V71(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop)
{
  eERRORRESULT Error = ERR_OK;
  bool DeviceReady = true;
  bool ForceStop   = false;
  bool DeviceWrite  = ((deviceAddress & 0x01) == 0);

  //--- Transfer data ---
  if (start)
  {
    Error = SoftI2C_Start_V71(pIntDev, deviceAddress); // Send a start if asked
    if (Error == ERR__I2C_NACK) DeviceReady = false;   // If the device receive a NAK, then the device is not ready
    if (Error != ERR_OK) ForceStop = true;             // If there is an error while starting the transfer then
  }
  if (ForceStop == false)
  {
    if (DeviceWrite) // Device write
    {
      while (byteCount > 0)
      {
        Error = SoftI2C_TxByte_V71(pIntDev, *data);             // Transmit byte
        if (Error == ERR__I2C_NACK) Error = ERR__I2C_NACK_DATA; // If we receive a NACK here then it is a NACK on data transfer
        if (Error != ERR_OK) { ForceStop = true; break; }       // If there is an error while receiving data from I2C then stop the transfer
        data++;
        byteCount--;
      }
    }
    else // Device read
    {
      while (byteCount > 0)
      {
        Error = SoftI2C_RxByte_V71(pIntDev, data, byteCount > 1); // Receive byte
        if (Error != ERR_OK) { ForceStop = true; break; }         // If there is an error while receiving data from I2C then stop the transfer
        data++;
        byteCount--;
      }
    }
  }
  if (stop || ForceStop)
  {
    //--- Stop transfer ---
    eERRORRESULT ErrorStop = SoftI2C_Stop_V71(pIntDev); // Stop I2C
    if (DeviceReady == false) return ERR__NOT_READY;
    return (Error != ERR_OK ? Error : ErrorStop);
  }

  return Error;
}



#else
//=============================================================================
// Hardware I2C driver interface configuration for the ATSAMV71
//=============================================================================
eERRORRESULT HardI2C_InterfaceInit_V71(void *pIntDev, const uint32_t sclFreq)
{
  if (pIntDev == NULL) return ERR__I2C_PARAMETER_ERROR;
  Twihs *I2C = (Twihs *)pIntDev;
//  if (I2Cconfigured) return ERR_OK;

  //--- Reset ---
  SDA0_SOFT_PIO_En;
  SDA0_SOFT_High;   // SDA = 1
  SDA0_SOFT_In;     // SDA in
  SCL0_SOFT_PIO_En;
  SCL0_SOFT_High;   // SCL = 1
  SCL0_SOFT_Out;    // SCL out
  size_t z = 9;                 // Clock up to 9 cycles. Here we force I2C SCL to clock until a device stuck in communication respond
  while (SDA0_SOFT_Status == 0) // Look for SDA high in each cycle while SCL is high and then break
  {
		delay_us(1);
		SCL0_SOFT_Low;  // SCL = 0
		delay_us(1);
		SCL0_SOFT_High; // SCL = 1
    if (--z == 0) break;
  }
  ioport_set_pin_mode(TWIHS0_DATA_GPIO, TWIHS0_DATA_FLAGS); // Restore SDA pin function
  ioport_disable_pin(TWIHS0_DATA_GPIO);                     // Restore SDA pin function
  ioport_set_pin_mode(TWIHS0_CLK_GPIO, TWIHS0_CLK_FLAGS);   // Restore SCL pin function
  ioport_disable_pin(TWIHS0_CLK_GPIO);                      // Restore SCL pin function

  //--- Configuration of the TWI interface ---
  twihs_options_t opt;
  opt.speed = sclFreq;
  if (twihs_master_setup(I2C, &opt) != TWIHS_SUCCESS) return ERR__I2C_CONFIG_ERROR;
  I2C->TWIHS_CR |= TWIHS_CR_STOP;

  //--- Clear receive buffer ---
  uint8_t Data = I2C->TWIHS_RHR;
  (void)Data; // Unused data
  //--- clear registers ---
  Data = I2C->TWIHS_SR;
  (void)Data; // Unused data

  I2Cconfigured = true;
  return ERR_OK;
}



//=============================================================================
// Hardware I2C - Transfer data through an I2C communication for the ATSAMV71
//=============================================================================
eERRORRESULT HardI2C_Tranfert_V71(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop)
{
  Twihs *I2C = (Twihs *)pIntDev;
  size_t RemainingBytes = byteCount;
  uint32_t Status;
  uint32_t Timeout = TWIHS_TIMEOUT;
  bool DeviceWrite  = ((deviceAddress & 0x01) == 0);
  I2C->TWIHS_MMR  = 0;
  I2C->TWIHS_MMR  = TWIHS_MMR_DADR(deviceAddress >> 1) | (DeviceWrite ? 0 : TWIHS_MMR_MREAD);
  I2C->TWIHS_IADR = 0; // Not used
  I2C->TWIHS_CR   = TWIHS_CR_MSEN | TWIHS_CR_SVDIS;

  //--- Device polling ? ---
  if ((data == NULL) || (byteCount <= 0))                      // Device polling only
  { // Little hack because TWI of V71 does not support device polling without using SMBus
    I2C->TWIHS_MMR &= ~TWIHS_MMR_MREAD;                        // The SMBus of this device does not support quick read command (no Stop will be sent)
    I2C->TWIHS_CR |= TWIHS_CR_SMBEN + TWIHS_CR_PECDIS;         // Enable SMBus
    I2C->TWIHS_CR |= TWIHS_CR_STOP;                            // Send a stop
    I2C->TWIHS_CR |= TWIHS_CR_QUICK;                           // Start the polling with a quick command

    Timeout = TWIHS_TIMEOUT;
    while (true)                                               // Wait the polling to finish
    {
      Status = I2C->TWIHS_SR;
      if ((Status & TWIHS_SR_NACK) > 0) return ERR__I2C_NACK;
      if (!Timeout--) return ERR__I2C_TIMEOUT;                 // Timeout ? return an error
      if ((Status & TWIHS_SR_TXCOMP) > 0) break;
    }
    return ERR_OK;
  }

  //--- Transfer data ---
  if (start) I2C->TWIHS_CR |= TWIHS_CR_START;                  // Send a start if asked
  if (DeviceWrite) // Device write
  {
    while (true)
    {
      Status = I2C->TWIHS_SR;
      if ((Status & TWIHS_SR_NACK) > 0) return ERR__I2C_NACK_DATA;
      if (!Timeout--) return ERR__I2C_TIMEOUT;                 // Timeout ? return an error
      if ((Status & TWIHS_SR_TXRDY) == 0) continue;
      Timeout = TWIHS_TIMEOUT;

      if (RemainingBytes == 0) break;                          // No data remaining to send, then break the loop
      I2C->TWIHS_THR = *data;                                  // Send next data byte
      data++;
      RemainingBytes--;
    }
    if (stop) I2C->TWIHS_CR |= TWIHS_CR_STOP;                  // Send a stop if asked
  }
  else // Device read
  {
    while (RemainingBytes > 0)
    {
      if ((RemainingBytes == 1) && stop)
        I2C->TWIHS_CR |= TWIHS_CR_STOP;                        // Last byte ? Send a stop if asked

      Timeout = TWIHS_TIMEOUT;
      while (true)                                             // Wait the polling to finish
      {
        Status = I2C->TWIHS_SR;
        if ((Status & TWIHS_SR_NACK) > 0)
        {
          if (RemainingBytes == byteCount)
               return ERR__I2C_NACK;
          else return ERR__I2C_NACK_DATA;
        }
        if (!Timeout--) return ERR__I2C_TIMEOUT;               // Timeout ? return an error
        if ((Status & TWIHS_SR_RXRDY) > 0) break;
      }

      *data = I2C->TWIHS_RHR;                                  // Get next data byte
      data++;
      RemainingBytes--;
    }
  }
  if (stop) while ((I2C->TWIHS_SR & TWIHS_SR_TXCOMP) == 0);    // Wait until both holding register and internal shifter are empty and STOP condition has been sent
  return ERR_OK;
}
#endif




//**********************************************************************************************************************************************************
//=============================================================================
// Get millisecond
//=============================================================================
uint32_t GetCurrentms_V71(void)
{
  return msCount;
}





//**********************************************************************************************************************************************************



//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------
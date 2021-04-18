/*******************************************************************************
    File name:    I2C_V71InterfaceSync.h
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 15/04/2020
    Description:  I2C interface for driver
    This interface implements the synchronous use of the I2C on a SAMV71
    and is also specific with the SAMV71 Xplained Ultra board
    History :
*******************************************************************************/
#ifndef I2C_V71INTERFACESYNC_H_INC
#define I2C_V71INTERFACESYNC_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include "Main.h"
//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------



#define BOARD_I2C_CLK_SPEED_HZ        ( 400000 ) //! I2C speed at 400kHz
#define BOARD_CLK_TWIHS_MUX_EEPROM    ( 0 )      //! TWIHS EEPROM clock pin peripheral

#define SCL0_SOFT_IRQn        ( PIOA_IRQn )
#define SCL0_SOFT_ID          ( ID_PIOA )
#define SCL0_SOFT_PIO         ( PIOA )
#define SCL0_SOFT_PORT        ( IOPORT_PIOA )
#define SCL0_SOFT_MASK        ( PIO_PA4 )
#define SCL0_SOFT_PIO_En        SCL0_SOFT_PIO->PIO_PER  |= SCL0_SOFT_MASK
#define SCL0_SOFT_PIO_Dis       SCL0_SOFT_PIO->PIO_PDR  |= SCL0_SOFT_MASK
#define SCL0_SOFT_PullUp_En     SCL0_SOFT_PIO->PIO_PUER |= SCL0_SOFT_MASK
#define SCL0_SOFT_PullUp_Dis    SCL0_SOFT_PIO->PIO_PUDR |= SCL0_SOFT_MASK
#define SCL0_SOFT_Filter_En     SCL0_SOFT_PIO->PIO_IFER |= SCL0_SOFT_MASK
#define SCL0_SOFT_Filter_Dis    SCL0_SOFT_PIO->PIO_IFDR |= SCL0_SOFT_MASK
#define SCL0_SOFT_Out           SCL0_SOFT_PIO->PIO_OER  |= SCL0_SOFT_MASK
#define SCL0_SOFT_In            SCL0_SOFT_PIO->PIO_ODR  |= SCL0_SOFT_MASK
#define SCL0_SOFT_High          SCL0_SOFT_PIO->PIO_SODR |= SCL0_SOFT_MASK
#define SCL0_SOFT_Low           SCL0_SOFT_PIO->PIO_CODR |= SCL0_SOFT_MASK
#define SCL0_SOFT_Status      ( SCL0_SOFT_PIO->PIO_PDSR &  SCL0_SOFT_MASK )

#define SDA0_SOFT_IRQn        ( PIOA_IRQn )
#define SDA0_SOFT_ID          ( ID_PIOA )
#define SDA0_SOFT_PIO         ( PIOA )
#define SDA0_SOFT_PORT        ( IOPORT_PIOA )
#define SDA0_SOFT_MASK        ( PIO_PA3 )
#define SDA0_SOFT_PIO_En        SDA0_SOFT_PIO->PIO_PER  |= SDA0_SOFT_MASK
#define SDA0_SOFT_PIO_Dis       SDA0_SOFT_PIO->PIO_PDR  |= SDA0_SOFT_MASK
#define SDA0_SOFT_PullUp_En     SDA0_SOFT_PIO->PIO_PUER |= SDA0_SOFT_MASK
#define SDA0_SOFT_PullUp_Dis    SDA0_SOFT_PIO->PIO_PUDR |= SDA0_SOFT_MASK
#define SDA0_SOFT_Filter_En     SDA0_SOFT_PIO->PIO_IFER |= SDA0_SOFT_MASK
#define SDA0_SOFT_Filter_Dis    SDA0_SOFT_PIO->PIO_IFDR |= SDA0_SOFT_MASK
#define SDA0_SOFT_Out           SDA0_SOFT_PIO->PIO_OER  |= SDA0_SOFT_MASK
#define SDA0_SOFT_In            SDA0_SOFT_PIO->PIO_ODR  |= SDA0_SOFT_MASK
#define SDA0_SOFT_High          SDA0_SOFT_PIO->PIO_SODR |= SDA0_SOFT_MASK
#define SDA0_SOFT_Low           SDA0_SOFT_PIO->PIO_CODR |= SDA0_SOFT_MASK
#define SDA0_SOFT_Status      ( SDA0_SOFT_PIO->PIO_PDSR &  SDA0_SOFT_MASK )


//*****************************************************************************



#if defined(SOFT_I2C)
/*! @brief Software I2C driver interface configuration for the ATSAMV71
 *
 * This function will be called at driver initialization to configure the interface driver soft I2C
 * @param[in] *pIntDev Is the EERAM47x16.InterfaceDevice of the device that call this function
 * @param[in] sclFreq Is the SCL frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SoftI2C_InterfaceInit_V71(void *pIntDev, const uint32_t sclFreq);


/*! @brief Software I2C - Start an I2C communication for the ATSAMV71
 *
 * This function will perform a software I2C start and then send the addrComp
 * In case of a restart, the function will handle it correctly
 * @param[in] *pIntDev Is the Interface Device pointer of the device that call this function
 * @param[in] addrComp Is the address of the component to start a communication with. Shall contain the R/W bit
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SoftI2C_Start_V71(void *pIntDev, uint8_t addrComp);


/*! @brief Software I2C - Stop an I2C communication for the ATSAMV71
 *
 * This function will perform a software I2C stop
 * @param[in] *pIntDev Is the Interface Device pointer of the device that call this function
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SoftI2C_Stop_V71(void *pIntDev);


/*! @brief Software I2C - Transmit a byte through an I2C communication for the ATSAMV71
 *
 * This function will perform a software I2C byte transmission
 * @param[in] *pIntDev Is the Interface Device pointer of the device that call this function
 * @param[in] dataByte Is the data byte to send over the I2C bus
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SoftI2C_TxByte_V71(void *pIntDev, const uint8_t dataByte);


/*! @brief Software I2C - Receive a byte through an I2C communication for the ATSAMV71
 *
 * This function will perform a software I2C byte reception
 * @param[in] *pIntDev Is the Interface Device pointer of the device that call this function
 * @param[out] *dataByte Is the data byte to be received over the I2C bus
 * @param[in] ack Is the acknowledgment that must be sent to the device for this data byte ('true' = ACK ; 'false' = NACK)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SoftI2C_RxByte_V71(void *pIntDev, uint8_t *dataByte, bool ack);


/*! @brief Software I2C - Transfer data through an I2C communication for the ATSAMV71
 *
 * This function will be called when the driver needs to transfer data over the I2C communication with the device
 * Can be read data of transmit data. It also indicate if it needs a start and/or a stop
 * @param[in] *pIntDev Is the Interface Device pointer of the device that call the I2C transfer
 * @param[in] deviceAddress Is the device address on the bus (8-bits only). The LSB bit indicate if it is a I2C Read (bit at '1') or a I2C Write (bit at '0')
 * @param[in,out] *data Is a pointer to memory data to write in case of I2C Write, or where the data received will be stored in case of I2C Read (can be NULL if no data transfer other than chip address)
 * @param[in] byteCount Is the byte count to write over the I2C bus or the count of byte to read over the bus
 * @param[in] start Indicate if the transfer needs a start (in case of a new transfer) or restart (if the previous transfer have not been stopped)
 * @param[in] stop Indicate if the transfer needs a stop after the last byte sent
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SoftI2C_Tranfert_V71(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop);

//********************************************************************************************************************



#else
/*! @brief Hardware I2C driver interface configuration for the ATSAMV71
 *
 * This function will be called at driver initialization to configure the interface driver soft I2C
 * @param[in] *pIntDev Is the EERAM47x16.InterfaceDevice of the device that call this function
 * @param[in] sclFreq Is the SCL frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT HardI2C_InterfaceInit_V71(void *pIntDev, const uint32_t sclFreq);


/*! @brief Hardware I2C - Transfer data through an I2C communication for the ATSAMV71
 *
 * This function will be called when the driver needs to transfer data over the I2C communication with the device
 * Can be read data of transmit data. It also indicate if it needs a start and/or a stop
 * @param[in] *pIntDev Is the Interface Device pointer of the device that call the I2C transfer
 * @param[in] deviceAddress Is the device address on the bus (8-bits only). The LSB bit indicate if it is a I2C Read (bit at '1') or a I2C Write (bit at '0')
 * @param[in,out] *data Is a pointer to memory data to write in case of I2C Write, or where the data received will be stored in case of I2C Read (can be NULL if no data transfer other than chip address)
 * @param[in] byteCount Is the byte count to write over the I2C bus or the count of byte to read over the bus
 * @param[in] start Indicate if the transfer needs a start (in case of a new transfer) or restart (if the previous transfer have not been stopped)
 * @param[in] stop Indicate if the transfer needs a stop after the last byte sent
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT HardI2C_Tranfert_V71(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop);
#endif

//********************************************************************************************************************



/*! @brief Get millisecond
 *
 * This function will be called when the driver need to get current millisecond
 */
uint32_t GetCurrentms_V71(void);

//********************************************************************************************************************





//-----------------------------------------------------------------------------
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
//-----------------------------------------------------------------------------
#endif /* I2C_V71INTERFACESYNC_H_INC */
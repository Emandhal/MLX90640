/*******************************************************************************
 * @file    MLX90640.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.2
 * @date    27/02/2021
 * @brief   MLX90640 driver
 *
 * 32x24 IR array
 * Follow datasheet 3901090640 Rev.12 (Dec 2019)
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

/* Revision history:
 * 1.0.2    Correct resolution error in delta Vdd calculus
 * 1.0.1    Add minimum and maximum value of To for each subframe in MLX90640_FrameTo
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef MLX90640_H_INC
#define MLX90640_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#include "ErrorsDef.h"
#include "Conf_MLX90640.h"
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

//! If the moving average filter is not defined, define it with a value that disable it
#ifndef MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT
#  define MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT  ( 0 )
#endif

//-----------------------------------------------------------------------------



// Limits definitions
#define MLX90640_I2CCLOCK_FMp_MAX  ( 1000000u ) //!< Max I2C clock frequency in FM+ mode (Fast-mode plus)
#define MLX90640_I2CCLOCK_FM_MAX   (  400000u ) //!< Max I2C clock frequency in FM mode (Fast-mode)



// Device definitions
#define MLX90640_I2C_READ                  ( 0x01 ) //!< Standard I2C LSB bit to set
#define MLX90640_I2C_WRITE                 ( 0xFE ) //!< Standard I2C bit mask which clear the LSB

#define MLX90640_CHIPADDRESS_DEFAULT  ( 0x33 << 1 ) //!< Base chip address (Default, can be changed)
#define MLX90640_CHIPADDRESS_MASK          ( 0xFE ) //!< Chip address mask

//-----------------------------------------------------------------------------

#define MLX90640_ROW_COUNT           ( 24 ) //!< The MLX90640 have 24 lines of pixels
#define MLX90640_COL_COUNT           ( 32 ) //!< The MLX90640 have 32 columns of pixels
#define MLX90640_TOTAL_PIXELS_COUNT  ( MLX90640_ROW_COUNT * MLX90640_COL_COUNT ) //!< Total pixels count of the MLX90640

#define MLX90640_MAX_DEFECT_PIXELS   ( 4 ) //!< The MLX90640 can have up to 4 defective pixels

//-----------------------------------------------------------------------------

#define MLX90640_VddV0  (  3.3f ) //!< Vdd_V0 is +3.3V
#define MLX90640_TaV0   ( 25.0f ) //!< Ta_V0 is +25°

//-----------------------------------------------------------------------------

//! Sign extending from a variable 'bitwidth' at 'pos' without branching (will be simplified at compile time with 'pos' and 'bitwidth' fixed
#define MLX90640_DATA_EXTRACT_TO_INT16(data,pos,bitwidth)  (int16_t)( ((((uint16_t)(data) >> (pos)) & ((1 << (bitwidth)) - 1)) ^ (1 << ((bitwidth) - 1))) - (1 << ((bitwidth) - 1)) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MLX90640 Register list
//********************************************************************************************************************

//! MLX90640 registers list
typedef enum
{
  // RAM Addresses
  RamMLX90640_StartAddress    = 0x0400, //!< RAM address start
  RamMLX90640_EndAddress      = 0x07FF, //!< RAM address end
  // EEPROM Addresses
  EepMLX90640_StartAddress    = 0x2400, //!< EEPROM address start
  EepMLX90640_Configuration   = 0x2403, //!< Configuration in EEPROM
  EepMLX90640_DeviceID        = 0x2407, //!< Device ID in EEPROM
  EepMLX90640_DeviceOption    = 0x240A, //!< Device Options in EEPROM
  EepMLX90640_I2Cconfig       = 0x240E, //!< I2C Configuration in EEPROM
  EepMLX90640_I2Caddress      = 0x240F, //!< I2C Address in EEPROM
  EepMLX90640_OffsetsData     = 0x2410, //!< Offsets data in EEPROM
  EepMLX90640_ScaleOCC        = 0x2410, //!< AlphaPTAT + Scale OCC parameters in EEPROM
  EepMLX90640_AlphasData      = 0x2420, //!< Alphas data in EEPROM
  EepMLX90640_GainParam       = 0x2430, //!< Gain parameter in EEPROM
  EepMLX90640_PTAT25Param     = 0x2431, //!< PTAT_25 parameters in EEPROM
  EepMLX90640_KPTATParam      = 0x2432, //!< K_PTAT parameters in EEPROM
  EepMLX90640_VddParam        = 0x2433, //!< Vdd parameters in EEPROM
  EepMLX90640_KvColRowParam   = 0x2434, //!< Kv for Columns and Row parameters in EEPROM
  EepMLX90640_ILchessParam    = 0x2435, //!< IL Chess parameters in EEPROM
  EepMLX90640_KtaColOddParam  = 0x2436, //!< Kta for Columns Odd parameters in EEPROM
  EepMLX90640_KtaColEvenParam = 0x2437, //!< Kta for Columns Even parameters in EEPROM
  EepMLX90640_KscaleParam     = 0x2438, //!< K Scales parameters in EEPROM
  EepMLX90640_AlphaCPspParam  = 0x2439, //!< Alpha Compensation Pixel subpages parameters in EEPROM
  EepMLX90640_OffsetCPspParam = 0x243A, //!< Offset Compensation Pixel subpages parameters in EEPROM
  EepMLX90640_KCP_Param       = 0x243B, //!< KtaCP and KvCP parameters in EEPROM
  EepMLX90640_TGC_KsTaParam   = 0x243C, //!< TGC and KsTa parameters in EEPROM
  EepMLX90640_KsToDataParam   = 0x243D, //!< KsTo data parameters in EEPROM
  EepMLX90640_CTtempParam     = 0x243F, //!< KsTo Scale and CTs temp data parameters in EEPROM
  EepMLX90640_PixelsParam     = 0x2440, //!< Pixels parameters start address in EEPROM
  EepMLX90640_EndAddress      = 0x273F, //!< EEPROM address end
  // Registers
  RegMLX90640_Status          = 0x8000, //!< Status Register
  RegMLX90640_Control1        = 0x800D, //!< Control 1 Register
  RegMLX90640_I2Cconfig       = 0x800F, //!< I2C Configuration Register
} eMLX90640_Registers;





//********************************************************************************************************************
// MLX90640 Specific Controller Registers
//********************************************************************************************************************

//! Status Register (Reg: 0x8000 ; EEPROM: 0x2403)
PACKITEM
typedef union __PACKED__ MLX90640_Status
{
  uint16_t Status;
  struct
  {
    uint16_t MEASSUB  :  3; //!< 0- 2 - Last measured subpage: '0b000' = Measurement of subpage 0 has been measured ; '0b001' = Measurement of subpage 1 has been measured
    uint16_t DATAAVAIL:  1; //!< 3    - New data available in RAM: '1' = A new data is available in RAM ; '0' = No new data is available in RAM (must be reset by the customer)
    uint16_t OVERWRENA:  1; //!< 4    - Enable overwrite: '1' = Data in RAM overwrite is enabled ; '0' = Data in RAM overwrite is disabled
    uint16_t          : 11; //!< 5-15
  } Bits;
} MLX90640_Status;
UNPACKITEM;
ControlItemSize(MLX90640_Status, 2);

//! Last measured subpage enum
typedef enum
{
  MLX90640_LAST_IS_SUBPAGE_0 = 0b000, //!< Measurement of subpage 0 has been measured
  MLX90640_LAST_IS_SUBPAGE_1 = 0b001, //!< Measurement of subpage 1 has been measured
} eMLX90640_LastMeasured;

#define MLX90640_LAST_SUBPAGE_Pos         0
#define MLX90640_LAST_SUBPAGE_Mask        (0x7u << MLX90640_LAST_SUBPAGE_Pos)
#define MLX90640_LAST_SUBPAGE_GET(value)  (((uint16_t)(value) & MLX90640_LAST_SUBPAGE_Mask) >> MLX90640_LAST_SUBPAGE_Pos) //!< Get last measurement of subpage
#define MLX90640_NEW_DATA_AVAILABLE       (0x1u << 3) //!< A new data is available in RAM
#define MLX90640_DATA_OVERWRITE_ENABLE    (0x1u << 4) //!< Enable data in RAM overwrite
#define MLX90640_DATA_OVERWRITE_DISABLE   (0x0u << 4) //!< Disable data in RAM overwrite

#define MLX90640_CLEAR_DEVICE_STATUS      ( MLX90640_DATA_OVERWRITE_ENABLE ) //!< This value clears the device's status

//-----------------------------------------------------------------------------



//! Control 1 Register (Reg: 0x800D ; EEPROM: 0x240C) [default: 0x1901]
PACKITEM
typedef union __PACKED__ MLX90640_Control1
{
  uint16_t Control1;
  struct
  {
    uint16_t SUBMODE    :  1; //!<  0    - Subpages mode: '1' = Subpage mode is activated (default) ; '0' = No subpages, only one page will be measured
    uint16_t            :  1; //!<
    uint16_t DATAHOLD   :  1; //!<  2    - Data hold: '1' = Transfer the data into storage RAM only if en_overwrite = 1 (check 0x8000) ; '0' = Transfer the data into storage RAM at each measured frame (default)
    uint16_t SUBREPEAT  :  1; //!<  3    - Subpages repeat: '1' Select subpage determines which subpage to be measured if Enable subpages mode = "1" ; '0' = Toggles between subpage "0" and subpage "1" if Enable subpages mode = "1" (default)
    uint16_t SELSUBPAGE :  3; //!<  4- 6 - Select subpage: '0b000' = Subpage 0 is selected (default) ; '0b001' = Subpage 1 is selected
    uint16_t REFRESHRAT :  3; //!<  7- 9 - Refresh rate control
    uint16_t RESOLUTION :  2; //!< 10-11 - Resolution control
    uint16_t READPATTERN:  1; //!< 12    - Reading pattern: '1' = Chess pattern (default) ; '0' = Interleaved (TV) mode
    uint16_t            :  3; //!< 13-15
  } Bits;
} MLX90640_Control1;
UNPACKITEM;
ControlItemSize(MLX90640_Control1, 2);

#define MLX90640_SUBPAGE_MODE_ENABLE     (0x1u << 0) //!< Enable subpade mode (default)
#define MLX90640_SUBPAGE_MODE_DISABLE    (0x0u << 0) //!< Disable subpade mode. No subpages, only one page will be measured
#define MLX90640_DATA_HOLD_ENABLE        (0x1u << 2) //!< Enable the data hold. Transfer the data into storage RAM only if en_overwrite = 1 (check 0x8000)
#define MLX90640_DATA_HOLD_DISABLE       (0x0u << 2) //!< Disable the data hold. Transfer the data into storage RAM at each measured frame (default)
#define MLX90640_SUBPAGE_REPEAT_ENABLE   (0x1u << 3) //!< Enable subpade repeat. Select subpage determines which subpage to be measured if Enable subpages mode = "1"
#define MLX90640_SUBPAGE_REPEAT_DISABLE  (0x0u << 3) //!< Disable subpade repeat. Toggles between subpage "0" and subpage "1" if Enable subpages mode = "1" (default)

//! Select subpage enum
typedef enum
{
  MLX90640_SUBPAGE_0_IS_SELECTED = 0b0, //!< Subpage 0 is selected (default)
  MLX90640_SUBPAGE_1_IS_SELECTED = 0b1, //!< Subpage 1 is selected
} eMLX90640_SelectSubpage;

#define MLX90640_SELECT_SUBPAGE_Pos         4
#define MLX90640_SELECT_SUBPAGE_Mask        (0x7u << MLX90640_SELECT_SUBPAGE_Pos)
#define MLX90640_SELECT_SUBPAGE_SET(value)  (((uint16_t)(value) << MLX90640_SELECT_SUBPAGE_Pos) & MLX90640_SELECT_SUBPAGE_Mask) //!< Set selected subpage
#define MLX90640_SELECT_SUBPAGE_GET(value)  (((uint16_t)(value) & MLX90640_SELECT_SUBPAGE_Mask) >> MLX90640_SELECT_SUBPAGE_Pos) //!< Get selected subpage

//! Refresh rate control enum
typedef enum
{
  MLX90640_IR_REFRESH_RATE_0Hz5 = 0b000, //!< IR refresh rate = 0.5Hz
  MLX90640_IR_REFRESH_RATE_1Hz  = 0b001, //!< IR refresh rate = 1Hz
  MLX90640_IR_REFRESH_RATE_2Hz  = 0b010, //!< IR refresh rate = 2Hz (default)
  MLX90640_IR_REFRESH_RATE_4Hz  = 0b011, //!< IR refresh rate = 4Hz
  MLX90640_IR_REFRESH_RATE_8Hz  = 0b100, //!< IR refresh rate = 8Hz
  MLX90640_IR_REFRESH_RATE_16Hz = 0b101, //!< IR refresh rate = 16Hz
  MLX90640_IR_REFRESH_RATE_32Hz = 0b110, //!< IR refresh rate = 32Hz
  MLX90640_IR_REFRESH_RATE_64Hz = 0b111, //!< IR refresh rate = 64Hz
} eMLX90640_RefreshRate;

#define MLX90640_IR_REFRESH_RATE_Pos         7
#define MLX90640_IR_REFRESH_RATE_Mask        (0x7u << MLX90640_IR_REFRESH_RATE_Pos)
#define MLX90640_IR_REFRESH_RATE_SET(value)  (((uint16_t)(value) << MLX90640_IR_REFRESH_RATE_Pos) & MLX90640_IR_REFRESH_RATE_Mask) //!< Set IR refresh rate
#define MLX90640_IR_REFRESH_RATE_GET(value)  (((uint16_t)(value) & MLX90640_IR_REFRESH_RATE_Mask) >> MLX90640_IR_REFRESH_RATE_Pos) //!< Get IR refresh rate

//! ADC resolution control enum
typedef enum
{
  MLX90640_ADC_RESOLUTION_16bits = 0b00, //!< ADC set to 16-bit resolution
  MLX90640_ADC_RESOLUTION_17bits = 0b01, //!< ADC set to 17-bit resolution
  MLX90640_ADC_RESOLUTION_18bits = 0b10, //!< ADC set to 18-bit resolution (default)
  MLX90640_ADC_RESOLUTION_19bits = 0b11, //!< ADC set to 19-bit resolution
} eMLX90640_ADCresolution;

#define MLX90640_ADC_RESOLUTION_Pos         10
#define MLX90640_ADC_RESOLUTION_Mask        (0x3u << MLX90640_ADC_RESOLUTION_Pos)
#define MLX90640_ADC_RESOLUTION_SET(value)  (((uint16_t)(value) << MLX90640_ADC_RESOLUTION_Pos) & MLX90640_ADC_RESOLUTION_Mask) //!< Set ADC resolution
#define MLX90640_ADC_RESOLUTION_GET(value)  (((uint16_t)(value) & MLX90640_ADC_RESOLUTION_Mask) >> MLX90640_ADC_RESOLUTION_Pos) //!< Get ADC resolution

//! Reading pattern enum
typedef enum
{
  MLX90640_READING_INTERLEAVE_MODE    = 0b0, //!< Reading in Interleaved (TV) mode
  MLX90640_READING_CHESS_PATTERN_MODE = 0b1, //!< Reading in Chess pattern (default)
} eMLX90640_ReadingPattern;

#define MLX90640_READING_PATTERN_Pos         12
#define MLX90640_READING_PATTERN_Mask        (0x1u << MLX90640_READING_PATTERN_Pos)
#define MLX90640_READING_PATTERN_SET(value)  (((uint16_t)(value) << MLX90640_READING_PATTERN_Pos) & MLX90640_READING_PATTERN_Mask) //!< Set reading pattern
#define MLX90640_READING_PATTERN_GET(value)  (((uint16_t)(value) & MLX90640_READING_PATTERN_Mask) >> MLX90640_READING_PATTERN_Pos) //!< Get reading pattern
#define MLX90640_READING_CHESS_PATTERN       (0x1u << MLX90640_READING_PATTERN_Pos) //!< Chess pattern (default)
#define MLX90640_READING_INTERLEAVED         (0x0u << MLX90640_READING_PATTERN_Pos) //!< Interleaved (TV) mode
#define MLX90640_INTERLEAVED_MODE            ( (uint8_t)MLX90640_READING_INTERLEAVED ) //!< Interleaved (TV) mode pattern

//-----------------------------------------------------------------------------

//! Subpage modes
typedef enum
{
  MLX90640_MEASURE_ONLY_SUBPAGE0      = 0x0, //!< Measure only subpage 0
  MLX90640_MEASURE_ONLY_SUBPAGE1      = 0x1, //!< Measure only subpage 1
  MLX90640_MEASURE_ALTERNATE_SUBPAGES = 0x2, //!< Measure alternate between subpage 0 and subpage 1 (0->1->0->1...)
} eMLX90640_SubpageMode;

//-----------------------------------------------------------------------------



//! I2C Configuration Register (Reg: 0x800F ; EEPROM: 0x240E) [default: 0x0000]
PACKITEM
typedef union __PACKED__ MLX90640_I2Cconfig
{
  uint16_t I2Cconfig;
  struct
  {
    uint16_t FMp       :  1; //!< 0    - FM+ mode: '1' = FM+ mode disabled ; '0' = FM+ mode enabled (default)
    uint16_t THRESOLD  :  1; //!< 1    - I2C threshold levels: '1' = 1.8V reffered threshold (1.8V mode) ; '0' = VDD reffered threshold (normal mode) (default)
    uint16_t SDACURRENT:  1; //!< 2    - SDA driver current limit control: '1' = SDA driver current limit is OFF ; '0' = SDA driver current limit is ON (default)
    uint16_t           : 13; //!< 3-15
  } Bits;
} MLX90640_I2Cconfig;
UNPACKITEM;
ControlItemSize(MLX90640_I2Cconfig, 2);

#define MLX90640_FMp_DISABLE                (0x1u << 0) //!< Disable FM+ mode
#define MLX90640_FMp_ENABLE                 (0x0u << 0) //!< Enable FM+ mode
#define MLX90640_THRESHOLD_1V8              (0x1u << 1) //!< 1.8V reffered threshold (1.8V mode)
#define MLX90640_THRESHOLD_VDD              (0x0u << 1) //!< VDD reffered threshold (normal mode) (default)
#define MLX90640_SDA_CURRENT_LIMIT_DISABLE  (0x1u << 2) //!< SDA driver current limit is OFF
#define MLX90640_SDA_CURRENT_LIMIT_ENABLE   (0x0u << 2) //!< SDA driver current limit is ON (default)

//-----------------------------------------------------------------------------



//! I2C Configuration Register (Reg: 0x8010 ; EEPROM: 0x240F) [default: 0xBE33]
PACKITEM
typedef union __PACKED__ MLX90640_I2Caddress
{
  uint16_t I2Caddress;
  struct
  {
    uint16_t ADDRESS : 8; //!< 0- 7 - I2C Address
    uint16_t RESERVED: 8; //!< 8-15
  } Bits;
} MLX90640_I2Caddress;
UNPACKITEM;
ControlItemSize(MLX90640_I2Caddress, 2);

#define MLX90640_I2C_ADDRESS_Pos            0
#define MLX90640_I2C_ADDRESS_Mask           (0xFFu << MLX90640_I2C_ADDRESS_Pos)
#define MLX90640_I2C_ADDRESS_SET(value)     (((uint16_t)(value) << MLX90640_I2C_ADDRESS_Pos) & MLX90640_I2C_ADDRESS_Mask) //!< Set I2C address
#define MLX90640_I2C_ADDRESS_GET(value)     (((uint16_t)(value) & MLX90640_I2C_ADDRESS_Mask) >> MLX90640_I2C_ADDRESS_Pos) //!< Get I2C address

//-----------------------------------------------------------------------------



// Device option Register (EEPROM: 0x240A)
#define MLX90640_CALIBRATION_MODE_Pos         11
#define MLX90640_CALIBRATION_MODE_Mask        (0x1u << MLX90640_CALIBRATION_MODE_Pos)
#define MLX90640_CALIBRATION_MODE_GET(value)  (((uint16_t)(value) & MLX90640_CALIBRATION_MODE_Mask) >> MLX90640_CALIBRATION_MODE_Pos) //!< Get calibration mode
#define MLX90640_FOV_Mask  ( 0x000C )

//! List of supported devices
typedef enum
{
  MLX90640BAA,            //!< MLX90640BAA - FOV = 110°x75°
  MLX90640BAB,            //!< MLX90640BAB - FOV = 55°x35°
  eMLX90640_DEVICE_COUNT, // Device count of this enum, keep last
} eMLX90640_Devices;

static const char* const MLX90640_DevicesNames[eMLX90640_DEVICE_COUNT] =
{
  "MLX90640BAA",
  "MLX90640BAB",
};

//-----------------------------------------------------------------------------



//! Scale OCC Register (EEPROM: 0x2410)
PACKITEM
typedef union __PACKED__ MLX90640_ScaleOCC
{
  uint16_t ScaleOCC;
  struct
  {
    uint16_t SCALE_REM  :  4; //!<  0- 3 - Scale_Occ_rem
    uint16_t SCALE_ROW  :  4; //!<  4- 7 - Scale_Occ_col
    uint16_t SCALE_COL  :  4; //!<  8-11 - Scale_Occ_row
    uint16_t ALPHA_PTAT :  4; //!< 12-15 - (Alpha PTAT - 8)*4
  } Bits;
} MLX90640_ScaleOCC;
UNPACKITEM;
ControlItemSize(MLX90640_ScaleOCC, 2);

#define MLX90640_SCALE_OCC_REM_Pos         0
#define MLX90640_SCALE_OCC_REM_Mask        (0xFu << MLX90640_SCALE_OCC_REM_Pos)
#define MLX90640_SCALE_OCC_REM_GET(value)  (uint8_t)(((uint16_t)(value) & MLX90640_SCALE_OCC_REM_Mask) >> MLX90640_SCALE_OCC_REM_Pos) //!< Get Scale OCC remnant
#define MLX90640_SCALE_OCC_COL_Pos         4
#define MLX90640_SCALE_OCC_COL_Mask        (0xFu << MLX90640_SCALE_OCC_COL_Pos)
#define MLX90640_SCALE_OCC_COL_GET(value)  (uint8_t)(((uint16_t)(value) & MLX90640_SCALE_OCC_COL_Mask) >> MLX90640_SCALE_OCC_COL_Pos) //!< Get Scale OCC column
#define MLX90640_SCALE_OCC_ROW_Pos         8
#define MLX90640_SCALE_OCC_ROW_Mask        (0xFu << MLX90640_SCALE_OCC_ROW_Pos)
#define MLX90640_SCALE_OCC_ROW_GET(value)  (uint8_t)(((uint16_t)(value) & MLX90640_SCALE_OCC_ROW_Mask) >> MLX90640_SCALE_OCC_ROW_Pos) //!< Get Scale OCC row
#define MLX90640_ALPHA_PTAT_Pos            12
#define MLX90640_ALPHA_PTAT_Mask           (0xFu << MLX90640_ALPHA_PTAT_Pos)
#define MLX90640_ALPHA_PTAT_GET(value)     (uint8_t)(((uint16_t)(value) & MLX90640_ALPHA_PTAT_Mask) >> MLX90640_ALPHA_PTAT_Pos)       //!< Get (Alpha PTAT - 8)*4

//-----------------------------------------------------------------------------



//! Row + column Register (EEPROM: for OCC 0x2412..0x241F ; for ACC 0x2422.. 0x242F)
PACKITEM
typedef union __PACKED__ MLX90640_RowColumn
{
  uint16_t Data[(MLX90640_ROW_COUNT / 4) + (MLX90640_COL_COUNT / 4)];
  struct
  {
    union
    {
      uint16_t Row[MLX90640_ROW_COUNT / 4];
      struct
      {
        int16_t ROW1 : 4; //!<  0- 3 - Row 1
        int16_t ROW2 : 4; //!<  4- 7 - Row 2
        int16_t ROW3 : 4; //!<  8-11 - Row 3
        int16_t ROW4 : 4; //!< 12-15 - Row 4
        int16_t ROW5 : 4; //!<  0- 3 - Row 5
        int16_t ROW6 : 4; //!<  4- 7 - Row 6
        int16_t ROW7 : 4; //!<  8-11 - Row 7
        int16_t ROW8 : 4; //!< 12-15 - Row 8
        int16_t ROW9 : 4; //!<  0- 3 - Row 9
        int16_t ROW10: 4; //!<  4- 7 - Row 10
        int16_t ROW11: 4; //!<  8-11 - Row 11
        int16_t ROW12: 4; //!< 12-15 - Row 12
        int16_t ROW13: 4; //!<  0- 3 - Row 13
        int16_t ROW14: 4; //!<  4- 7 - Row 14
        int16_t ROW15: 4; //!<  8-11 - Row 15
        int16_t ROW16: 4; //!< 12-15 - Row 16
        int16_t ROW17: 4; //!<  0- 3 - Row 17
        int16_t ROW18: 4; //!<  4- 7 - Row 18
        int16_t ROW19: 4; //!<  8-11 - Row 19
        int16_t ROW20: 4; //!< 12-15 - Row 20
        int16_t ROW21: 4; //!<  0- 3 - Row 21
        int16_t ROW22: 4; //!<  4- 7 - Row 22
        int16_t ROW23: 4; //!<  8-11 - Row 23
        int16_t ROW24: 4; //!< 12-15 - Row 24
      } Bits;
    } Row;
    union
    {
      uint16_t Column[MLX90640_COL_COUNT / 4];
      struct
      {
        int16_t COL1 : 4; //!<  0- 3 - Column 1
        int16_t COL2 : 4; //!<  4- 7 - Column 2
        int16_t COL3 : 4; //!<  8-11 - Column 3
        int16_t COL4 : 4; //!< 12-15 - Column 4
        int16_t COL5 : 4; //!<  0- 3 - Column 5
        int16_t COL6 : 4; //!<  4- 7 - Column 6
        int16_t COL7 : 4; //!<  8-11 - Column 7
        int16_t COL8 : 4; //!< 12-15 - Column 8
        int16_t COL9 : 4; //!<  0- 3 - Column 9
        int16_t COL10: 4; //!<  4- 7 - Column 10
        int16_t COL11: 4; //!<  8-11 - Column 11
        int16_t COL12: 4; //!< 12-15 - Column 12
        int16_t COL13: 4; //!<  0- 3 - Column 13
        int16_t COL14: 4; //!<  4- 7 - Column 14
        int16_t COL15: 4; //!<  8-11 - Column 15
        int16_t COL16: 4; //!< 12-15 - Column 16
        int16_t COL17: 4; //!<  0- 3 - Column 17
        int16_t COL18: 4; //!<  4- 7 - Column 18
        int16_t COL19: 4; //!<  8-11 - Column 19
        int16_t COL20: 4; //!< 12-15 - Column 20
        int16_t COL21: 4; //!<  0- 3 - Column 21
        int16_t COL22: 4; //!<  4- 7 - Column 22
        int16_t COL23: 4; //!<  8-11 - Column 23
        int16_t COL24: 4; //!< 12-15 - Column 24
        int16_t COL25: 4; //!<  0- 3 - Column 25
        int16_t COL26: 4; //!<  4- 7 - Column 26
        int16_t COL27: 4; //!<  8-11 - Column 27
        int16_t COL28: 4; //!< 12-15 - Column 28
        int16_t COL29: 4; //!<  0- 3 - Column 29
        int16_t COL30: 4; //!<  4- 7 - Column 30
        int16_t COL31: 4; //!<  8-11 - Column 31
        int16_t COL32: 4; //!< 12-15 - Column 32
      } Bits;
    } Column;
  };
} MLX90640_RowColumn;
UNPACKITEM;
ControlItemSize(MLX90640_RowColumn, 28); // 12 + 16

#define MLX90640_EXTRACT_NIBBLE_0(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,4)
#define MLX90640_EXTRACT_NIBBLE_1(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,4,4)
#define MLX90640_EXTRACT_NIBBLE_2(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,8,4)
#define MLX90640_EXTRACT_NIBBLE_3(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,12,4)

//-----------------------------------------------------------------------------



//! Scale ACC Register (EEPROM: 0x2420)
PACKITEM
typedef union __PACKED__ MLX90640_ScaleACC
{
  uint16_t ScaleACC;
  struct
  {
    uint16_t SCALE_REM  :  4; //!<  0- 3 - scale_Acc_rem
    uint16_t SCALE_ROW  :  4; //!<  4- 7 - scale_Acc_col
    uint16_t SCALE_COL  :  4; //!<  8-11 - scale_Acc_row
    uint16_t ALPHA_SCALE:  4; //!< 12-15 - Alpha scale - 30
  } Bits;
} MLX90640_ScaleACC;
UNPACKITEM;
ControlItemSize(MLX90640_ScaleACC, 2);

#define MLX90640_SCALE_ACC_REM_Pos         0
#define MLX90640_SCALE_ACC_REM_Mask        (0xFu << MLX90640_SCALE_ACC_REM_Pos)
#define MLX90640_SCALE_ACC_REM_GET(value)  (((uint16_t)(value) & MLX90640_SCALE_ACC_REM_Mask) >> MLX90640_SCALE_ACC_REM_Pos) //!< Get Scale_Acc_rem
#define MLX90640_SCALE_ACC_COL_Pos         4
#define MLX90640_SCALE_ACC_COL_Mask        (0xFu << MLX90640_SCALE_ACC_COL_Pos)
#define MLX90640_SCALE_ACC_COL_GET(value)  (((uint16_t)(value) & MLX90640_SCALE_ACC_COL_Mask) >> MLX90640_SCALE_ACC_COL_Pos) //!< Get Scale_Acc_col
#define MLX90640_SCALE_ACC_ROW_Pos         8
#define MLX90640_SCALE_ACC_ROW_Mask        (0xFu << MLX90640_SCALE_ACC_ROW_Pos)
#define MLX90640_SCALE_ACC_ROW_GET(value)  (((uint16_t)(value) & MLX90640_SCALE_ACC_ROW_Mask) >> MLX90640_SCALE_ACC_ROW_Pos) //!< Scale_Acc_row
#define MLX90640_ALPHA_SCALE_Pos           12
#define MLX90640_ALPHA_SCALE_Mask          (0xFu << MLX90640_ALPHA_SCALE_Pos)
#define MLX90640_ALPHA_SCALE_GET(value)    (((uint16_t)(value) & MLX90640_ALPHA_SCALE_Mask) >> MLX90640_ALPHA_SCALE_Pos)     //!< Get Alpha scale - 30

//-----------------------------------------------------------------------------



//! K PTAT Register (EEPROM: 0x2432)
PACKITEM
typedef union __PACKED__ MLX90640_KPTAT
{
  int16_t KPTAT;
  struct
  {
    int16_t Kt_PTAT: 10; //!<  0- 9 - Kt PTAT
    int16_t Kv_PTAT:  6; //!< 10-15 - Kv PTAT
  } Bits;
} MLX90640_KPTAT;
UNPACKITEM;
ControlItemSize(MLX90640_KPTAT, 2);

#define MLX90640_Kt_PTAT_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,10) //!< Get Kt PTAT
#define MLX90640_Kv_PTAT_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,10,6) //!< Get Kv PTAT

//-----------------------------------------------------------------------------



// Vdd Register (EEPROM: 0x2433)
#define MLX90640_Vdd_25_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get Vdd25
#define MLX90640_Kv_Vdd_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get Kv Vdd

//-----------------------------------------------------------------------------



//! Kv Average Register (EEPROM: 0x2434)
PACKITEM
typedef union __PACKED__ MLX90640_KvAvg
{
  uint16_t KvAvg;
  struct
  {
    uint16_t KvAvg_RowEven_ColEven: 4; //!<  0- 3 - Kv Average on Row Even and Column Even
    uint16_t KvAvg_RowOdd_ColEven : 4; //!<  4- 7 - Kv Average on Row Odd and Column Even
    uint16_t KvAvg_RowEven_ColOdd : 4; //!<  8-11 - Kv Average on Row Even and Column Odd
    uint16_t KvAvg_RowOdd_ColOdd  : 4; //!< 12-15 - Kv Average on Row Odd and Column Odd
  } Bits;
} MLX90640_KvAvg;
UNPACKITEM;
ControlItemSize(MLX90640_KvAvg, 2);

#define MLX90640_KvAVG_ROW_EVEN_COL_EVEN_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,4)  //!< Get Kv Average on Row Even and Column Even
#define MLX90640_KvAVG_ROW_ODD_COL_EVEN_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,4,4)  //!< Get Kv Average on Row Odd and Column Even
#define MLX90640_KvAVG_ROW_EVEN_COL_ODD_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,8,4)  //!< Get Kv Average on Row Even and Column Odd
#define MLX90640_KvAVG_ROW_ODD_COL_ODD_GET(value)    MLX90640_DATA_EXTRACT_TO_INT16(value,12,4) //!< Get Kv Average on Row Odd and Column Odd

//-----------------------------------------------------------------------------



#define MLX90640_C1  ( 0 )
#define MLX90640_C2  ( 1 )
#define MLX90640_C3  ( 2 )

//! IL Chess Register (EEPROM: 0x2435)
PACKITEM
typedef union __PACKED__ MLX90640_ILchess
{
  uint16_t ILchess;
  struct
  {
    uint16_t IL_CHESS_C1: 6; //!<  0- 5 - IL Chess C1
    uint16_t IL_CHESS_C2: 5; //!<  6-10 - IL Chess C2
    uint16_t IL_CHESS_C3: 5; //!< 11-15 - IL Chess C3
  } Bits;
} MLX90640_ILchess;
UNPACKITEM;
ControlItemSize(MLX90640_ILchess, 2);

#define MLX90640_IL_CHESS_C1_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,6)  //!< Get IL Chess C1
#define MLX90640_IL_CHESS_C2_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,6,5)  //!< Get IL Chess C2
#define MLX90640_IL_CHESS_C3_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,11,5) //!< Get IL Chess C3

//-----------------------------------------------------------------------------



// Kta Average Column Odd Register (EEPROM: 0x2436)
#define MLX90640_KtaAvg_ROW_EVEN_COL_ODD_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get Kta Average Row Even Column Odd
#define MLX90640_KtaAvg_ROW_ODD_COL_ODD_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get Kta Average Row Odd Column Odd

//-----------------------------------------------------------------------------



// Kta Average Column Even Register (EEPROM: 0x2437)
#define MLX90640_KtaAvg_ROW_EVEN_COL_EVEN_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get Kta Average Row Even Column Even
#define MLX90640_KtaAvg_ROW_ODD_COL_EVEN_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get Kta Average Row Odd Column Even

//-----------------------------------------------------------------------------



//! K scale Register (EEPROM: 0x2438)
PACKITEM
typedef union __PACKED__ MLX90640_Kscale
{
  uint16_t Kscale;
  struct
  {
    int16_t  Kta_Scale2: 4; //!<  0- 3 - Kta Scale2
    int16_t  Kta_Scale1: 4; //!<  4- 7 - Kta Scale1
    int16_t  Kv_Scale  : 4; //!<  8-11 - Kv Scale
    uint16_t ResCtrlCal: 2; //!< 12-13 - Res control calib
    uint16_t           : 2; //!< 14-15
  } Bits;
} MLX90640_Kscale;
UNPACKITEM;
ControlItemSize(MLX90640_Kscale, 2);

#define MLX90640_Kta_SCALE2_Pos           0
#define MLX90640_Kta_SCALE2_Mask          (0xFu << MLX90640_Kta_SCALE2_Pos)
#define MLX90640_Kta_SCALE2_GET(value)    (((uint16_t)(value) & MLX90640_Kta_SCALE2_Mask) >> MLX90640_Kta_SCALE2_Pos)    //!< Get Kta Scale 2
#define MLX90640_Kta_SCALE1_Pos           4
#define MLX90640_Kta_SCALE1_Mask          (0xFu << MLX90640_Kta_SCALE1_Pos)
#define MLX90640_Kta_SCALE1_GET(value)    (((uint16_t)(value) & MLX90640_Kta_SCALE1_Mask) >> MLX90640_Kta_SCALE1_Pos)    //!< Get Kta Scale 1
#define MLX90640_Kv_SCALE_Pos             8
#define MLX90640_Kv_SCALE_Mask            (0xFu << MLX90640_Kv_SCALE_Pos)
#define MLX90640_Kv_SCALE_GET(value)      (((uint16_t)(value) & MLX90640_Kv_SCALE_Mask) >> MLX90640_Kv_SCALE_Pos)        //!< Get Kv Scale
#define MLX90640_RES_CTRL_CAL_Pos         12
#define MLX90640_RES_CTRL_CAL_Mask        (0x3u << MLX90640_RES_CTRL_CAL_Pos)
#define MLX90640_RES_CTRL_CAL_GET(value)  (uint8_t)(((value) & MLX90640_RES_CTRL_CAL_Mask) >> MLX90640_RES_CTRL_CAL_Pos) //!< Get Res control calib

//-----------------------------------------------------------------------------



//! Alpha Compensation Pixel subpage Register (EEPROM: 0x2439)
PACKITEM
typedef union __PACKED__ MLX90640_AlphaCP
{
  uint16_t AlphaCPsubpage;
  struct
  {
    uint16_t AlphaCPsubpage0  : 10; //!<  0- 9 - Alpha CP subpage_0
    uint16_t AlphaCP_P1P0ratio:  6; //!< 10-15 - Alpha CP subpage_1/subpage_0 ratio (CP subpage_1 / CP subpage_0 - 1)*2^7
  } Bits;
} MLX90640_AlphaCP;
UNPACKITEM;
ControlItemSize(MLX90640_AlphaCP, 2);

#define MLX90640_ALPHA_CP_SUBPAGE0_Pos           0
#define MLX90640_ALPHA_CP_SUBPAGE0_Mask          (0x3FFu << MLX90640_ALPHA_CP_SUBPAGE0_Pos)
#define MLX90640_ALPHA_CP_SUBPAGE0_GET(value)    (((uint16_t)(value) & MLX90640_ALPHA_CP_SUBPAGE0_Mask) >> MLX90640_ALPHA_CP_SUBPAGE0_Pos) //!< Get Alpha CP subpage_0
#define MLX90640_ALPHA_CP_P1P0_RATIO_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,10,6)                                                //!< Get Alpha CP subpage_1/subpage_0 ratio (CP subpage_1 / CP subpage_0 - 1)*2^7

//-----------------------------------------------------------------------------



//! Offset Compensation Pixel subpage Register (EEPROM: 0x243A)
PACKITEM
typedef union __PACKED__ MLX90640_OffsetCP
{
  uint16_t OffsetCPsubpage;
  struct
  {
    uint16_t OffsetCPsubpage   : 10; //!<  0- 9 - Offset CP subpage_0
    uint16_t OffsetCP_P1P0delta:  6; //!< 10-15 - Offset delta (CP subpage_1 - CP subpage_0)
  } Bits;
} MLX90640_OffsetCP;
UNPACKITEM;
ControlItemSize(MLX90640_OffsetCP, 2);

#define MLX90640_OFFSET_CP_SUBPAGE_GET(value)     MLX90640_DATA_EXTRACT_TO_INT16(value,0,10) //!< Get Offset CP subpage_0
#define MLX90640_OFFSET_CP_P1P0_DELTA_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,10,6) //!< Get Offset delta (CP subpage_1 - CP subpage_0)

//-----------------------------------------------------------------------------



// Kta Compensation Pixel & Kv Compensation Pixel Register (EEPROM: 0x243B)
#define MLX90640_Kta_CP_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get Kta Compensation Pixel
#define MLX90640_Kv_CP_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get Kv Compensation Pixel

//-----------------------------------------------------------------------------



// Temperature Gradient Coefficient & KsTa Register (EEPROM: 0x243C)
#define MLX90640_TGC_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get Temperature Gradient Coefficient (±4)*2^7
#define MLX90640_KsTa_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get KsTa*2^13

//-----------------------------------------------------------------------------



#define MLX90640_KsTo1  ( 0 )
#define MLX90640_KsTo2  ( 1 )

// KsTo range 1 (<0°C) & range 2 (0°C..CT1°C) Register (EEPROM: 0x243D)
#define MLX90640_KsTo_CT_RANGE_1_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get KsTo Corner Temperature range 1 (<0°C)
#define MLX90640_KsTo_CT_RANGE_2_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get KsTo Corner Temperature range 2 (0°C..CT1°C)

//-----------------------------------------------------------------------------



#define MLX90640_KsTo3  ( 2 )
#define MLX90640_KsTo4  ( 3 )

// KsTo range 3 (CT1°C..CT2°C) & range 4 (CT2°C..) Register (EEPROM: 0x243E)
#define MLX90640_KsTo_CT_RANGE_3_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,0,8) //!< Get KsTo Corner Temperature range 3 (CT1°C..CT2°C)
#define MLX90640_KsTo_CT_RANGE_4_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,8,8) //!< Get KsTo Corner Temperature range 4 (CT2°C..)

//-----------------------------------------------------------------------------



#define MLX90640_CT1  ( 0 )
#define MLX90640_CT2  ( 1 )
#define MLX90640_CT3  ( 2 )
#define MLX90640_CT4  ( 3 )

//! CT and Temp Register (EEPROM: 0x243F)
PACKITEM
typedef union __PACKED__ MLX90640_CTtemp
{
  uint16_t CTtemp;
  struct
  {
    uint16_t KsToScaleOff: 4; //!<  0- 3 - KsTo Scale offset - 8
    uint16_t CT3         : 4; //!<  4- 7 - CT3
    uint16_t CT4         : 4; //!<  8-11 - CT4
    uint16_t TempStep    : 2; //!< 12-13 - Temp Step
    uint16_t             : 2; //!< 14-15
  } Bits;
} MLX90640_CTtemp;
UNPACKITEM;
ControlItemSize(MLX90640_CTtemp, 2);

#define MLX90640_KsTo_SCALE_OFFSET_Pos         0
#define MLX90640_KsTo_SCALE_OFFSET_Mask        (0xFu << MLX90640_KsTo_SCALE_OFFSET_Pos)
#define MLX90640_KsTo_SCALE_OFFSET_GET(value)  (((uint16_t)(value) & MLX90640_KsTo_SCALE_OFFSET_Mask) >> MLX90640_KsTo_SCALE_OFFSET_Pos) //!< Get KsTo Scale offset - 8
#define MLX90640_CT3_Pos                       4
#define MLX90640_CT3_Mask                      (0xFu << MLX90640_CT3_Pos)
#define MLX90640_CT3_GET(value)                (((uint16_t)(value) & MLX90640_CT3_Mask) >> MLX90640_CT3_Pos)             //!< Get CT3
#define MLX90640_CT4_Pos                       8
#define MLX90640_CT4_Mask                      (0xFu << MLX90640_CT4_Pos)
#define MLX90640_CT4_GET(value)                (((uint16_t)(value) & MLX90640_CT4_Mask) >> MLX90640_CT4_Pos)             //!< Get CT4
#define MLX90640_TEMP_STEP_Pos                 12
#define MLX90640_TEMP_STEP_Mask                (0x3u << MLX90640_TEMP_STEP_Pos)
#define MLX90640_TEMP_STEP_GET(value)          (((uint16_t)(value) & MLX90640_TEMP_STEP_Mask) >> MLX90640_TEMP_STEP_Pos) //!< Get Temp Step

//-----------------------------------------------------------------------------



//! Pixel data Register (EEPROM: 0x2440..0x273F)
PACKITEM
typedef union __PACKED__ MLX90640_PixelData
{
  uint16_t Pixel;
  struct
  {
    uint16_t Outlier  : 1; //!<  0    - Outlier
    uint16_t Kta      : 3; //!<  1- 3 - Pixel Kta
    uint16_t AlphaPix : 6; //!<  4- 9 - Alpha pixel
    uint16_t OffsetPix: 6; //!< 10-15 - Offset pixel
  } Bits;
} MLX90640_PixelData;
UNPACKITEM;
ControlItemSize(MLX90640_PixelData, 2);

#define MLX90640_PIXEL_IS_BROKEN(value)   ( (value) == 0x0000 )        //!< The pixel is considered as broken i.e. the pixel has failed
#define MLX90640_PIXEL_IS_OUTLIER(value)  ( ((value) & (1 << 0)) > 0 ) //!< The pixel is considered as Outlier i.e. the sensor accuracy is not guaranteed by the calibration
#define MLX90640_Kta_GET(value)           MLX90640_DATA_EXTRACT_TO_INT16(value,1,3)  //!< Get pixel Kta
#define MLX90640_ALPHA_PIXEL_GET(value)   MLX90640_DATA_EXTRACT_TO_INT16(value,4,6)  //!< Get Alpha pixel
#define MLX90640_OFFSET_PIXEL_GET(value)  MLX90640_DATA_EXTRACT_TO_INT16(value,10,6) //!< Get Offset pixel

//********************************************************************************************************************

//! Type of pixel fault enum
typedef enum
{
  MLX90640_NOT_DETECTIVE, //!< Pixel is not effective
  MLX90640_BROKEN ,       //!< Broken pixel
  MLX90640_OUTLIER,       //!< Outlier pixel
} eMLX90640_FaultType;

//! Defective pixels informations
typedef struct MLX90640_DefectPixels
{
  eMLX90640_FaultType Type; //!< Type of pixel fault
  uint8_t X;                //!< Column position of the pixel
  uint8_t Y;                //!< Row position of the pixel
} MLX90640_DefectPixels;

//********************************************************************************************************************



//! EEPROM registers (EEPROM start address: 0x2400 ; each address is 16-bits)
PACKITEM
typedef union __PACKED__ MLX90640_EEPROM
{
  uint16_t Words[ 0x2740 - 0x2400     ]; //!< EEPROM words: starts at address 0x2400 and finish at address 0x2740
  uint8_t  Bytes[(0x2740 - 0x2400) * 2]; //!< EEPROM bytes: starts at address 0x2400 and finish at address 0x2740
  struct
  {
    uint16_t OscTrim;            //!< (Offset16: 0x00) Oscillator Trim
    uint16_t AnaTrim;            //!< (Offset16: 0x01) Analog Trim
    uint16_t Reserved0;
    MLX90640_Status ConfReg;     //!< (Offset16: 0x03) Configuration
    uint16_t Reserved1[3];
    uint16_t ID[3];              //!< (Offset16: 0x07) Device identification
    uint16_t DeviceOptions;      //!< (Offset16: 0x0A) Device options
    uint16_t Reserved2;
    MLX90640_Control1 ContReg1;  //!< (Offset16: 0x0C) Control 1
    uint16_t ContReg2;           //!< (Offset16: 0x0D) Control 2 (not documented)
    MLX90640_I2Cconfig I2CConf;  //!< (Offset16: 0x0E) I2C Configuration
    MLX90640_I2Caddress I2CAddr; //!< (Offset16: 0x0F) I2C Address
    MLX90640_ScaleOCC ScaleOCC;  //!< (Offset16: 0x10) Scale OCC
    int16_t PixOsAvg;            //!< (Offset16: 0x11) Pix Os Average
    MLX90640_RowColumn Occ;      //!< (Offset16: 0x12) OCC row + column
    MLX90640_ScaleACC ScaleACC;  //!< (Offset16: 0x20) Scale ACC
    int16_t PixSensitivityAvg;   //!< (Offset16: 0x21) Pix Sensitivity Average
    MLX90640_RowColumn Acc;      //!< (Offset16: 0x22) ACC row + column
    int16_t Gain;                //!< (Offset16: 0x30) Gain
    int16_t PTAT_25;             //!< (Offset16: 0x31) PTAT25
    MLX90640_KPTAT K_PTAT;       //!< (Offset16: 0x32) K PTAT
    uint16_t Vdd;                //!< (Offset16: 0x33) KvVdd & Vdd25
    MLX90640_KvAvg Kv_Avg;       //!< (Offset16: 0x34) Kv Avg
    MLX90640_ILchess IL_Chess;   //!< (Offset16: 0x35) IL Chess
    uint16_t KtaAvg_ColumnOdd;   //!< (Offset16: 0x36) Kta Average Row Even Column Odd & Kta Average Row Odd Column Odd
    uint16_t KtaAvg_ColumnEven;  //!< (Offset16: 0x37) Kta Average Row Even Column Even & Kta Average Row Odd Column Even
    MLX90640_Kscale K_Scale;     //!< (Offset16: 0x38) Kta, Kv Scale and Res control calib
    MLX90640_AlphaCP Alpha_CP;   //!< (Offset16: 0x39) Alpha Compensation Pixel subpage
    MLX90640_OffsetCP Offset_CP; //!< (Offset16: 0x3A) Offset Compensation Pixel subpage
    uint16_t K_CP;               //!< (Offset16: 0x3B) Kta Compensation Pixel & Kv Compensation Pixel
    uint16_t TGC_KsTa;           //!< (Offset16: 0x3C) Temperature Gradient Coefficient (±4)*2^7 & KsTa*2^13
    uint16_t KsTo_Range1_2;      //!< (Offset16: 0x3D) KsTo range 1 (<0°C) & KsTo range 2 (0°C..CT1°C)
    uint16_t KsTo_Range3_4;      //!< (Offset16: 0x3E) KsTo range 3 (CT1°C..CT2°C) & range 4 (CT2°C..)
    MLX90640_CTtemp CT_Temp;     //!< (Offset16: 0x3F) CT and Temp
    union
    {
      MLX90640_PixelData PixelYXData[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< (Offset16: 0x40) 32x24 Pixels data from pixel (1,1) to (24x32)
      MLX90640_PixelData PixelData[MLX90640_TOTAL_PIXELS_COUNT];              //!< (Offset16: 0x40) 768 Pixels data from pixel (1,1) to (24x32)
    };
  };
} MLX90640_EEPROM;
UNPACKITEM;
ControlItemSize(MLX90640_EEPROM, 1664); // Start at address 0x2400 and finish at address 0x2740 => 0x340 => 832*2 for bytes

#define MLX90640_PIXEL_DATA_AT(row,column)     ( (PixelData[(((row) - 1) * 32) + ((column) - 1)] ) //!< Get the pixel in PixelData[]. 'row' should be > 0 and <= 24. 'column' should be > 0 and <= 32
#define MLX90640_PIXEL_YX_DATA_AT(row,column)  ( (PixelYXData[(row) - 1][(column) - 1] )           //!< Get the pixel in PixelYXData[][]. 'row' should be > 0 and <= 24. 'column' should be > 0 and <= 32

//********************************************************************************************************************



//! Device pre-calculated data extracted from EEPROM data (need for each subpage data)
typedef struct MLX90640_Parameters
{
  // Gain parameter
  int16_t Gain;       //!< Gain parameter for gain calculation (common for all pixels)
  // Others
  uint8_t CalibrationPattern;
  // Vdd parameters
  uint8_t Resolution; //!< Resolution parameter for supply voltage value calculation
  int16_t Kv_Vdd;     //!< Kv_Vdd parameter for supply voltage value calculation
  int16_t Vdd_25;     //!< Vdd_25 parameter for supply voltage value calculation
  // PTAT parameters
  float KvPTAT;       //!< KvPTAT parameter for ambient temperature calculation
  float KtPTAT;       //!< KtPTAT parameter for ambient temperature calculation
  float AlphaPTAT;    //!< AlphaPTAT parameter for ambient temperature calculation
  uint16_t PTAT_25;   //!< V_PTAT25 parameter for ambient temperature calculation
  // Offset parameters
  int16_t CPsubpageOffset[2]; //!< Offset for compensation pixel, one for each subpages
  // IL Chess parameters
  float ILchess[3];           //!< IL Chess C1, C2 and C3 value calculation
  // Sensitivity parameters
  float CPsubpageAlpha[2]; //!< Sensitivity (Alpha) for compensation pixel, one for each subpages
  // Kta parameters
  float KtaCP;             //!< Kta compensation pixel coefficient parameter
  // Kv parameters
  float KvCP;              //!< Kv compensation pixel coefficient parameter
  // KsTa parameter
  float KsTa;              //!< KsTa coefficient parameter
  // TGC parameter
  float TGC;               //!< Temperature Gradient Coefficient parameter
  // Corner Temperature parameters
  float KsTo[4];
  int16_t CT[4];
  // Pixels defect
  MLX90640_DefectPixels DefectivePixels[MLX90640_MAX_DEFECT_PIXELS]; //!< This is the defective pixel list. The imager can have up to 4 defective pixels

  // Pre-calculated parameters
#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
  union
  {
    int16_t OffsetsYX[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< Offset calculation of all 32x24 Pixels data from pixel (1,1) to (24x32)
    int16_t Offsets[MLX90640_TOTAL_PIXELS_COUNT];              //!< Offset calculation of all 768 Pixels data from pixel (1,1) to (24x32)
  };
  union
  {
    float AlphasYX[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< Sensitivity (Alpha) calculation of all 32x24 Pixels data from pixel (1,1) to (24x32)
    float Alphas[MLX90640_TOTAL_PIXELS_COUNT];              //!< Sensitivity (Alpha) calculation of all 768 Pixels data from pixel (1,1) to (24x32)
  };
  union
  {
    float KtaCoeffYX[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< Kta coefficients calculation of all 32x24 Pixels data from pixel (1,1) to (24x32)
    float KtaCoeff[MLX90640_TOTAL_PIXELS_COUNT];              //!< Kta coefficients calculation of all 768 Pixels data from pixel (1,1) to (24x32)
  };
  union
  {
    float KvCoeffYX[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< Kv coefficients calculation of all 32x24 Pixels data from pixel (1,1) to (24x32)
    float KvCoeff[MLX90640_TOTAL_PIXELS_COUNT];              //!< Kv coefficients calculation of all 768 Pixels data from pixel (1,1) to (24x32)
  };
#endif
} MLX90640_Parameters;

//********************************************************************************************************************



//! Frame data (RAM start address: 0x0400 ; each address is 16-bits)
PACKITEM
typedef union __PACKED__ MLX90640_FrameData
{
  uint16_t Words[ 0x0740 - 0x0400 + 1     ]; //!< RAM words: starts at address 0x0400 and finish at address 0x0740 + 1 register
  uint8_t  Bytes[(0x0740 - 0x0400 + 1) * 2]; //!< RAM bytes: starts at address 0x0400 and finish at address 0x0740 + 1 register
  struct
  {
    // Frame data parameters
    union
    {
      int16_t FrameYX[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< Frame data of all 32x24 Pixels from pixel (1,1) to (24x32)
      int16_t Frame[MLX90640_TOTAL_PIXELS_COUNT];              //!< Frame data of all 768 Pixels from pixel (1,1) to (24x32)
    };
    // Auxiliary data
    union
    {
      uint16_t AuxWords[0x0740 - 0x0700 + 1]; //!< RAM words: starts at address 0x0700 and finish at address 0x0740
      struct
      {
        int16_t  VbeTa;                       //!< (Offset16: 0x300) Vbe of ambient temperature
        uint16_t Reserved0[7];
        int16_t  CPsubpage0;                  //!< (Offset16: 0x308) Coefficient Parameter pixel of subpage 0
        uint16_t Reserved1;
        int16_t  Gain;                        //!< (Offset16: 0x30A) Gain
        uint16_t Reserved2[21];
        int16_t  VtaPTAT;                     //!< (Offset16: 0x320) VPTAT of ambient temperature
        uint16_t Reserved3[7];
        int16_t  CPsubpage1;                  //!< (Offset16: 0x328) Coefficient Parameter pixel of subpage 1
        uint16_t Reserved4;
        int16_t  VddPix;                      //!< (Offset16: 0x32A) Vdd of the frame
        uint16_t Reserved5[21];
        MLX90640_Status StatusReg;            //!< (Image of 0x8000) Status register
      };
    };
  };
} MLX90640_FrameData;
UNPACKITEM;
ControlItemSize(MLX90640_FrameData, 1666); // Start at address 0x0400 and finish at address 0x0740 plus 1 register => 0x340+1 => 833*2 for bytes



//! Frame To (object temperature)
typedef struct MLX90640_FrameTo
{
  // Frame data parameters
  union
  {
    float PixelYX[MLX90640_ROW_COUNT][MLX90640_COL_COUNT]; //!< To value of all 32x24 Pixels from pixel (1,1) to (24x32)
    float Pixel[MLX90640_TOTAL_PIXELS_COUNT];              //!< To value of all 768 Pixels from pixel (1,1) to (24x32)
  };
  // Auxiliary data
  float Vdd;             //!< Vdd of the last subframe
  float Ta;              //!< Ambient Temperature of the last subframe
  // Min, Max value
  float MinToSubpage[2]; //!< Minimum To value of each subframes
  float MaxToSubpage[2]; //!< Maximum To value of each subframes
  // Moving Average Filter
#if (MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT > 1)
  float PixGainCPSP0_Filter[MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT]; //!< Pixels Gain_CP_SP0 Filter stored values
  size_t Filter0_Index;                                                   //!< Pixels Gain_CP_SP0 Filter value current index
  float PixGainCPSP1_Filter[MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT]; //!< Pixels Gain_CP_SP1 Filter stored values
  size_t Filter1_Index;                                                   //!< Pixels Gain_CP_SP0 Filter value current index
#endif
} MLX90640_FrameTo;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MLX90640 Driver API
//********************************************************************************************************************

#define MLX90640_DEV_PARAMETERIZED          (1u << 0)    // Device is parameterized
#define MLX90640_DEV_NOT_PARAMETERIZED      (0u << 0)    // Device is not parameterized
#define MLX90640_DEV_NOT_PARAMETERIZED_SET  (~(1u << 0)) // Mask to set the device as not parameterized

#define MLX90640_ENDIANNESS_Pos           ( 1 )
#define MLX90640_LITTLE_ENDIAN            (0u << MLX90640_ENDIANNESS_Pos) // Select the little endianness
#define MLX90640_BIG_ENDIAN               (1u << MLX90640_ENDIANNESS_Pos) // Select the big endianness
#define MLX90640_IS_LITTLE_ENDIAN(value)  (((value) & (1u << MLX90640_ENDIANNESS_Pos)) == 0) // Little endian CPU detected ?

//-----------------------------------------------------------------------------



typedef struct MLX90640 MLX90640; //! Typedef of MLX90640 device object structure
typedef uint16_t TMLX90640DriverInternal; //! Alias for Driver Internal data flags



/*! @brief Interface function for driver initialization of the MLX90640
 *
 * This function will be called at driver initialization to configure the interface driver
 * @param[in] *pIntDev Is the MLX90640.InterfaceDevice of the device that call the interface initialization
 * @param[in] sclFreq Is the SCL frequency in Hz to set at the interface initialization
 * @return Returns an #eERRORRESULT value enum
 */
typedef eERRORRESULT (*MLX90640_I2CInit_Func)(void *pIntDev, const uint32_t sclFreq);


/*! @brief Interface function for I2C transfer of the MLX90640
 *
 * This function will be called when the driver needs to transfer data over the I2C communication with the device
 * Can be a read of data or a transmit of data. It also indicate if it needs a start and/or a stop
 * The maximim data that the driver will transmit between a start and a stop/restart is 832*2 bytes
 * @warning A I2CInit_Func() must be called before using this function. DO NOT CHANGE THE ENDIANNESS OF THE BYTES RETURNED BY THIS FUNCTION
 * @param[in] *pIntDev Is the MLX90640.InterfaceDevice of the device that call the I2C transfer
 * @param[in] deviceAddress Is the device address on the bus (8-bits only). The LSB bit indicate if it is a I2C Read (bit at '1') or a I2C Write (bit at '0')
 * @param[in,out] *data Is a pointer to memory data to write in case of I2C Write, or where the data received will be stored in case of I2C Read (can be NULL if no data transfer other than chip address)
 * @param[in] byteCount Is the byte count to write over the I2C bus or the count of byte to read over the bus
 * @param[in] start Indicate if the transfer needs a start (in case of a new transfer) or restart (if the previous transfer have not been stopped)
 * @param[in] stop Indicate if the transfer needs a stop after the last byte sent
 * @return Returns an #eERRORRESULT value enum
 */
typedef eERRORRESULT (*MLX90640_I2CTransfer_Func)(void *pIntDev, const uint8_t deviceAddress, uint8_t *data, size_t byteCount, bool start, bool stop);


/*! @brief Function that give the current millisecond of the system to the driver
 *
 * This function will be called when the driver needs to get current millisecond
 * @return Returns the current millisecond of the system
 */
typedef uint32_t (*GetCurrentms_Func)(void);



//! MLX90640 device object structure
struct MLX90640
{
  void *UserDriverData;                     //!< Optional, can be used to store driver data or NULL
  TMLX90640DriverInternal InternalConfig;   //!< DO NOT USE OR CHANGE THIS VALUE, IT'S THE INTERNAL DRIVER CONFIGURATION

  //--- Interface driver params and call functions ---
  uint8_t I2Caddress;                       //!< Device address set into the I2C address (0x800F) register. Use MLX90640_CHIPADDRESS_DEFAULT if you do not have change the address
  void *InterfaceDevice;                    //!< This is the pointer that will be in the first parameter of all interface call functions
  uint32_t I2CclockSpeed;                   //!< Clock frequency of the I2C interface in Hertz
  MLX90640_I2CInit_Func fnI2C_Init;         //!< This function will be called at driver initialization to configure the interface driver
  MLX90640_I2CTransfer_Func fnI2C_Transfer; //!< This function will be called when the driver needs to transfer data over the I2C communication with the device

  //--- Time call function ---
  GetCurrentms_Func fnGetCurrentms;         //!< This function will be called when the driver need to get current millisecond

  //--- Device EEPROM ---
#if !defined(MLX90640_PRECALCULATE_PIXELS_COEFFS)
  MLX90640_EEPROM* EEPROM;                  //!< Device EEPROM. Need to store a dump of the device's EEPROM for further calculations
#endif

  //--- Device parameters ---
  MLX90640_Parameters* Params;              //!< Device parameters. Will be filled after calling the function MLX90640_ExtractDeviceParameters()
};

//-----------------------------------------------------------------------------



//! MCP251XFD Controller and CAN configuration structure
typedef struct MLX90640_Config
{
  //--- Subpage configuration ---
  eMLX90640_SubpageMode SubpageMode;       //!< Determine the working mode of the subpages
  eMLX90640_RefreshRate RefreshRate;       //!< IR refresh rate of subpages
  eMLX90640_ReadingPattern ReadingPattern; //!< Reading pattern of the IR array
  eMLX90640_ADCresolution ADCresolution;   //!< Select the ADC resolution of the IR sensors

  //--- I2C configuration ---
  bool I2C_FMpEnable;                      //!< I2C FM+: 'true' = Enable (SCL @ 1MHz) ; 'false' = Disable (SCL @ 400kHz)
  bool SetThresholdTo1V8;                  //!< I2C threshold level to 1.8V: 'true' = threshold to 1.8V ; 'false' = threshold to Vdd
  bool SetSDAdriverCurrentLimit;           //!< I2C driver current limit: 'true' = Enable ; 'false' = Disable
} MLX90640_Config;

//********************************************************************************************************************





/*! @brief MLX90640 device initialization
 *
 * This function initializes the MLX90640 driver and call the initialization of the interface driver (I2C).
 * Next it checks parameters and configures the MLX90640
 * @param[in] *pComp Is the pointed structure of the device to be initialized
 * @param[in] *pConf Is the pointed structure of the device configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_MLX90640(MLX90640 *pComp, const MLX90640_Config *pConf);



/*! @brief Is the MLX90640 device ready
 *
 * Poll the acknowledge from the MLX90640
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns 'true' if ready else 'false'
 */
bool MLX90640_IsReady(MLX90640 *pComp);



/*! @brief Is a frame available on the MLX90640 device
 *
 * Poll the flag of a new data available in RAM from the MLX90640
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns 'true' if a frame is available else 'false'
 */
bool MLX90640_IsFrameAvailable(MLX90640 *pComp);



/*! @brief Get actual device of the MLX90640
 *
 * @param[in] *pComp Is the pointed structure of the device
 * @param[out] *device Is the device found (MLX90640BAA or MLX90640BAB)
 * @param[out] *deviceId1 Is the returned device ID1 (This parameter can be NULL if not needed)
 * @param[out] *deviceId2 Is the returned device ID2 (This parameter can be NULL if not needed)
 * @param[out] *deviceId3 Is the returned device ID3 (This parameter can be NULL if not needed)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_GetDeviceID(MLX90640 *pComp, eMLX90640_Devices* device, uint16_t* deviceId1, uint16_t* deviceId2, uint16_t* deviceId3);

//********************************************************************************************************************



/*! @brief Read data from the MLX90640 device
 *
 * This function reads data from the MLX90640 device. This function will convert the big-endian data (the device communicates in big endian) to the endianness of the CPU
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the address to read
 * @param[in] *data Is where the data array will be stored
 * @param[in] size Is the size of the data array to read (count of 16-bits data)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_ReadData(MLX90640 *pComp, const uint16_t address, uint16_t* data, size_t size);


/*! @brief Read a register from the MLX90640 device
 *
 * This function reads a register from the MLX90640 device
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address to read
 * @param[in] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MLX90640_ReadRegister(MLX90640 *pComp, const uint16_t address, uint16_t* data)
{
  return MLX90640_ReadData(pComp, address, data, 1);
}


/*! @brief Write data to the MLX90640 device
 *
 * This function writes data to the MLX90640 device. This function will convert data the from endianness of the CPU to big endian (the device communicates in big endian)
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the address where data will be written (address will be incremented automatically)
 * @param[in] *data Is the data array to store
 * @param[in] size Is the size of the data array to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_WriteData(MLX90640 *pComp, const uint16_t address, const uint16_t* data, size_t size);


/*! @brief Write a register to the MLX90640 device
 *
 * This function writes data to a register of the MLX90640 device
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the address where data will be written (address will be incremented automatically)
 * @param[in] *data Is the data to store
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MLX90640_WriteRegister(MLX90640 *pComp, const uint16_t address, const uint16_t data)
{
  return MLX90640_WriteData(pComp, address, &data, 1);
}


/*! @brief Dump the entire EEPROM data from the MLX90640 device
 *
 * This function reads the entire address range of the internal EEPROM of the MLX90640 device
 * The function will take care of the max EEPROM I2C speed operations (see note 5 of the table 5 of the datasheet)
 * Took around 40ms to dump the EEPROM from the MLX90640 device with SCL@400kHz
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *eepromDump Is where the EEPROM dump will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_DumpEEPROM(MLX90640 *pComp, MLX90640_EEPROM *eepromDump);


/*! @brief Get the last frame data on the MLX90640 device
 *
 * This function reads the entire address range of the internal RAM of the MLX90640 device
 * Took around 40ms to read the RAM from the MLX90640 device with SCL@400kHz
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *frameData Is where the frame data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_GetFrameData(MLX90640 *pComp, MLX90640_FrameData* frameData);

//********************************************************************************************************************



/*! @brief Configure the I2C on the MLX90640 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] i2cFMpEnable Indicate if the FM+ mode of the device should be activated
 * @param[in] setThresholdTo1V8 Set the I2C threshold of the device. 'true' to set to 1.8V, 'false' to set to Vdd
 * @param[in] setSDAdriverCurrentLimit Indicate if the device should limit its current on SDA pin
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_ConfigureDeviceI2C(MLX90640 *pComp, bool i2cFMpEnable, bool setThresholdTo1V8, bool setSDAdriverCurrentLimit);


/*! @brief Configure the MLX90640 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] SubpageMode Set the subpage mode
 * @param[in] RefreshRate Set the refresh rate of the frame
 * @param[in] ReadingPattern Set the reading pattern of the subpages
 * @param[in] ADCresolution Set the ADC resolution of Vdd
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_ConfigureDevice(MLX90640 *pComp, eMLX90640_SubpageMode subpageMode, eMLX90640_RefreshRate refreshRate, eMLX90640_ReadingPattern readingPattern, eMLX90640_ADCresolution adcResolution);


/*! @brief Change the I2C address of the MLX90640 device
 *
 * After using this function, reset the device to use it at its new address
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] newAddress Is the new I2C address to set. The value shall be > 0x00 and < 0x7F
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_ChangeI2Caddress(MLX90640 *pComp, uint8_t newAddress);

//********************************************************************************************************************



/*! @brief Extract device’s parameters from an EEPROM dump of a MLX90640 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in,out] *eepromDump Is where the EEPROM dump will be stored if extracted or the EEPROM dump to use for parameters extraction
 * @param[in] dumpEEPROM Indicate if the EEPROM shall be extracted and stored in *eepromDump
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_ExtractDeviceParameters(MLX90640 *pComp, MLX90640_EEPROM *eepromDump, bool dumpEEPROM);

//********************************************************************************************************************



/*! @brief Calculate Object Temperature of the frame on the MLX90640 device
 *
 * This function calculates the subframe To (Object Temperature) of the frame data previously extracted. It will only update the associated subframe's pixels and let others pixels untouched
 * In order to compensate correctly for the emissivity and achieve best accuracy we need to know the surrounding temperature which is responsible for the second component of the IR signal namely the reflected part -Tr. In case this temperature is not available and cannot be provided it might be replaced by Tr ~= Ta-8.
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *frameData Is the extracted frame data from the device
 * @param[in] emissivity Is the IR signal emitted by the object, if unknown set to 1.0f
 * @param[in] tr Is the IR signal reflected from the object (the source of this signal is surrounding environment of the sensor), if unknown set to Ta-8
 * @param[out] *result Is where the result will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_CalculateTo(MLX90640 *pComp, MLX90640_FrameData* frameData, float emissivity, float tr, MLX90640_FrameTo *result);


/*! @brief Correct bad pixels in a frame on the MLX90640 device
 *
 * This function corrects the defective pixel value by replacing its value by an interpolation of its neighboring pixels (See §9 of datasheet), here it is a mean of the neighboring pixels (X-1, X+1, Y-1, and Y+1, when available)
 * This function changes only defective pixels, others will not be touched
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in,out] *result Is where the result will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MLX90640_CorrectBadPixels(MLX90640 *pComp, MLX90640_FrameTo *result);

//********************************************************************************************************************



/*! @brief Invert endianness of an array of data
 *
 * Invert the endianness of the data array from big endian to little endian
 * In processor with little endianness, this function is useful because the device is communicating data in big endian
 * @param[in,out] *data Is the data array where to invert endianness
 * @param[in] size Count of uint16_t data in the array
 */
inline void MLX90640_BigEndianToLittleEndian(uint16_t *data, size_t size)
{
  if (data == NULL) return;
  for (size_t z = 0; z < size; ++z) { *data = ((*data & 0xFF) << 8) | ((*data >> 8) & 0xFF); data++; }
}

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
#endif /* MLX90640_H_INC */
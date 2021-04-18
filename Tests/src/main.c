/*
 * Hardware setup:
 *
 * If use of I2C Mux 4 board for connection:
 * 1) Plug the mikroBus Xplained Pro adapter board into connector EXT1 or EXT2 of the SAM V71 Xplained Ultra evaluation kit.
 * 2) Plug a I2C Mux 4 click into adapter board with address set to A1 = 0 and A0 = 0.
 * 3) Connect a MLX90640 board (like Pimoroni MLX90640 board) in one of the I2C Mux 4 outuput
 * 4) Power SAM V71 Xplained by connecting a USB cable to the DEBUG connector and plugging it into your PC.
 *
 * If the MLX90640 is connected directly to the SAM V71 Xplained Ultra evaluation kit:
 * 1) Plug the mikroBus Xplained Pro adapter board into connector EXT1 or EXT2 of the SAM V71 Xplained Ultra evaluation kit.
 * 3) Connect a MLX90640 board (like Pimoroni MLX90640 board) on the TWIHS0 port available on the board
 * 4) Power SAM V71 Xplained by connecting a USB cable to the DEBUG connector and plugging it into your PC.
 */
//=============================================================================

//-----------------------------------------------------------------------------
#include <asf.h> // Use Atmel Software Framework (ASF)
#include "string.h"
#include "Main.h"
#include "DriversParams.h"
#include "StringTools.h"
#include "I2C_V71InterfaceSync.h"
//-----------------------------------------------------------------------------


// Variables of program
volatile uint32_t msCount; //! Milli-seconds count from start of the system
static bool I2Cmux_DevicePresent = false;
static bool EERAM_DevicePresent = false;
static bool IrCAM_EEPROM_Dumped = false;
static MLX90640_FrameTo IrFrame;

//-----------------------------------------------------------------------------



//=============================================================================
// Dump the EEPROM memory
//=============================================================================
static void IrCAM_ShowEEPROM(MLX90640_EEPROM* pIrCAM_EEPROM)
{
  //--- Dump the data read ---
  static const char* Hexa = "0123456789ABCDEF";

  #define ROW_LENGTH  16             // 16 words per row
  char HexaDump[ROW_LENGTH * 5];     // [4 digit hexa + space] - 1 space + 1 zero terminal
  char HexaChar[ROW_LENGTH * 2 + 1]; // [2 char per words] + 1 zero terminal

  char CharSrc;
  uint16_t Address = 0x2400;
  uint16_t *pSrc = &pIrCAM_EEPROM->Words[0];
  uint16_t Size = sizeof(MLX90640_EEPROM) / sizeof(uint16_t);
  LOGINFO("Dump %d word at 0x2400 of device", Size);
  HexaChar[ROW_LENGTH] = 0;
  for (int32_t i = ((Size+ROW_LENGTH-1) / ROW_LENGTH); --i >= 0; pSrc += ROW_LENGTH, Size -= ROW_LENGTH, Address += ROW_LENGTH)
  {
    memset(HexaDump, ' ', sizeof(HexaDump));
    memset(HexaChar, '.', ROW_LENGTH);
    for (int j = (Size >= ROW_LENGTH ? ROW_LENGTH : Size); --j >= 0;)
    {
      HexaDump[j * 5 + 0] = Hexa[(pSrc[j] >> 12) & 0xF];
      HexaDump[j * 5 + 1] = Hexa[(pSrc[j] >>  8) & 0xF];
      HexaDump[j * 5 + 2] = Hexa[(pSrc[j] >>  4) & 0xF];
      HexaDump[j * 5 + 3] = Hexa[(pSrc[j] >>  0) & 0xF];
      //HexaDump[j * 5 + 4] = ' ';
      CharSrc = ((pSrc[j] >> 8) & 0xFF);
      HexaChar[j * 2 + 0] = (CharSrc < 0x20) ? '.' : CharSrc;
      CharSrc = ((pSrc[j] >> 0) & 0xFF);
      HexaChar[j * 2 + 1] = (CharSrc < 0x20) ? '.' : CharSrc;
    }
    HexaDump[ROW_LENGTH * 5 - 1] = 0;
    HexaChar[ROW_LENGTH * 2] = 0;
    LOGINFO("  %04X : %s \"%s\"", (unsigned int)Address, HexaDump, HexaChar);
  }
  #undef ROW_LENGTH
}



//=============================================================================
// Show the Parameters memory
//=============================================================================
const char* const  DefectStringsNames[] =
{
  "BROKEN",
  "OUTLIER",
};
  
static void IrCAM_ShowParams(MLX90640_Parameters* pIrCAM_Params)
{
  char Buffer0[22], Buffer1[22], Buffer2[12], Buffer3[12];
  LOGINFO("Gain         = %d", (int)pIrCAM_Params->Gain);
  LOGINFO("Resolution   = %u", (unsigned int)pIrCAM_Params->Resolution);
  LOGINFO("Kv_Vdd       = %d", (int)pIrCAM_Params->Kv_Vdd);
  LOGINFO("Vdd_25       = %d", (int)pIrCAM_Params->Vdd_25);
  Float_ToString(pIrCAM_Params->KvPTAT, &Buffer0[0], sizeof(Buffer0), 0, 8, true);
  LOGINFO("KvPTAT       = %s", Buffer0);
  Float_ToString(pIrCAM_Params->KtPTAT, &Buffer0[0], sizeof(Buffer0), 0, 6, true);
  LOGINFO("KtPTAT       = %s", Buffer0);
  Float_ToString(pIrCAM_Params->AlphaPTAT, &Buffer0[0], sizeof(Buffer0), 0, 6, false);
  LOGINFO("AlphaPTAT    = %s", Buffer0);
  LOGINFO("PTAT_25      = %u", (unsigned int)pIrCAM_Params->PTAT_25);
  Float_ToString(pIrCAM_Params->ILchess[0], &Buffer0[0], sizeof(Buffer0), 0, 4, true);
  Float_ToString(pIrCAM_Params->ILchess[1], &Buffer1[0], sizeof(Buffer1), 0, 4, true);
  Float_ToString(pIrCAM_Params->ILchess[2], &Buffer2[0], sizeof(Buffer2), 0, 4, true);
  LOGINFO("ILchess[3]   = %s,%s,%s", Buffer0, Buffer1, Buffer2);
  Float_ToString(pIrCAM_Params->KtaCP, &Buffer0[0], sizeof(Buffer0), 0, 8, true);
  LOGINFO("KtaCP        = %s", Buffer0);
  Float_ToString(pIrCAM_Params->KvCP, &Buffer0[0], sizeof(Buffer0), 0, 6, true);
  LOGINFO("KvCP         = %s", Buffer0);
  Float_ToString(pIrCAM_Params->KsTa, &Buffer0[0], sizeof(Buffer0), 0, 8, true);
  LOGINFO("KsTa         = %s", Buffer0);
  Float_ToString(pIrCAM_Params->TGC, &Buffer0[0], sizeof(Buffer0), 0, 18, true);
  LOGINFO("TGC          = %s", Buffer0);
  Float_ToString(pIrCAM_Params->KsTo[0], &Buffer0[0], sizeof(Buffer0), 0, 8, true);
  Float_ToString(pIrCAM_Params->KsTo[1], &Buffer1[0], sizeof(Buffer1), 0, 8, true);
  Float_ToString(pIrCAM_Params->KsTo[2], &Buffer2[0], sizeof(Buffer2), 0, 8, true);
  Float_ToString(pIrCAM_Params->KsTo[3], &Buffer3[0], sizeof(Buffer3), 0, 8, true);
  LOGINFO("KsTo[4]      = %s,%s,%s,%s", Buffer0, Buffer1, Buffer2, Buffer3);
  LOGINFO("CT[4]        = %d,%d,%d,%d", (int)pIrCAM_Params->CT[0], (int)pIrCAM_Params->CT[1], (int)pIrCAM_Params->CT[2], (int)pIrCAM_Params->CT[3]);

  Float_ToString(pIrCAM_Params->CPsubpageAlpha[0], &Buffer0[0], sizeof(Buffer0), 0, 18, true);
  Float_ToString(pIrCAM_Params->CPsubpageAlpha[1], &Buffer1[0], sizeof(Buffer1), 0, 18, true);
  LOGINFO("CPsubpages   : [0]-> Alpha = %s ; Offset = %d", Buffer0, (int)pIrCAM_Params->CPsubpageOffset[0]);
  LOGINFO("             : [1]-> Alpha = %s ; Offset = %d", Buffer1, (int)pIrCAM_Params->CPsubpageOffset[1]);
  LOGINFO("CalibPattern = %u", (unsigned int)pIrCAM_Params->CalibrationPattern);

#ifdef MLX90640_PRECALCULATE_PIXELS_COEFFS
  for (size_t y = 0; y < MLX90640_ROW_COUNT; ++y)
    for (size_t x = 0; x < MLX90640_COL_COUNT; ++x)
    {
      size_t Pos = (y * MLX90640_COL_COUNT) + x;
      //--- Pixel defect? ---
      bool PixelIsDefect = false;
      for (size_t z = 0; z < MLX90640_MAX_DEFECT_PIXELS; ++z)
        if (pIrCAM_Params->DefectivePixels[z].Type != MLX90640_NOT_DETECTIVE)
          if ((pIrCAM_Params->DefectivePixels[z].X == x) && (pIrCAM_Params->DefectivePixels[z].Y == y))
          {
            LOGINFO("Pixel[%3d]   : X = %2d ; Y = %2d ; Defective: %s", (int)Pos, (int)x, (int)y, DefectStringsNames[(int)pIrCAM_Params->DefectivePixels[z].Type - 1]);
            PixelIsDefect = true;
            break;
          }
      if (PixelIsDefect) continue;
      
      //--- Show pixel ---
      Float_ToString(pIrCAM_Params->AlphasYX[y][x]  , &Buffer0[0], sizeof(Buffer0), 0, 18, true);
      Float_ToString(pIrCAM_Params->KtaCoeffYX[y][x], &Buffer2[0], sizeof(Buffer2), 0,  9, true);
      Float_ToString(pIrCAM_Params->KvCoeffYX[y][x] , &Buffer3[0], sizeof(Buffer3), 0,  6, true);
      LOGINFO("Pixel[%3d]   : X = %2d ; Y = %2d ; Alpha = %s ; Offset = %4d ; Kta = %s ; Kv = %s", (int)Pos, (int)x, (int)y, Buffer0, (int)pIrCAM_Params->OffsetsYX[y][x], Buffer2, Buffer3);
    }
#else
  for (size_t z = 0; z < MLX90640_MAX_DEFECT_PIXELS; ++z)
    if (pIrCAM_Params->DefectivePixels[z].Type != MLX90640_NOT_DETECTIVE)
    {
      size_t x = pIrCAM_Params->DefectivePixels[z].X;
      size_t y = pIrCAM_Params->DefectivePixels[z].Y;
      size_t Pos = (y * MLX90640_ROW_COUNT) + x;
      LOGINFO("Pixel[%3d]   : X = %2d ; Y = %2d ; Defective: %s", (int)Pos, (int)x, (int)y, DefectStringsNames[(int)pIrCAM_Params->DefectivePixels[z].Type - 1]);
    }
#endif
}



//=============================================================================
// Show the current error
//=============================================================================
void ShowError(eERRORRESULT error)
{
  char* pStr = NULL;
/*  switch (error)
  {
#   define X(a, b, c) case a: pStr = (char*)c; break;
    ERRORS_TABLE
#   undef X
  }*/
  pStr = (char*)ERR_ErrorStrings[error];

  if (pStr != NULL)
       LOGERROR("Device error: %s", pStr);
  else LOGERROR("Device error: Unknown error (%u)", (unsigned int)error);
}





//=============================================================================
// Process command in buffer
//=============================================================================
static void ProcessCommand(void)
{
  eERRORRESULT Error;
  if (CommandInput.ToProcess == false) return;
  CommandInput.ToProcess = false;
  if (CommandInput.Buffer[0] != '*') return;       // First char should be a '*'
  CommandInput.Buffer[0] = '\0';                   // Break the current command in the buffer

  for (size_t z = 1; z < 7; z++) // Put string in lower case
  CommandInput.Buffer[z] = LowerCase(CommandInput.Buffer[z]);

  char* pBuf = &CommandInput.Buffer[1];

  //--- Check the command ---
  eConsoleCommand ConsoleCmd = NO_COMMAND;
  if (strncmp(pBuf, "dumpee"    ,  6) == 0) ConsoleCmd = DUMPEE;     // "*DumpEE" command
  if (strncmp(pBuf, "recallee"  ,  8) == 0) ConsoleCmd = RECALLEE;   // "*RecallEE" command
  if (strncmp(pBuf, "showee"    ,  6) == 0) ConsoleCmd = SHOWEE;     // "*ShowEE" command
  if (strncmp(pBuf, "saveee"    ,  6) == 0) ConsoleCmd = SAVEEE;     // "*SaveEE" command
  if (strncmp(pBuf, "showparams", 10) == 0) ConsoleCmd = SHOWPARAMS; // "*ShowParams" command

  if (ConsoleCmd == NO_COMMAND) return;
  SetStrToConsoleBuffer(CONSOLE_TX, "\r\n");

  //--- Do stuff ---
  switch (ConsoleCmd)
  {
    case DUMPEE:
      Error = MLX90640_DumpEEPROM(IrCAM, &IrCAM_EEPROM);
      if (Error == ERR_OK)
      {
        IrCAM_EEPROM_Dumped = true;
        LOGTRACE("Dump done");
      }
      else ShowError(Error);
      break;
    case RECALLEE:
      Error = EERAM47x16_ReadSRAMData(EERAM_47L16, 0, &IrCAM_EEPROM.Bytes[0], sizeof(IrCAM_EEPROM));
      if (Error == ERR_OK)
      {
        IrCAM_EEPROM_Dumped = true;
        LOGTRACE("Recall done");
      }
      else ShowError(Error);
      break;
    case SHOWEE:
      IrCAM_ShowEEPROM(&IrCAM_EEPROM);
      break;
    case SAVEEE:
      Error = EERAM47x16_WriteSRAMData(EERAM_47L16, 0, &IrCAM_EEPROM.Bytes[0], sizeof(IrCAM_EEPROM));
      if (Error != ERR_OK) ShowError(Error);
      else
      {
        Error = EERAM47x16_StoreSRAMtoEEPROM(EERAM_47L16, false, true);
        if (Error == ERR_OK)
             LOGTRACE("Store done");
        else ShowError(Error);
      }
      break;
    case SHOWPARAMS:
      IrCAM_ShowParams(&IrCAM_Params);
      break;

    case NO_COMMAND:
    default: return;
  }
}





//=============================================================================
// SysTick Handler
//=============================================================================
void SysTick_Handler(void)
{
  msCount++;
}





//=============================================================================
// Main
//=============================================================================
int main (void)
{
  eERRORRESULT Error;
  
  wdt_disable(WDT);

  //--- Configure system clock --------------------------
  sysclk_init();
  SystemCoreClock = sysclk_get_cpu_hz();

  //--- Initialize board --------------------------------
  board_init();
  ioport_set_pin_mode(TWIHS0_DATA_GPIO, TWIHS0_DATA_FLAGS);
	ioport_disable_pin(TWIHS0_DATA_GPIO);
  ioport_set_pin_mode(TWIHS0_CLK_GPIO, TWIHS0_CLK_FLAGS);
	ioport_disable_pin(TWIHS0_CLK_GPIO);
  
  EXT2_PWM_PIO_En;
  EXT2_PWM_High;
	EXT2_PWM_Out;

  //--- Initialize the console UART ---------------------
  InitConsoleTx(CONSOLE_TX);
  InitConsoleRx(CONSOLE_RX);

  //--- Demo start --------------------------------------
  printf("\r\n\r\n");
  LOGTITLE("MLX90640 Demo start...");

  //--- Configure SysTick base timer --------------------
  SysTick_Config(SystemCoreClock * SYSTEM_TICK_MS / 1000); // (Fmck(Hz)*1/1000)=1ms

  //--- Configure 47L16 --------------------------------
  Error = Init_EERAM47x16(EERAM_47L16);
  if (Error == ERR_OK)
  {
    LOGTRACE("Device 47L16 [2kB EERAM (2048x1)] detected");
    EERAM47x16_Status_Register Status;
    Error = EERAM47x16_GetStatus(EERAM_47L16, &Status);
    if (Error == ERR_OK) LOGTRACE("    47L16 Status: %02X", (unsigned int)Status.Status);

    ioport_set_pin_level(LED1_GPIO, LED1_INACTIVE_LEVEL);
    EERAM_DevicePresent = true;
  }

  //--- Configure TCA9543A ------------------------------
  EXT2_RST_PIO_En;
  EXT2_RST_High;
  EXT2_RST_Out;
  delay_ms(1);
  EXT2_RST_Low;
  delay_us(1);   // Here, the TCA9543A needs at least 4ns
  EXT2_RST_High;
  Error = Init_TCA9543A(I2CMUX);
  if (Error == ERR_OK)
  {
    LOGTRACE("Device TCA9543A detected, 2 new I2C channels available");
    ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
    I2Cmux_DevicePresent = true;
  }
  else
  {
    LOGERROR("Device TCA9543A not detected");
    ioport_set_pin_level(LED0_GPIO, LED0_ACTIVE_LEVEL);
    I2Cmux_DevicePresent = false;
  }
  
  //--- Configure MLX90640 ------------------------------
  float Tr = 25.0f - 8.0f; // Set the first Tr at 25-8°. Next will be get from the last calculated frame
  if (I2Cmux_DevicePresent == false)
  {
#if defined(SOFT_I2C)
    MLX90640_V71.fnI2C_Init     = SoftI2C_InterfaceInit_V71;
    MLX90640_V71.fnI2C_Transfer = SoftI2C_Tranfert_V71;
#else
    MLX90640_V71.fnI2C_Init     = HardI2C_InterfaceInit_V71;
    MLX90640_V71.fnI2C_Transfer = HardI2C_Tranfert_V71;
#endif
  }
  Error = Init_MLX90640(IrCAM, &IrCAM_Config);
  if (Error == ERR_OK)
  {
    eMLX90640_Devices Device;
    uint16_t DeviceID[3];
    Error = MLX90640_GetDeviceID(IrCAM, &Device, &DeviceID[0], &DeviceID[1], &DeviceID[2]);
    if (Error == ERR_OK)
    {
      LOGTRACE("Device %s detected, device ID: %04X%04X%04X", MLX90640_DevicesNames[(size_t)Device], (unsigned short)DeviceID[0], (unsigned short)DeviceID[1], (unsigned short)DeviceID[2]);

//      Error = MLX90640_ExtractDeviceParameters(IrCAM, &IrCAM_EEPROM, false);  // Do not dump EEPROM
      Error = MLX90640_ExtractDeviceParameters(IrCAM, &IrCAM_EEPROM, true); // Dump EEPROM
      if (Error == ERR_OK)
      {
        ioport_set_pin_level(LED1_GPIO, LED1_INACTIVE_LEVEL);
      }
      else ShowError(Error);
    }
    else ShowError(Error);
  }
  else
  {
    ioport_set_pin_level(LED1_GPIO, LED1_ACTIVE_LEVEL);
    ShowError(Error);
    LOGFATAL("Error on MLX90640, END OF DEMO");
    while (true) TrySendingNextCharToConsole(CONSOLE_TX); // Stay stuck here
  }    

  //--- Reset watchdog ----------------------------------
  wdt_restart(WDT);

  //--- Log ---------------------------------------------
  LOGTRACE("Initialization complete");

  //--- Display menu ------------------------------------
  LOGINFO("Available commands:");
  LOGINFO("  *DumpEE    : Dump the EEPROM from MLX90640");
  LOGINFO("  *ShowEE    : Show the EEPROM in an hex table");
  LOGINFO("  *ShowParams: Show the parameters");
  if (EERAM_DevicePresent)
  {
    LOGINFO("  *RecallEE  : Recall the EEPROM from 47L16 EERAM");
    LOGINFO("  *SaveEE    : Save the EEPROM to the 47L16 EERAM");
  }

  //=== The main loop ===================================
  while(1)
  {
    //--- Flush char by char console buffer ---
    TrySendingNextCharToConsole(CONSOLE_TX);

    //--- Process command if any available ---
    ProcessCommand();
    
    //--- Frame available ? ---
    if (MLX90640_IsFrameAvailable(IrCAM))
    {
      //--- Get subframe frame data ---
      Error = MLX90640_GetFrameData(IrCAM, &IrCAM_Frame);
      if (Error == ERR_OK)
      {
        EXT2_PWM_Low;
        //--- Calculate the subframe To
        Error = MLX90640_CalculateTo(IrCAM, &IrCAM_Frame, 1.0f, Tr, &IrFrame);
        if (Error == ERR_OK)
        {
          Tr = IrFrame.Ta - 8.0f; // Calculate next Tr: Tr = Ta - 8°
          
          // Show the frame <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
          
          
        } else ShowError(Error);
        EXT2_PWM_High;
      } else ShowError(Error);
    }

    nop();
  }
}

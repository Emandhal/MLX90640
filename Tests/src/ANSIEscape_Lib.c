/*!*****************************************************************************
 * @file    ANSIEscape_Lib.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/12/2017
 * @brief   Some functions for ANSI Escape console communication
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "stdarg.h"
#include "stdio.h"
#include "Main.h"
#include "ANSIEscape_Lib.h"
#include "string.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//-----------------------------------------------------------------------------





//=============================================================================
// Convert and add a uint8_t to a char array (string)
//=============================================================================
static void FillByteInArray(uint8_t value, char **array)
{
  bool HasHundred = false;
  if (value > 99)
  {
    **array = (char)((value / 100) + 48);
    value %= 100;
    (*array)++; // Next index of the original array
    HasHundred = true;
  }
  if ((value > 9) || HasHundred)
  {
    **array = (char)((value / 10) + 48);
    value %= 10;
    (*array)++; // Next index of the original array
  }
  **array = (char)(value + 48);
  (*array)++; // Next index of the original array
}





//**********************************************************************************************************************************************************
#ifdef USE_ANSI_ESCAPE_TX

//=============================================================================
// Function for Cursor Movement with ANSI Escape sequences
//=============================================================================
// \e[#A - Moves the cursor up by the specified number of lines without changing columns. If the cursor is already on the top line, this sequence is ignored. \e[A is equivalent to \e[1A.
// \e[#B - Moves the cursor down by the specified number of lines without changing columns. If the cursor is already on the bottom line, this sequence is ignored. \e[B is equivalent to \e[1B.
// \e[#C - Moves the cursor forward by the specified number of columns without changing lines. If the cursor is already in the rightmost column, this sequence is ignored. \e[C is equivalent to \e[1C.
// \e[#D - Moves the cursor back by the specified number of columns without changing lines. If the cursor is already in the leftmost column, this sequence is ignored. \e[D is equivalent to \e[1D.
// \e[#E - Moves the cursor down the indicated # of rows, to column 1. \e[E is equivalent to \e[1E.
// \e[#F - Moves the cursor up the indicated # of rows, to column 1. \e[F is equivalent to \e[1F.
// \e[#G - Moves the cursor to indicated column in current row. \e[G is equivalent to \e[1G.
// \e[s - Save Cursor Position: Saves the current cursor position. You can move the cursor to the saved cursor position by using the Restore Cursor Position sequence.
// \e[u - Restore Cursor Position: Returns the cursor to the position stored by the Save Cursor Position sequence.
void ANSIESC_Cursor(ConsoleTx* pApi, eANSIESC_Cursor movement, uint8_t count)
{
  char EscSequence[7] = { ESC_CMD, ESC_BRACKET_CMD }; // ESC_CMD + ESC_BRACKET_CMD + 3 char for uint8 to string + Sequence type char + null terminal
  char *pSeq = &EscSequence[2];  // Go right after the ESC_BRACKET_CMD
  FillByteInArray(count, &pSeq); // Add the count of the movement and move the pointer after the last char
  *pSeq = (char)movement;        // Add the movement sequence type
  pSeq++;                        // Next index
  *pSeq = '\0';                  // Null terminal
  SetStrToConsoleBuffer(pApi, &EscSequence[0]);
}



//=============================================================================
// Function for set cursor position with ANSI Escape sequences
//=============================================================================
// \e[#;#H - Moves the cursor to the specified position. The first # specifies the line number, the second # specifies the column. If you do not specify a position, the cursor moves to the home position: the upper-left corner of the screen (line 1, column 1).
void ANSIESC_SetCursorPosition(ConsoleTx* pApi, uint8_t line, uint8_t column)
{
  char EscSequence[11] = { ESC_CMD, ESC_BRACKET_CMD }; // ESC_CMD + ESC_BRACKET_CMD + 3 char for uint8 to string X-axis + ESC_SEPARATOR + 3 char for uint8 to string X-axis + Sequence type char + null terminal
  char *pSeq = &EscSequence[2];   // Go right after the ESC_BRACKET_CMD
  FillByteInArray(line, &pSeq);   // Add the line position and move the pointer after the last char
  *pSeq = (char)ESC_SEPARATOR;    // Add the argument separator
  pSeq++;                         // Next index
  FillByteInArray(column, &pSeq); // Add the column position and move the pointer after the last char
  *pSeq = (char)TO_POSITION;      // Add the position sequence type
  pSeq++;                         // Next index
  *pSeq = '\0';                   // Null terminal
  SetStrToConsoleBuffer(pApi, &EscSequence[0]);
}



//=============================================================================
// Function to clear screen or line with ANSI Escape sequences
//=============================================================================
// \e[0J - Clears the screen from cursor to end of display. The cursor position is unchanged.
// \e[1J - Clears the screen from start to cursor. The cursor position is unchanged.
// \e[2J - Clears the screen and moves the cursor to the home position (line 1, column 1).
// \e[J is equivalent to \e[0J. (Some terminal/emulators respond to \e[J as if it were \e[2J. Here, the default is 0; it is the norm)
// \e[0K - Clears all characters from the cursor position to the end of the line (including the character at the cursor position). The cursor position is unchanged.
// \e[1K - Clears all characters from start of line to the cursor position. (including the character at the cursor position). The cursor position is unchanged.
// \e[2K - Clears all characters of the whole line. The cursor position is unchanged.
// \e[K is equivalent to \e[0K. (Some terminal/emulators respond to \e[K as if it were \e[2K. Here, the default is 0; it is the norm)
void ANSIESC_Clear(ConsoleTx* pApi, eANSIESC_Clear clear, eANSIESC_ClearDirection direction)
{
  char EscSequence[5] = { ESC_CMD, ESC_BRACKET_CMD }; // ESC_CMD + ESC_BRACKET_CMD + Clear line or screen + Clear direction + null terminal
  EscSequence[2] = (char)direction; // Add the direction command right after the ESC_BRACKET_CMD
  EscSequence[3] = (char)clear;     // Add the clear type command right after the direction command
  EscSequence[4] = '\0';            // Null terminal
  SetStrToConsoleBuffer(pApi, &EscSequence[0]);
}



//=============================================================================
// Function for insertion/delete with ANSI Escape sequences
//=============================================================================
// \e[#L - The cursor line and all lines below it move down # lines, leaving blank space. The cursor position is unchanged. The bottommost # lines are lost. \e[L is equivalent to \e[1L.
// \e[#M - The block of # lines at and below the cursor are deleted; all lines below them move up # lines to fill in the gap, leaving # blank lines at the bottom of the screen. The cursor position is unchanged. \e[M is equivalent to \e[1M.
// \e[#@ - The cursor character and all characters to the right of it move right # columns, leaving behind blank space. The cursor position is unchanged. The rightmost # characters on the line are lost. \e[@ is equivalent to \e[1@.
// \e[#P - The block of # characters at and to the right of the cursor are deleted; all characters to the right of it move left # columns, leaving behind blank space. The cursor position is unchanged. \e[P is equivalent to \e[1P.
void ANSIESC_InsertDelete(ConsoleTx* pApi, eANSIESC_InsertDelete command, uint8_t count)
{
  char EscSequence[7] = { ESC_CMD, ESC_BRACKET_CMD }; // ESC_CMD + ESC_BRACKET_CMD + 3 char for uint8 to string + Sequence type char + null terminal
  char *pSeq = &EscSequence[2];  // Go right after the ESC_BRACKET_CMD
  FillByteInArray(count, &pSeq); // Add the count of the command and move the pointer after the last char
  *pSeq = (char)command;         // Add the command sequence type
  pSeq++;                        // Next index
  *pSeq = '\0';                  // Null terminal
  SetStrToConsoleBuffer(pApi, &EscSequence[0]);
}



//=============================================================================
// Function for setting graphic mode with ANSI Escape sequences
//=============================================================================
// \e[#;...;#m - Set Graphics Mode: Calls the graphics functions specified by the following values. These specified functions remain active until the next occurrence of this escape sequence. Graphics mode changes the colors and attributes of text (such as bold and underline) displayed on the screen. \e[m is equivalent to \e[0m.}
void ANSIESC_SetGraphicMode(ConsoleTx* pApi, uint8_t count, ...)
{
  if (count == 0) return;
  char EscSequence[5] = { ESC_CMD, ESC_BRACKET_CMD, 0 }; // ESC_CMD + ESC_BRACKET_CMD + null terminal
  SetStrToConsoleBuffer(pApi, &EscSequence[0]);          // Send the beginning of the escape sequence
  char *pSeq;

  // Send each parameters one by one
  va_list args;
  va_start(args, count);
  while (count > 0)
  {
    count--;
    pSeq = &EscSequence[0];                                      // Set to first char
    FillByteInArray((uint8_t)va_arg(args, unsigned int), &pSeq); // Add the count of the command and move the pointer after the last char
    if (count > 0) { *pSeq = ESC_SEPARATOR; pSeq++; }            // If there are other parameters after this one, put the ';' separator
    *pSeq = '\0';                                                // Null terminal
    SetStrToConsoleBuffer(pApi, &EscSequence[0]);                // Send parameter
  }
  va_end(args);

  SetCharToConsoleBuffer(pApi, (char)ANSIESC_SETGRAPHICSMODE); // Send the graphic mode sequence char
}

//-----------------------------------------------------------------------------
#endif /* USE_ANSI_ESCAPE_TX */





//**********************************************************************************************************************************************************
#ifdef USE_ANSI_ESCAPE_RX



//-----------------------------------------------------------------------------
#endif /* USE_ANSI_ESCAPE_RX */





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
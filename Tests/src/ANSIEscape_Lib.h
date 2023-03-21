/*!*****************************************************************************
 * @file    ANSIEscape_Lib.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/12/2017
 * @brief   Some functions for ANSI Escape console communication
 ******************************************************************************/
#ifndef ANSIESCAPELIB_H_
#define ANSIESCAPELIB_H_
//=============================================================================

//-----------------------------------------------------------------------------
#include "stdio.h"
#include <stdarg.h>
#include "Console.h"

#ifndef __cplusplus
#else
extern "C" {
#endif
//-----------------------------------------------------------------------------





//******************************************************************************
//* ANSI escape sequence is a sequence of ASCII characters, the first two of
//* which are the ASCII "Escape" character 27 (1Bh) and the left-bracket character
//* " [ " (5Bh). The character or characters following the escape and left-bracket
//* characters specify an alphanumeric code that controls a keyboard or display function.
#define ESC_CMD          '\x1B' // \e
#define ESC_BRACKET_CMD  '['
#define ESC_SEPARATOR    ';'
//******************************************************************************

//! ANSI Escape sequences for Cursor Movement
typedef enum
{
  MOVE_UP          = 'A', //!< \e[#A - Moves the cursor up by the specified number of lines without changing columns. If the cursor is already on the top line, this sequence is ignored. \e[A is equivalent to \e[1A.
  MOVE_DOWN        = 'B', //!< \e[#B - Moves the cursor down by the specified number of lines without changing columns. If the cursor is already on the bottom line, this sequence is ignored. \e[B is equivalent to \e[1B.
  MOVE_FORWARD     = 'C', //!< \e[#C - Moves the cursor forward by the specified number of columns without changing lines. If the cursor is already in the rightmost column, this sequence is ignored. \e[C is equivalent to \e[1C.
  MOVE_BACKWARD    = 'D', //!< \e[#D - Moves the cursor back by the specified number of columns without changing lines. If the cursor is already in the leftmost column, this sequence is ignored. \e[D is equivalent to \e[1D.
  MOVE_NEXTLINE    = 'E', //!< \e[#E - Moves the cursor down the indicated # of rows, to column 1. \e[E is equivalent to \e[1E.
  MOVE_PREVLINE    = 'F', //!< \e[#F - Moves the cursor up the indicated # of rows, to column 1. \e[F is equivalent to \e[1F.
  TO_COLUMN        = 'G', //!< \e[#G - Moves the cursor to indicated column in current row. \e[G is equivalent to \e[1G.
  TO_POSITION      = 'H', //!< \e[#;#H - Moves the cursor to the specified position. The first # specifies the line number, the second # specifies the column. If you do not specify a position, the cursor moves to the home position: the upper-left corner of the screen (line 1, column 1).
  SAVE_POSITION    = 's', //!< \e[s - Save Cursor Position: Saves the current cursor position. You can move the cursor to the saved cursor position by using the Restore Cursor Position sequence.
  RESTORE_POSITION = 'u', //!< \e[u - Restore Cursor Position: Returns the cursor to the position stored by the Save Cursor Position sequence.
} eANSIESC_Cursor;


//! ANSI Escape sequences for clear direction (associated with #eANSIESC_Clear)
typedef enum
{
  FROM_CURSOR_TO_END   = '0', //!< Clears from cursor to end. The cursor position is unchanged.
  FROM_START_TO_CURSOR = '1', //!< Clears from start to cursor. The cursor position is unchanged.
  CLEAR_ALL            = '2', //!< Clears and moves the cursor to the home position (line 1, column 1).
} eANSIESC_ClearDirection;


//! ANSI Escape sequences for clearing (associated with #eANSIESC_ClearDirection)
typedef enum
{
  CLEAR_SCREEN = 'J', //!< Clears from cursor to end. The cursor position is unchanged. \e[J is equivalent to \e[0J. (Some terminal/emulators respond to \e[J as if it were \e[2J. Here, the default is 0; it is the norm)
  CLEAR_LINE   = 'K', //!< Clears from start to cursor. The cursor position is unchanged. \e[K is equivalent to \e[0K. (Some terminal/emulators respond to \e[K as if it were \e[2K. Here, the default is 0; it is the norm)
} eANSIESC_Clear;


//! ANSI Escape sequences for insert or delete (associated with #eANSIESC_ClearDirection)
typedef enum
{
  INSERT_LINES      = 'L', //!< \e[#L - The cursor line and all lines below it move down # lines, leaving blank space. The cursor position is unchanged. The bottommost # lines are lost. \e[L is equivalent to \e[1L.
  DELETE_LINES      = 'M', //!< \e[#M - The block of # lines at and below the cursor are deleted; all lines below them move up # lines to fill in the gap, leaving # blank lines at the bottom of the screen. The cursor position is unchanged. \e[M is equivalent to \e[1M.
  INSERT_CHARACTERS = '@', //!< \e[#@ - The cursor character and all characters to the right of it move right # columns, leaving behind blank space. The cursor position is unchanged. The rightmost # characters on the line are lost. \e[@ is equivalent to \e[1@.
  DELETE_CHARACTERS = 'P', //!< \e[#P - The block of # characters at and to the right of the cursor are deleted; all characters to the right of it move left # columns, leaving behind blank space. The cursor position is unchanged. \e[P is equivalent to \e[1P.
} eANSIESC_InsertDelete;


//! ANSI Escape sequences
typedef enum
{
// Escape sequences for Cursor Movement
  ANSIESC_CURSOR_UP         = 'A', //!< \e[#A - Moves the cursor up by the specified number of lines without changing columns. If the cursor is already on the top line, this sequence is ignored. \e[A is equivalent to \e[1A.
  ANSIESC_CURSOR_DOWN       = 'B', //!< \e[#B - Moves the cursor down by the specified number of lines without changing columns. If the cursor is already on the bottom line, this sequence is ignored. \e[B is equivalent to \e[1B.
  ANSIESC_CURSOR_FORWARD    = 'C', //!< \e[#C - Moves the cursor forward by the specified number of columns without changing lines. If the cursor is already in the rightmost column, this sequence is ignored. \e[C is equivalent to \e[1C.
  ANSIESC_CURSOR_BACKWARD   = 'D', //!< \e[#D - Moves the cursor back by the specified number of columns without changing lines. If the cursor is already in the leftmost column, this sequence is ignored. \e[D is equivalent to \e[1D.
  ANSIESC_CURSOR_NEXTLINE   = 'E', //!< \e[#E - Moves the cursor down the indicated # of rows, to column 1. \e[E is equivalent to \e[1E.
  ANSIESC_CURSOR_PREVLINE   = 'F', //!< \e[#F - Moves the cursor up the indicated # of rows, to column 1. \e[F is equivalent to \e[1F.
  ANSIESC_CURSOR_CURABSH    = 'G', //!< \e[#G - Moves the cursor to indicated column in current row. \e[G is equivalent to \e[1G.
  ANSIESC_CURSOR_CURPOS     = 'H', //!< \e[#;#H - Moves the cursor to the specified position. The first # specifies the line number, the second # specifies the column. If you do not specify a position, the cursor moves to the home position: the upper-left corner of the screen (line 1, column 1).
  ANSIESC_CURSOR_CURSAVE    = 's', //!< \e[s - Save Cursor Position: Saves the current cursor position. You can move the cursor to the saved cursor position by using the Restore Cursor Position sequence.
  ANSIESC_CURSOR_CURRESTO   = 'u', //!< \e[u - Restore Cursor Position: Returns the cursor to the position stored by the Save Cursor Position sequence.

// Escape sequences for Line and Char Edition
  ANSIESC_INSERT_LINES      = 'L', //!< \e[#L - The cursor line and all lines below it move down # lines, leaving blank space. The cursor position is unchanged. The bottommost # lines are lost. \e[L is equivalent to \e[1L.
  ANSIESC_DELETE_LINES      = 'M', //!< \e[#M - The block of # lines at and below the cursor are deleted; all lines below them move up # lines to fill in the gap, leaving # blank lines at the bottom of the screen. The cursor position is unchanged. \e[M is equivalent to \e[1M.
  ANSIESC_INSERT_CHARACTERS = '@', //!< \e[#\@ - The cursor character and all characters to the right of it move right # columns, leaving behind blank space. The cursor position is unchanged. The rightmost # characters on the line are lost. \e[\@ is equivalent to \e[1\@.
  ANSIESC_DELETE_CHARACTERS = 'P', //!< \e[#P - The block of # characters at and to the right of the cursor are deleted; all characters to the right of it move left # columns, leaving behind blank space. The cursor position is unchanged. \e[P is equivalent to \e[1P.

// Escape sequences for Set Graphics Rendition
  ANSIESC_SETGRAPHICSMODE   = 'm', //!< \e[#;...;#m - Set Graphics Mode: Calls the graphics functions specified by the following values. These specified functions remain active until the next occurrence of this escape sequence. Graphics mode changes the colors and attributes of text (such as bold and underline) displayed on the screen. \e[m is equivalent to \e[0m.}
} eANSIESC_Sequence;






#define ANSIESC_BLACK            0
#define ANSIESC_RED              1
#define ANSIESC_GREEN            2
#define ANSIESC_BLUE             4
#define ANSIESC_FOREGROUND      30
#define ANSIESC_FOREGROUND_HI   90
#define ANSIESC_BACKGROUND      40
#define ANSIESC_BACKGROUND_HI  100

//! ANSI Escape graphic mode foreground colors values (associated with #eANSIESC_Sequence::ANSIESC_SETGRAPHICSMODE)
typedef enum
{
  // Foreground colors
  FOREGROUND_BLACK      = ANSIESC_FOREGROUND + ANSIESC_BLACK                              , //!< 30 => Graphic mode foreground Black
  FOREGROUND_RED        = ANSIESC_FOREGROUND + ANSIESC_RED                                , //!< 31 => Graphic mode foreground Red
  FOREGROUND_GREEN      = ANSIESC_FOREGROUND + ANSIESC_GREEN                              , //!< 32 => Graphic mode foreground Green
  FOREGROUND_YELLOW     = ANSIESC_FOREGROUND + ANSIESC_RED  + ANSIESC_GREEN               , //!< 33 => Graphic mode foreground Yellow
  FOREGROUND_BLUE       = ANSIESC_FOREGROUND + ANSIESC_BLUE                               , //!< 34 => Graphic mode foreground Blue
  FOREGROUND_MAGENTA    = ANSIESC_FOREGROUND + ANSIESC_BLUE + ANSIESC_RED                 , //!< 35 => Graphic mode foreground Magenta
  FOREGROUND_CYAN       = ANSIESC_FOREGROUND + ANSIESC_BLUE + ANSIESC_GREEN               , //!< 36 => Graphic mode foreground Cyan
  FOREGROUND_WHITE      = ANSIESC_FOREGROUND + ANSIESC_RED  + ANSIESC_GREEN + ANSIESC_BLUE, //!< 37 => Graphic mode foreground White
  // Foreground colors High intensity
  FOREGROUND_HI_BLACK   = ANSIESC_FOREGROUND_HI + ANSIESC_BLACK                              , //!< 90 => Graphic mode foreground Black
  FOREGROUND_HI_RED     = ANSIESC_FOREGROUND_HI + ANSIESC_RED                                , //!< 91 => Graphic mode foreground Red
  FOREGROUND_HI_GREEN   = ANSIESC_FOREGROUND_HI + ANSIESC_GREEN                              , //!< 92 => Graphic mode foreground Green
  FOREGROUND_HI_YELLOW  = ANSIESC_FOREGROUND_HI + ANSIESC_RED  + ANSIESC_GREEN               , //!< 93 => Graphic mode foreground Yellow
  FOREGROUND_HI_BLUE    = ANSIESC_FOREGROUND_HI + ANSIESC_BLUE                               , //!< 94 => Graphic mode foreground Blue
  FOREGROUND_HI_MAGENTA = ANSIESC_FOREGROUND_HI + ANSIESC_BLUE + ANSIESC_RED                 , //!< 95 => Graphic mode foreground Magenta
  FOREGROUND_HI_CYAN    = ANSIESC_FOREGROUND_HI + ANSIESC_BLUE + ANSIESC_GREEN               , //!< 96 => Graphic mode foreground Cyan
  FOREGROUND_HI_WHITE   = ANSIESC_FOREGROUND_HI + ANSIESC_RED  + ANSIESC_GREEN + ANSIESC_BLUE, //!< 97 => Graphic mode foreground White
} eGraphicMode_ForegroundColor;


//! ANSI Escape graphic mode background colors values (associated with #eANSIESC_Sequence::ANSIESC_SETGRAPHICSMODE)
typedef enum
{
  // Background colors
  BACKGROUND_BLACK      = ANSIESC_BACKGROUND + ANSIESC_BLACK                              , //!< 40 => Graphic mode background Black
  BACKGROUND_RED        = ANSIESC_BACKGROUND + ANSIESC_RED                                , //!< 41 => Graphic mode background Red
  BACKGROUND_GREEN      = ANSIESC_BACKGROUND + ANSIESC_GREEN                              , //!< 42 => Graphic mode background Green
  BACKGROUND_YELLOW     = ANSIESC_BACKGROUND + ANSIESC_RED  + ANSIESC_GREEN               , //!< 43 => Graphic mode background Yellow
  BACKGROUND_BLUE       = ANSIESC_BACKGROUND + ANSIESC_BLUE                               , //!< 44 => Graphic mode background Blue
  BACKGROUND_MAGENTA    = ANSIESC_BACKGROUND + ANSIESC_BLUE + ANSIESC_RED                 , //!< 45 => Graphic mode background Magenta
  BACKGROUND_CYAN       = ANSIESC_BACKGROUND + ANSIESC_BLUE + ANSIESC_GREEN               , //!< 46 => Graphic mode background Cyan
  BACKGROUND_WHITE      = ANSIESC_BACKGROUND + ANSIESC_RED  + ANSIESC_GREEN + ANSIESC_BLUE, //!< 47 => Graphic mode background White
  // Background colors High intensity
  BACKGROUND_HI_BLACK   = ANSIESC_BACKGROUND_HI + ANSIESC_BLACK                              , //!< 100 => Graphic mode background Black
  BACKGROUND_HI_RED     = ANSIESC_BACKGROUND_HI + ANSIESC_RED                                , //!< 101 => Graphic mode background Red
  BACKGROUND_HI_GREEN   = ANSIESC_BACKGROUND_HI + ANSIESC_GREEN                              , //!< 102 => Graphic mode background Green
  BACKGROUND_HI_YELLOW  = ANSIESC_BACKGROUND_HI + ANSIESC_RED  + ANSIESC_GREEN               , //!< 103 => Graphic mode background Yellow
  BACKGROUND_HI_BLUE    = ANSIESC_BACKGROUND_HI + ANSIESC_BLUE                               , //!< 104 => Graphic mode background Blue
  BACKGROUND_HI_MAGENTA = ANSIESC_BACKGROUND_HI + ANSIESC_BLUE + ANSIESC_RED                 , //!< 105 => Graphic mode background Magenta
  BACKGROUND_HI_CYAN    = ANSIESC_BACKGROUND_HI + ANSIESC_BLUE + ANSIESC_GREEN               , //!< 106 => Graphic mode background Cyan
  BACKGROUND_HI_WHITE   = ANSIESC_BACKGROUND_HI + ANSIESC_RED  + ANSIESC_GREEN + ANSIESC_BLUE, //!< 107 => Graphic mode background White
} eGraphicMode_BackgroundColor;



#define ANSIESC_ATTRIBUTE_OFF  20

//! ANSI Escape graphic mode text attributes values (associated with #eANSIESC_Sequence::ANSIESC_SETGRAPHICSMODE)
typedef enum
{
  TEXT_ATTR_ALL_OFF           = 0                                              , //!<  0 => All text attributes Off
  TEXT_ATTR_BOLD              = 1                                              , //!<  1 => Bold On
  TEXT_ATTR_ITALIC            = 3                                              , //!<  3 => Italic On
  TEXT_ATTR_UNDERSCORE        = 4                                              , //!<  4 => Underscore On
  TEXT_ATTR_REVERSE_VIDEO     = 7                                              , //!<  7 => Inverse or reverse; swap foreground and background
  TEXT_ATTR_CONCEALED         = 8                                              , //!<  8 => Concealed On
  TEXT_ATTR_CROSSED_OUT       = 9                                              , //!<  9 => Crossed out On
  TEXT_ATTR_BOLD_OFF          = ANSIESC_ATTRIBUTE_OFF + TEXT_ATTR_BOLD         , //!< 21 => Bold Off
  TEXT_ATTR_ITALIC_OFF        = ANSIESC_ATTRIBUTE_OFF + TEXT_ATTR_ITALIC       , //!< 23 => Italic Off
  TEXT_ATTR_UNDERSCORE_OFF    = ANSIESC_ATTRIBUTE_OFF + TEXT_ATTR_UNDERSCORE   , //!< 24 => Underscore Off
  TEXT_ATTR_REVERSE_VIDEO_OFF = ANSIESC_ATTRIBUTE_OFF + TEXT_ATTR_REVERSE_VIDEO, //!< 27 => Reverse video Off
  TEXT_ATTR_CONCEALED_OFF     = ANSIESC_ATTRIBUTE_OFF + TEXT_ATTR_CONCEALED    , //!< 28 => Concealed Off
  TEXT_ATTR_CROSSED_OUT_OFF   = ANSIESC_ATTRIBUTE_OFF + TEXT_ATTR_CROSSED_OUT  , //!< 29 => Crossed out Off
} eGraphicMode_TextAttributes;





//********************************************************************************************************************
// Functions to transmit ANSI Escape sequences
//********************************************************************************************************************
#ifdef USE_ANSI_ESCAPE_TX

/*! @brief Function for Cursor Movement with ANSI Escape sequences
 *
 * \e[#A - Moves the cursor up by the specified number of lines without changing columns. If the cursor is already on the top line, this sequence is ignored. \e[A is equivalent to \e[1A.
 * \e[#B - Moves the cursor down by the specified number of lines without changing columns. If the cursor is already on the bottom line, this sequence is ignored. \e[B is equivalent to \e[1B.
 * \e[#C - Moves the cursor forward by the specified number of columns without changing lines. If the cursor is already in the rightmost column, this sequence is ignored. \e[C is equivalent to \e[1C.
 * \e[#D - Moves the cursor back by the specified number of columns without changing lines. If the cursor is already in the leftmost column, this sequence is ignored. \e[D is equivalent to \e[1D.
 * \e[#E - Moves the cursor down the indicated # of rows, to column 1. \e[E is equivalent to \e[1E.
 * \e[#F - Moves the cursor up the indicated # of rows, to column 1. \e[F is equivalent to \e[1F.
 * \e[#G - Moves the cursor to indicated column in current row. \e[G is equivalent to \e[1G.
 * \e[s - Save Cursor Position: Saves the current cursor position. You can move the cursor to the saved cursor position by using the Restore Cursor Position sequence.
 * \e[u - Restore Cursor Position: Returns the cursor to the position stored by the Save Cursor Position sequence.
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] movement Is the movement of the cursor to apply
 * @param[in] count Is the count of movement to do
 */
void ANSIESC_Cursor(ConsoleTx* pApi, eANSIESC_Cursor movement, uint8_t count);



/*! @brief Function for set cursor position with ANSI Escape sequences
 *
 * \e[#;#H - Moves the cursor to the specified position. The first # specifies the line number, the second # specifies the column. If you do not specify a position, the cursor moves to the home position: the upper-left corner of the screen (line 1, column 1).
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] line Is the line where the cursor will be put
 * @param[in] column Is the column where the cursor will be put
 */
void ANSIESC_SetCursorPosition(ConsoleTx* pApi, uint8_t line, uint8_t column);



/*! @brief Function to clear screen or line with ANSI Escape sequences
 *
 * \e[0J - Clears the screen from cursor to end of display. The cursor position is unchanged.
 * \e[1J - Clears the screen from start to cursor. The cursor position is unchanged.
 * \e[2J - Clears the screen and moves the cursor to the home position (line 1, column 1).
 * \e[J is equivalent to \e[0J. (Some terminal/emulators respond to \e[J as if it were \e[2J. Here, the default is 0; it is the norm)
 * \e[0K - Clears all characters from the cursor position to the end of the line (including the character at the cursor position). The cursor position is unchanged.
 * \e[1K - Clears all characters from start of line to the cursor position. (including the character at the cursor position). The cursor position is unchanged.
 * \e[2K - Clears all characters of the whole line. The cursor position is unchanged.
 * \e[K is equivalent to \e[0K. (Some terminal/emulators respond to \e[K as if it were \e[2K. Here, the default is 0; it is the norm)
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] clear Indicate if this is a screen or a line to clear
 * @param[in] direction Is the direction of the clear
 */
void ANSIESC_Clear(ConsoleTx* pApi, eANSIESC_Clear clear, eANSIESC_ClearDirection direction);



/*! @brief Function for insertion/delete with ANSI Escape sequences
 *
 * \e[#L - The cursor line and all lines below it move down # lines, leaving blank space. The cursor position is unchanged. The bottommost # lines are lost. \e[L is equivalent to \e[1L.
 * \e[#M - The block of # lines at and below the cursor are deleted; all lines below them move up # lines to fill in the gap, leaving # blank lines at the bottom of the screen. The cursor position is unchanged. \e[M is equivalent to \e[1M.
 * \e[#@ - The cursor character and all characters to the right of it move right # columns, leaving behind blank space. The cursor position is unchanged. The rightmost # characters on the line are lost. \e[@ is equivalent to \e[1@.
 * \e[#P - The block of # characters at and to the right of the cursor are deleted; all characters to the right of it move left # columns, leaving behind blank space. The cursor position is unchanged. \e[P is equivalent to \e[1P.
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] command Indicate if this is an insert of a delete of a line or a char
 * @param[in] count Is the count of movement to do
 */
void ANSIESC_InsertDelete(ConsoleTx* pApi, eANSIESC_InsertDelete command, uint8_t count);



/*! @brief Function for setting graphic mode with ANSI Escape sequences
 *
 * \e[#;...;#m - Set Graphics Mode: Calls the graphics functions specified by the following values. These specified functions remain active until the next occurrence of this escape sequence. Graphics mode changes the colors and attributes of text (such as bold and underline) displayed on the screen. \e[m is equivalent to \e[0m.}
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] count Is the count of graphic mode parameters to send
 * @param[in] ... Are the set of graphic mode to apply
 */
void ANSIESC_SetGraphicMode(ConsoleTx* pApi, uint8_t count, ...);

//-----------------------------------------------------------------------------
#endif /* USE_ANSI_ESCAPE_TX */

//**********************************************************************************************************************************************************





//********************************************************************************************************************
// Functions to receive ANSI Escape sequences
//********************************************************************************************************************
#ifdef USE_ANSI_ESCAPE_RX



//-----------------------------------------------------------------------------
#endif /* USE_ANSI_ESCAPE_RX */





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* ANSIESCAPELIB_H_ */
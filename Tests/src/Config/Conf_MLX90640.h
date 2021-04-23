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
#ifndef CONF_MLX90640_H
#define CONF_MLX90640_H
//=============================================================================



// If in debug mode, check NULL parameters that are mandatory in each functions of the driver
#ifdef DEBUG
#  define CHECK_NULL_PARAM
#endif


// This define specify to the driver to pre-calculate Offset, Sensitivity, Kta, and Kv of each pixel and save them in the MLX90640_Parameters
// If the following define is set, the driver will take 768x2+768x4x3 = 10704 bytes of RAM more to store theses values
// If unset then the driver will take less ram but more time to calculate To of each pixels
#define MLX90640_PRECALCULATE_PIXELS_COEFFS


// In order to limit the noise in the final To calculation it is advisable to filter the CP readings at this point of calculation
// This filter is only useful with device with thermal gradient compensation
// A good practice would be to apply a Moving Average Filter with length of 16 or higher
// If set to < 2, the filter will be disabled
//#define MLX90640_MOVING_AVERAGE_FILTER_VALUES_COUNT  ( 16 )



//-----------------------------------------------------------------------------
#endif /* CONF_MLX90640_H */

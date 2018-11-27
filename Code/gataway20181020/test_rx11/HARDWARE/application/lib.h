/*! ----------------------------------------------------------------------------
 *  @file    lib.h
 *  @brief   DecaWave header for general purpose library functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _LIB_H_
#define _LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

// Give the rounded up result of a division between two positive integers.
// param  a  numerator
// param  b  denominator
// return  rounded up result of the division
#define CEIL_DIV(a,b) (((a) + (b) - 1) / (b))

#ifdef __cplusplus
}
#endif

#endif // _LIB_H_

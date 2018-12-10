/*
********************************************************************************
*                            Library.h
*
* File          : Library.h
* Version       : V1.0
* Author        : whq
* Mode          : Thumb2
* Toolchain     :                      
* Description   : 库函数
*
* Date          : 2013/12/11
* History		: 
* 
*******************************************************************************/


#ifndef _LIBRARY_H_
#define _LIBRARY_H_

#include "stdint.h"


#ifdef  __cplusplus
extern "C"
{
#endif


/******************************宏定义*******************************************/


//#define _OS_WINDOWS_
//#define _OS_LINUX_


#define MAX(x,y)                (((x) > (y))? (x) : (y))
#define MIN(x,y)                (((x) < (y))? (x) : (y))  
#define ABS(x)                  ((x)>0 ? (x) : -(x))  
#define CHECKVAL(val, min,max)  ((val < min || val > max) ? 0 : 1)

/******************************类型声明*****************************************/


typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
}BCDTIME_t;                 //BCD 时间结构




typedef struct {
    int8_t state;           //当前状态
    int16_t judgeCount;     //已经判定的次数
}STATE_JUDGE_t;




/******************************函数声明****************************************/

uint8_t LIB_BcdToInt8(uint8_t val);
uint8_t LIB_Int8ToBcd(uint8_t val);
uint32_t LIB_StringBcdToInt(uint8_t *str, uint16_t len);
void LIB_IntToStringBcd(uint8_t *str, uint16_t size, uint32_t val);
uint16_t LIB_Int16ByteReversed(uint16_t val);
uint32_t LIB_Int32ByteReversed(uint32_t val);
uint8_t LIB_CheckXOR(uint8_t *pBuf, uint16_t len);
uint8_t LIB_CheckSum(uint8_t *pBuf, uint16_t len);
uint8_t LIB_CheckXORExt(uint8_t cXor, uint8_t *pBuf, uint16_t len);
uint8_t LIB_CheckSumExt(uint8_t sum, uint8_t *pBuf, uint16_t len);
uint32_t LIB_BCDTime2Sec(BCDTIME_t *time);
void LIB_Sec2BCDTime(BCDTIME_t *tim, uint32_t sec);

int32_t LIB_StatusFilter(STATE_JUDGE_t *pState, int8_t newState, int16_t judgeCount);

#ifdef  __cplusplus
}
#endif

#endif
/******************************END*********************************************/

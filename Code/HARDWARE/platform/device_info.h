#ifndef _DEVICE_INFO_H_
#define _DEVICE_INFO_H_
#include "stm32f10x.h"	

#define SN   0x01
#define SW   0x02
#define DECA 0x03
#define ADMI 0x04
#define PSWD 0x05
#define SVIP 0x06
#define SVPT 0x07
typedef struct
{
	u16 flag;
	u16 start_count;
	u16 device_switch;
	u16 hw_version[4];
	u16 sw_version[4];
} System_Para_TypeDef; 

extern u16 FLAH_BUFF0[16];
extern u8 buff_version[19];
extern u8 buff_sn[41];
extern u8 buff_wifipassword[10];
extern System_Para_TypeDef sys_para;


u8 Check_cmd(u8* RXbuff);
u8 Get_UniqueID(void);
void Flash_Configuration(void);
u8 RouterInfo_Write_to_Flash(u8 OBJ, u8* buffer);
void My_STMFLASH_Write(void);
#endif


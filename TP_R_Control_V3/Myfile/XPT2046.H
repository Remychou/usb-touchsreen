/********************************** (C) COPYRIGHT *******************************
* File Name          : XPT2046.H
* Author             : RZ
* Version            : V1.00
* Date               : 2017-6-10 11:42:03
* Description        : 触摸控制芯片XPT2046头文件
*******************************************************************************/
#ifndef __XPT2046_H_
#define __XPT2046_H_

#include "CH554.h"


/* GPIO REMAP */
sbit XPT2046_CLK 	= P1^7;
sbit XPT2046_DOUT 	= P1^6; /* MISO */
sbit XPT2046_DIN 	= P1^5;	/* MOSI */
sbit XPT2046_CS 	= P1^4;
sbit XPT2046_BUSY 	= P3^1; 
sbit XPT2046_PEN 	= P3^4;

/* CMD DEFINE */
/********************************************************************************
* +---+----+----+----+------+----------+-----+-----+
* | 7 |  6 |  5 |  4 |   3  |    2     |  1  |	0  |
* +---+----+----+----+------+----------+-----+-----+
* | S | A2 | A1 | A0 | MODE | SER/DEFR | PD1 | PD0 |
* +---+----+----+----+------+----------+-----+-----+
* S: 		START OF DATA
* A2-A0: 	CHANNEL SELEC
* MODE: 	RESOLUTION SELECT
* SEF/DFR: 	SINGLE/DIFFERENCE INPUT SELECT
* PD1-PD0:  POWER MODE, 11: POWERED, 00: SHUT
********************************************************************************/
#define		DEF_XPOS			(0XD3)	
#define 	DEF_YPOS			(0X93)
#define 	DEF_SAMPLE_CNT		(16)

/* VARIABLE DEFINE */
struct POINT
{
	UINT16 X_Pos;
	UINT16 Y_Pos;
};

UINT16 	Low_Pass_Filter			( UINT16 New_Data, UINT16 Old_Data, UINT8 Factor );
void 	XPT2046_SendCommand		(UINT8 ucCommand);			//发送一个命令
void 	XPT2046_Init			( void );
UINT16 	XPT2046_ReadData		(UINT8 ucCommand);			//读取一个数据
UINT16 	xpt2046_Get_Touch_Data	( UINT8 XorY );
UINT16 	Pos_ABS					( UINT16 m, UINT16 n );

#endif

/* END OF FILE  */
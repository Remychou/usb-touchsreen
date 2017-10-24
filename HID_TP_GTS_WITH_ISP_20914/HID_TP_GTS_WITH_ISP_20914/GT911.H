/* REG INFO */
/********************************** (C) COPYRIGHT *******************************
* File Name          : GT911.H
* Author             : RZ
* Version            : V1.0
* Date               : 2017-5-15 
* Description        : define of TP IC GT911
*******************************************************************************/

/******************************************************************************
* MAX FREQ 400K Hz
* DEV_ADDR_T = DEV_ADDR + W/R
* HOST WRITE 
* +-------+----------+-----------+-----+------------+-----+------------+-----+----------+-----+-----+------+
* | START | DEV_ADDR | WRITE (0) | ACK | REG_ADDR_H | ACK | REG_ADDR_L | ACK | DATA ... | ... | ACK | STOP |
* +-------+----------+-----------+-----+------------+-----+------------+-----+----------+-----+-----+------+
* HOST READ 
* +-------+----------+----------+-----+------------+-----+------------+------+-------+----------+----------+-----+------+-----+-----+-----+-----+------+
* | START | DEV_ADDR | READ (1) | ACK | REG_ADDR_H | ACK | REG_ADDR_L | STOP | START | DEV_ADDR | READ (1) | ACK | DATA | ACK | ... | ... | NAK | STOP |
* +-------+----------+----------+-----+------------+-----+------------+------+-------+----------+----------+-----+------+-----+-----+-----+-----+------+
******************************************************************************/

#ifndef __GT911_H__
#define __GT911_H__

#ifdef __cplusplus
extern "C" {
#endif

#include 	"CH554.H"

#define		DEF_CHANGE_X_Y								(1)
	
#define 	BIT0										(0X01)
#define 	BIT1										(0X02)
#define 	BIT2										(0X04)
#define 	BIT3										(0X08)
#define 	BIT4										(0X10)
#define 	BIT5										(0X20)
#define 	BIT6										(0X40)
#define 	BIT7										(0X80)

#define		DEF_SCREEN_MAX_X							(0)
#define 	DEF_SCREEN_MAX_Y							(0)	

#define 	DEF_X_MULTI									(1024/800)			/* 1.28 */
#define		DEF_Y_MULTI									(600/480)			/* 1.25 */
	
	
#define 	DEF_IIC_MAX_FREQ							(400*1000)
/* DEVICE ADDRESS */	
//#define		DEF_TP_DEV_ADDR								(0x5D)			/* DEV_ADDR */
#define		DEF_TP_DEV_ADDR								(0x14)				/* DEV_ADDR */
//#define 	GT_CMD_WR									(DEF_TP_DEV_ADDR << 1 + 0)
//#define		GT_CMD_RD									(DEF_TP_DEV_ADDR << 1 + 1)

//I2C读写命令	
#define GT_CMD_WR 		0X28     	//写命令
#define GT_CMD_RD 		0X29		//读命令
  
//GT9147 部分寄存器定义 
#define GT_CTRL_REG 	0X8040   	//GT9147控制寄存器
#define GT_CFGS_REG 	0X8047   	//GT9147配置起始地址寄存器
#define GT_CHECK_REG 	0X80FF   	//GT9147校验和寄存器
#define GT_PID_REG 		0X8140   	//GT9147产品ID寄存器

#define GT_GSTID_REG 	0X814E   	//GT9147当前检测到的触摸情况
#define GT_TP1_REG 		0X814F  	//第一个触摸点数据地址
#define GT_TP2_REG 		0X8157		//第二个触摸点数据地址
#define GT_TP3_REG 		0X815F		//第三个触摸点数据地址
#define GT_TP4_REG 		0X8167		//第四个触摸点数据地址
#define GT_TP5_REG 		0X816F		//第五个触摸点数据地址  
#define GT_TP6_REG 		0X8177		//第六个触摸点数据地址  
#define GT_TP7_REG 		0X817F		//第七个触摸点数据地址  
#define GT_TP8_REG 		0X8187		//第八个触摸点数据地址  
#define GT_TP9_REG 		0X818F		//第九个触摸点数据地址  
#define GT_TP10_REG 	0X8197		//第十个触摸点数据地址  

/* CMD RIGHT TIME, WRITE ONLY */
#define 	DEF_GTS_CMD									(0X8040)			/* WRITE ONLY */
#define 	DEF_GTS_ESD_CHK								(0X8041)

/* CFG INFO */
#define 	DEF_GTS_CFG_VERSION							 (0X8047)			/* 'A' ~ 'Z' */

/* VARIABLE DEFINE   */
#define 	POINTER_NUM									 (1)				/*  5 OR 10*/

typedef struct
{
	UINT8 Tip_Switch;
	UINT8 Contact_Identifier;
	UINT16 X_pos;		/* coordinate X */
	UINT16 Y_pos;		/* coordinate Y */
	UINT16 Resolution_Multi;
//	UINT8 Pressure;
//	UINT8 EventId;		/* 0: none; 1: down; 2: move; 3: stay; 4 up */
}POINTER;

typedef struct 
{
	UINT16 	x_max_pos;
	UINT16 	y_max_pos;
	UINT8 	x_mirror;
	UINT8	y_mirror;
	UINT8 	x_y_swap;
	double 	X_Resolution;
	double 	Y_Resolution;	
	UINT8 	Point_Num;
	UINT8 	IRQ_Flag;
}_GT911_Info ;
	
extern POINTER TP[POINTER_NUM];
extern _GT911_Info GT911_Info;

/* Function Define */
extern UINT8 	GT911_Init					( void );
extern void 	GT911_Get_Touch_Info		( void );
extern UINT8 	GT911_Touch_Check			( void );
extern void 	GT911_Gesture_Check			( void );
extern void 	GT911_Get_Gesture_Info		( void );
extern void 	GT911_Config				( void );
extern UINT8 	GT911_Scan					( void );
#ifdef __cplusplus
}
#endif

#endif

/* END OF FILE */
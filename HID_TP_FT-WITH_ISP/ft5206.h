/* REG INFO */
/********************************** (C) COPYRIGHT *******************************
* File Name          : FT5206.H
* Author             : RZ
* Version            : V1.0
* Date               : 2017-5-15 
* Description        : define of TP IC FT5206
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

#ifndef __FT5206_H__
#define __FT5206_H__

#ifdef __cplusplus
extern "C" {
#endif

#include 	"CH554.H"



#define 	BIT0					(0X01)
#define 	BIT1					(0X02)
#define 	BIT2					(0X04)
#define 	BIT3					(0X08)
#define 	BIT4					(0X10)
#define 	BIT5					(0X20)
#define 	BIT6					(0X40)
#define 	BIT7					(0X80)

sbit 		FT5206_RST_PIN	=  		P1^2;
sbit		FT5206_INT_PIN	= 		P3^2;
sbit		FT5206_SCL		=		P1^4;
sbit		FT5206_SDA		=		P1^5;

//I2C��д����
#define DEF_TP_DEV_ADDR 			0X38    	/* ADDRESS */
#define FT_CMD_WR 					0X70    	/* д���� */
#define FT_CMD_RD 					0X71		/* ������ */
                                         
//FT5206 ���ּĴ�������                   
#define DEF_FT_DEVIDE_MODE 			0x00   		/* FT5206ģʽ���ƼĴ��� */
#define DEF_FT_REG_NUM_FINGER       0x02		/* ����״̬�Ĵ��� */
                                           
#define DEF_FT_TP1_REG 				0X03	  	/* ��һ�����������ݵ�ַ */
#define DEF_FT_TP2_REG 				0X09		/* �ڶ������������ݵ�ַ */
#define DEF_FT_TP3_REG 				0X0F		/* ���������������ݵ�ַ */
#define DEF_FT_TP4_REG 				0X15		/* ���ĸ����������ݵ�ַ */
#define DEF_FT_TP5_REG 				0X1B		/* ��������������ݵ�ַ */
                                            
                        
#define	DEF_FT_ID_G_LIB_VERSION		0xA1		/* �汾 */
#define DEF_FT_ID_G_MODE 			0xA4   		/* FT5206�ж�ģʽ���ƼĴ��� */
#define DEF_FT_ID_G_THGROUP			0x80   		/* ������Чֵ���üĴ��� */
#define DEF_FT_ID_G_PERIODACTIVE	0x88   		/* ����״̬�������üĴ��� */

#define TP_PRES_DOWN 0x80  //����������	  
#define TP_CATH_PRES 0x40  //�а��������� 
#define POINTER_NUM					0x05
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
	double 	X_Resolution;
	double 	Y_Resolution;
	UINT8 	x_y_swap;
	UINT8 	Point_Num;
	UINT8 	IRQ_Flag;
}_FT5206_Info ;



extern POINTER TP[POINTER_NUM];
extern _FT5206_Info FT5206_Info;

/* Function Define */
extern UINT8 	FT5206_Init					( void );
extern void 	FT5206_Get_Touch_Info		( void );
extern UINT8 	FT5206_Touch_Check			( void );
extern void 	FT5206_Gesture_Check		( void );
extern void 	FT5206_Get_Gesture_Info		( void );
extern void 	FT5206_Config				( void );
extern void  	FT5206_Scan					( void );
#ifdef __cplusplus
}
#endif

#endif

/* END OF FILE */



















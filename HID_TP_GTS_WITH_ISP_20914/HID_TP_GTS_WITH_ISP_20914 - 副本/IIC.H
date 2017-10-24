/********************************** (C) COPYRIGHT *******************************
* File Name          : IIC.H
* Author             : RZ
* Version            : V1.00
* Date               : 2017-5-15
* Description        : I2C ¶ÁÐ´
*******************************************************************************/

#ifndef __IIC_H__
#define __IIC_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "CH554.H"
void 	I2c_Init		( void );
void 	IIC_Start		( void );
void 	IIC_Stop		( void );
void 	IIC_Ack			( void );
void 	IIC_Nack		( void );
UINT8	IIC_Wait_Ack	( void );
UINT8 	IIC_Wait_Ack	( void );
void 	IIC_Send_Byte	( UINT8 txd );
UINT8  	IIC_Read_Byte	( UINT8 ack );
	
#ifdef __cplusplus
}
#endif

#endif  

/* END OF FILE */

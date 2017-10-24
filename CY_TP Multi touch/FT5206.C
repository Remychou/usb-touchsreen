/********************************** (C) COPYRIGHT *******************************
* File Name          : FT5206.C
* Author             : RZ
* Version            : V1.00
* Date               : 2017-5-15 
* Description        : 电容触摸IC触摸IC驱动
*******************************************************************************/

#include "FT5206.H"
#include "FLASH_IC.H"
#include "IIC.H"
#include "DEBUG.H"
#include "DEVICE.H"
#include <stdio.h>
#include <string.h>

//sbit 		FT5206_RST_PIN	=  		P1^2;
//sbit		FT5206_INT_PIN	= 		P3^2;

#define 	FT5206_RST( x )				( FT5206_RST_PIN = x )
#define		FT5206_INT( x )				( FT5206_INT_PIN = x )
#define 	FT5206_INT_DAT( )			( FT5206_INT_PIN )

const UINT16 FT5206_TPX_TBL[5] =
{
	DEF_FT_TP1_REG,
	DEF_FT_TP2_REG,
	DEF_FT_TP3_REG,
	DEF_FT_TP4_REG,
	DEF_FT_TP5_REG
};

POINTER 		TP[POINTER_NUM]		= 		{0};
_FT5206_Info 	idata FT5206_Info 		= 		{0};

/*******************************************************************************
* Function Name  : FT5206_WR_Reg
* Description    : 向FT5206写入一次数据
                   reg:起始寄存器地址
                   buf:数据缓缓存区
                   len:写数据长度
                   返回值:0,成功;1,失败.
* Input          : UINT16 Addr ,UINT8 dat
* Output         : None
* Return         : bit
*******************************************************************************/
UINT8 FT5206_WR_Reg(UINT16 reg,UINT8 *buf,UINT8 len)
{
	UINT8 i;
	UINT8 ret=0;
	IIC_Start();	 
	IIC_Send_Byte(FT_CMD_WR); 
	IIC_Wait_Ack(); 
	IIC_Send_Byte(reg>>8); 
	IIC_Wait_Ack(); 
	IIC_Send_Byte(reg&0XFF); 
	IIC_Wait_Ack(); 
	for(i=0;i<len;i++)
	{ 
    	IIC_Send_Byte(buf[i]); 
		ret=IIC_Wait_Ack();
		if(ret)break; 
	}
    IIC_Stop(); 
	return ret; 
}
#if 0
/*******************************************************************************
* Function Name  : FT5206_RD_Reg
* Description    : 从FT5206读出一次数据
                   reg:起始寄存器地址
                   buf:数据缓缓存区
                   len:读数据长度			 
* Input          : UINT16 reg,
				   UINT8 *buf,
				   UINT8 len
* Output         : None
* Return         : None
*******************************************************************************/
void FT5206_RD_Reg(UINT16 reg,UINT8 *buf,UINT8 len)
{
	UINT8 i; 
 	IIC_Start();	
 	IIC_Send_Byte(FT_CMD_WR);   	//发送写命令 	 
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	IIC_Wait_Ack();  
 	IIC_Start();  	 	   
	IIC_Send_Byte(FT_CMD_RD);   	//发送读命令		   
	IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    IIC_Stop();//产生一个停止条件     
}
#endif
/*
* IIC protocol 
* READ DATA:
* Device Address, Packet length LSB, Packet length MSB, Packet ID, Packet Data 1, Packet Data 2, ... stop.
*/
#define 	PACKET_ID		(0XF6)
#define		PACKET_LEN		(0XEE)

void CY568_RD_Reg(UINT16 packet_id,UINT8 *buf,UINT8 packet_len)
{
	UINT8 i; 
 	IIC_Start();	
 	IIC_Send_Byte(FT_CMD_WR);   				/* send read addr */
	IIC_Wait_Ack(); 	 										  		   
 	
	IIC_Send_Byte( packet_len & 0xff );   		/* packet length LSB */
	IIC_Wait_Ack();
	IIC_Send_Byte( packet_len >> 8 );   		/* packet length MSB */
	IIC_Wait_Ack();  

	IIC_Send_Byte( packet_id );   				/* packet ID */
	IIC_Wait_Ack();  
	
	for(i=0;i<packet_len;i++)					/*  */
	{	   
    	buf[i]=IIC_Read_Byte(i==(packet_len-1)?0:1); //发数据	  
	} 
    IIC_Stop();//产生一个停止条件     
} 



void FT5206_RD_Reg(UINT16 reg,UINT8 *buf,UINT8 len)
{
	UINT8 i;
	UINT8 readedLen=1;
 	IIC_Start();	
 	IIC_Send_Byte(0x48);
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Send_Byte(0x03);
	IIC_Wait_Ack(); 	 										  		   
 	IIC_Send_Byte(0x00);
	IIC_Wait_Ack(); 	 										  		   
	IIC_Stop();

 	IIC_Start();	
 	IIC_Send_Byte(0x49);
	IIC_Wait_Ack(); 	 										  		   

	for(i=0;i<readedLen;i++){
    buf[i]=IIC_Read_Byte(1); 
		readedLen=buf[0];
	} 
	IIC_Stop();
}



/*******************************************************************************
* Function Name  : FT5206_Init
* Description    : 初始化FT5206触摸屏
                   返回值:0,初始化成功;1,初始化失败 
* Input          : None
* Output         : None
* Return         : UINT8
*******************************************************************************/
UINT8 FT5206_Init(void)
{
	FT5206_INT_PIN = 1;
	FT5206_RST(0);												/* 复位 */
	mDelaymS(20);
 	FT5206_RST(1);												/* 释放复位 */
	mDelaymS(50);  	
	
	Get_Config_Info();
	FT5206_Info.X_Resolution = 2048 / (double)FT5206_Info.x_max_pos;
	FT5206_Info.Y_Resolution = 2048 / (double)FT5206_Info.y_max_pos;
	
//	printf("Resolution:%f,%f\n",X_Resolution,Y_Resolution);
	return 1;
}

UINT8 SerialData[10] = { 
	0X54, 0X55,						/* start */
	0X00,0X00,0X00,0X00,0X00,		/* pos */
	0X0FF,0X00,						/* end */
	0X00};							/* check out */		

/*******************************************************************************
* Function Name  : FT5206_Scan
* Description    : 扫描触摸屏(采用查询方式)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 FT5206_Scan( void )
{
	UINT8 i=0;
	UINT8 TP_Value[PACKET_LEN] = {0};
//	static t;
//	t++;
//	if( t > 10 )
//	{
//		t = 0;
		
//		memset( TP_Value, 0,  PACKET_LEN );

		FT5206_RD_Reg( 0X03,TP_Value,0X11);//读取触摸点的状态  
		if( TP_Value[0] >= 0x11 )
		{
			FT5206_Info.Point_Num = ( TP_Value[0] -7 )/10;
		}
		else
		{
			return 0;
		}		
//		printf("%d\n",(UINT16)FT5206_Info.Point_Num );

		
		for( i = 0; i < FT5206_Info.Point_Num; i++ )
		{
			TP[i].Tip_Switch = (TP_Value[8+10*i] & 0X80)? 1:0;
			TP[i].Contact_Identifier = TP_Value[8+10*i] & 0X0F;
			TP[i].Resolution_Multi = 0X0030;
			
			if( FT5206_Info.x_y_swap == 1 )
			{
				TP[i].Y_pos = ( TP_Value[ 9+10*i] + ( (UINT16)TP_Value[10+10*i]<<8 ) )  * FT5206_Info.Y_Resolution; 	
				TP[i].X_pos = ( TP_Value[11+10*i] + ( (UINT16)TP_Value[12+10*i]<<8 ) )  * FT5206_Info.X_Resolution; 				
			}
			else
			{

				TP[i].X_pos = ( TP_Value[ 9+10*i] + ( (UINT16)TP_Value[10+10*i]<<8 ) )  * FT5206_Info.X_Resolution; 
				TP[i].Y_pos = ( TP_Value[11+10*i] + ( (UINT16)TP_Value[12+10*i]<<8 ) )  * FT5206_Info.Y_Resolution; 				
			}
			
			
			if ( FT5206_Info.x_mirror == 1 )
			{
				TP[i].X_pos = 2048 - TP[i].X_pos;
			}
			
			
			if ( FT5206_Info.y_mirror == 1 )
			{
				TP[i].Y_pos = 2048 - TP[i].Y_pos;
			}
			
		}
#if 0
		for( i = 0; i < 30; i++ )
		{
			printf("%d  ",(UINT16)TP_Value[i]);
		}
		printf("\n");

//		printf("Resolution:%f,%f\n",X_Resolution,Y_Resolution);

		for( i = 0; i < FT5206_Info.Point_Num ; i++ )
		{
			printf("TP_Switch:%d\tID:%d\tX_pos:%d\tY_pos:%d\n",
				(UINT16)TP[i].Tip_Switch,(UINT16)TP[i].Contact_Identifier,(UINT16)TP[i].X_pos,(UINT16)TP[i].Y_pos );
		}
#endif		
		return 1;
//	}
//	
//	return 0;


}

/*******************************************************************************
* Function Name  : FT5206_Touch_Check
* Description    : 按键检测与上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 FT5206_Touch_Check( void )
{
//	if( FT5206_Info.IRQ_Flag == 1 )
//	{
//		FT5206_Info.IRQ_Flag = 0;		
		
		if( FT5206_Scan() == 1)
		{
			Absolute_Up_Pack( &TP, FT5206_Info.Point_Num );
		}
//		EX0 = 1;
//	}

	return 0;
}

/*******************************************************************************
* Function Name  : FT5206_ISR
* Description    : 外部中断0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void    FT5206_ISR( void ) interrupt INT_NO_INT0 using 1                    //USB中断服务程序,使用寄存器组1
{
	EX0 = 0;
	FT5206_Info.IRQ_Flag = 1;
#if DE_PRINTF	
	printf("INT\n");
#endif	
}

/* END OF FILE */
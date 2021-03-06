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

sbit 		LED_IO			=		P1^3;
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
_FT5206_Info 	FT5206_Info 		= 		{0};

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
	UINT8 temp[2];
	FT5206_RST(0);				//复位
	mDelaymS(20);
 	FT5206_RST(1);				//释放复位		    
	mDelaymS(50);

	LED_IO = 0;

#if 0
	temp[0]=0;
	FT5206_WR_Reg(DEF_FT_DEVIDE_MODE,temp,1);	//进入正常操作模式 
	temp[0]=0;
	FT5206_WR_Reg(DEF_FT_ID_G_MODE,temp,1);		//查询模式 
	temp[0]=12;								//触摸有效值，22，越小越灵敏	
	FT5206_WR_Reg(DEF_FT_ID_G_THGROUP,temp,1);	//设置触摸有效值
	temp[0]=12;								//激活周期，不能小于12，最大14
	FT5206_WR_Reg(DEF_FT_ID_G_PERIODACTIVE,temp,1); 
	//读取版本号，参考值：0x3003
	FT5206_RD_Reg(DEF_FT_ID_G_LIB_VERSION,&temp[0],2);  
	if(temp[0]==0X30&&temp[1]==0X03)//版本:0X3003
	{
#if DE_PRINTF		
		printf("CTP ID:%x\r\n",((UINT16)temp[0]<<8)+temp[1]);
		FT5206_RD_Reg(0xA8,&temp[0],1); 
		printf("A8h:0x%02x\r\n",((UINT16)temp[0]));//0x82
		FT5206_RD_Reg(0xA6,&temp[0],1); 
		printf("A6h:0x%02x\r\n",((UINT16)temp[0]));//0x17 
		FT5206_RD_Reg(0xA8,&temp[0],1); 
		printf("A2h:0x%02x\r\n",((UINT16)temp[0]));//0x01 
		FT5206_RD_Reg(0xA3,&temp[0],1);
		printf("A3h:0x%02x\r\n",((UINT16)temp[0]));//0x54  
		FT5206_RD_Reg(0x01,&temp[0],1); 
		printf("01h:0x%02x\r\n",((UINT16)temp[0]));//0xff
		FT5206_RD_Reg(0x02,&temp[0],1);
		printf("0x02:0x%02x\r\n",((UINT16)temp[0]));//
#endif		
		return 0;
	}
#endif
	Get_Config_Info();
	FT5206_Info.X_Resolution = 2048 / (double)FT5206_Info.x_max_pos;
	FT5206_Info.Y_Resolution = 2048 / (double)FT5206_Info.y_max_pos;
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
void FT5206_Scan( void )
{
	UINT8 mode;
	UINT8 buf[4];
	UINT8 i=0;
	
	FT5206_RD_Reg(DEF_FT_REG_NUM_FINGER,&mode,1);//读取触摸点的状态  
	FT5206_Info.Point_Num = mode;
#if DE_PRINTF	
	printf("Point_Num:%d\n",(UINT16)FT5206_Info.Point_Num);
#endif
	for( i=0; i < POINTER_NUM; i++)
	{
		FT5206_RD_Reg( FT5206_TPX_TBL[i], buf, 4 );	//读取XY坐标值 
		
		TP[i].Tip_Switch = ( ( buf[0] & 0XF0 ) != 0x80 ) ? 0 : 1;
		if( FT5206_Info.Point_Num == 0 )
		{
			continue;
		}
		if( ( TP[i].Tip_Switch != 0 ) || ( i == 0 ) )
		{
			TP[i].Contact_Identifier = i;

			if( FT5206_Info.x_y_swap == 1 )
			{			
				TP[i].Y_pos = ( ( (UINT16)( buf[2] & 0X0F ) << 8 ) + buf[3] ) * FT5206_Info.X_Resolution;
				TP[i].X_pos = ( ( (UINT16)( buf[0] & 0X0F ) << 8 ) + buf[1] ) * FT5206_Info.Y_Resolution;
			}
			else
			{
				TP[i].X_pos = ( ( (UINT16)( buf[2] & 0X0F ) << 8 ) + buf[3] ) * FT5206_Info.X_Resolution;
				TP[i].Y_pos = ( ( (UINT16)( buf[0] & 0X0F ) << 8 ) + buf[1] ) * FT5206_Info.Y_Resolution;
	
			}
			if ( FT5206_Info.x_mirror == 1 )
			{
				TP[i].X_pos = 2048 - TP[i].X_pos;
			}
			
			
			if ( FT5206_Info.y_mirror == 1 )
			{
				TP[i].Y_pos = 2048 - TP[i].Y_pos;
			}		
			
			TP[i].Resolution_Multi = 0x0030;
		}
		else
		{
			TP[i].Contact_Identifier = 0;
			TP[i].Y_pos = 0;
			TP[i].X_pos = 0;
			TP[i].Resolution_Multi = 0;
		}
		
#if DE_PRINTF
		printf("buf[%d]:%02x\n",(UINT16)i,(UINT16)(buf[0]));
			printf("TP[%d].Tip_Switch:%02d\tTP[%d].Contact:%02d\n",
				(UINT16)i,(UINT16)TP[i].Tip_Switch,(UINT16)i,(UINT16)TP[i].Contact_Identifier);					
			printf("TP[%d].X_pos:%d\tTP[%d].Y_pos:%d\t\n",
			(UINT16)i,(UINT16)TP[i].X_pos,(UINT16)i,(UINT16)TP[i].Y_pos );

#endif
	}

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
	static UINT8 t;
	if( FT5206_Info.IRQ_Flag == 1 )
	{
		FT5206_Info.IRQ_Flag = 0;		
		
		FT5206_Scan();
		Absolute_Up_Pack( &TP, FT5206_Info.Point_Num );
		
		EX0 = 1;
		if( t++ > 10  )
		{
			t = 0;
			LED_IO ^= 1;
		}
		
	}
//	else
//	{
//		LED_IO = 0;
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
//	printf("INT\n");
}

/* END OF FILE */
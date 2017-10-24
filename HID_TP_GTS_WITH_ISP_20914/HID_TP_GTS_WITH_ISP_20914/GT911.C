/********************************** (C) COPYRIGHT *******************************
* File Name          : GT911.C
* Author             : RZ
* Version            : V1.00
* Date               : 2017-5-15 
* Description        : 电容触摸IC触摸IC驱动
*******************************************************************************/

#include "GT911.H"
#include "FLASH_IC.H"
#include "IIC.H"
#include "DEBUG.H"
#include "DEVICE.H"
#include <stdio.h>
#include <string.h>

sbit 		GT911_RST_PIN	=  		P1^2;
sbit		GT911_INT_PIN	= 		P3^2;

#define 	GT911_RST( x )			( GT911_RST_PIN = x )
#define		GT911_INT( x )			( GT911_INT_PIN = x )
#define 	GT911_INT_DAT( )		( GT911_INT_PIN )

POINTER 	TP[POINTER_NUM] = 		{ 0 };
_GT911_Info GT911_Info		=		{ 0 };

const UINT16 GT911_TPX_TBL[10]=			/* 兼容所有系列，10个手指都支持 */
{			
	GT_TP1_REG,
	GT_TP2_REG,
	GT_TP3_REG,
	GT_TP4_REG,
	GT_TP5_REG,
	GT_TP6_REG,
	GT_TP7_REG, 
	GT_TP8_REG, 
	GT_TP9_REG, 
	GT_TP10_REG
};


const UINT8 GT911_CFG_TBL[]=
{ 
	0X67,				/* CFG VERSION */
	0xff,0X00,			/* X_MAX = 0X01E0 (480) */
	0xff,0x00,			/* Y_MAX = 0X0320 (800) */
	0X10,				/* TOUCH_NUM = 5 */
	0X35,				/* MODULE1 = 0X35 BIT3 CHANGE XY AXIS */
	0x00,0x01,0x08,		/* MODULE2, SHAKE_CNT, FILTER */
	0X28,0X05,0X5A,		/* LARGE_TOUCH, NOISE, TOUCH_LEVER */
	0X3C,0X03,0X10,		/* LEAVE_LEVEL, LPM_CTL, REFRESH_RATE ... */
	
};  


/*******************************************************************************
* Function Name  : GT911_WR_Reg
* Description    : 向GT911写入一次数据
* Input          : reg:起始寄存器地址
                   buf:数据缓缓存区
                   len:写数据长度
* Output         : None
* Return         : 0,成功;1,失败.
*******************************************************************************/
static UINT8 GT911_WR_Reg(UINT16 reg,UINT8 *buf,UINT8 len)
{
	UINT8 i;
	UINT8 ret=0;
	IIC_Start();
 	IIC_Send_Byte(GT_CMD_WR);   	//发送写命令 	 
	IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);   	//发送高8位地址
	IIC_Wait_Ack();
	IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	IIC_Wait_Ack();
	for(i=0;i<len;i++)
	{
    	IIC_Send_Byte(buf[i]);  	//发数据
		ret=IIC_Wait_Ack();
		if(ret)break;  
	}
    IIC_Stop();					//产生一个停止条件	    
	return ret; 
}

/*******************************************************************************
* Function Name  : GT911_RD_Reg
* Description    : 从GT911读出一次数据
* Input          : reg:起始寄存器地址
                   buf:数据缓缓存区
                   len:读数据长度			  
* Output         : None
* Return         : None
*******************************************************************************/
static void GT911_RD_Reg(UINT16 reg,UINT8 *buf,UINT8 len)
{
	UINT8 i; 
 	IIC_Start();	
 	IIC_Send_Byte(GT_CMD_WR);   					/* 发送写命令 */
	IIC_Wait_Ack(); 
 	IIC_Send_Byte(reg>>8);   						/* 发送高8位地址 */
	IIC_Wait_Ack(); 
 	IIC_Send_Byte(reg&0XFF);   						/* 发送低8位地址 */
	IIC_Wait_Ack(); 
	
 	IIC_Start(); 
	IIC_Send_Byte(GT_CMD_RD);   					/* 发送读命令 */
	IIC_Wait_Ack(); 
	for(i=0;i<len;i++) 
	{
    	buf[i]=IIC_Read_Byte(i==(len-1)?0:1); 		/* 发数据 */
	} 
    IIC_Stop();										/* 产生一个停止条件 */
} 

/*******************************************************************************
* Function Name  : GT911_Config
* Description    : 获取与配置触摸IC配置信息
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 GT911_Send_Cfg(UINT8 mode)
{
	UINT8 Cfg_Info[32] = {0};
	UINT8 buf[2];
	UINT8 i=0;

	buf[0]=0;
	buf[1]=mode;								/*是否写入到GT911 FLASH?  即是否掉电保存 */
	for( i=0; i < sizeof( GT911_CFG_TBL ); i++ )
	{
		buf[0] += GT911_CFG_TBL[i];				/* 计算校验和*/
	}
    buf[0] = ( ~buf[0] ) + 1;
	
	GT911_WR_Reg( GT_CFGS_REG, GT911_CFG_TBL, sizeof(GT911_CFG_TBL) );//发送寄存器配置
	GT911_WR_Reg( GT_CHECK_REG, buf, 2 );			/* 写入校验和,和配置更新 */

#if DE_PRINTF
	GT911_RD_Reg( DEF_GTS_CFG_VERSION, Cfg_Info, sizeof(Cfg_Info) );	

	printf("Config Info:\n");
	for( i = 0; i < sizeof(Cfg_Info); i++ )
	{
		printf("%02x\t",(UINT16)Cfg_Info[i]);
	}
	printf("\n");
	
	buf[0] = 0x00;
	buf[1] = 0x04;
	GT911_WR_Reg( 0x8146, buf, 2 );
	buf[0] = 0x58;
	buf[1] = 0x02;
	GT911_WR_Reg( 0x8148, buf, 2 );
	
	GT911_RD_Reg( 0x8140, Cfg_Info, sizeof(Cfg_Info) );
	printf("Coordinate Info\n");
	for( i = 0; i < sizeof(Cfg_Info); i++ )
	{
		printf("%02x\t",(UINT16)Cfg_Info[i]);
	}
	printf("\n");	
#endif
	return 0;
}

/*******************************************************************************
* Function Name  : GT911_Init
* Description    : 触摸板初始化，获取ID，确认是否工作
				   	 SET DEV_ADDRESS	
				   ******** 0X14 0X28/0X29 *************
				   * RST  ________——————
				   * INT  ____——————————
				   ******** 0X5D 0XBA/0XBB *************
				   *  RST  ________——————
				   *  INT  ______________
				   *********************************
* Input          : None
* Output         : None
* Return         : IC VERSION
*******************************************************************************/
UINT8 GT911_Init(void)
{

	/* SET ADDRESS 0x28/0x29*/
#if (DEF_TP_DEV_ADDR == 0X14)	
	GT911_RST(0);
	GT911_INT(0);
	mDelayuS(10);
	GT911_INT(1);
	mDelayuS(100);
	GT911_RST(1);
#elif (DEF_TP_DEV_ADDR == 0X5D)	
	GT911_RST(0);
	GT911_INT(0);
	mDelayuS(100);
	GT911_RST(1);
#endif	
	mDelaymS(5);
	/* INT 浮空输入 */
	P3_MOD_OC &= ~BIT2;														/* P3^2 浮空输入 */
	P3_DIR_PU &= ~BIT2;
	I2c_Init();
	/* Config system infomation */	
//	GT911_Send_Cfg(1);
	
	Get_Config_Info();
	GT911_Info.X_Resolution = 2048 /(double)GT911_Info.x_max_pos;
	GT911_Info.Y_Resolution = 2048 /(double)GT911_Info.y_max_pos;	
#if DE_PRINTF	
	printf("%f\t%f\n",GT911_Info.X_Resolution,GT911_Info.Y_Resolution);
#endif	
	return 1;
}

/*******************************************************************************
* Function Name  : GT911_Touch_Check
* Description    : 按键检测与上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 GT911_Touch_Check( void )
{
	
	if( GT911_Info.IRQ_Flag == 1 )
	{
		if( GT911_Scan() == 1 )
		{
			if(( TP[0].X_pos != 0 )||( TP[0].Y_pos != 0 ) )
			{
				Absolute_Up_Pack( &TP, GT911_Info.Point_Num );
			}
		}
		
		GT911_Info.IRQ_Flag = 0;		
		EX0 = 1;
	
	}
	return 0;
}


	
/*******************************************************************************
* Function Name  : GT911_Scan
* Description    : 扫描触摸屏(采用查询方式)
* Input          : None
* Output         : None
* Return         : 0,触屏无触摸;1,触屏有触摸
*******************************************************************************/
UINT8 GT911_Scan( void )
{
	UINT8 buf[5];
	UINT8 mode;

	UINT8 i;
	UINT8 temp;
	
	INT1 = 0;		
	GT911_RD_Reg(GT_GSTID_REG,&mode,1);								//读取触摸点的状态 
	
	if( ( mode & BIT7 ) == 0 )
	{
		temp = 0;	
		GT911_WR_Reg(GT_GSTID_REG,&temp,1);							//清标志,如果延迟较长可以去掉			
		return 0;
	}

	GT911_Info.Point_Num =  mode&0x0F;

	for( i = 0; i < POINTER_NUM; i++ )
	{
		GT911_RD_Reg( GT911_TPX_TBL[i], buf, 5 );
		
//		printf("ID[%d]:%d\t",(UINT16)i,(UINT16)buf[0]);
		if( i == 0 )
		{
			TP[i].Tip_Switch = GT911_Info.Point_Num ? 1 : 0;
		}
		else
		{
			TP[i].Tip_Switch = ( buf[0] ? 1 : 0 );
		}
		
		if( TP[i].Tip_Switch == 0 )
		{
			continue;
		}
		
		TP[i].Contact_Identifier = buf[0];
		
		if( ( TP[i].Tip_Switch == 1 ) || (i == 0) )
		{
			if( GT911_Info.x_y_swap == 1 )
			{
				TP[i].Y_pos = ( ((UINT16)buf[2]<<8)+buf[1] ) * GT911_Info.X_Resolution;
				TP[i].X_pos = ( ((UINT16)buf[4]<<8)+buf[3] ) * GT911_Info.Y_Resolution;
			}
			else
			{
				TP[i].X_pos = ( ((UINT16)buf[2]<<8)+buf[1] ) * GT911_Info.X_Resolution;
				TP[i].Y_pos = ( ((UINT16)buf[4]<<8)+buf[3] ) * GT911_Info.Y_Resolution;					
//				TP[i].X_pos = ( ((UINT16)buf[2]<<8)+buf[1] );
//				TP[i].Y_pos = ( ((UINT16)buf[4]<<8)+buf[3] );		
			}
			TP[i].Resolution_Multi = 0x0030;
			
			if ( GT911_Info.x_mirror == 1 )
			{
				TP[i].X_pos = 2048 - TP[i].X_pos;
			}
			
			if ( GT911_Info.y_mirror == 1 )
			{
				TP[i].Y_pos = 2048 - TP[i].Y_pos;
			}
		}
		else 
		{
			TP[i].X_pos = 0;
			TP[i].Y_pos = 0;
			TP[i].Resolution_Multi = 0;
		}
#if DE_PRINTF	
	printf("sw:%d\tid%d\t%d\t%d\n",(UINT16)TP[i].Tip_Switch,(UINT16)TP[i].Contact_Identifier,TP[i].X_pos,TP[i].Y_pos);
#endif
	}
#if DE_PRINTF	
	printf("num:%d\n",(UINT16)GT911_Info.Point_Num);
#endif
	temp = 0;
	GT911_WR_Reg(GT_GSTID_REG,&temp,1);							//清标志 		
	
	INT1 = 1;		
	return 1;
}

/*******************************************************************************
* Function Name  : GT911_ISR
* Description    : 外部中断0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void    GT911_ISR( void ) interrupt INT_NO_INT0 using 1                    //USB中断服务程序,使用寄存器组1
{
	EX0 = 0;
	GT911_Info.IRQ_Flag = 1;
}

/* END OF FILE */
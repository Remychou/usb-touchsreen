#include "XPT2046.h"
#include "CH554.h"
#include "DEBUG.h"
#include "mytype.h"
#include <intrins.h>

/*******************************************************************************
* Function Name  : XPT2046_Init
* Description    : INIT THE GPIO 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void XPT2046_Init( void )
{
	P1_MOD_OC &= ~( BIT6 + BIT4 + BIT7 );		/* SET PUSU-PULL OUTPUT */
	P1_DIR_PU |=  ( BIT6 + BIT4 + BIT7 ); 
	
//	P1_MOD_OC &= ~BIT5;							/* SET FLOAT INPUT */
//	P1_DIR_PU &= ~BIT5; 
}   
/*******************************************************************************
* Function Name  : XPT2046_SendCommand
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void XPT2046_SendCommand(UINT8 ucCommand)   //发送一个命令
{
	UINT8 i;
	for(i = 0;i < 8;i++)
	{
		if(ucCommand & 0x80)
		{
			XPT2046_DIN = 1;
		}
		else
		{
			XPT2046_DIN = 0;
		}
		ucCommand <<= 1;
		mDelayuS(1);
		XPT2046_CLK = 0;
		mDelayuS(1);
		XPT2046_CLK = 1;
		mDelayuS(1);
	}	
}

/*******************************************************************************
* Function Name  : 
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT16 XPT2046_ReadData(UINT8 ucCommand)	//读取一个数据
{
	UINT8  i;
	UINT16 uiBackValue;
	
	XPT2046_CLK = 0; //先拉低时钟
		mDelayuS(1);
	XPT2046_CS = 0;	//使能片选
		mDelayuS(1);
	XPT2046_SendCommand(ucCommand);	//发送控制字
	while(XPT2046_BUSY);
		mDelayuS(1);
	XPT2046_CLK = 1;
		mDelayuS(1);
	XPT2046_CLK = 0;
		mDelayuS(1);

	for( i = 0; i < 16;i++)
	{
		uiBackValue <<= 1;
		XPT2046_CLK = 1;
		mDelayuS(1);
		XPT2046_CLK = 0;
		mDelayuS(1);
		if(XPT2046_DOUT)uiBackValue++;
		mDelayuS(1);
	}

	uiBackValue >>= 4;
	XPT2046_CS = 1;

	return uiBackValue;

		
}

/*******************************************************************************
* Function Name  : xpt2046_Get_Touch_Data
* Description    : 
* Input          : UINT8 XorY 
* Output         : None
* Return         : UINT16 uiBackValue
*******************************************************************************/
UINT16 xpt2046_Get_Touch_Data( UINT8 XorY )
{
	UINT8 i,j;
	UINT16 volatile ucCorData[DEF_SAMPLE_CNT];
	UINT16 volatile k;
	UINT32 uiBackValue;
	
	uiBackValue = 0;
	for( i = 0; i < DEF_SAMPLE_CNT - 1; i++ )
	{
		ucCorData[i] = XPT2046_ReadData(XorY);
	}
	if( DEF_XPOS == XorY )
		ucCorData[ DEF_SAMPLE_CNT - 1 ] = XPT2046_ReadData( XorY & 0XFC );
	else
		ucCorData[ DEF_SAMPLE_CNT - 1 ] = XPT2046_ReadData( XorY );
	
	for( i = 0;i < DEF_SAMPLE_CNT;i++)
	{
		for( j = i + 1;j < DEF_SAMPLE_CNT;j++)
		{
			if(ucCorData[i] < ucCorData[j])
			{
				k = ucCorData[i];
				ucCorData[i] = ucCorData[j];
				ucCorData[j] = k;
			}
		}
	}
	
	for( i = 5;i < DEF_SAMPLE_CNT - 5;i++)
	{
		uiBackValue += ucCorData[i];
	}

	return ( UINT16 )( ( uiBackValue / ( DEF_SAMPLE_CNT -10 ) ) );
}

/*******************************************************************************
* Function Name  : Low_Pass_Filter
* Description    : 1st Low pass filter 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT16 Low_Pass_Filter( UINT16 New_Data, UINT16 Old_Data, UINT8 Factor )
{
	UINT16 Result;
	if( New_Data < Old_Data )
	{
		Result = Old_Data - New_Data;
		Result = Result * Factor;
		Result = Result + 128;
		Result = Result / 256;
		Result = Old_Data - Result;
	}
	else if( New_Data > Old_Data )
	{
		Result = New_Data - Old_Data;
		Result = Result * Factor;
		Result = Result + 128;
		Result = Result / 256;
		Result = Old_Data + Result;		
	}	
	else
		Result = Old_Data;
	return ((UINT16)Result);
}
/*******************************************************************************
* Function Name  : Pos_ABS
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT16 Pos_ABS( UINT16 m, UINT16 n )
{
	if( m > n )
	{
		return( m - n );
	}
	else 
	{
		return( n - m );
	}
}

/* END OF FILE */
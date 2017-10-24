/********************************** (C) COPYRIGHT *******************************
* File Name          : MAIN.C
* Author             : RZ
* Version            : V1.00
* Date               : 2017-6-10 11:17:50
* Description        : CH554 XPT2046驱动电阻屏
*******************************************************************************/
#include "CH554.H"
#include "DEBUG.H"
#include "DEVICE.H"
#include "XPT2046.H"
#include "PRINTF.H"

struct POINT TP;
sbit LED = P1^1;
void main(void)
{
	UINT16 x_temp=0,y_temp=0;
	UINT8 t=0;
	CfgFsys();
	mDelaymS(3);
	
	mInitSTDIO();														/* Enable printf */
	DEBUG_PRINTF( "DEBUG_DATE:"__DATE__""__TIME__"\n" );				
	XPT2046_Init();														/* Init xpt2046 */	
	CH55X_USBD_Init();													/* Init USB module */
	LED = 0;
	mDelaymS(50);	
	mDelaymS(50);	
	LED = 1;
	CH554WDTModeSelect(1);												/* Enable WDT */
	while(1)
	{
		CH554WDTFeed(0);										/* Feed WDT */

		if(XPT2046_PEN == 0)
		{
			mDelaymS(20);			

			TP.Y_Pos = xpt2046_Get_Touch_Data( DEF_YPOS );
			TP.X_Pos = xpt2046_Get_Touch_Data( DEF_XPOS );
			while(XPT2046_PEN == 0)
			{
				CH554WDTFeed(0);										/* Feed WDT */
				
				if(t++> 5)
				{
					t = 0;
					LED ^= 1;
				}	
					
				x_temp = TP.X_Pos;
				y_temp = TP.Y_Pos;
				

				TP.Y_Pos = xpt2046_Get_Touch_Data( DEF_YPOS );
				TP.X_Pos = xpt2046_Get_Touch_Data( DEF_XPOS );				
				TP.X_Pos = Low_Pass_Filter( TP.X_Pos, x_temp, 100 );
				TP.Y_Pos = Low_Pass_Filter( TP.Y_Pos, y_temp, 100 );	

				MOUSE_Absolute_Up_Pack_Soft( 0x83, x_temp, y_temp );

			}
			MOUSE_Absolute_Up_Pack_Soft( 0x82, x_temp, y_temp );
			LED = 1;
		}
		
		/* 如果接收到USB HID类命令包,则进行命令包处理 */
		if( HID_Cmd_Pack_Flag )
		{		
			HID_CMD_Pack_Deal( );												/* HID自定义设备命令包处理 */
			HID_Cmd_Pack_Flag = 0x00;											/* 接收到HID命令包标志 */
		}
	}	 

}		


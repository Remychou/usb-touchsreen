/********************************** (C) COPYRIGHT *******************************
* File Name          : MAIN.C
* Author             : RZ
* Version            : V1.00
* Date               : 2017-5-16
* Description        : 用CH554，模拟Multi Touch设备实现多指触控功能
*******************************************************************************/
#include "CH554.H"
#include "GT911.H"
#include "DEBUG.H"
#include "DEVICE.H"
#include <stdio.h>


/*******************************************************************************
* Function Name  : main
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void main( void )
{

    CfgFsys( );                                                           	/* CH559时钟选择配置 */
    mDelaymS(5);                                                          	/* 修改主频等待内部晶振稳定,必加 */
    mInitSTDIO( );                                                        	/* 串口0初始化 */
#if DE_PRINTF
    printf("CH554_HID_TP_V100 DEBUG\n DEBUG_DATA: "__DATE__""__TIME__" \n");
#endif	
	
    USBDeviceInit();                                                     	/* USB设备模式初始化 */
	GT911_Init();
    UEP1_T_LEN = 0;                                                      	/* 预使用发送长度一定要清空 */
    UEP2_T_LEN = 0;                                                      	/* 预使用发送长度一定要清空 */
		

	IT0 = 1;
//	EX0 = 1;
	EA = 1;                                                              	/* 允许单片机中断 */
	CH554WDTModeSelect(1);

    while(1)
    {
		CH554WDTFeed(0);

		GT911_Touch_Check();
    }
}
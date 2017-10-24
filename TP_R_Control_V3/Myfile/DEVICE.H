/********************************** (C) COPYRIGHT *******************************
* File Name          : DEVICE.H
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : CH55X USB设备操作相关定义
*******************************************************************************/



#ifndef __DEVICE_H__
#define __DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************/
/* 鼠标相关宏定义 */
#define	DEF_MOUSE_PACK_LEN   	7											 	/* 鼠标数据单包字节数 */

#define	DEF_MOUSE_X_MAX		   	0x8000											/* 绝对鼠标X轴最大值 */
#define	DEF_MOUSE_Y_MAX		   	0x8000											/* 绝对鼠标Y轴最大值 */

/***********************************************************************************************************************/
/* 变量外扩 */
extern UINT8X	HID_Cmd_Pack_Flag;												/* 接收到HID命令包标志 */

/***********************************************************************************************************************/
/* 函数外扩 */
extern void CH55X_USBD_ModeCfg( void );											/* CH55X USB设备工作模式配置 */
extern void CH55X_USBD_IntCfg( void );											/* CH55X USB设备模式中断配置 */
extern void CH55X_USBD_EnpdCfg( void );											/* CH55X USB设备模式端点配置 */
extern void CH55X_USBD_Init( void );											/* CH55X USB设备模式初始化 */
extern void CH55X_USBD_Interrupt( void );										/* CH55X USB中断服务程序 */
extern void CH55X_USB_Endp1_Up( UINT8 *pbuf, UINT8 len );						/* CH55X USB端点1数据上传 */

extern void HID_CMD_Pack_Deal( void );	 										/* HID自定义设备命令包处理 */
extern void MOUSE_Relative_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z );	/* 相对鼠标上传一包数据 */
extern void MOUSE_Absolute_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z );	/* 绝对鼠标上传一包数据 */

extern void MOUSE_Absolute_Up_Pack_Soft( UINT8 key, UINT16 x, UINT16 y );
#ifdef __cplusplus
}
#endif

#endif

/***********************************************************************************************************************/



/********************************** (C) COPYRIGHT *******************************
* File Name          : DEVICE.H
* Author             : RZ
* Version            : V1.00
* Date               : 2017-5-17
* Description        : USBÇý¶¯º¯Êý
*******************************************************************************/

#ifndef  	__DEVICE_H__
#define 	__DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif
	
extern UINT8 	USB_Enum_OK;
extern UINT8X 	EP_UP_BUF[84];
extern void 	USBDeviceInit		( void );
extern UINT8 	Absolute_Up_Pack	( POINTER * pTP, UINT8 Point_Num );
extern UINT8 	Test				( void );
	
#ifdef __cplusplus
}
#endif

#endif

/* END OF FILE */

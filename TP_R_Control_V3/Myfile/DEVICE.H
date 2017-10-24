/********************************** (C) COPYRIGHT *******************************
* File Name          : DEVICE.H
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : CH55X USB�豸������ض���
*******************************************************************************/



#ifndef __DEVICE_H__
#define __DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************/
/* �����غ궨�� */
#define	DEF_MOUSE_PACK_LEN   	7											 	/* ������ݵ����ֽ��� */

#define	DEF_MOUSE_X_MAX		   	0x8000											/* �������X�����ֵ */
#define	DEF_MOUSE_Y_MAX		   	0x8000											/* �������Y�����ֵ */

/***********************************************************************************************************************/
/* �������� */
extern UINT8X	HID_Cmd_Pack_Flag;												/* ���յ�HID�������־ */

/***********************************************************************************************************************/
/* �������� */
extern void CH55X_USBD_ModeCfg( void );											/* CH55X USB�豸����ģʽ���� */
extern void CH55X_USBD_IntCfg( void );											/* CH55X USB�豸ģʽ�ж����� */
extern void CH55X_USBD_EnpdCfg( void );											/* CH55X USB�豸ģʽ�˵����� */
extern void CH55X_USBD_Init( void );											/* CH55X USB�豸ģʽ��ʼ�� */
extern void CH55X_USBD_Interrupt( void );										/* CH55X USB�жϷ������ */
extern void CH55X_USB_Endp1_Up( UINT8 *pbuf, UINT8 len );						/* CH55X USB�˵�1�����ϴ� */

extern void HID_CMD_Pack_Deal( void );	 										/* HID�Զ����豸��������� */
extern void MOUSE_Relative_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z );	/* �������ϴ�һ������ */
extern void MOUSE_Absolute_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z );	/* ��������ϴ�һ������ */

extern void MOUSE_Absolute_Up_Pack_Soft( UINT8 key, UINT16 x, UINT16 y );
#ifdef __cplusplus
}
#endif

#endif

/***********************************************************************************************************************/


/******************** (C) COPYRIGHT 2013 MJX ***********************************
* File Name          : USB_DESC.H
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : USB�豸����������
*******************************************************************************/


#ifndef __USB_DESC_H
#define __USB_DESC_H

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/* ͷ�ļ����� */

/******************************************************************************/
/* ��������غ궨�� */
#define DEF_DEVICE_CFG_DESC_SIZE   34											/* USB�������������ȶ��� */
#define DEF_REPORT1_DESC_SIZE      332											/* USB����������1���ȶ��� */	
#define DEF_REPORT2_DESC_SIZE      0x25											/* USB����������1���ȶ��� */
#define DEF_HID_PACK_SIZE		   0x20											/* USB HID���ݰ���С���� */	

/******************************************************************************/
/* �豸������ */
const UINT8 Device_Descriptor[ 18 ] =
{
    0x12,                       								 /* bLength */
    0x01, 														 /* bDescriptorType */
    0x10,                       								 /* bcdUSB */
    0x01,
    0x00,                       								 /* bDeviceClass */
    0x00,                       								 /* bDeviceSubClass */
    0x00,                       								 /* bDeviceProtocol */
    0x08,                       								 /* bMaxPacketSize 8 */
    0xEF,                       								 /* idVendor (0x0483) */
    0x0E,
    0x01,                       								 /* idProduct = 0x5710 */
    0x00,
    0x00,                       								 /* bcdDevice rel. 2.00 */
    0x01,
    0x01,                          								 /* Index of string descriptor describing manufacturer */
    0x02,                          								 /* Index of string descriptor describing product */
    0x00,                          								 /* Index of string descriptor describing the device serial number */
    0x01                        								 /* bNumConfigurations */
}; 

/******************************************************************************/
/* USB���������� */
// const UINT8 Config_Descriptor[ DEF_DEVICE_CFG_DESC_SIZE ] =
// {
//     0x09, 														 /* bLength: Configuation Descriptor size */
//     0x02,							 							 /* bDescriptorType: Configuration */
//     DEF_DEVICE_CFG_DESC_SIZE, 									 /* wTotalLength: Bytes returned */
//     0x00,
//     0x02,         												 /* bNumInterfaces: 2 interface */
//     0x01,         												 /* bConfigurationValue: Configuration value */
//     0x00,         												 /* iConfiguration: Index of string descriptor describing the configuration */
//     0xE0,         												 /* bmAttributes: bus powered */
//     0xA0,         												 /* MaxPower 100 mA: this current is used for detecting Vbus */

// 	/**************************************�ӿ�0*****************************************/
//     /************** Descriptor of Mouse interface ****************/    
//     /* 10 */
// 	0x09,         												 /* bLength: Interface Descriptor size */
//     0x04,														 /* bDescriptorType: Interface descriptor type */
//     0x00,         												 /* bInterfaceNumber: Number of Interface */
//     0x00,         												 /* bAlternateSetting: Alternate setting */
//     0x01,         												 /* bNumEndpoints */
//     0x03,         												 /* bInterfaceClass: HID */
//     0x01,         												 /* bInterfaceSubClass : 1=BOOT, 0=no boot */
//     0x02,         												 /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
//     0x00,          												 /* iInterface: Index of string descriptor */
// 	
// 	/******************** Descriptor of KeyBoard HID ********************/
//     /* 19 */
// 	0x09,         												 /* bLength: HID Descriptor size */
//     0x21,				 										 /* bDescriptorType: HID */
//     0x10,         												 /* bcdHID: HID Class Spec release number */
//     0x01,
//     0x00,          												 /* bCountryCode: Hardware target country */
//     0x01,         												 /* bNumDescriptors: Number of HID class descriptors to follow */
//     0x22,         												 /* bDescriptorType */
//     DEF_REPORT1_DESC_SIZE,										 /* wItemLength: Total length of Report descriptor */
//     0x00,

//     /******************** Descriptor of KeyBoard endpoint ********************/
//     /* 28 */
// 	0x07,          												 /* bLength: Endpoint Descriptor size */
//     0x05,						 								 /* bDescriptorType: */
//     0x81,          												 /* bEndpointAddress: Endpoint Address (IN) */
//     0x03,          												 /* bmAttributes: Interrupt endpoint */
//     0x08,          												 /* wMaxPacketSize: 8 Byte max */
//     0x00,
//     0x0A,          												 /* bInterval: Polling Interval (32 ms) */

// 	/**************************************�ӿ�1*****************************************/
//     /************** Descriptor of �Զ���HID interface ****************/ 
//  
//     /* 35 */
// 	0x09,         												 /* bLength: Interface Descriptor size */
//     0x04,														 /* bDescriptorType: Interface descriptor type */
//     0x01,         												 /* bInterfaceNumber: Number of Interface */
//     0x00,         												 /* bAlternateSetting: Alternate setting */
//     0x02,         												 /* bNumEndpoints */
//     0x03,         												 /* bInterfaceClass: HID */
//     0x00,         												 /* bInterfaceSubClass : 1=BOOT, 0=no boot */
//     0x00,         												 /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
//     0x00,          												 /* iInterface: Index of string descriptor */

// 	/******************** Descriptor of KeyBoard HID ********************/
//     /* 44 */
// 	0x09,         												 /* bLength: HID Descriptor size */
//     0x21, 														 /* bDescriptorType: HID */
//     0x10,         												 /* bcdHID: HID Class Spec release number */
//     0x01,
//     0x00,         												 /* bCountryCode: Hardware target country */
//     0x01,         												 /* bNumDescriptors: Number of HID class descriptors to follow */
//     0x22,         												 /* bDescriptorType */
//     DEF_REPORT2_DESC_SIZE,									 	 /* wItemLength: Total length of Report descriptor */
//     0x00,

//     /******************** Descriptor of KeyBoard endpoint ********************/
//     /* 53 */
// 	0x07,          												 /* bLength: Endpoint Descriptor size */
//     0x05, 														 /* bDescriptorType: */
//     0x82,          												 /* bEndpointAddress: Endpoint Address (IN) */
//     0x03,          												 /* bmAttributes: Interrupt endpoint */
//     0x20,          												 /* wMaxPacketSize: 20 Byte max */
//     0x00,
//     0x0A,          												 /* bInterval: Polling Interval (32 ms) */  

// 	0x07,          												 /* bLength: Endpoint Descriptor size */
//     0x05, 														 /* bDescriptorType: */
//     0x02,          												 /* bEndpointAddress: Endpoint Address (IN) */
//     0x03,          												 /* bmAttributes: Interrupt endpoint */
//     0x20,          												 /* wMaxPacketSize: 8 Byte max */
//     0x00,
//     0x0A          												 /* bInterval: Polling Interval (32 ms) */ 
// };

// ����������
UINT8C	Config_Descriptor[DEF_DEVICE_CFG_DESC_SIZE] = {
		   0x09,0x02,0x22,0x00,0x01,0x01,0x01,0xA0,0x32,  //0x32 ��    0x22   MyCfgDescr ���� 
		   0x09,0x04,0x00,0x00,0x01,0x03,0x00,0x00,0x00,  //  0x01, 0x02  // bNumEndpoints
		   0x09,0x21,0x00,0x01,0x00,0x01,0x22,0x42,0x01,  //0x72 �� 0x66 ��������������  
		   0x07,0x05,0x81,0x03,0x10,0x00,0x05
     // , 	 // 0x0a �� 0x09 ���ͳ���   0x09 --> 0x10
     // 0x07,0x05,0x02,0x03,  0x09,0x00,0x0A		// ����ڼ��ʱ�� 0x0a��Ϊ0x06 		   
		 // 09 02 22 00  01 01 01 a0  32 09 04 00  00 01 03 00                               
     // 00 00 09 21  00 01 00 01  22 42 01 07  05 81 03 10                                                                
     //   00 03 		   		
						};
/*HID�౨��������*/ 
UINT8C Mouse_Report1_Descriptor[DEF_REPORT1_DESC_SIZE] = { 
 0x05,0x0d,0x09,0x04,0xa1,0x01,0x85,0x02,0x09,0x20,0xa1,0x00,0x09,0x42,0x09,0x32,0x15,0x00,0x25,0x01,0x95,0x02,0x75,0x01,0x81,0x02,0x95,0x06,0x75,0x01,0x81,0x03 
,0x05,0x01,0x09,0x30,0x75,0x10,0x95,0x01,0xa4,0x55,0x0d,0x65,0x33,0x36,0x00,0x00,0x46,0x99,0x28,0x16,0x00,0x00,0x26,0xff,0x0f,0x81,0x02,0x09,0x31,0x16,0x00,0x00 
,0x26,0xff,0x0f,0x36,0x00,0x00,0x46,0xaf,0x19,0x81,0x02,0xb4,0xc0,0xc0,0x05,0x01,0x09,0x01,0xa1,0x01,0x85,0x01,0x09,0x01,0xa1,0x00,0x05,0x09,0x19,0x01,0x29,0x02 
,0x15,0x00,0x25,0x01,0x95,0x02,0x75,0x01,0x81,0x02,0x95,0x01,0x75,0x06,0x81,0x01,0x05,0x01,0x09,0x30,0x09,0x31,0x16,0x00,0x00,0x26,0xff,0x0f,0x36,0x00,0x00,0x46 
,0xff,0x0f,0x66,0x00,0x00,0x75,0x10,0x95,0x02,0x81,0x02,0xc0,0xc0,0x06,0x00,0xff,0x09,0x01,0xa1,0x01,0x09,0x01,0x15,0x00,0x26,0xff,0x00,0x85,0x03,0x75,0x08,0x95 
,0x05,0x81,0x02,0x06,0x00,0xff,0x09,0x01,0x15,0x00,0x26,0xff,0x00,0x75,0x08,0x95,0x05,0x91,0x02,0xc0,0x05,0x0d,0x09,0x04,0xa1,0x01,0x85,0x04,0x09,0x22,0xa1,0x00 
,0x09,0x42,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x01,0x81,0x02,0x09,0x32,0x15,0x00,0x25,0x01,0x81,0x02,0x09,0x51,0x75,0x05,0x95,0x01,0x16,0x00,0x00,0x26,0x10,0x00 
,0x81,0x02,0x09,0x47,0x75,0x01,0x95,0x01,0x15,0x00,0x25,0x01,0x81,0x02,0x05,0x01,0x09,0x30,0x75,0x10,0x95,0x01,0x55,0x0d,0x65,0x33,0x35,0x00,0x46,0x60,0x17,0x26 
,0xff,0x0f,0x81,0x02,0x09,0x31,0x75,0x10,0x95,0x01,0x55,0x0d,0x65,0x33,0x35,0x00,0x46,0x26,0x0e,0x26,0xff,0x0f,0x81,0x02,0x05,0x0d,0x09,0x55,0x25,0x08,0x75,0x08 
,0x95,0x01,0xb1,0x02,0xc0,0xc0,0x05,0x0d,0x09,0x0e,0xa1,0x01,0x85,0x05,0x09,0x22,0xa1,0x00,0x09,0x52,0x09,0x53,0x15,0x00,0x25,0x0a,0x75,0x08,0x95,0x02,0xb1,0x02 
,0xc0,0xc0
 };	
/* ���������� */
const UINT8 Lang_Descriptor[ ] = { 0x04, 0x03, 0x09, 0x04 };

/* ��Ʒ��Ϣ */
const UINT8 ProdInfo_Descriptor[ ] = { 0x28,3,'U',0, 'S',0,'B',0,0x20,0, 'T', 0, 'o', 0, 'u', 0, 'c', 0, 'h', 0, 'C', 0,'o',0,'n',0,'t',0,'r',0,'o',0,'l',0,'l',0,'e',0,'r',0 };

/* ������Ϣ */
const UINT8 ManuInfo_Descriptor[ ] = { 0x0C, 0x03, 'M', 0, 'o', 0, 'u', 0, 's', 0, 'e', 0 };


/******************************************************************************/
/* USB��걨�������� */
// const UINT8 Mouse_Report1_Descriptor[ DEF_REPORT1_DESC_SIZE ] =
// {
// 	0x05, 0x01, 												 /* USAGE_PAGE (Generic Desktop)        ͨ�������豸 */
// 	0x09, 0x02, 			 									 /* USAGE (Mouse)                       ��� */
// 	0xa1, 0x01,													 /* COLLECTION (Application)            ���� */
// 	0x85, 0x01, 
// 	0x09, 0x01, 			 								     /* USAGE (Pointer)                     ָ���豸 */
// 	0xa1, 0x01, 		   										 /* COLLECTION (Physical)               ���� */
// 	
// 	0x05, 0x09, 												 /* USAGE_PAGE (Button)                 ���� */	
// 	0x19, 0x01, 												 /* USAGE_MINIMUM (Button 1)            ʹ����Сֵ1 */
//     0x29, 0x05, 												 /* USAGE_MAXIMUM (Button 3)            ʹ�����ֵ5��1��ʾ�����2��ʾ�Ҽ���3��ʾ�м� */	
// 	0x15, 0x00, 		  										 /* LOGICAL_MINIMUM (0)                 �߼���Сֵ0 */
// 	0x25, 0x01, 		 										 /* LOGICAL_MAXIMUM (1)                 �߼����ֵ1 */	
// 	0x95, 0x05, 	 											 /* REPORT_COUNT (5)                    ����Ϊ5 */
// 	0x75, 0x01, 			 									 /* REPORT_SIZE (1)                     ��СΪ1bit */
// 	0x81, 0x02, 	  										  	 /* INPUT (Data,Var,Abs)                ���룬��������ֵ������ֵ */
// 	0x95, 0x01, 	  										 	 /* REPORT_COUNT (1)   ���5��bit������һ���ֽ� */
// 	0x75, 0x03, 	  											 /* REPORT_SIZE (3) */
//     0x81, 0x01, 			 									 /* INPUT (Cnst,Var,Abs) */
// 	
// 	0x05, 0x01,   												 /* USAGE_PAGE (Generic Desktop)        ��;ҳΪͨ������ */
// 	0x09, 0x38,    												 /* USAGE (Wheel)                       ��;Ϊ���� */
// 	0x15, 0x81, 												 /* LOGICAL_MINIMUM (-127)              �߼���СֵΪ-127 */
// 	0x25, 0x7f, 	 											 /* LOGICAL_MAXIMUM (127)               �߼����ֵΪ+127 */
// 	0x75, 0x08, 												 /* REPORT_SIZE (8)                     ��СΪ8��bits */
// 	0x95, 0x01, 												 /* REPORT_COUNT (1)                    ����Ϊ1�� ���� */
// 	0x81, 0x06,													 /* INPUT (Data,Var,Rel)                ���룬������ֵ�����ֵ */ 

//     0x09, 0x30, 												 /* USAGE (X)                           ��;ΪX */
// 	0x09, 0x31, 	   											 /* USAGE (Y)                           ��;ΪY */
// 	0x75, 0x10, 	   											 /* REPORT_SIZE (16)                    ��СΪ16��bits */
// 	0x16, 0x01, 0x80, 
// 	0x26, 0xFF, 0x7f,	 
// 	0x95, 0x02, 												 /* REPORT_COUNT (2)                    ����Ϊ2�������ֱ����x,y,���� */
// 	0x81, 0x06,   												/* INPUT (Data,Var,Rel)                ���룬������ֵ�����ֵ */
//     0xc0, 0xc0, 
// 	
// 	
// 	0x05, 0x01, 
// 	0x09, 0x02, 
// 	0xa1, 0x01, 
// 	0x85, 0x02, 
// 	0x09, 0x01, 
// 	0xa1, 0x00, 
// 	0x05, 0x09, 
//     0x19, 0x01, 
// 	0x29, 0x05, 
// 	0x15, 0x00, 
// 	0x25, 0x01, 
// 	0x75, 0x01, 
// 	0x95, 0x05, 
// 	0x81, 0x02, 
// 	0x75, 0x01, 
//     0x95, 0x03, 
// 	0x81, 0x01, 
// 	0x75, 0x08, 
// 	0x95, 0x01, 
// 	0x05, 0x01, 
// 	0x09, 0x38, 
// 	0x15, 0x81, 
// 	0x25, 0x7f, 
//     0x81, 0x06, 
// 	0xc0, 
// 	
// 	0x09, 0x01, 
// 	0xa1, 0x00, 
// 	0x05, 0x01, 
// 	0x09, 0x30,  												 /* USAGE (X)                           ��;ΪX */							
// 	0x15, 0x00, 											     /* LOGICAL_MINIMUM (0)                 �߼���СֵΪ0 */
// 	0x26, 0x00, 0x80, 		    								 /* LOGICAL_MAXIMUM (0x8000)            �߼����ֵΪx8000 */
//     0x35, 0x00, 
// 	0x46, 0x00, 0x80, 	
// 	0x75, 0x10,   												 /* REPORT_SIZE (16)                    ��СΪ16��bits */
// 	0x95, 0x01, 			   									 /* REPORT_COUNT (1)                    ����Ϊ1��������x */
// 	0x81, 0x02, 	   											 /* INPUT (Data,Var,Abs)                �����ã�������ֵ������ֵ */

// 	0x09, 0x31, 	   											 /* USAGE (Y)                           ��;ΪY */ 
// 	0x15, 0x00, 
// 	0x26, 0x00, 0x80, 
// 	0x35, 0x00, 
// 	0x46, 0x00, 0x80, 
// 	0x75, 0x10, 												 /* REPORT_SIZE (16)                    ��СΪ16��bits */
// 	0x95, 0x01, 		   									 	 /* REPORT_COUNT (1)                    ����Ϊ1��������y */
// 	0x81, 0x02, 												 /* INPUT (Data,Var,Abs)                �����ã�������ֵ������ֵ */
// 	0xc0, 
// 	0xc0
// }; 

// /******************************************************************************/
// /* USB�Զ���HID���豸���������� */
// const UINT8 Hid_Report2_Descriptor[ DEF_REPORT2_DESC_SIZE ] =
// {
//     0x06,
//     0xA0,
//     0xff,
//     0x09,
//     0x01,
//     0xA1,
//     0x01,
//     0x09,
//     0x01,
//     0x15,
//     0x00,
//     0x26,
//     0xFF,
//     0x00,
//     0x75,
//     0x08,
//     0x95,
//     DEF_HID_PACK_SIZE,											 /* �˵㴫�䳤�� */
//     0x81,
//     0x02,
//     0x09,
//     0x02,
//     0x75,
//     0x08,
//     0x95,
//     DEF_HID_PACK_SIZE,											 /* �˵㴫�䳤�� */
//     0x91,
//     0x02,
//     0x09,
//     0x03,
//     0x75,
//     0x08,
//     0x95,
//     0x05,
//     0xB1,
//     0x02,
//     0xC0  
// }; 

#ifdef __cplusplus
}
#endif

#endif 
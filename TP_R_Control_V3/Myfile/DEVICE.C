/********************************** (C) COPYRIGHT *******************************
* File Name          : DEVICE.C
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : CH559模拟USB复合设备,USB鼠标及HID类自定义设备,支持类命令
*******************************************************************************/



/******************************************************************************/
/* 头文件包含 */
#include <stdio.h>
#include <string.h>
#define NO_XSFR_DEFINE															/* 在CH559.H前定义该宏 */
#include "..\myfile\CH554.H"													/* CH559相关头文件 */
#include "DEBUG.H"
#include "..\myfile\USB_DESC.H"													/* CH55X USB描述符相关头文件 */
#include "..\myfile\DEVICE.H"	 												/* CH55X USB设备模式操作相关头文件 */

/******************************************************************************/
/* 常、变量定义 */
#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define DEF_USB_DEBUG  	 	       0x00											/* USB调试打印控制 */

UINT8X  Ep0Buffer[ THIS_ENDP0_SIZE ] _at_ 0x0000;                               /* 端点0 OUT&IN缓冲区,必须是偶地址 */
UINT8X	Ep1Buffer[ MAX_PACKET_SIZE ] _at_ 0x0040;                               /* 端点1 IN缓冲区,必须是偶地址 */
UINT8X	Ep2Buffer[ MAX_PACKET_SIZE * 2 ] _at_ 0x0080;                           /* 端点2 IN缓冲区,必须是偶地址 */

volatile UINT8X  Com_Buf[ 100 ];												/* 临时共用缓冲区 */
volatile UINT8X  Mouse_Up_Buf[ 8 ];											 	/* 鼠标数据上传缓冲区 */
volatile UINT8X  gUSB_Up_Flag = 0x00;											/* USB数据包上传标志(LED点亮、熄灭切换标志) */

UINT8 	Mouse_Up_Succ_Flag;
UINT8 	gModule_USB_Status;
UINT16X SetupLen;																/* Setup包长度 */
UINT8X  SetupReq,Ready,Count,FLAG,UsbConfig;
PUINT8  pDescr;                                                                 /* USB配置标志 */

USB_SETUP_REQ   SetupReqBuf;                                                    /* 暂存Setup包 */


#define UsbSetupBuf                ( (PUSB_SETUP_REQ)Ep0Buffer )

UINT8X	HID_Cmd_Pack_Flag = 0x00;												/* 接收到HID命令包标志 */

/*******************************************************************************
* Function Name  : CH55X_USBD_ModeCfg
* Description    : CH55X USB设备工作模式配置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_ModeCfg( void )
{
	USB_CTRL = 0x00;                                                            /* 清空USB控制寄存器 */
	USB_CTRL &= ~bUC_HOST_MODE;                                                 /* 该位为选择设备模式 */
	USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;					    /* USB设备和内部上拉使能,在中断期间中断标志未清除前自动返回NAK */
	USB_DEV_AD = 0x00;                                                          /* 设备地址初始化 */
//	UDEV_CTRL &= ~bUD_RECV_DIS;                                                 /* 使能接收器 */

#if 0
	USB_CTRL |= bUC_LOW_SPEED;
	UDEV_CTRL |= bUD_LOW_SPEED;                                                 /* 选择低速1.5M模式 */
#endif

#if  1
	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED;                                             	/* 选择全速12M模式,默认方式 */
#endif

	UDEV_CTRL |= bUD_PD_DIS;                          						    /* 禁止DM、DP下拉电阻 */
	UDEV_CTRL |= bUD_PORT_EN;                                                 	/* 使能物理端口 */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_IntCfg
* Description    : USB设备模式中断配置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_IntCfg( void )
{
	USB_INT_EN |= bUIE_SUSPEND;                                               	/* 使能设备挂起中断 */
	USB_INT_EN |= bUIE_TRANSFER;                                                /* 使能USB传输完成中断 */
	USB_INT_EN |= bUIE_BUS_RST;                                                 /* 使能设备模式USB总线复位中断 */
	USB_INT_FG |= 0x1F;                                                         /* 清中断标志 */
	IE_USB = 1;                                                                 /* 使能USB中断 */
	EA = 1; 																    /* 允许单片机中断 */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_EnpdCfg
* Description    : CH55X USB设备模式端点配置
*				   当前模拟复合设备,除了端点0的控制传输,还包括端点
*				   1、2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_EnpdCfg( void )
{
	UEP1_DMA = Ep1Buffer;                                                       /* 端点1数据传输地址 */
	UEP4_1_MOD |= bUEP1_TX_EN;                                                  /* 端点1发送使能 */
	UEP4_1_MOD &= ~bUEP1_RX_EN;                                                 /* 端点1接收禁止 */
	UEP4_1_MOD &= ~bUEP1_BUF_MOD;                                               /* 端点1单64字节发送缓冲区 */
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;								    /* 端点1自动翻转同步标志位，IN事务返回NAK */

	UEP2_DMA = Ep2Buffer;                                                       /* 端点2数据传输地址 */
	UEP2_3_MOD |= bUEP2_TX_EN;                                                  /* 端点2发送使能 */
	UEP2_3_MOD |= bUEP2_RX_EN;                          	                    /* 端点2接收使能 */
	UEP2_3_MOD &= ~bUEP2_BUF_MOD;                                               /* 端点2单64字节发送缓冲区 */
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;								    /* 端点2自动翻转同步标志位，IN事务返回NAK */

	UEP0_DMA = Ep0Buffer;                                                       /* 端点0数据传输地址 */
	UEP4_1_MOD &= ~( bUEP4_RX_EN | bUEP4_TX_EN );								/* 端点0单64字节收发缓冲区 */
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                  /* OUT事务返回ACK,IN事务返回NAK */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_Init
* Description    : CH55X USB设备模式初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_Init( void )
{
	CH55X_USBD_ModeCfg( );														/* CH55X USB设备工作模式配置 */
	CH55X_USBD_EnpdCfg( );														/* CH55X USB设备模式端点配置 */
	CH55X_USBD_IntCfg( );														/* CH55X USB设备模式中断配置 */
	UEP1_T_LEN = 0;	                                                      		/* 预使用发送长度一定要清空 */
	UEP2_T_LEN = 0;	                                                      		/* 预使用发送长度一定要清空 */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_Interrupt
* Description    : CH55X USB中断服务程序,使用寄存器组1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_Interrupt( void ) interrupt INT_NO_USB using 1
{
	UINT16 len;
	UINT8 i;

#if	( DEF_USB_DEBUG == 1 )
	printf("%02X ",(UINT16)USB_INT_FG);
#endif
	if( UIF_TRANSFER )                                                          /* USB传输完成中断 */
	{
		switch( USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
		{
		case UIS_TOKEN_IN | 2:                                              /* 端点2上传成功中断 */
			UEP2_T_LEN = 0;	                                                /* 预使用发送长度一定要清空 */
//				UEP1_CTRL ^= bUEP_T_TOG;                                        /* 如果不设置自动翻转则需要手动翻转 */
			UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;        /* 默认应答NAK */
			break;

		case UIS_TOKEN_IN | 1:                                              /* 端点1上传成功中断 */
			UEP1_T_LEN = 0;                                                 /* 预使用发送长度一定要清空 */
//				UEP2_CTRL ^= bUEP_T_TOG;                                        /* 如果不设置自动翻转则需要手动翻转 */
			UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;        /* 默认应答NAK */

			Mouse_Up_Succ_Flag = 0x01;										 /* 鼠标数据上传成功标志 */
			break;

		case UIS_TOKEN_OUT | 2:                                             /* 端点2数据下传成功中断 */
			if( U_TOG_OK )                                                	/* 不同步的数据包将丢弃 */
			{
//					UEP2_CTRL ^= bUEP_R_TOG;                               		/* 已自动翻转 */
				len = USB_RX_LEN;
				HID_Cmd_Pack_Flag = 0x01;									/* 接收到HID命令包标志 */
			}
			break;

		case UIS_TOKEN_SETUP | 0:                                           /* SETUP事务 */
			len = USB_RX_LEN;
			if( len == ( sizeof(USB_SETUP_REQ) ) )
			{
				SetupLen = UsbSetupBuf->wLengthH;
				SetupLen <<= 8;
				SetupLen |= UsbSetupBuf->wLengthL;
				len = 0;                                                    /* 默认为成功并且上传0长度 */

				/* 处理STEP包 */
				SetupReq = UsbSetupBuf->bRequest;
#if	( DEF_USB_DEBUG == 1 )
				printf("REQ %02X ",(UINT16)SetupReq);
#endif
				switch( SetupReq )                                          /* 请求码 */
				{
				case USB_GET_DESCRIPTOR:
					switch( UsbSetupBuf->wValueH )
					{
					case 1:	                                        /* 获取设备描述符 */
						pDescr = Device_Descriptor;                 /* 上传指针指向设备描述符 */
						len = sizeof( Device_Descriptor );			/* 计算上传长度 */
						break;

					case 2:									        /* 获取配置描述符 */
						pDescr = Config_Descriptor;                 /* 上传指针指向配置描述符 */
						len = sizeof( Config_Descriptor );
						break;

					case 3:                                         /* 字符串描述符	*/
						switch( UsbSetupBuf->wValueL )
						{
						case 0:
							pDescr = (PUINT8)( &Lang_Descriptor[ 0 ] );
							len = sizeof( Lang_Descriptor );
							break;

						case 1:
							pDescr = (PUINT8)( &ManuInfo_Descriptor[ 0 ] );
							len = sizeof( ManuInfo_Descriptor );
							break;
						case 2:
							pDescr = (PUINT8)( &ProdInfo_Descriptor[ 0 ] );
							len = sizeof( ProdInfo_Descriptor );
							break;

						default:
							len = 0xFFFF;
							break;
						}
						break;

					case 0x22:                                      /* 获取HID报表描述符 */
						if( UsbSetupBuf->wIndexL == 0 )             /* 接口0报表描述符 */
						{
							pDescr = Mouse_Report1_Descriptor;   	/* 上传指针指向第一个HID报表描述符 */
							len = sizeof( Mouse_Report1_Descriptor );
						}
// 									else if( UsbSetupBuf->wIndexL == 1 )        /* 接口1报表描述符 */
// 									{
// 										pDescr = Hid_Report2_Descriptor;   		/* 上传指针指向第二个HID报表描述符 */
// 										len = sizeof( Hid_Report2_Descriptor );
// 										Ready = 1;
// 									}
						else
						{
							len = 0xFFFF;    						/* 异常情况 */
						}
						break;

					default:
						len = 0xFFFF;                               /* 不支持的命令或者出错 */
						break;
					}
					if( SetupLen > len )
					{
						SetupLen = len;    								/* 限制总长度 */
					}
					len = SetupLen >= 8 ? 8 : SetupLen;                 /* 本次传输长度 */
					memcpy( Ep0Buffer, pDescr, len );                   /* 加载上传数据 */
					SetupLen -= len;
					pDescr += len;
					break;

				case USB_SET_ADDRESS:									/* 设置地址 */
					SetupLen = UsbSetupBuf->wValueL;                    /* 暂存USB设备地址 */
					break;

				case USB_GET_CONFIGURATION:								/* 获取配置值 */
					Ep0Buffer[ 0 ] = UsbConfig;
					if( SetupLen >= 1 )
					{
						len = 1;
					}
					break;

				case USB_SET_CONFIGURATION:								/* 设置配置值 */
					UsbConfig = UsbSetupBuf->wValueL;

					/* 设置枚举成功标志 */
					gModule_USB_Status = 0x01;

					break;

				case 0x0A:
					break;


				case 0:
					UsbConfig = UsbSetupBuf->bRequestType;
					break;

				default:
					len = 0xFFFF;                                       /* 操作失败 */
					break;
				}
			}
			else
			{
				len = 0xFFFF;                                               /* 包长度错误 */
			}

			if( len == 0xFFFF )
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; /* STALL */
			}
			else if( len <= 8 )                                             /* 上传数据或者状态阶段返回0长度包 */
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; /* 默认数据包是DATA1，返回应答ACK */
#if	( DEF_USB_DEBUG == 1 )
				printf("S_U\n");
#endif
			}
			else
			{
				/* 虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段 */
				UEP0_T_LEN = 0;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; /* 默认数据包是DATA1,返回应答ACK */
			}
			break;

		case UIS_TOKEN_IN | 0:                                              /* 端点0上传成功中断 */
			switch( SetupReq )
			{
			case USB_GET_DESCRIPTOR:
				len = SetupLen >= 8 ? 8 : SetupLen;                     /* 本次传输长度 */
				memcpy( Ep0Buffer, pDescr, len );                       /* 加载上传数据 */
				SetupLen -= len;
				pDescr += len;

				/* 启动数据上传 */
				UEP0_T_LEN = len;
				UEP0_CTRL ^= bUEP_T_TOG;                                /* 同步标志位翻转 */
				break;

			case USB_SET_ADDRESS:
				USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;

			default:
				UEP0_T_LEN = 0;                                         /* 状态阶段完成中断或者是强制上传0长度数据包结束控制传输 */
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			break;

		case UIS_TOKEN_OUT | 0:  											/* 端点0下传成功中断 */
			len = USB_RX_LEN;
			for(i=0; i<len; i++)
			{
				Com_Buf[i]= Ep0Buffer[i];
			}
			if(UsbConfig == 0x40)
			{
				HID_Cmd_Pack_Flag	= 1;
			}
			/* 虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段 */
			UEP0_T_LEN = 0;
			UEP0_CTRL ^= bUEP_R_TOG;                                /* 同步标志位翻转 */						/* 默认数据包是DATA0,返回应答ACK */
			break;

		default:
			break;
		}
		UIF_TRANSFER = 0;                                                       /*　写0清空中断 */
	}

	if( UIF_BUS_RST )                                                           /* USB总线复位中断 */
	{
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;                                                        /* 清中断标志 */
	}

	if( UIF_SUSPEND )                                                           /* USB总线挂起/唤醒完成中断 */
	{
		UIF_SUSPEND = 0;
		if( USB_MIS_ST & bUMS_SUSPEND )                                         /* 挂起 */
		{
			while( XBUS_AUX & bUART0_TX )
			{
				;    															/* 等待发送完成 */
			}
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                             /* USB或者RXD0有信号时可被唤醒 */
			PCON |= PD;                                                         /* 睡眠 */
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
		}
	}
	else
	{                                                                         	/* 意外的中断,不可能发生的情况 */
		USB_INT_FG = 0x00;                                                      /* 清中断标志 */
	}
}


/*******************************************************************************
* Function Name  : CH55X_Endp1_UpPack
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USB_Endp1_Up( UINT8 *pbuf, UINT8 len )
{
	memcpy( Ep1Buffer, pbuf, len );				                              /* 加载上传数据 */
	UEP1_T_LEN = len; 			                                              /* 设置上传数据长度 */
	UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                 /* 有数据时上传数据并应答ACK */
}

/*******************************************************************************
* Function Name  : CH55X_USB_Endp2_Up
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USB_Endp2_Up( UINT8 *pbuf, UINT8 len )
{
	memcpy( &Ep2Buffer[ MAX_PACKET_SIZE ], pbuf, len );				         /* 加载上传数据 */
	UEP2_T_LEN = len; 			                                             /* 设置上传数据长度 */
	UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;                 /* 有数据时上传数据并应答ACK */
}

/*******************************************************************************
* Function Name  : HID_CMD_Pack_Deal
* Description    : HID自定义设备，命令包处理
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void HID_CMD_Pack_Deal( void )
{
	UINT8 i;
	switch(Com_Buf[2])
	{
	case 0x41:
		for(i=0; i<(Com_Buf[1]+2); i++)
		{
			Mouse_Up_Buf[i]= Com_Buf[i];
		}
// 			 if(Com_Buf[6]>i)
// 			 {
//        		for(;i<Com_Buf[6];i++)
//        		{
//					Mouse_Up_Buf[i]= Mouse_Up_Buf[i-3];
//        		}
// 			 }
		CH55X_USB_Endp1_Up( Mouse_Up_Buf, i);						/* CH55X USB端点1数据上传 */

		/* 超时等待上传成功 */
		gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
		for( i = 0; i < 100; i++ )
		{
			if( Mouse_Up_Succ_Flag )
			{
				break;
			}
			mDelayuS( 100 );
		}
		break;
	case 0x43:
		for(i=0; i<(Com_Buf[1]+2); i++)
		{
			Mouse_Up_Buf[i]= Com_Buf[i];
		}
		CH55X_USB_Endp1_Up( Mouse_Up_Buf, i+2);						/* CH55X USB端点1数据上传 */

		/* 超时等待上传成功 */
		gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
		for( i = 0; i < 100; i++ )
		{
			if( Mouse_Up_Succ_Flag )
			{
				break;
			}
			mDelayuS( 100 );
		}
		break;
	case 0x46:

		Mouse_Up_Buf[0]= 0x0a;
		Mouse_Up_Buf[1]= 6;
		Mouse_Up_Buf[2]= 'F';
		Mouse_Up_Buf[3]= '4';
		Mouse_Up_Buf[4]= 'W';
		Mouse_Up_Buf[5]= 'I';
		Mouse_Up_Buf[6]= 'R';
		Mouse_Up_Buf[7]= 'E';

		CH55X_USB_Endp1_Up( Mouse_Up_Buf, 8 );						/* CH55X USB端点1数据上传 */

		/* 超时等待上传成功 */
		gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
		for( i = 0; i < 100; i++ )
		{
			if( Mouse_Up_Succ_Flag )
			{
				break;
			}
			mDelayuS( 100 );
		}
		break;

	case 0x45:
		Mouse_Up_Buf[0]= 0x0a;
		Mouse_Up_Buf[1]= 4;
		Mouse_Up_Buf[2]= 'E';
		Mouse_Up_Buf[3]= 'U';
		Mouse_Up_Buf[4]= 'S';
		Mouse_Up_Buf[5]= 'B';

		CH55X_USB_Endp1_Up( Mouse_Up_Buf, 6 );						/* CH55X USB端点1数据上传 */

		/* 超时等待上传成功 */
		gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
		for( i = 0; i < 100; i++ )
		{
			if( Mouse_Up_Succ_Flag )
			{
				break;
			}
			mDelayuS( 100 );
		}
		break;
	case 0x44:
		/* 0A 05 44 32  2E 31 39    */
		Mouse_Up_Buf[0]= 0X0A;
		Mouse_Up_Buf[1]= 0X05;
		Mouse_Up_Buf[2]= 0X44;
		Mouse_Up_Buf[3]= '2';
		Mouse_Up_Buf[4]= '.';
		Mouse_Up_Buf[5]= '1';
		Mouse_Up_Buf[6]= '9';
		CH55X_USB_Endp1_Up( Mouse_Up_Buf, 7 );						/* CH55X USB端点1数据上传 */

		/* 超时等待上传成功 */
		gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
		for( i = 0; i < 100; i++ )
		{
			if( Mouse_Up_Succ_Flag )
			{
				break;
			}
			mDelayuS( 100 );
		}
		break;

	default:
		break;
	}
}

/*******************************************************************************
* Function Name  : MOUSE_Relative_Up_Pack
* Description    : 相对鼠标上传一包数据
*				   数据规则: 第1个字节: REPORT ID(0x01);
*							 第2个字节: 按键状态(BIT2:中键,BIT1:右键,BIT0:左键);
*							 第3个字节: 滚轮滚动齿数(0x01---0x7F向上滚动; 0x81---0xFF向下滚动);
*							 第4个字节: X轴移动距离(0x01---0x7F向右滚动; 0x81---0xFF向左滚动);
*							 第5个字节: 0x00;
*							 第6个字节: Y轴移动距离(0x01---0x7F向下滚动; 0x81---0xFF向上滚动);
*							 第7个字节: 0x00;
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MOUSE_Relative_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z )
{
	UINT8  i;

	/* 上传鼠标USB数据包 */
	Mouse_Up_Buf[ 0 ] = 0x01;
	Mouse_Up_Buf[ 1 ] = key;
	Mouse_Up_Buf[ 2 ] = z;
	Mouse_Up_Buf[ 3 ] = (UINT8)( x );
	Mouse_Up_Buf[ 4 ] = (UINT8)( x >> 8 );
	Mouse_Up_Buf[ 5 ] = (UINT8)( y );
	Mouse_Up_Buf[ 6 ] = (UINT8)( y >> 8 );
	Mouse_Up_Succ_Flag = 0x00;									 				/* 清鼠标数据上传成功标志 */
	CH55X_USB_Endp1_Up( Mouse_Up_Buf, DEF_MOUSE_PACK_LEN );						/* CH55X USB端点1数据上传 */

	/* 超时等待上传成功 */
	gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
	for( i = 0; i < 100; i++ )
	{
		if( Mouse_Up_Succ_Flag )
		{
			break;
		}
		mDelayuS( 100 );
	}
}

/*******************************************************************************
* Function Name  : MOUSE_Absolute_Up_Pack
* Description    : 绝对鼠标上传一包数据
*				   数据规则: 第1个字节: REPORT ID(0x02);
*							 第2个字节: 按键状态(BIT2:中键,BIT1:右键,BIT0:左键);
*							 第3个字节: 滚轮滚动齿数(0x01---0x7F向上滚动; 0x81---0xFF向下滚动);
*							 第4个字节: X轴方向坐标低字节;
*							 第5个字节: X轴方向坐标高字节;
*							 第6个字节: Y轴方向坐标低字节;
*							 第7个字节: Y轴方向坐标高字节;
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MOUSE_Absolute_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z )
{
	UINT8  i;

	/* 上传鼠标USB数据包 */
	Mouse_Up_Buf[ 0 ] = 0x02;
	Mouse_Up_Buf[ 1 ] = key;
	Mouse_Up_Buf[ 2 ] = z;
	Mouse_Up_Buf[ 3 ] = (UINT8)( x );
	Mouse_Up_Buf[ 4 ] = (UINT8)( x >> 8 );
	Mouse_Up_Buf[ 5 ] = (UINT8)( y );
	Mouse_Up_Buf[ 6 ] = (UINT8)( y >> 8 );
	Mouse_Up_Succ_Flag = 0x00;									 				/* 清鼠标数据上传成功标志 */
	CH55X_USB_Endp1_Up( Mouse_Up_Buf, DEF_MOUSE_PACK_LEN );						/* CH55X USB端点1数据上传 */

	/* 超时等待上传成功 */
	gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
	for( i = 0; i < 100; i++ )
	{
		if( Mouse_Up_Succ_Flag )
		{
			break;
		}
		mDelayuS( 100 );
	}
}

void MOUSE_Absolute_Up_Pack_Soft( UINT8 key, UINT16 x, UINT16 y )
{
	UINT8  i;

	/* 上传鼠标USB数据包 */
	Mouse_Up_Buf[ 0 ] = key;
	Mouse_Up_Buf[ 1 ] = (UINT8)( ( x >> 7 )&0xFF);
	Mouse_Up_Buf[ 2 ] = (UINT8)(   x & 0X7F );
	Mouse_Up_Buf[ 3 ] = (UINT8)( ( y >> 7 )&0XFF);
	Mouse_Up_Buf[ 4 ] = (UINT8)(   y & 0X7F );
	Mouse_Up_Succ_Flag = 0x00;									 				/* 清鼠标数据上传成功标志 */
	CH55X_USB_Endp1_Up( Mouse_Up_Buf, 5 );						/* CH55X USB端点1数据上传 */

	/* 超时等待上传成功 */
	gUSB_Up_Flag = 0x01;														/* 数据包上传LED点亮、熄灭切换标志 */
	for( i = 0; i < 100; i++ )
	{
		if( Mouse_Up_Succ_Flag )
		{
			break;
		}
		mDelayuS( 100 );
	}
}
/********************************** (C) COPYRIGHT *******************************
* File Name          : DEVICE.C
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : CH559ģ��USB�����豸,USB��꼰HID���Զ����豸,֧��������
*******************************************************************************/



/******************************************************************************/
/* ͷ�ļ����� */
#include <stdio.h>
#include <string.h>
#define NO_XSFR_DEFINE															/* ��CH559.Hǰ����ú� */
#include "..\myfile\CH554.H"													/* CH559���ͷ�ļ� */
#include "DEBUG.H"
#include "..\myfile\USB_DESC.H"													/* CH55X USB���������ͷ�ļ� */
#include "..\myfile\DEVICE.H"	 												/* CH55X USB�豸ģʽ�������ͷ�ļ� */

/******************************************************************************/
/* ������������ */
#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define DEF_USB_DEBUG  	 	       0x00											/* USB���Դ�ӡ���� */

UINT8X  Ep0Buffer[ THIS_ENDP0_SIZE ] _at_ 0x0000;                               /* �˵�0 OUT&IN������,������ż��ַ */
UINT8X	Ep1Buffer[ MAX_PACKET_SIZE ] _at_ 0x0040;                               /* �˵�1 IN������,������ż��ַ */
UINT8X	Ep2Buffer[ MAX_PACKET_SIZE * 2 ] _at_ 0x0080;                           /* �˵�2 IN������,������ż��ַ */

volatile UINT8X  Com_Buf[ 100 ];												/* ��ʱ���û����� */
volatile UINT8X  Mouse_Up_Buf[ 8 ];											 	/* ��������ϴ������� */
volatile UINT8X  gUSB_Up_Flag = 0x00;											/* USB���ݰ��ϴ���־(LED������Ϩ���л���־) */

UINT8 	Mouse_Up_Succ_Flag;
UINT8 	gModule_USB_Status;
UINT16X SetupLen;																/* Setup������ */
UINT8X  SetupReq,Ready,Count,FLAG,UsbConfig;
PUINT8  pDescr;                                                                 /* USB���ñ�־ */

USB_SETUP_REQ   SetupReqBuf;                                                    /* �ݴ�Setup�� */


#define UsbSetupBuf                ( (PUSB_SETUP_REQ)Ep0Buffer )

UINT8X	HID_Cmd_Pack_Flag = 0x00;												/* ���յ�HID�������־ */

/*******************************************************************************
* Function Name  : CH55X_USBD_ModeCfg
* Description    : CH55X USB�豸����ģʽ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_ModeCfg( void )
{
	USB_CTRL = 0x00;                                                            /* ���USB���ƼĴ��� */
	USB_CTRL &= ~bUC_HOST_MODE;                                                 /* ��λΪѡ���豸ģʽ */
	USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;					    /* USB�豸���ڲ�����ʹ��,���ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK */
	USB_DEV_AD = 0x00;                                                          /* �豸��ַ��ʼ�� */
//	UDEV_CTRL &= ~bUD_RECV_DIS;                                                 /* ʹ�ܽ����� */

#if 0
	USB_CTRL |= bUC_LOW_SPEED;
	UDEV_CTRL |= bUD_LOW_SPEED;                                                 /* ѡ�����1.5Mģʽ */
#endif

#if  1
	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED;                                             	/* ѡ��ȫ��12Mģʽ,Ĭ�Ϸ�ʽ */
#endif

	UDEV_CTRL |= bUD_PD_DIS;                          						    /* ��ֹDM��DP�������� */
	UDEV_CTRL |= bUD_PORT_EN;                                                 	/* ʹ�������˿� */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_IntCfg
* Description    : USB�豸ģʽ�ж�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_IntCfg( void )
{
	USB_INT_EN |= bUIE_SUSPEND;                                               	/* ʹ���豸�����ж� */
	USB_INT_EN |= bUIE_TRANSFER;                                                /* ʹ��USB��������ж� */
	USB_INT_EN |= bUIE_BUS_RST;                                                 /* ʹ���豸ģʽUSB���߸�λ�ж� */
	USB_INT_FG |= 0x1F;                                                         /* ���жϱ�־ */
	IE_USB = 1;                                                                 /* ʹ��USB�ж� */
	EA = 1; 																    /* ������Ƭ���ж� */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_EnpdCfg
* Description    : CH55X USB�豸ģʽ�˵�����
*				   ��ǰģ�⸴���豸,���˶˵�0�Ŀ��ƴ���,�������˵�
*				   1��2���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_EnpdCfg( void )
{
	UEP1_DMA = Ep1Buffer;                                                       /* �˵�1���ݴ����ַ */
	UEP4_1_MOD |= bUEP1_TX_EN;                                                  /* �˵�1����ʹ�� */
	UEP4_1_MOD &= ~bUEP1_RX_EN;                                                 /* �˵�1���ս�ֹ */
	UEP4_1_MOD &= ~bUEP1_BUF_MOD;                                               /* �˵�1��64�ֽڷ��ͻ����� */
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;								    /* �˵�1�Զ���תͬ����־λ��IN���񷵻�NAK */

	UEP2_DMA = Ep2Buffer;                                                       /* �˵�2���ݴ����ַ */
	UEP2_3_MOD |= bUEP2_TX_EN;                                                  /* �˵�2����ʹ�� */
	UEP2_3_MOD |= bUEP2_RX_EN;                          	                    /* �˵�2����ʹ�� */
	UEP2_3_MOD &= ~bUEP2_BUF_MOD;                                               /* �˵�2��64�ֽڷ��ͻ����� */
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;								    /* �˵�2�Զ���תͬ����־λ��IN���񷵻�NAK */

	UEP0_DMA = Ep0Buffer;                                                       /* �˵�0���ݴ����ַ */
	UEP4_1_MOD &= ~( bUEP4_RX_EN | bUEP4_TX_EN );								/* �˵�0��64�ֽ��շ������� */
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                  /* OUT���񷵻�ACK,IN���񷵻�NAK */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_Init
* Description    : CH55X USB�豸ģʽ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH55X_USBD_Init( void )
{
	CH55X_USBD_ModeCfg( );														/* CH55X USB�豸����ģʽ���� */
	CH55X_USBD_EnpdCfg( );														/* CH55X USB�豸ģʽ�˵����� */
	CH55X_USBD_IntCfg( );														/* CH55X USB�豸ģʽ�ж����� */
	UEP1_T_LEN = 0;	                                                      		/* Ԥʹ�÷��ͳ���һ��Ҫ��� */
	UEP2_T_LEN = 0;	                                                      		/* Ԥʹ�÷��ͳ���һ��Ҫ��� */
}

/*******************************************************************************
* Function Name  : CH55X_USBD_Interrupt
* Description    : CH55X USB�жϷ������,ʹ�üĴ�����1
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
	if( UIF_TRANSFER )                                                          /* USB��������ж� */
	{
		switch( USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
		{
		case UIS_TOKEN_IN | 2:                                              /* �˵�2�ϴ��ɹ��ж� */
			UEP2_T_LEN = 0;	                                                /* Ԥʹ�÷��ͳ���һ��Ҫ��� */
//				UEP1_CTRL ^= bUEP_T_TOG;                                        /* ����������Զ���ת����Ҫ�ֶ���ת */
			UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;        /* Ĭ��Ӧ��NAK */
			break;

		case UIS_TOKEN_IN | 1:                                              /* �˵�1�ϴ��ɹ��ж� */
			UEP1_T_LEN = 0;                                                 /* Ԥʹ�÷��ͳ���һ��Ҫ��� */
//				UEP2_CTRL ^= bUEP_T_TOG;                                        /* ����������Զ���ת����Ҫ�ֶ���ת */
			UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;        /* Ĭ��Ӧ��NAK */

			Mouse_Up_Succ_Flag = 0x01;										 /* ��������ϴ��ɹ���־ */
			break;

		case UIS_TOKEN_OUT | 2:                                             /* �˵�2�����´��ɹ��ж� */
			if( U_TOG_OK )                                                	/* ��ͬ�������ݰ������� */
			{
//					UEP2_CTRL ^= bUEP_R_TOG;                               		/* ���Զ���ת */
				len = USB_RX_LEN;
				HID_Cmd_Pack_Flag = 0x01;									/* ���յ�HID�������־ */
			}
			break;

		case UIS_TOKEN_SETUP | 0:                                           /* SETUP���� */
			len = USB_RX_LEN;
			if( len == ( sizeof(USB_SETUP_REQ) ) )
			{
				SetupLen = UsbSetupBuf->wLengthH;
				SetupLen <<= 8;
				SetupLen |= UsbSetupBuf->wLengthL;
				len = 0;                                                    /* Ĭ��Ϊ�ɹ������ϴ�0���� */

				/* ����STEP�� */
				SetupReq = UsbSetupBuf->bRequest;
#if	( DEF_USB_DEBUG == 1 )
				printf("REQ %02X ",(UINT16)SetupReq);
#endif
				switch( SetupReq )                                          /* ������ */
				{
				case USB_GET_DESCRIPTOR:
					switch( UsbSetupBuf->wValueH )
					{
					case 1:	                                        /* ��ȡ�豸������ */
						pDescr = Device_Descriptor;                 /* �ϴ�ָ��ָ���豸������ */
						len = sizeof( Device_Descriptor );			/* �����ϴ����� */
						break;

					case 2:									        /* ��ȡ���������� */
						pDescr = Config_Descriptor;                 /* �ϴ�ָ��ָ������������ */
						len = sizeof( Config_Descriptor );
						break;

					case 3:                                         /* �ַ���������	*/
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

					case 0x22:                                      /* ��ȡHID���������� */
						if( UsbSetupBuf->wIndexL == 0 )             /* �ӿ�0���������� */
						{
							pDescr = Mouse_Report1_Descriptor;   	/* �ϴ�ָ��ָ���һ��HID���������� */
							len = sizeof( Mouse_Report1_Descriptor );
						}
// 									else if( UsbSetupBuf->wIndexL == 1 )        /* �ӿ�1���������� */
// 									{
// 										pDescr = Hid_Report2_Descriptor;   		/* �ϴ�ָ��ָ��ڶ���HID���������� */
// 										len = sizeof( Hid_Report2_Descriptor );
// 										Ready = 1;
// 									}
						else
						{
							len = 0xFFFF;    						/* �쳣��� */
						}
						break;

					default:
						len = 0xFFFF;                               /* ��֧�ֵ�������߳��� */
						break;
					}
					if( SetupLen > len )
					{
						SetupLen = len;    								/* �����ܳ��� */
					}
					len = SetupLen >= 8 ? 8 : SetupLen;                 /* ���δ��䳤�� */
					memcpy( Ep0Buffer, pDescr, len );                   /* �����ϴ����� */
					SetupLen -= len;
					pDescr += len;
					break;

				case USB_SET_ADDRESS:									/* ���õ�ַ */
					SetupLen = UsbSetupBuf->wValueL;                    /* �ݴ�USB�豸��ַ */
					break;

				case USB_GET_CONFIGURATION:								/* ��ȡ����ֵ */
					Ep0Buffer[ 0 ] = UsbConfig;
					if( SetupLen >= 1 )
					{
						len = 1;
					}
					break;

				case USB_SET_CONFIGURATION:								/* ��������ֵ */
					UsbConfig = UsbSetupBuf->wValueL;

					/* ����ö�ٳɹ���־ */
					gModule_USB_Status = 0x01;

					break;

				case 0x0A:
					break;


				case 0:
					UsbConfig = UsbSetupBuf->bRequestType;
					break;

				default:
					len = 0xFFFF;                                       /* ����ʧ�� */
					break;
				}
			}
			else
			{
				len = 0xFFFF;                                               /* �����ȴ��� */
			}

			if( len == 0xFFFF )
			{
				SetupReq = 0xFF;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; /* STALL */
			}
			else if( len <= 8 )                                             /* �ϴ����ݻ���״̬�׶η���0���Ȱ� */
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; /* Ĭ�����ݰ���DATA1������Ӧ��ACK */
#if	( DEF_USB_DEBUG == 1 )
				printf("S_U\n");
#endif
			}
			else
			{
				/* ��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶� */
				UEP0_T_LEN = 0;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; /* Ĭ�����ݰ���DATA1,����Ӧ��ACK */
			}
			break;

		case UIS_TOKEN_IN | 0:                                              /* �˵�0�ϴ��ɹ��ж� */
			switch( SetupReq )
			{
			case USB_GET_DESCRIPTOR:
				len = SetupLen >= 8 ? 8 : SetupLen;                     /* ���δ��䳤�� */
				memcpy( Ep0Buffer, pDescr, len );                       /* �����ϴ����� */
				SetupLen -= len;
				pDescr += len;

				/* ���������ϴ� */
				UEP0_T_LEN = len;
				UEP0_CTRL ^= bUEP_T_TOG;                                /* ͬ����־λ��ת */
				break;

			case USB_SET_ADDRESS:
				USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;

			default:
				UEP0_T_LEN = 0;                                         /* ״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ��� */
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			break;

		case UIS_TOKEN_OUT | 0:  											/* �˵�0�´��ɹ��ж� */
			len = USB_RX_LEN;
			for(i=0; i<len; i++)
			{
				Com_Buf[i]= Ep0Buffer[i];
			}
			if(UsbConfig == 0x40)
			{
				HID_Cmd_Pack_Flag	= 1;
			}
			/* ��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶� */
			UEP0_T_LEN = 0;
			UEP0_CTRL ^= bUEP_R_TOG;                                /* ͬ����־λ��ת */						/* Ĭ�����ݰ���DATA0,����Ӧ��ACK */
			break;

		default:
			break;
		}
		UIF_TRANSFER = 0;                                                       /*��д0����ж� */
	}

	if( UIF_BUS_RST )                                                           /* USB���߸�λ�ж� */
	{
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;                                                        /* ���жϱ�־ */
	}

	if( UIF_SUSPEND )                                                           /* USB���߹���/��������ж� */
	{
		UIF_SUSPEND = 0;
		if( USB_MIS_ST & bUMS_SUSPEND )                                         /* ���� */
		{
			while( XBUS_AUX & bUART0_TX )
			{
				;    															/* �ȴ�������� */
			}
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                             /* USB����RXD0���ź�ʱ�ɱ����� */
			PCON |= PD;                                                         /* ˯�� */
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
		}
	}
	else
	{                                                                         	/* ������ж�,�����ܷ�������� */
		USB_INT_FG = 0x00;                                                      /* ���жϱ�־ */
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
	memcpy( Ep1Buffer, pbuf, len );				                              /* �����ϴ����� */
	UEP1_T_LEN = len; 			                                              /* �����ϴ����ݳ��� */
	UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                 /* ������ʱ�ϴ����ݲ�Ӧ��ACK */
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
	memcpy( &Ep2Buffer[ MAX_PACKET_SIZE ], pbuf, len );				         /* �����ϴ����� */
	UEP2_T_LEN = len; 			                                             /* �����ϴ����ݳ��� */
	UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;                 /* ������ʱ�ϴ����ݲ�Ӧ��ACK */
}

/*******************************************************************************
* Function Name  : HID_CMD_Pack_Deal
* Description    : HID�Զ����豸�����������
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
		CH55X_USB_Endp1_Up( Mouse_Up_Buf, i);						/* CH55X USB�˵�1�����ϴ� */

		/* ��ʱ�ȴ��ϴ��ɹ� */
		gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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
		CH55X_USB_Endp1_Up( Mouse_Up_Buf, i+2);						/* CH55X USB�˵�1�����ϴ� */

		/* ��ʱ�ȴ��ϴ��ɹ� */
		gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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

		CH55X_USB_Endp1_Up( Mouse_Up_Buf, 8 );						/* CH55X USB�˵�1�����ϴ� */

		/* ��ʱ�ȴ��ϴ��ɹ� */
		gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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

		CH55X_USB_Endp1_Up( Mouse_Up_Buf, 6 );						/* CH55X USB�˵�1�����ϴ� */

		/* ��ʱ�ȴ��ϴ��ɹ� */
		gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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
		CH55X_USB_Endp1_Up( Mouse_Up_Buf, 7 );						/* CH55X USB�˵�1�����ϴ� */

		/* ��ʱ�ȴ��ϴ��ɹ� */
		gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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
* Description    : �������ϴ�һ������
*				   ���ݹ���: ��1���ֽ�: REPORT ID(0x01);
*							 ��2���ֽ�: ����״̬(BIT2:�м�,BIT1:�Ҽ�,BIT0:���);
*							 ��3���ֽ�: ���ֹ�������(0x01---0x7F���Ϲ���; 0x81---0xFF���¹���);
*							 ��4���ֽ�: X���ƶ�����(0x01---0x7F���ҹ���; 0x81---0xFF�������);
*							 ��5���ֽ�: 0x00;
*							 ��6���ֽ�: Y���ƶ�����(0x01---0x7F���¹���; 0x81---0xFF���Ϲ���);
*							 ��7���ֽ�: 0x00;
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MOUSE_Relative_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z )
{
	UINT8  i;

	/* �ϴ����USB���ݰ� */
	Mouse_Up_Buf[ 0 ] = 0x01;
	Mouse_Up_Buf[ 1 ] = key;
	Mouse_Up_Buf[ 2 ] = z;
	Mouse_Up_Buf[ 3 ] = (UINT8)( x );
	Mouse_Up_Buf[ 4 ] = (UINT8)( x >> 8 );
	Mouse_Up_Buf[ 5 ] = (UINT8)( y );
	Mouse_Up_Buf[ 6 ] = (UINT8)( y >> 8 );
	Mouse_Up_Succ_Flag = 0x00;									 				/* ����������ϴ��ɹ���־ */
	CH55X_USB_Endp1_Up( Mouse_Up_Buf, DEF_MOUSE_PACK_LEN );						/* CH55X USB�˵�1�����ϴ� */

	/* ��ʱ�ȴ��ϴ��ɹ� */
	gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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
* Description    : ��������ϴ�һ������
*				   ���ݹ���: ��1���ֽ�: REPORT ID(0x02);
*							 ��2���ֽ�: ����״̬(BIT2:�м�,BIT1:�Ҽ�,BIT0:���);
*							 ��3���ֽ�: ���ֹ�������(0x01---0x7F���Ϲ���; 0x81---0xFF���¹���);
*							 ��4���ֽ�: X�᷽��������ֽ�;
*							 ��5���ֽ�: X�᷽��������ֽ�;
*							 ��6���ֽ�: Y�᷽��������ֽ�;
*							 ��7���ֽ�: Y�᷽��������ֽ�;
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void MOUSE_Absolute_Up_Pack( UINT8 key, UINT16 x, UINT16 y, UINT8 z )
{
	UINT8  i;

	/* �ϴ����USB���ݰ� */
	Mouse_Up_Buf[ 0 ] = 0x02;
	Mouse_Up_Buf[ 1 ] = key;
	Mouse_Up_Buf[ 2 ] = z;
	Mouse_Up_Buf[ 3 ] = (UINT8)( x );
	Mouse_Up_Buf[ 4 ] = (UINT8)( x >> 8 );
	Mouse_Up_Buf[ 5 ] = (UINT8)( y );
	Mouse_Up_Buf[ 6 ] = (UINT8)( y >> 8 );
	Mouse_Up_Succ_Flag = 0x00;									 				/* ����������ϴ��ɹ���־ */
	CH55X_USB_Endp1_Up( Mouse_Up_Buf, DEF_MOUSE_PACK_LEN );						/* CH55X USB�˵�1�����ϴ� */

	/* ��ʱ�ȴ��ϴ��ɹ� */
	gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
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

	/* �ϴ����USB���ݰ� */
	Mouse_Up_Buf[ 0 ] = key;
	Mouse_Up_Buf[ 1 ] = (UINT8)( ( x >> 7 )&0xFF);
	Mouse_Up_Buf[ 2 ] = (UINT8)(   x & 0X7F );
	Mouse_Up_Buf[ 3 ] = (UINT8)( ( y >> 7 )&0XFF);
	Mouse_Up_Buf[ 4 ] = (UINT8)(   y & 0X7F );
	Mouse_Up_Succ_Flag = 0x00;									 				/* ����������ϴ��ɹ���־ */
	CH55X_USB_Endp1_Up( Mouse_Up_Buf, 5 );						/* CH55X USB�˵�1�����ϴ� */

	/* ��ʱ�ȴ��ϴ��ɹ� */
	gUSB_Up_Flag = 0x01;														/* ���ݰ��ϴ�LED������Ϩ���л���־ */
	for( i = 0; i < 100; i++ )
	{
		if( Mouse_Up_Succ_Flag )
		{
			break;
		}
		mDelayuS( 100 );
	}
}

/********************************** (C) COPYRIGHT *******************************
* File Name          : HID_TP.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/05/09
* Description        : CH554模拟HID Touch设备上传数据
                       数据来源于下游I2C触摸驱动芯片GT911
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "CH554.H"
#include "DEBUG.H"
#include "GT911.H"
#include "USB_DESC.H"


UINT8X  Ep0Buffer[8<(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;        //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X  Ep2Buffer[128<(2*MAX_PACKET_SIZE+4)?128:(2*MAX_PACKET_SIZE+4)] _at_ 0x0044;//端点2 IN&OUT缓冲区,必须是偶地址
UINT8   SetupReq,Ready,Count,FLAG,UsbConfig;
UINT16 SetupLen;
PUINT8  pDescr;                                                                    //USB配置标志
USB_SETUP_REQ   SetupReqBuf;                                                       //暂存Setup包
UINT8 USB_Enum_OK;
#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)  

#pragma  NOAREGS


UINT8X EP_UP_BUF[84] = {0};
UINT8X UserEp2Buf[64];                                            //用户数据定义

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{   
	UINT8 i;
	  IE_USB = 0;
	  USB_CTRL = 0x00;                                                           // 先设定USB设备模式
#ifndef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED;                                                //选择低速1.5M模式
#else
    UDEV_CTRL &= ~bUD_LOW_SPEED;                                               //选择全速12M模式，默认方式
#endif
    UEP2_DMA = Ep2Buffer;                                                      //端点2数据传输地址
    UEP2_3_MOD |= bUEP2_TX_EN;                                                 //端点2发送使能
    UEP2_3_MOD |= bUEP2_RX_EN;                                                 //端点2接收使能
    UEP2_3_MOD &= ~bUEP2_BUF_MOD;                                              //端点2收发各64字节缓冲区
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;                 //端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
    UEP0_DMA = Ep0Buffer;                                                      //端点0数据传输地址
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //端点0单64字节收发缓冲区
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT事务返回ACK，IN事务返回NAK
		
	USB_DEV_AD = 0x00;
	UDEV_CTRL = bUD_PD_DIS;                                                    // 禁止DP/DM下拉电阻
	USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
	UDEV_CTRL |= bUD_PORT_EN;                                                  // 允许USB端口
	USB_INT_FG = 0xFF;                                                         // 清中断标志
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	IE_USB = 1;
	
	for(i=0; i<64; i++)                                                   //准备演示数据
    {
        UserEp2Buf[0] = 0;
    }
	FLAG = 0;
    Ready = 0;
}

/*******************************************************************************
* Function Name  : Enp2BlukIn()
* Description    : USB设备模式端点2的批量上传
* Input          : UINT8X buf, UINT8 n 
* Output         : None
* Return         : None
*******************************************************************************/
static void Enp2BlukIn( UINT8 * buf, UINT8 n )
{
    memcpy( Ep2Buffer+MAX_PACKET_SIZE, buf, n);        //加载上传数据
    UEP2_T_LEN = n;                                              //上传最大包长度
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                  //有数据时上传数据并应答ACK
    while(UEP2_CTRL&UEP_T_RES_ACK);                                            //等待传输完成
}

/*******************************************************************************
* Function Name  : DeviceInterrupt
* Description    : CH559USB中断处理函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void    DeviceInterrupt( void ) interrupt INT_NO_USB using 1                    //USB中断服务程序,使用寄存器组1
{
    UINT8 i;
    UINT16 len;
    if(UIF_TRANSFER)                                                            //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 端点批量上传
             UEP2_T_LEN = 0;                                                    //预使用发送长度一定要清空
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            break;
        case UIS_TOKEN_OUT | 2:                                                 //endpoint 2# 端点批量下传
//             if ( U_TOG_OK )                                                     // 不同步的数据包将丢弃
//             {
//                 len = USB_RX_LEN;                                               //接收数据长度，数据从Ep2Buffer首地址开始存放
//                 for ( i = 0; i < len; i ++ )
//                 {
//                     Ep2Buffer[MAX_PACKET_SIZE+i] = Ep2Buffer[i] ^ 0xFF;         // OUT数据取反到IN由计算机验证
//                 }
//                 UEP2_T_LEN = len;
//                 UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;       // 允许上传
//             }
            break;
        case UIS_TOKEN_SETUP | 0:                                               //SETUP事务
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthH;
                SetupLen <<= 8;							
                SetupLen |= UsbSetupBuf->wLengthL;
                len = 0;                                                         // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;							
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/*HID类命令*/
                {
					switch( SetupReq )                                             
					{
						case 0x01:                                                   //GetReport				
							Ep0Buffer[0] = 0x02;
							Ep0Buffer[1] = 0x0A;										
							len = 2;
							break;
						case 0x02:                                                   //GetIdle
							break;	
						case 0x03:                                                   //GetProtocol
							break;				
						case 0x09:                                                   //SetReport										
							break;
						case 0x0A:                                                   //SetIdle
							break;	
						case 0x0B:                                                   //SetProtocol
							break;
						default:
							len = 0xFF;  				                                   /*命令不支持*/					
							break;
				  }		
                }
                else                                                             //标准请求
                {
                    switch(SetupReq)                                             //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                                  //设备描述符
                            pDescr = DevDesc;                                    //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                                  //配置描述符
                            pDescr = CfgDesc;                                    //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                                               //报表描述符
                            pDescr = HIDRepDesc;                                 //数据准备上传
                            len = sizeof(HIDRepDesc);
                            Ready = 1;                                           //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
                            break;
						case 3:                                          // 字符串描述符
							switch( UsbSetupBuf->wValueL ) {
								case 1:
									pDescr = (PUINT8)( &MyManuInfo[0] );
									len = sizeof( MyManuInfo );
									break;
								case 2:
									pDescr = (PUINT8)( &MyProdInfo[0] );
									len = sizeof( MyProdInfo );
									break;
								case 0:
									pDescr = (PUINT8)( &MyLangDescr[0] );
									len = sizeof( MyLangDescr );
									break;
								default:
									len = 0xFF;                               // 不支持的字符串描述符
									break;
							}
							break;												
                        default:
                            len = 0xff;                                          //不支持的命令或者出错
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //限制总长度
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;//本次传输长度
                        memcpy(Ep0Buffer,pDescr,len);                            //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                         //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
						USB_Enum_OK = 1;
						EX0 = 1;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                      //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                       // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF;                                           // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                                         /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )        /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF;                                    /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                        /* 操作失败 */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )    /* 设置端点 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                                     /* 操作失败 */
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                         /* 操作失败 */
                            }
                        }
                        else
                        {
                            len = 0xFF;                                             /* 操作失败 */
                        } 
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff;                                                  //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                          //包长度错误
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len <= THIS_ENDP0_SIZE)                                         //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                                      //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;     //本次传输长度
                memcpy( Ep0Buffer, pDescr, len );                                   //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                            //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                                      //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if(SetupReq == 0x09)
            {
							if ( U_TOG_OK )                                                     // 不同步的数据包将丢弃
							{
									len = USB_RX_LEN;                                                 //接收数据长度，数据从Ep1Buffer首地址开始存放
									for ( i = 0; i < len; i ++ )
									{
											UserEp2Buf[i] = Ep0Buffer[i] ^ 0xFF;		// OUT数据取反到IN由计算机验证									
									}							
                  UEP0_T_LEN = 0;
                  UEP0_CTRL = (UEP0_CTRL ^ bUEP_R_TOG) | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK										
							}
            }
            UEP0_T_LEN = 0;  															//虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;									//默认数据包是DATA0,返回应答ACK
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                           	//写0清空中断
    }
    if(UIF_BUS_RST)                                                                 	//设备模式USB总线复位中断
    {
		EX0 = 0;
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                             	//清中断标志
    }
    if (UIF_SUSPEND)                                                                 	//USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                             	//挂起
        {
#ifdef DE_PRINTF
            printf( "zz" );                                                          	//睡眠状态
#endif
            while ( XBUS_AUX & bUART0_TX )
            {
                ;    //等待发送完成
            }
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                                   	//USB或者RXD0有信号时可被唤醒
            PCON |= PD;                                                               	//睡眠
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
    }
    else
	{                                                                             		//意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                                         	    //清中断标志
//      printf("UnknownInt  N");
    }
}
/*******************************************************************************
* Function Name  : Absolute_Up_Pack
* Description    : 上传触摸板数据
* Input          : POINTER * pTP, UINT8 point_num
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 Absolute_Up_Pack( POINTER * pTP, UINT8 Point_Num )
{
	UINT8 i;
	UINT8 dat[84] = {0};
	static UINT16 Contact_Cnt = 0x0000;	

	dat[0] = 0x01;
	for( i = 0; i < POINTER_NUM; i ++ ) 
	{
		dat[1 + i * 8] = ( Point_Num == 0 ) ? 0 : ( pTP + i ) -> Tip_Switch;
		dat[2 + i * 8] = ( pTP + i ) -> Contact_Identifier;
		dat[3 + i * 8] = ( ( pTP + i ) -> X_pos ) & 0x00ff;
		dat[4 + i * 8] = ( ( pTP + i ) -> X_pos >> 8 ) & 0x00ff ;
		dat[5 + i * 8] = ( pTP + i ) -> Y_pos & 0x00ff;
		dat[6 + i * 8] = ( ( pTP + i ) -> Y_pos >> 8 ) & 0x00ff ;
		dat[7 + i * 8] = ( pTP + i ) -> Resolution_Multi & 0x00ff;
		dat[8 + i * 8] = ( ( pTP + i ) -> Resolution_Multi >> 8 ) & 0x00ff ;
	}

#if 0
	for( i = 0; i < 84; i++ )
	{
		printf( "dat[%02d]=%04x\t",(UINT16)i, (UINT16)dat[i] );
	}
	printf("\n");
#endif	
	
	Contact_Cnt++;
	dat[81] = Contact_Cnt & 0X00FF;
	dat[82] = (Contact_Cnt >> 8) & 0x00ff;
//	dat[83] = ( Point_Num == 0) ? 1: 1;
	dat[83] = 1;
	EX0 = 0;
	Enp2BlukIn( dat, 64 );
	mDelaymS(2);
	Enp2BlukIn( &dat[64], 20 );
	EX0 = 1;
	
	return 1;
}


/* END OF FILE */

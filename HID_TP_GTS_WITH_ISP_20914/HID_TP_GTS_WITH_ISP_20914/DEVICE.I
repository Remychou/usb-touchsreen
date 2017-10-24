
#line 1 "DEVICE.C" /0
 







 
 
  
#line 1 "C:\Keil\C51\Inc\stdio.h" /0






 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 typedef unsigned int size_t;
 
 
 #pragma SAVE
 #pragma REGPARMS
 extern char _getkey (void);
 extern char getchar (void);
 extern char ungetchar (char);
 extern char putchar (char);
 extern int printf   (const char *, ...);
 extern int sprintf  (char *, const char *, ...);
 extern int vprintf  (const char *, char *);
 extern int vsprintf (char *, const char *, char *);
 extern char *gets (char *, int n);
 extern int scanf (const char *, ...);
 extern int sscanf (char *, const char *, ...);
 extern int puts (const char *);
 
 #pragma RESTORE
 
 
 
#line 11 "DEVICE.C" /0
 
  
#line 1 "C:\Keil\C51\Inc\string.h" /0






 
 
 
 
 
 
#line 13 "C:\Keil\C51\Inc\string.h" /1
  
 
 
#line 16 "C:\Keil\C51\Inc\string.h" /0
 
 
#line 18 "C:\Keil\C51\Inc\string.h" /1
  
 
#line 20 "C:\Keil\C51\Inc\string.h" /0
 
 #pragma SAVE
 #pragma REGPARMS
 extern char *strcat (char *s1, char *s2);
 extern char *strncat (char *s1, char *s2, int n);
 
 extern char strcmp (char *s1, char *s2);
 extern char strncmp (char *s1, char *s2, int n);
 
 extern char *strcpy (char *s1, char *s2);
 extern char *strncpy (char *s1, char *s2, int n);
 
 extern int strlen (char *);
 
 extern char *strchr (const char *s, char c);
 extern int strpos (const char *s, char c);
 extern char *strrchr (const char *s, char c);
 extern int strrpos (const char *s, char c);
 
 extern int strspn (char *s, char *set);
 extern int strcspn (char *s, char *set);
 extern char *strpbrk (char *s, char *set);
 extern char *strrpbrk (char *s, char *set);
 extern char *strstr  (char *s, char *sub);
 extern char *strtok  (char *str, const char *set);
 
 extern char memcmp (void *s1, void *s2, int n);
 extern void *memcpy (void *s1, void *s2, int n);
 extern void *memchr (void *s, char val, int n);
 extern void *memccpy (void *s1, void *s2, char val, int n);
 extern void *memmove (void *s1, void *s2, int n);
 extern void *memset  (void *s, char val, int n);
 #pragma RESTORE
 
 
#line 12 "DEVICE.C" /0
 
  
#line 1 "CH554.H" /0







 
 
 
 
 
 
 
 
#line 16 "CH554.H" /1
 
 
#line 18 "CH554.H" /0
 
 
 
 
 
 
 
 
#line 26 "CH554.H" /1
  
 
#line 28 "CH554.H" /0
 
 
 typedef bit                             BOOL;
 
 
 typedef unsigned char                   UINT8;
 
 
 typedef unsigned short                  UINT16;
 
 
 typedef unsigned long                   UINT32;
 
 
 typedef unsigned char  data             UINT8D;
 
 
 typedef unsigned short data             UINT16D;
 
 
 typedef unsigned long  data             UINT32D;
 
 
 typedef unsigned char  idata            UINT8I;
 
 
 typedef unsigned short idata            UINT16I;
 
 
 typedef unsigned long  idata            UINT32I;
 
 
 typedef unsigned char  xdata            UINT8X;
 
 
 typedef unsigned short xdata            UINT16X;
 
 
 typedef unsigned long  xdata            UINT32X;
 
 
 typedef unsigned char volatile          UINT8V;
 
 
 typedef unsigned char volatile data     UINT8DV;
 
 
 typedef unsigned char volatile xdata    UINT8XV;
 
 
 typedef unsigned char volatile pdata    UINT8PV;
 
 
 typedef const unsigned char code        UINT8C;
 
 
 typedef unsigned char                 *PUINT8;
 
 
 typedef unsigned short                *PUINT16;
 
 
 typedef unsigned long                 *PUINT32;
 
 
 typedef unsigned char  idata          *PUINT8I;
 
 
 typedef unsigned short idata          *PUINT16I;
 
 
 typedef unsigned long  idata          *PUINT32I;
 
 
 typedef unsigned char  xdata          *PUINT8X;
 
 
 typedef unsigned short xdata          *PUINT16X;
 
 
 typedef unsigned long  xdata          *PUINT32X;
 
 
 typedef unsigned char volatile        *PUINT8V;
 
 
 typedef unsigned char volatile data   *PUINT8DV;
 
 
 typedef unsigned char volatile xdata  *PUINT8XV;
 
 
 typedef unsigned char volatile pdata  *PUINT8PV;
 
 
 typedef const unsigned char code      *PUINT8C;
 
 
 
 
 
 
 
#line 131 "CH554.H" /1
 
 
#line 133 "CH554.H" /0
 
 
 
 
 
 
 
#line 140 "CH554.H" /1
 
 
#line 142 "CH554.H" /0
 
 
 
 
 
 sfr PSW             = 0xD0;          
 sbit CY            = PSW^7;         
 sbit AC            = PSW^6;         
 sbit F0            = PSW^5;         
 sbit RS1           = PSW^4;         
 sbit RS0           = PSW^3;         
 
 
 
 
 
 
 sbit OV            = PSW^2;         
 sbit F1            = PSW^1;         
 sbit P             = PSW^0;         
 sfr ACC             = 0xE0;          
 sfr B               = 0xF0;          
 sfr SP              = 0x81;          
 
 sfr DPL             = 0x82;          
 sfr DPH             = 0x83;          
 sfr SAFE_MOD        = 0xA1;          
 
 
 sfr GLOBAL_CFG      = 0xB1;          
 
 
 
 
 
 
 
 
 sfr PCON            = 0x87;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr CLOCK_CFG       = 0xB9;          
 
 
 
 
 
 
 
 














 
 sfr WAKE_CTRL       = 0xA9;          
 
 
 
 
 
 
 
 
 sfr RESET_KEEP      = 0xFE;          
 sfr WDOG_COUNT      = 0xFF;          
 
 
 sfr IE              = 0xA8;          
 sbit EA            = IE^7;          
 sbit E_DIS         = IE^6;          
 sbit ET2           = IE^5;          
 sbit ES            = IE^4;          
 sbit ET1           = IE^3;          
 sbit EX1           = IE^2;          
 sbit ET0           = IE^1;          
 sbit EX0           = IE^0;          
 sfr IP              = 0xB8;          
 sbit PH_FLAG       = IP^7;          
 sbit PL_FLAG       = IP^6;          
 
 
 
 
 
 sbit PT2           = IP^5;          
 sbit PS            = IP^4;          
 sbit PT1           = IP^3;          
 sbit PX1           = IP^2;          
 sbit PT0           = IP^1;          
 sbit PX0           = IP^0;          
 sfr IE_EX           = 0xE8;          
 sbit IE_WDOG       = IE_EX^7;       
 sbit IE_GPIO       = IE_EX^6;       
 sbit IE_PWMX       = IE_EX^5;       
 sbit IE_UART1      = IE_EX^4;       
 sbit IE_ADC        = IE_EX^3;       
 sbit IE_USB        = IE_EX^2;       
 sbit IE_TKEY       = IE_EX^1;       
 sbit IE_SPI0       = IE_EX^0;       
 sfr IP_EX           = 0xE9;          
 
 
 
 
 
 
 
 
 sfr GPIO_IE         = 0xC7;          
 
 
 
 
 
 
 
 
 
 
 sfr16 ROM_ADDR      = 0x84;          
 sfr ROM_ADDR_L      = 0x84;          
 sfr ROM_ADDR_H      = 0x85;          
 sfr16 ROM_DATA      = 0x8E;          
 sfr ROM_DATA_L      = 0x8E;          
 sfr ROM_DATA_H      = 0x8F;          
 sfr ROM_CTRL        = 0x86;          
 
 
 
 
 
 
 
 
 sfr P1              = 0x90;          
 sbit SCK           = P1^7;          
 sbit TXD1          = P1^7;          
 sbit TIN5          = P1^7;          
 sbit MISO          = P1^6;          
 sbit RXD1          = P1^6;          
 sbit TIN4          = P1^6;          
 sbit MOSI          = P1^5;          
 sbit PWM1          = P1^5;          
 sbit TIN3          = P1^5;          
 sbit UCC2          = P1^5;          
 sbit AIN2          = P1^5;          
 sbit T2_           = P1^4;          
 sbit CAP1_         = P1^4;          
 sbit SCS           = P1^4;          
 sbit TIN2          = P1^4;          
 sbit UCC1          = P1^4;          
 sbit AIN1          = P1^4;          
 sbit TXD_          = P1^3;          
 sbit RXD_          = P1^2;          
 sbit T2EX          = P1^1;          
 sbit CAP2          = P1^1;          
 sbit TIN1          = P1^1;          
 sbit VBUS2         = P1^1;          
 sbit AIN0          = P1^1;          
 sbit T2            = P1^0;          
 sbit CAP1          = P1^0;          
 sbit TIN0          = P1^0;          
 sfr P1_MOD_OC       = 0x92;          
 sfr P1_DIR_PU       = 0x93;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr P2              = 0xA0;          
 sfr P3              = 0xB0;          
 sbit UDM           = P3^7;          
 sbit UDP           = P3^6;          
 sbit T1            = P3^5;          
 sbit PWM2          = P3^4;          
 sbit RXD1_         = P3^4;          
 sbit T0            = P3^4;          
 sbit INT1          = P3^3;          
 sbit TXD1_         = P3^2;          
 sbit INT0          = P3^2;          
 sbit VBUS1         = P3^2;          
 sbit AIN3          = P3^2;          
 sbit PWM2_         = P3^1;          
 sbit TXD           = P3^1;          
 sbit PWM1_         = P3^0;          
 sbit RXD           = P3^0;          
 sfr P3_MOD_OC       = 0x96;          
 sfr P3_DIR_PU       = 0x97;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr PIN_FUNC        = 0xC6;          
 
 
 
 
 
 
 
 
 sfr XBUS_AUX        = 0xA2;          
 
 
 
 
 
 
 
 
 sfr TCON            = 0x88;          
 sbit TF1           = TCON^7;        
 sbit TR1           = TCON^6;        
 sbit TF0           = TCON^5;        
 sbit TR0           = TCON^4;        
 sbit IE1           = TCON^3;        
 sbit IT1           = TCON^2;        
 sbit IE0           = TCON^1;        
 sbit IT0           = TCON^0;        
 sfr TMOD            = 0x89;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr TL0             = 0x8A;          
 sfr TL1             = 0x8B;          
 sfr TH0             = 0x8C;          
 sfr TH1             = 0x8D;          
 
 
 sfr SCON            = 0x98;          
 sbit SM0           = SCON^7;        
 sbit SM1           = SCON^6;        
 
 
 
 
 
 sbit SM2           = SCON^5;        
 
 sbit REN           = SCON^4;        
 sbit TB8           = SCON^3;        
 sbit RB8           = SCON^2;        
 sbit TI            = SCON^1;        
 sbit RI            = SCON^0;        
 sfr SBUF            = 0x99;          
 
 
 sfr T2CON           = 0xC8;          
 sbit TF2           = T2CON^7;       
 sbit CAP1F         = T2CON^7;       
 sbit EXF2          = T2CON^6;       
 sbit RCLK          = T2CON^5;       
 sbit TCLK          = T2CON^4;       
 sbit EXEN2         = T2CON^3;       
 sbit TR2           = T2CON^2;       
 sbit C_T2          = T2CON^1;       
 sbit CP_RL2        = T2CON^0;       
 sfr T2MOD           = 0xC9;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr16 RCAP2         = 0xCA;          
 sfr RCAP2L          = 0xCA;          
 sfr RCAP2H          = 0xCB;          
 sfr16 T2COUNT       = 0xCC;          
 sfr TL2             = 0xCC;          
 sfr TH2             = 0xCD;          
 sfr16 T2CAP1        = 0xCE;          
 sfr T2CAP1L         = 0xCE;          
 sfr T2CAP1H         = 0xCF;          
 
 
 sfr PWM_DATA2       = 0x9B;          
 sfr PWM_DATA1       = 0x9C;          
 sfr PWM_CTRL        = 0x9D;          
 
 
 
 
 
 
 
 sfr PWM_CK_SE       = 0x9E;          
 
 
 sfr SPI0_STAT       = 0xF8;          
 sbit S0_FST_ACT    = SPI0_STAT^7;   
 sbit S0_IF_OV      = SPI0_STAT^6;   
 sbit S0_IF_FIRST   = SPI0_STAT^5;   
 sbit S0_IF_BYTE    = SPI0_STAT^4;   
 sbit S0_FREE       = SPI0_STAT^3;   
 sbit S0_T_FIFO     = SPI0_STAT^2;   
 sbit S0_R_FIFO     = SPI0_STAT^0;   
 sfr SPI0_DATA       = 0xF9;          
 sfr SPI0_CTRL       = 0xFA;          
 
 
 
 
 
 
 
 
 sfr SPI0_CK_SE      = 0xFB;          
 
 
 sfr SPI0_SETUP      = 0xFC;          
 
 
 
 
 
 
 
 
 
 sfr SCON1           = 0xC0;          
 sbit U1SM0         = SCON1^7;       
 sbit U1SMOD        = SCON1^5;       
 sbit U1REN         = SCON1^4;       
 sbit U1TB8         = SCON1^3;       
 sbit U1RB8         = SCON1^2;       
 sbit U1TI          = SCON1^1;       
 sbit U1RI          = SCON1^0;       
 sfr SBUF1           = 0xC1;          
 sfr SBAUD1          = 0xC2;          
 
 
 sfr ADC_CTRL        = 0x80;          
 sbit CMPO          = ADC_CTRL^7;    
 sbit CMP_IF        = ADC_CTRL^6;    
 sbit ADC_IF        = ADC_CTRL^5;    
 sbit ADC_START     = ADC_CTRL^4;    
 sbit CMP_CHAN      = ADC_CTRL^3;    
 sbit ADC_CHAN1     = ADC_CTRL^1;    
 sbit ADC_CHAN0     = ADC_CTRL^0;    
 
 
 
 
 
 sfr ADC_CFG         = 0x9A;          
 
 
 
 sfr ADC_DATA        = 0x9F;          
 
 
 sfr TKEY_CTRL       = 0xC3;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr16 TKEY_DAT      = 0xC4;          
 sfr TKEY_DATL       = 0xC4;          
 sfr TKEY_DATH       = 0xC5;          
 
 
 
 sfr USB_C_CTRL      = 0x91;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr UDEV_CTRL       = 0xD1;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr UEP1_CTRL       = 0xD2;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr UEP1_T_LEN      = 0xD3;          
 sfr UEP2_CTRL       = 0xD4;          
 sfr UEP2_T_LEN      = 0xD5;          
 sfr UEP3_CTRL       = 0xD6;          
 sfr UEP3_T_LEN      = 0xD7;          
 sfr USB_INT_FG      = 0xD8;          
 sbit U_IS_NAK      = USB_INT_FG^7;  
 sbit U_TOG_OK      = USB_INT_FG^6;  
 sbit U_SIE_FREE    = USB_INT_FG^5;  
 sbit UIF_FIFO_OV   = USB_INT_FG^4;  
 sbit UIF_HST_SOF   = USB_INT_FG^3;  
 sbit UIF_SUSPEND   = USB_INT_FG^2;  
 sbit UIF_TRANSFER  = USB_INT_FG^1;  
 sbit UIF_DETECT    = USB_INT_FG^0;  
 sbit UIF_BUS_RST   = USB_INT_FG^0;  
 sfr USB_INT_ST      = 0xD9;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr USB_MIS_ST      = 0xDA;          
 
 
 
 
 
 
 
 
 sfr USB_RX_LEN      = 0xDB;          
 sfr UEP0_CTRL       = 0xDC;          
 sfr UEP0_T_LEN      = 0xDD;          
 sfr UEP4_CTRL       = 0xDE;          
 sfr UEP4_T_LEN      = 0xDF;          
 sfr USB_INT_EN      = 0xE1;          
 
 
 
 
 
 
 
 
 sfr USB_CTRL        = 0xE2;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr USB_DEV_AD      = 0xE3;          
 
 
 sfr16 UEP2_DMA      = 0xE4;          
 sfr UEP2_DMA_L      = 0xE4;          
 sfr UEP2_DMA_H      = 0xE5;          
 sfr16 UEP3_DMA      = 0xE6;          
 sfr UEP3_DMA_L      = 0xE6;          
 sfr UEP3_DMA_H      = 0xE7;          
 sfr UEP4_1_MOD      = 0xEA;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr UEP2_3_MOD      = 0xEB;          
 
 
 
 
 
 
 sfr16 UEP0_DMA      = 0xEC;          
 sfr UEP0_DMA_L      = 0xEC;          
 sfr UEP0_DMA_H      = 0xED;          
 sfr16 UEP1_DMA      = 0xEE;          
 sfr UEP1_DMA_L      = 0xEE;          
 sfr UEP1_DMA_H      = 0xEF;          
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

















 
 
 
#line 881 "CH554.H" /1
 
 
#line 883 "CH554.H" /0
 
 
 
 
 
 
 
#line 890 "CH554.H" /1
 
 
#line 892 "CH554.H" /0
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 typedef struct _USB_SETUP_REQ {
 UINT8 bRequestType;
 UINT8 bRequest;
 UINT8 wValueL;
 UINT8 wValueH;
 UINT8 wIndexL;
 UINT8 wIndexH;
 UINT8 wLengthL;
 UINT8 wLengthH;
 } USB_SETUP_REQ, *PUSB_SETUP_REQ;
 
 typedef USB_SETUP_REQ xdata *PXUSB_SETUP_REQ;
 
 typedef struct _USB_DEVICE_DESCR {
 UINT8 bLength;
 UINT8 bDescriptorType;
 UINT8 bcdUSBL;
 UINT8 bcdUSBH;
 UINT8 bDeviceClass;
 UINT8 bDeviceSubClass;
 UINT8 bDeviceProtocol;
 UINT8 bMaxPacketSize0;
 UINT8 idVendorL;
 UINT8 idVendorH;
 UINT8 idProductL;
 UINT8 idProductH;
 UINT8 bcdDeviceL;
 UINT8 bcdDeviceH;
 UINT8 iManufacturer;
 UINT8 iProduct;
 UINT8 iSerialNumber;
 UINT8 bNumConfigurations;
 } USB_DEV_DESCR, *PUSB_DEV_DESCR;
 
 typedef USB_DEV_DESCR xdata *PXUSB_DEV_DESCR;
 
 typedef struct _USB_CONFIG_DESCR {
 UINT8 bLength;
 UINT8 bDescriptorType;
 UINT8 wTotalLengthL;
 UINT8 wTotalLengthH;
 UINT8 bNumInterfaces;
 UINT8 bConfigurationValue;
 UINT8 iConfiguration;
 UINT8 bmAttributes;
 UINT8 MaxPower;
 } USB_CFG_DESCR, *PUSB_CFG_DESCR;
 
 typedef USB_CFG_DESCR xdata *PXUSB_CFG_DESCR;
 
 typedef struct _USB_INTERF_DESCR {
 UINT8 bLength;
 UINT8 bDescriptorType;
 UINT8 bInterfaceNumber;
 UINT8 bAlternateSetting;
 UINT8 bNumEndpoints;
 UINT8 bInterfaceClass;
 UINT8 bInterfaceSubClass;
 UINT8 bInterfaceProtocol;
 UINT8 iInterface;
 } USB_ITF_DESCR, *PUSB_ITF_DESCR;
 
 typedef USB_ITF_DESCR xdata *PXUSB_ITF_DESCR;
 
 typedef struct _USB_ENDPOINT_DESCR {
 UINT8 bLength;
 UINT8 bDescriptorType;
 UINT8 bEndpointAddress;
 UINT8 bmAttributes;
 UINT8 wMaxPacketSizeL;
 UINT8 wMaxPacketSizeH;
 UINT8 bInterval;
 } USB_ENDP_DESCR, *PUSB_ENDP_DESCR;
 
 typedef USB_ENDP_DESCR xdata *PXUSB_ENDP_DESCR;
 
 typedef struct _USB_CONFIG_DESCR_LONG {
 USB_CFG_DESCR   cfg_descr;
 USB_ITF_DESCR   itf_descr;
 USB_ENDP_DESCR  endp_descr[1];
 } USB_CFG_DESCR_LONG, *PUSB_CFG_DESCR_LONG;
 
 typedef USB_CFG_DESCR_LONG xdata *PXUSB_CFG_DESCR_LONG;
 
 typedef struct _USB_HUB_DESCR {
 UINT8 bDescLength;
 UINT8 bDescriptorType;
 UINT8 bNbrPorts;
 UINT8 wHubCharacteristicsL;
 UINT8 wHubCharacteristicsH;
 UINT8 bPwrOn2PwrGood;
 UINT8 bHubContrCurrent;
 UINT8 DeviceRemovable;
 UINT8 PortPwrCtrlMask;
 } USB_HUB_DESCR, *PUSB_HUB_DESCR;
 
 typedef USB_HUB_DESCR xdata *PXUSB_HUB_DESCR;
 
 typedef struct _USB_HID_DESCR {
 UINT8 bLength;
 UINT8 bDescriptorType;
 UINT8 bcdHIDL;
 UINT8 bcdHIDH;
 UINT8 bCountryCode;
 UINT8 bNumDescriptors;
 UINT8 bDescriptorTypeX;
 UINT8 wDescriptorLengthL;
 UINT8 wDescriptorLengthH;
 } USB_HID_DESCR, *PUSB_HID_DESCR;
 
 typedef USB_HID_DESCR xdata *PXUSB_HID_DESCR;
 
 typedef struct _UDISK_BOC_CBW {          
 UINT8 mCBW_Sig0;
 UINT8 mCBW_Sig1;
 UINT8 mCBW_Sig2;
 UINT8 mCBW_Sig3;
 UINT8 mCBW_Tag0;
 UINT8 mCBW_Tag1;
 UINT8 mCBW_Tag2;
 UINT8 mCBW_Tag3;
 UINT8 mCBW_DataLen0;
 UINT8 mCBW_DataLen1;
 UINT8 mCBW_DataLen2;
 UINT8 mCBW_DataLen3;                 
 UINT8 mCBW_Flag;                     
 UINT8 mCBW_LUN;
 UINT8 mCBW_CB_Len;                   
 UINT8 mCBW_CB_Buf[16];               
 } UDISK_BOC_CBW, *PUDISK_BOC_CBW;
 
 typedef UDISK_BOC_CBW xdata *PXUDISK_BOC_CBW;
 
 typedef struct _UDISK_BOC_CSW {          
 UINT8 mCSW_Sig0;
 UINT8 mCSW_Sig1;
 UINT8 mCSW_Sig2;
 UINT8 mCSW_Sig3;
 UINT8 mCSW_Tag0;
 UINT8 mCSW_Tag1;
 UINT8 mCSW_Tag2;
 UINT8 mCSW_Tag3;
 UINT8 mCSW_Residue0;                 
 UINT8 mCSW_Residue1;
 UINT8 mCSW_Residue2;
 UINT8 mCSW_Residue3;                 
 UINT8 mCSW_Status;                   
 } UDISK_BOC_CSW, *PUDISK_BOC_CSW;
 
 typedef UDISK_BOC_CSW xdata *PXUDISK_BOC_CSW;
 
 
#line 1214 "CH554.H" /1
 
 
#line 1216 "CH554.H" /0
 
 
#line 13 "DEVICE.C" /0
 
  
#line 1 "DEBUG.H" /0








 
 
 
 
 
 
#line 15 "DEBUG.H" /1
 
 
#line 17 "DEBUG.H" /0
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 void	CfgFsys					( void );                    
 void	mDelayuS				( UINT16 n );              	 
 void	mDelaymS				( UINT16 n );              	 
 void  	CH554UART0Alter			( void );                 	 
 void	mInitSTDIO				( void );                    
 UINT8 	CH554UART0RcvByte		( void );              		 
 void  	CH554UART0SendByte		(UINT8 SendDat); 			 
 
 void	UART1Setup				( void );                    
 UINT8 	CH554UART1RcvByte		( void );              		 
 void  	CH554UART1SendByte		(UINT8 SendDat); 			 
 
 void 	CH554WDTModeSelect		(UINT8 mode);    			 
 void 	CH554WDTFeed			(UINT8 tim);           		 
 
 
#line 69 "DEBUG.H" /1
 
 
#line 71 "DEBUG.H" /0
 
 
 
 
#line 14 "DEVICE.C" /0
 
  
#line 1 "GT911.H" /0
 






 
 











 
 
 
 
 
 
#line 27 "GT911.H" /1
 
 
#line 29 "GT911.H" /0
 
  
#line 1 "CH554.H" /0







 
 
 
 
 
#line 13 "CH554.H" /1
  
 
 
 
 
 
 
 
 
  
  
 
 
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
 
 
 
 
 
 
 
#line 135 "CH554.H" /0
 
 
#line 137 "CH554.H" /1
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
 
  
  
  
  
  
  
 
 
 
  
 
  
  
  
  
  
  
  
 
 
 
 
 
  
  
  
 
  
  
  
  
  
  
  
  














 
 
  
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
  
 
  
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
  
  
 
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
  
  
  
  
  
  
  
  
 
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
 
 
 
 
 
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
 
  
  
  
  
 
 
 
 
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
  
 
 
  
 
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
 
 
 
 
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
  
 
 
 
  
  
  
  
  
  
  
  
 
 
 
 
 
 
  
  
  
  
  
  
 
  
  
  
  
  
  
  
 
  
  
  
  
  
  
  
  
  
  
 
 
 
 
 
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
  
  
 
 
 
 
 
  
  
 
  
  
  
  
  
  
  
  
 
 
 
 
 
 
  
  
  
  
  
  
  
  
 
  
  
  
  
  
  
 
 
 
 
 
 
 
 
  
  
  
  
 
  
  
 
 
 
 
 
 
 
  
  
  
 
 
 
 
 
 
 
 
  
  
 
 
 
 
 
 
 
  
  
  
  
  
  
 
 
 
 
 
 
 
  
  
  
 
  
  
  
  
 
  
  
  
 
  
  
  
  
 
  
 
  
  
  
 
 
 
 
  
  
 
 
 
 
 
  
 
  
 
  
 
  
 
  
 
  
 
 
 
  
 
 
  
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
 
  
  
  
  
  
  
 

















 
 
 
 
 
 
 
#line 885 "CH554.H" /0
 
 
#line 887 "CH554.H" /1
  
 
 
 
 
 
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
  
  
  
  
 
 
 
 
  
  
  
  
  
  
  
 
 
 
  
 
 
  
 
 
  
 
 
  
  
 
 
  
  
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 30 "GT911.H" /0
#line 30 "GT911.H" /0
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 typedef struct
 {
 UINT8 Tip_Switch;
 UINT8 Contact_Identifier;
 UINT16 X_pos;		 
 UINT16 Y_pos;		 
 UINT16 Resolution_Multi;
 
 
 }POINTER;
 
 typedef struct 
 {
 UINT16 	x_max_pos;
 UINT16 	y_max_pos;
 UINT8 	x_mirror;
 UINT8	y_mirror;
 UINT8 	x_y_swap;
 double 	X_Resolution;
 double 	Y_Resolution;	
 UINT8 	Point_Num;
 UINT8 	IRQ_Flag;
 }_GT911_Info ;
 
 extern POINTER TP[(1)];
 extern _GT911_Info GT911_Info;
 
 
 extern UINT8 	GT911_Init					( void );
 extern void 	GT911_Get_Touch_Info		( void );
 extern UINT8 	GT911_Touch_Check			( void );
 extern void 	GT911_Gesture_Check			( void );
 extern void 	GT911_Get_Gesture_Info		( void );
 extern void 	GT911_Config				( void );
 extern UINT8 	GT911_Scan					( void );
 
#line 125 "GT911.H" /1
 
 
#line 127 "GT911.H" /0
 
 
 
 
#line 15 "DEVICE.C" /0
 
  
#line 1 "USB_DESC.H" /0
 






 
 
 
 
 
#line 13 "USB_DESC.H" /1
 
 
#line 15 "USB_DESC.H" /0
 
 
 
 
 
#line 20 "USB_DESC.H" /1
  
 
#line 22 "USB_DESC.H" /0
 
 
 UINT8C DevDesc[18] = 
 {
 0x12, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 64, 0xae, 0x26, 0x06, 0x85, 0x00, 0x00, 0x01, 0x02, 
 0x00, 0x01
 };
 
 UINT8C CfgDesc[41] = 
 {
 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x20, 
 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 
 0x09, 0x21, 0x01, 0x01, 0x00, 0x01, 0x22, 0xc9, 0x03,
 0x07, 0x05, 0x82, 0x03, 64, 0x00, 0x01, 
 0x07, 0x05, 0x02, 0x03, 64, 0x00, 0x01
 };
 
 
 
 UINT8C	MyLangDescr[] = { 0x04, 0x03, 0x09, 0x04 };
 
 UINT8C	MyManuInfo[] = { 0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 };
 
 UINT8C	MyProdInfo[] = 
 {
 0x2a, 0x03, 0x43, 0x00, 0x68, 0x00, 0x69, 0x00, 0x70, 0x00, 0x4f, 0x00, 0x6e, 0x00, 0x65, 0x00, 
 0x20, 0x00, 0x48, 0x00, 0x69, 0x00, 0x64, 0x00, 0x20, 0x00, 0x44, 0x00, 0x46, 0x00, 0x55, 0x00, 
 0x20, 0x00, 0x54, 0x00, 0x6f, 0x00, 0x6f, 0x00, 0x6c, 0x00
 };
 
 
 UINT8C HIDRepDesc[969] = 
 {
 
#line 56 "USB_DESC.H" /1
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 118 "USB_DESC.H" /0
 
 0x05, 0x0d,  
 0x09, 0x04,  
 
 0xa1, 0x01,  
 
 
 0x85, 0x01,  
 0x09, 0x22,  
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0,
 
 0x09, 0x22,  
 
 0xa1, 0x02,	 
 0x09, 0x42,	 
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,	 
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79,	0x08,   
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c,	0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0,
 
 0x09, 0x22, 
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0, 
 
 0x09, 0x22, 
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0, 
 
 0x09, 0x22, 
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0, 
 
 0x09, 0x22,
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0,  
 
 0x09, 0x22,
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0,  
 
 0x09, 0x22, 
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d, 
 0x26, 0xff, 0x00, 
 0x46, 0xff, 0x00,
 0x95, 0x01, 
 0x09, 0x48, 
 0x81, 0x02, 
 0xc0, 
 
 0x09, 0x22,
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0,  
 
 0x09, 0x22, 
 
 0xa1, 0x02,  
 0x09, 0x42,  
 0x15, 0x00,  
 0x25, 0x01,  
 0x75, 0x01,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x95, 0x01,  
 0x81, 0x03,  
 0x95, 0x06,  
 0x81, 0x03,  
 0x75, 0x08,  
 0x09, 0x51,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x05, 0x01,  
 0x26, 0x00, 0x08,  
 0x75, 0x10,  
 0x55, 0x0e,  
 0x65, 0x11,  
 0x09, 0x30,  
 0x35, 0x00,  
 0x46, 0x79, 0x08,  
 0x95, 0x01,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 0x46, 0x4c, 0x05,  
 0x26, 0x00, 0x08,  
 0x09, 0x31,  
 0x81, 0x02,  
 0x35, 0x01,  
 0x35, 0x00,  
 
 0x05, 0x0d,  
 0x26, 0xff, 0x00,  
 0x46, 0xff, 0x00,  
 0x95, 0x01,  
 0x09, 0x48,  
 0x81, 0x02,  
 0xc0,  
 
 0x05, 0x0d,  
 0x27, 0xff,	0xff, 0x00,  
 0x00, 
 0x75, 0x10,  
 0x95, 0x01,  
 0x09, 0x56,  
 0x81, 0x02,  
 0x09, 0x54,  
 0x25, 0x10,  
 0x95, 0x01,  
 0x75, 0x08,  
 0x81, 0x02,  
 
 
 0x85, 0x02,  
 0x09, 0x55,  
 0x95, 0x01,  
 0x25, 0x05,  
 0xb1, 0x02,  
 0x06, 0x00, 0xff,  
 0x09, 0xc5,
 
 0x85, 0x05,  
 0x15, 0x00,  
 0x26, 0xff, 0x00,  
 0x75, 0x08,  
 0x96, 0x00, 0x01,  
 0xb1, 0x02,  
 0xc0, 
 
 
 0x06, 0x01, 0xff, 
 0x09, 0x00, 
 
 0xa1, 0x01,
 0x15, 0x00,  
 0x26, 0xff, 0x00,  
 0x75, 0x08,  
 0x85, 0x07,  
 0x95, 0x40,  
 0x09, 0x00,  
 0x81, 0x02,  
 0x09, 0x00,  
 0x15, 0x00,  
 0x26, 0xff, 0x00,  
 0x75, 0x08,  
 0x95, 0x40,  
 0x91, 0x02,  
 0xc0
 
 
 
 };
 
 
#line 626 "USB_DESC.H" /1
 
 
#line 628 "USB_DESC.H" /0
 
 
#line 16 "DEVICE.C" /0
 
 
 
 UINT8X  Ep0Buffer[8<(64+2)?8:(64+2)] _at_ 0x0000;         
 UINT8X  Ep2Buffer[128<(2*64+4)?128:(2*64+4)] _at_ 0x0044; 
 UINT8   SetupReq,Ready,Count,FLAG,UsbConfig;
 UINT16 SetupLen;
 PUINT8  pDescr;                                                                     
 USB_SETUP_REQ   SetupReqBuf;                                                        
 UINT8 USB_Enum_OK;
 
 
 #pragma NOAREGS
 
 
 UINT8X EP_UP_BUF[84] = {0};
 UINT8X UserEp2Buf[64];                                             
 






 
 void USBDeviceInit()
 {   
 UINT8 i;
 IE_USB = 0;
 USB_CTRL = 0x00;                                                            
 
#line 47 "DEVICE.C" /1
 
 
#line 49 "DEVICE.C" /0
 UDEV_CTRL &= ~0x04;                                                
 
 UEP2_DMA = Ep2Buffer;                                                       
 UEP2_3_MOD |= 0x04;                                                  
 UEP2_3_MOD |= 0x08;                                                  
 UEP2_3_MOD &= ~0x01;                                               
 UEP2_CTRL = 0x10 | 0x02 | 0x00;                  
 UEP0_DMA = Ep0Buffer;                                                       
 UEP4_1_MOD &= ~(0x08 | 0x04);                                 
 UEP0_CTRL = 0x00 | 0x02;                                  
 
 USB_DEV_AD = 0x00;
 UDEV_CTRL = 0x80;                                                     
 USB_CTRL = 0x20 | 0x08 | 0x01;                       
 UDEV_CTRL |= 0x01;                                                   
 USB_INT_FG = 0xFF;                                                          
 USB_INT_EN = 0x04 | 0x02 | 0x01;
 IE_USB = 1;
 
 for(i=0; i<64; i++)                                                    
 {
 UserEp2Buf[0] = 0;
 }
 FLAG = 0;
 Ready = 0;
 }
 






 
 static void Enp2BlukIn( UINT8 * buf, UINT8 n )
 {
 memcpy( Ep2Buffer+64, buf, n);         
 UEP2_T_LEN = n;                                               
 UEP2_CTRL = UEP2_CTRL & ~ 0x03 | 0x00;                   
 while(UEP2_CTRL&0x00);                                             
 }
 






 
 void    DeviceInterrupt( void ) interrupt 8 using 1                     
 {
 UINT8 i;
 UINT16 len;
 if(UIF_TRANSFER)                                                             
 {
 switch (USB_INT_ST & (0x30 | 0x0F))
 {
 case 0x20 | 2:                                                   
 UEP2_T_LEN = 0;                                                     
 
 UEP2_CTRL = UEP2_CTRL & ~ 0x03 | 0x02;            
 break;
 case 0x00 | 2:                                                  
 
 
 
 
 
 
 
 
 
 
 break;
 case 0x30 | 0:                                                
 len = USB_RX_LEN;
 if(len == (sizeof(USB_SETUP_REQ)))
 {
 SetupLen = ((PUSB_SETUP_REQ)Ep0Buffer)->wLengthH;
 SetupLen <<= 8;							
 SetupLen |= ((PUSB_SETUP_REQ)Ep0Buffer)->wLengthL;
 len = 0;                                                          
 SetupReq = ((PUSB_SETUP_REQ)Ep0Buffer)->bRequest;							
 if ( ( ((PUSB_SETUP_REQ)Ep0Buffer)->bRequestType & 0x60 ) != 0x00 ) 
 {
 switch( SetupReq )                                             
 {
 case 0x01:                                                    
 Ep0Buffer[0] = 0x02;
 Ep0Buffer[1] = 0x0A;										
 len = 2;
 break;
 case 0x02:                                                    
 break;	
 case 0x03:                                                    
 break;				
 case 0x09:                                                    
 break;
 case 0x0A:                                                    
 break;	
 case 0x0B:                                                    
 break;
 default:
 len = 0xFF;  				                                    
 break;
 }		
 }
 else                                                              
 {
 switch(SetupReq)                                              
 {
 case 0x06:
 switch(((PUSB_SETUP_REQ)Ep0Buffer)->wValueH)
 {
 case 1:                                                   
 pDescr = DevDesc;                                     
 len = sizeof(DevDesc);
 break;
 case 2:                                                   
 pDescr = CfgDesc;                                     
 len = sizeof(CfgDesc);
 break;
 case 0x22:                                                
 pDescr = HIDRepDesc;                                  
 len = sizeof(HIDRepDesc);
 Ready = 1;                                            
 break;
 case 3:                                           
 switch( ((PUSB_SETUP_REQ)Ep0Buffer)->wValueL ) {
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
 len = 0xFF;                                
 break;
 }
 break;												
 default:
 len = 0xff;                                           
 break;
 }
 if ( SetupLen > len )
 {
 SetupLen = len;     
 }
 len = SetupLen >= 64 ? 64 : SetupLen; 
 memcpy(Ep0Buffer,pDescr,len);                             
 SetupLen -= len;
 pDescr += len;
 break;
 case 0x05:
 SetupLen = ((PUSB_SETUP_REQ)Ep0Buffer)->wValueL;                          
 break;
 case 0x08:
 Ep0Buffer[0] = UsbConfig;
 if ( SetupLen >= 1 )
 {
 len = 1;
 }
 break;
 case 0x09:
 UsbConfig = ((PUSB_SETUP_REQ)Ep0Buffer)->wValueL;
 USB_Enum_OK = 1;
 EX0 = 1;
 break;
 case 0x0A:
 break;
 case 0x01:                                       
 if ( ( ((PUSB_SETUP_REQ)Ep0Buffer)->bRequestType & 0x1F ) == 0x02 ) 
 {
 switch( ((PUSB_SETUP_REQ)Ep0Buffer)->wIndexL )
 {
 case 0x82:
 UEP2_CTRL = UEP2_CTRL & ~ ( 0x40 | 0x03 ) | 0x02;
 break;
 case 0x81:
 UEP1_CTRL = UEP1_CTRL & ~ ( 0x40 | 0x03 ) | 0x02;
 break;
 case 0x02:
 UEP2_CTRL = UEP2_CTRL & ~ ( 0x80 | 0x0C ) | 0x00;
 break;
 default:
 len = 0xFF;                                        
 break;
 }
 }
 else
 {
 len = 0xFF;                                            
 }
 break;
 case 0x03:                                          
 if( ( ((PUSB_SETUP_REQ)Ep0Buffer)->bRequestType & 0x1F ) == 0x00 )         
 {
 if( ( ( ( UINT16 )((PUSB_SETUP_REQ)Ep0Buffer)->wValueH << 8 ) | ((PUSB_SETUP_REQ)Ep0Buffer)->wValueL ) == 0x01 )
 {
 if( CfgDesc[ 7 ] & 0x20 )
 {
 
 }
 else
 {
 len = 0xFF;                                     
 }
 }
 else
 {
 len = 0xFF;                                         
 }
 }
 else if( ( ((PUSB_SETUP_REQ)Ep0Buffer)->bRequestType & 0x1F ) == 0x02 )     
 {
 if( ( ( ( UINT16 )((PUSB_SETUP_REQ)Ep0Buffer)->wValueH << 8 ) | ((PUSB_SETUP_REQ)Ep0Buffer)->wValueL ) == 0x00 )
 {
 switch( ( ( UINT16 )((PUSB_SETUP_REQ)Ep0Buffer)->wIndexH << 8 ) | ((PUSB_SETUP_REQ)Ep0Buffer)->wIndexL )
 {
 case 0x82:
 UEP2_CTRL = UEP2_CTRL & (~0x40) | 0x03; 
 break;
 case 0x02:
 UEP2_CTRL = UEP2_CTRL & (~0x80) | 0x0C; 
 break;
 case 0x81:
 UEP1_CTRL = UEP1_CTRL & (~0x40) | 0x03; 
 break;
 default:
 len = 0xFF;                                      
 break;
 }
 }
 else
 {
 len = 0xFF;                                          
 }
 }
 else
 {
 len = 0xFF;                                              
 } 
 break;
 case 0x00:
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
 len = 0xff;                                                   
 break;
 }
 }
 }
 else
 {
 len = 0xff;                                                           
 }
 if(len == 0xff)
 {
 SetupReq = 0xFF;
 UEP0_CTRL = 0x80 | 0x40 | 0x0C | 0x03; 
 }
 else if(len <= 64)                                          
 {
 UEP0_T_LEN = len;
 UEP0_CTRL = 0x80 | 0x40 | 0x00 | 0x00; 
 }
 else
 {
 UEP0_T_LEN = 0;   
 UEP0_CTRL = 0x80 | 0x40 | 0x00 | 0x00; 
 }
 break;
 case 0x20 | 0:                                                       
 switch(SetupReq)
 {
 case 0x06:
 len = SetupLen >= 64 ? 64 : SetupLen;      
 memcpy( Ep0Buffer, pDescr, len );                                    
 SetupLen -= len;
 pDescr += len;
 UEP0_T_LEN = len;
 UEP0_CTRL ^= 0x40;                                             
 break;
 case 0x05:
 USB_DEV_AD = USB_DEV_AD & 0x80 | SetupLen;
 UEP0_CTRL = 0x00 | 0x02;
 break;
 default:
 UEP0_T_LEN = 0;                                                       
 UEP0_CTRL = 0x00 | 0x02;
 break;
 }
 break;
 case 0x00 | 0:   
 len = USB_RX_LEN;
 if(SetupReq == 0x09)
 {
 if ( U_TOG_OK )                                                      
 {
 len = USB_RX_LEN;                                                  
 for ( i = 0; i < len; i ++ )
 {
 UserEp2Buf[i] = Ep0Buffer[i] ^ 0xFF;		 
 }							
 UEP0_T_LEN = 0;
 UEP0_CTRL = (UEP0_CTRL ^ 0x80) | 0x40 | 0x00 | 0x00; 
 }
 }
 UEP0_T_LEN = 0;  															 
 UEP0_CTRL = 0x00 | 0x00;									 
 break;
 default:
 break;
 }
 UIF_TRANSFER = 0;                                                           	 
 }
 if(UIF_BUS_RST)                                                                 	 
 {
 EX0 = 0;
 UEP0_CTRL = 0x00 | 0x02;
 UEP1_CTRL = 0x10 | 0x00;
 UEP2_CTRL = 0x10 | 0x00 | 0x02;
 USB_DEV_AD = 0x00;
 UIF_SUSPEND = 0;
 UIF_TRANSFER = 0;
 UIF_BUS_RST = 0;                                                             	 
 }
 if (UIF_SUSPEND)                                                                 	 
 {
 UIF_SUSPEND = 0;
 if ( USB_MIS_ST & 0x04 )                                             	 
 {
 
 printf( "zz" );                                                          	 
 
 while ( XBUS_AUX & 0x80 )
 {
 ;     
 }
 SAFE_MOD = 0x55;
 SAFE_MOD = 0xAA;
 WAKE_CTRL = 0x80 | 0x01;                                   	 
 PCON |= 0x02;                                                               	 
 SAFE_MOD = 0x55;
 SAFE_MOD = 0xAA;
 WAKE_CTRL = 0x00;
 }
 }
 else
 {                                                                             		 
 USB_INT_FG = 0xFF;                                                         	     
 
 }
 }






 
 UINT8 Absolute_Up_Pack( POINTER * pTP, UINT8 Point_Num )
 {
 UINT8 i;
 UINT8 dat[84] = {0};
 static UINT16 Contact_Cnt = 0x0000;	
 
 dat[0] = 0x01;
 for( i = 0; i < (1); i ++ ) 
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
 
 
#line 445 "DEVICE.C" /1
 
 
 
 
 
 
#line 451 "DEVICE.C" /0
 
 Contact_Cnt++;
 dat[81] = Contact_Cnt & 0X00FF;
 dat[82] = (Contact_Cnt >> 8) & 0x00ff;
 
 dat[83] = 1;
 EX0 = 0;
 Enp2BlukIn( dat, 64 );
 mDelaymS(2);
 Enp2BlukIn( &dat[64], 20 );
 EX0 = 1;
 
 return 1;
 }
 
 
 

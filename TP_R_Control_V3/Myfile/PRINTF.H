/******************** (C) COPYRIGHT ********************************************
* File Name          : PRINTF.H
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : ���ڴ�ӡ�������ͷ�ļ�
*******************************************************************************/

#ifndef	__PRINTF_H__
#define __PRINTF_H__


/*******************************************************************************/
/* ���崮�ڴ�ӡ������� */
#define DEF_ARM_MCU		           0x00				 			 				/* ARMģʽ��51ģʽѡ�񿪹� */ 	
#define DEF_DEBUG_PRINTF           0x01				 			 				/* �����ӡ���ӿ��� */ 	

/*******************************************************************************/
/* ͨ���궨��DEF_ARM_MCUѡ��ARMģʽ����51ģʽ */
#if	( DEF_ARM_MCU == 1 )

/*******************************************************************************/
/* ARMģʽ */
/* ͨ���궨��DEF_DEBUG_PRINTFѡ���ӡ���ǲ���ӡ */
#if	( DEF_DEBUG_PRINTF == 1 )
#define DEBUG_PRINTF( format, arg... )    printf( format, ##arg )		 			/* ���ڴ�ӡ��� */
#else
#define DEBUG_PRINTF( format, arg... )    do{ if( 0 )printf( format, ##arg ); }while( 0 );
#endif

#else

/*******************************************************************************/
/* 51ģʽ */
/* ͨ���궨��DEF_DEBUG_PRINTFѡ���ӡ���ǲ���ӡ */
#if	( DEF_DEBUG_PRINTF == 1 )
#define DEBUG_PRINTF( x ) 			    printf( x )
#else
#define DEBUG_PRINTF( x )
#endif


#endif

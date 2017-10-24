/******************** (C) COPYRIGHT ********************************************
* File Name          : PRINTF.H
* Author             : TECH2
* Version            : V1.00
* Date               : 2017/05/09
* Description        : 串口打印输出控制头文件
*******************************************************************************/

#ifndef	__PRINTF_H__
#define __PRINTF_H__


/*******************************************************************************/
/* 定义串口打印输出开关 */
#define DEF_ARM_MCU		           0x00				 			 				/* ARM模式与51模式选择开关 */ 	
#define DEF_DEBUG_PRINTF           0x01				 			 				/* 常归打印监视开关 */ 	

/*******************************************************************************/
/* 通过宏定义DEF_ARM_MCU选择ARM模式还是51模式 */
#if	( DEF_ARM_MCU == 1 )

/*******************************************************************************/
/* ARM模式 */
/* 通过宏定义DEF_DEBUG_PRINTF选择打印还是不打印 */
#if	( DEF_DEBUG_PRINTF == 1 )
#define DEBUG_PRINTF( format, arg... )    printf( format, ##arg )		 			/* 串口打印输出 */
#else
#define DEBUG_PRINTF( format, arg... )    do{ if( 0 )printf( format, ##arg ); }while( 0 );
#endif

#else

/*******************************************************************************/
/* 51模式 */
/* 通过宏定义DEF_DEBUG_PRINTF选择打印还是不打印 */
#if	( DEF_DEBUG_PRINTF == 1 )
#define DEBUG_PRINTF( x ) 			    printf( x )
#else
#define DEBUG_PRINTF( x )
#endif


#endif


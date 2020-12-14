/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  �嵭����Ԥϡ��ʲ���(1)���(4)�ξ������������˸¤ꡤ�ܥ��եȥ���
 *  �����ܥ��եȥ���������Ѥ�����Τ�ޤࡥ�ʲ�Ʊ���ˤ���ѡ�ʣ������
 *  �ѡ������ۡʰʲ������ѤȸƤ֡ˤ��뤳�Ȥ�̵���ǵ������롥
 *  (1) �ܥ��եȥ������򥽡��������ɤη������Ѥ�����ˤϡ��嵭������
 *      ��ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ��꤬�����Τޤޤη��ǥ���
 *      ����������˴ޤޤ�Ƥ��뤳�ȡ�
 *  (2) �ܥ��եȥ������򡤥饤�֥������ʤɡ�¾�Υ��եȥ�������ȯ�˻�
 *      �ѤǤ�����Ǻ����ۤ�����ˤϡ������ۤ�ȼ���ɥ�����ȡ�����
 *      �ԥޥ˥奢��ʤɡˤˡ��嵭�����ɽ�����������Ѿ�浪��Ӳ���
 *      ��̵�ݾڵ����Ǻܤ��뤳�ȡ�
 *  (3) �ܥ��եȥ������򡤵�����Ȥ߹���ʤɡ�¾�Υ��եȥ�������ȯ�˻�
 *      �ѤǤ��ʤ����Ǻ����ۤ�����ˤϡ����Τ����줫�ξ�����������
 *      �ȡ�
 *    (a) �����ۤ�ȼ���ɥ�����ȡ����Ѽԥޥ˥奢��ʤɡˤˡ��嵭����
 *        �ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ����Ǻܤ��뤳�ȡ�
 *    (b) �����ۤη��֤��̤�������ˡ�ˤ�äơ�TOPPERS�ץ��������Ȥ�
 *        ��𤹤뤳�ȡ�
 *  (4) �ܥ��եȥ����������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������뤤���ʤ�»
 *      ������⡤�嵭����Ԥ����TOPPERS�ץ��������Ȥ����դ��뤳�ȡ�
 *      �ޤ����ܥ��եȥ������Υ桼���ޤ��ϥ���ɥ桼������Τ����ʤ���
 *      ͳ�˴�Ť����ᤫ��⡤�嵭����Ԥ����TOPPERS�ץ��������Ȥ�
 *      ���դ��뤳�ȡ�
 * 
 *  �ܥ��եȥ������ϡ�̵�ݾڤ��󶡤���Ƥ����ΤǤ��롥�嵭����Ԥ�
 *  ���TOPPERS�ץ��������Ȥϡ��ܥ��եȥ������˴ؤ��ơ�����λ�����Ū
 *  ���Ф���Ŭ������ޤ�ơ������ʤ��ݾڤ�Ԥ�ʤ����ޤ����ܥ��եȥ���
 *  �������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������������ʤ�»���˴ؤ��Ƥ⡤��
 *  ����Ǥ�����ʤ���
 *
 *  $Id: sipffsdemo.h 2416 2020-12-10 08:06:20Z fukuen $
 */

/*
 *  SPIFFS �ǥ�ץ������Υإå��ե�����
 */

/*
 *  �������åȰ�¸�����
 */
#include "target_test.h"
#include "glcd_disp.h"
#include "i2c.h"

/*
 *  �ƥ�������ͥ���٤����
 */

#define MAIN_PRIORITY	5		/* �ᥤ�󥿥�����ͥ���� */
								/* HIGH_PRIORITY���⤯���뤳�� */

/*
 *  �������åȤ˰�¸�����ǽ���Τ�����������
 */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* ʸ�����Ϥ��륷�ꥢ��ݡ���ID */
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define	STACK_SIZE		8192		/* �������Υ����å������� */
#endif /* STACK_SIZE */

#if defined(MAIXAMIGO) || defined(MAIXCUBE)
#define SIPEED_ST7789_RST_PIN    37
#define SIPEED_ST7789_DCX_PIN    38
#define SIPEED_ST7789_SS_PIN     36
#define SIPEED_ST7789_SCLK_PIN   39
#define SIPEED_ST7789_MOSI_PIN   -1
#define SIPEED_ST7789_MISO_PIN   -1
#else
#define SIPEED_ST7789_RST_PIN    21
#define SIPEED_ST7789_DCX_PIN    20
#define SIPEED_ST7789_SS_PIN     22
#define SIPEED_ST7789_SCLK_PIN   19
#define SIPEED_ST7789_MOSI_PIN   18
#define SIPEED_ST7789_MISO_PIN   -1
#endif

// default peripheral
#define SIPEED_ST7789_RST_GPIONUM  6
#define SIPEED_ST7789_DCX_GPIONUM  7
#define SIPEED_ST7789_SS           3

#if defined(MAIXAMIGO)
#define SPI_SCK_PIN   11
#define SPI_MISO_PIN  6
#define SPI_MOSI_PIN  10
#define SPI_SS_PIN    26
#define LED_PIN       15 //green
#elif defined(MAIXCUBE)
#define SPI_SCK_PIN   27
#define SPI_MISO_PIN  26
#define SPI_MOSI_PIN  28
#define SPI_SS_PIN    29
#define LED_PIN       14 //blue
#else
#define SPI_SCK_PIN   30
#define SPI_MISO_PIN  31
#define SPI_MOSI_PIN  33
#define SPI_SS_PIN    32
#define LED_PIN       6  //3		/* D13 */
#endif

#define SPI_PORTID    SPI1_PORTID
#define INHNO_SPI     IRQ_VECTOR_SPI0	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_SPI     IRQ_VECTOR_SPI0	/* ������ֹ� */
#define INTPRI_SPI    -5		/* �����ͥ���� */
#define INTATR_SPI    0			/* �����°�� */

#define SIPEED_DMA_CH DMA_CHANNEL3
#define INHNO_DMATX   IRQ_VECTOR_DMA3	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_DMATX   IRQ_VECTOR_DMA3	/* ������ֹ� */
#define INTPRI_DMATX  -4		/* �����ͥ���� */
#define INTATR_DMATX  0			/* �����°�� */

#ifndef PORTID
#define PORTID        1				/* arduino D15/D14 */
#endif

#if PORTID == 1
#define I2C_PORTID    I2C1_PORTID
#define INHNO_I2CEV   IRQ_VECTOR_I2C0	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_I2CEV   IRQ_VECTOR_I2C0	/* ������ֹ� */
#define INTPRI_I2CEV  -5			/* �����ͥ���� */
#define INTATR_I2CEV  0				/* �����°�� */
#else
#define I2C_PORTID    I2C2_PORTID
#define INHNO_I2CEV   IRQ_VECTOR_I2C1	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_I2CEV   IRQ_VECTOR_I2C1	/* ������ֹ� */
#define INTPRI_I2CEV  -5			/* �����ͥ���� */
#define INTATR_I2CEV  0				/* �����°�� */
#endif

#ifndef TOPPERS_MACRO_ONLY

/*
 *  �ҡ����ΰ������
 */
extern intptr_t heap_param[2];

/*
 *  �ؿ��Υץ��ȥ��������
 */

extern void	main_task(intptr_t exinf);
extern void heap_init(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */
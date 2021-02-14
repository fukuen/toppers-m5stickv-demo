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
 *    (b) �����ۤη��֤��̤�������ˡ�ˤ�äơ�TOPPERS�ץ������Ȥ�
 *        ��𤹤뤳�ȡ�
 *  (4) �ܥ��եȥ����������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������뤤���ʤ�»
 *      ������⡤�嵭����Ԥ����TOPPERS�ץ������Ȥ����դ��뤳�ȡ�
 *      �ޤ����ܥ��եȥ������Υ桼���ޤ��ϥ���ɥ桼������Τ����ʤ���
 *      ͳ�˴�Ť����ᤫ��⡤�嵭����Ԥ����TOPPERS�ץ������Ȥ�
 *      ���դ��뤳�ȡ�
 * 
 *  �ܥ��եȥ������ϡ�̵�ݾڤ��󶡤���Ƥ����ΤǤ��롥�嵭����Ԥ�
 *  ���TOPPERS�ץ������Ȥϡ��ܥ��եȥ������˴ؤ��ơ�����λ�����Ū
 *  ���Ф���Ŭ������ޤ�ơ������ʤ��ݾڤ�Ԥ�ʤ����ޤ����ܥ��եȥ���
 *  �������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������������ʤ�»���˴ؤ��Ƥ⡤��
 *  ����Ǥ�����ʤ���
 *
 *  $Id: main.h 2416 2021-02-07 08:06:20Z fukuen $
 */

/*
 *	FMmelody�Υإå��ե�����
 */

/*
 *  �������åȰ�¸�����
 */
#include "target_test.h"
#include "i2s.h"

/*
 *  �ƥ�������ͥ���٤����
 */

#define MAIN_PRIORITY	5		/* �ᥤ�󥿥�����ͥ���� */
								/* HIGH_PRIORITY���⤯���뤳�� */

#define HIGH_PRIORITY	9		/* �¹Լ¹Ԥ���륿������ͥ���� */
#define MID_PRIORITY	10
#define LOW_PRIORITY	11

/*
 *  �������åȤ˰�¸�����ǽ���Τ�����������
 */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* ʸ�����Ϥ��륷�ꥢ��ݡ���ID */
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define	STACK_SIZE		8192		/* �������Υ����å������� */
#endif /* STACK_SIZE */

#ifndef I2SPORTID
#define I2SPORTID        1
#endif
#define I2S1_PORTID        1 //temp

#if I2SPORTID == 1
#define I2S_PORTID    I2S1_PORTID
#define INHNO_I2SEV   IRQ_VECTOR_I2S0	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_I2SEV   IRQ_VECTOR_I2S0	/* ������ֹ� */
#define INTPRI_I2SEV  -5			/* �����ͥ���� */
#define INTATR_I2SEV  0				/* �����°�� */
#else
#define I2S_PORTID    I2S2_PORTID
#define INHNO_I2SEV   IRQ_VECTOR_I2S1	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_I2SEV   IRQ_VECTOR_I2S1	/* ������ֹ� */
#define INTPRI_I2SEV  -5			/* �����ͥ���� */
#define INTATR_I2SEV  0				/* �����°�� */
#endif

#define I2S_DMA_CH    DMA_CHANNEL4
#define INHNO_DMATX4  IRQ_VECTOR_DMA4	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_DMATX4  IRQ_VECTOR_DMA4	/* ������ֹ� */
#define INTPRI_DMATX4 -4		/* �����ͥ���� */
#define INTATR_DMATX4 0			/* �����°�� */

#define I2S_DMA1_CH   DMA_CHANNEL5
#define INHNO_DMARX5  IRQ_VECTOR_DMA5	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_DMARX5  IRQ_VECTOR_DMA5	/* ������ֹ� */
#define INTPRI_DMARX5 -4		/* �����ͥ���� */
#define INTATR_DMARX5 0			/* �����°�� */

#ifndef PORTID
#define PORTID        1				/* arduino D15/D14 */
#endif

#if PORTID == 1
#define I2C_PORTID    PORTID
#define INHNO_I2CEV   IRQ_VECTOR_I2C0	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_I2CEV   IRQ_VECTOR_I2C0	/* ������ֹ� */
#define INTPRI_I2CEV  -5			/* �����ͥ���� */
#define INTATR_I2CEV  0				/* �����°�� */
#else
#define I2C_PORTID    PORTID
#define INHNO_I2CEV   IRQ_VECTOR_I2C1	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_I2CEV   IRQ_VECTOR_I2C1	/* ������ֹ� */
#define INTPRI_I2CEV  -5			/* �����ͥ���� */
#define INTATR_I2CEV  0				/* �����°�� */
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TOPPERS_MACRO_ONLY

/*
 *  �ҡ����ΰ������
 */
extern intptr_t heap_param[2];

/*
 *  �ؿ��Υץ�ȥ��������
 */
extern void	main_task(intptr_t exinf);
extern void heap_init(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

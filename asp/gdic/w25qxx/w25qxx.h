/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
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
 *  $Id: w25qxx.h 2416 2020-11-30 11:03:25Z fukuen $
 */

/*
 *  W25QXX ����ץ����Υإå��ե�����
 */

#ifndef _W25QXX_H_
#define _W25QXX_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "device.h"
#include "spi.h"

#define DATALENGTH                          8

#define w25qxx_FLASH_PAGE_SIZE              256
#define w25qxx_FLASH_SECTOR_SIZE            4096
#define w25qxx_FLASH_PAGE_NUM_PER_SECTOR    16
#define w25qxx_FLASH_CHIP_SIZE              (16777216 UL)

#define WRITE_ENABLE                        0x06
#define WRITE_DISABLE                       0x04
#define READ_REG1                           0x05
#define READ_REG2                           0x35
#define READ_REG3                           0x15
#define WRITE_REG1                          0x01
#define WRITE_REG2                          0x31
#define WRITE_REG3                          0x11
#define READ_DATA                           0x03
#define FAST_READ                           0x0B
#define FAST_READ_DUAL_OUTPUT               0x3B
#define FAST_READ_QUAL_OUTPUT               0x6B
#define FAST_READ_DUAL_IO                   0xBB
#define FAST_READ_QUAL_IO                   0xEB
#define DUAL_READ_RESET                     0xFFFF
#define QUAL_READ_RESET                     0xFF
#define PAGE_PROGRAM                        0x02
#define QUAD_PAGE_PROGRAM                   0x32
#define SECTOR_ERASE                        0x20
#define BLOCK_32K_ERASE                     0x52
#define BLOCK_64K_ERASE                     0xD8
#define CHIP_ERASE                          0x60
#define W25QXX_READ_ID                      0x90
#define ENABLE_QPI                          0x38
#define EXIT_QPI                            0xFF
#define ENABLE_RESET                        0x66
#define RESET_DEVICE                        0x99

#define REG1_BUSY_MASK                      0x01
#define REG2_QUAL_MASK                      0x02

#define LETOBE(x)     ((x >> 24) | ((x & 0x00FF0000) >> 8) | ((x & 0x0000FF00) << 8) | (x << 24))

//#define SPI_PORTID    SPI4_PORTID
#define INHNO_SPI4    IRQ_VECTOR_SPI3		/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_SPI4    IRQ_VECTOR_SPI3		/* ������ֹ� */
#define INTPRI_SPI4   -5					/* �����ͥ���� */
#define INTATR_SPI4   0						/* �����°�� */

#define SPI4_DMA0_CH  DMA_CHANNEL0
#define INHNO_SPI4_DMATX   IRQ_VECTOR_DMA0	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_SPI4_DMATX   IRQ_VECTOR_DMA0	/* ������ֹ� */
#define INTPRI_SPI4_DMATX  -4				/* �����ͥ���� */
#define INTATR_SPI4_DMATX  0				/* �����°�� */

#define SPI4_DMA1_CH  DMA_CHANNEL1
#define INHNO_SPI4_DMARX   IRQ_VECTOR_DMA1	/* ����ߥϥ�ɥ��ֹ� */
#define INTNO_SPI4_DMARX   IRQ_VECTOR_DMA1	/* ������ֹ� */
#define INTPRI_SPI4_DMARX  -4				/* �����ͥ���� */
#define INTATR_SPI4_DMARX  0				/* �����°�� */


#ifndef TOPPERS_MACRO_ONLY

/**
 * w25qxx read/write operating mode
 */
typedef enum _w25qxx_read
{
	W25QXX_STANDARD = 0,
	W25QXX_STANDARD_FAST,
//	W25QXX_DUAL,
//	W25QXX_DUAL_FAST,
//	W25QXX_QUAD,
//	W25QXX_QUAD_FAST,
} w25qxx_read_t;

typedef struct
{
	SPI_Handle_t            *hspi;
	uint8_t                 spi_index;
	uint8_t                 spi_ss;
} w25qxx_Handler_t;

extern ER w25qxx_init(w25qxx_Handler_t *hw25);
extern ER w25qxx_is_busy(w25qxx_Handler_t *hw25);
extern ER w25qxx_chip_erase(w25qxx_Handler_t *hw25);
extern ER w25qxx_enable_quad_mode(w25qxx_Handler_t *hw25);
extern ER w25qxx_disable_quad_mode(w25qxx_Handler_t *hw25);
extern ER w25qxx_sector_erase(w25qxx_Handler_t *hw25, uint32_t addr);
extern ER w25qxx_32k_block_erase(w25qxx_Handler_t *hw25, uint32_t addr);
extern ER w25qxx_64k_block_erase(w25qxx_Handler_t *hw25, uint32_t addr);
extern ER w25qxx_read_status_reg1(w25qxx_Handler_t *hw25, uint8_t *reg_data);
extern ER w25qxx_read_status_reg2(w25qxx_Handler_t *hw25, uint8_t *reg_data);
extern ER w25qxx_write_status_reg(w25qxx_Handler_t *hw25, uint8_t reg1_data, uint8_t reg2_data);
extern ER w25qxx_read_id(w25qxx_Handler_t *hw25, uint8_t *manuf_id, uint8_t *device_id);
extern ER w25qxx_write_data(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf, uint32_t length);
extern ER w25qxx_read_data(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf, uint32_t length, w25qxx_read_t mode);


#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _W25QXX_H_ */


/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
 *
 *  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: w25qxx.c 2416 2020-11-30 18:45:11Z fuuen $
 */

/* 
 *  W25QXX 制御プログラムの本体
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdio.h>
#include <string.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "device.h"
#include "spi.h"
#include "w25qxx.h"
#include "sysctl.h"
#include <stdlib.h>

#define SPI_CORE_WAIT_TIME 500

static ER
w25qxx_receive_data(w25qxx_Handler_t *hw25, uint8_t *cmd_buff, uint8_t cmd_len, uint8_t *rx_buff, uint32_t rx_len)
{
	size_t i;
	uint32_t *write_cmd;
	uint32_t *read_buf;
	size_t v_recv_len;
	size_t v_cmd_len;

	write_cmd = (uint32_t *)malloc((cmd_len + rx_len) * sizeof(uint32_t));
	if (write_cmd == NULL)
	{
		syslog(LOG_ERROR, "malloc error");
		return E_NORES;
	}
	for (i = 0; i < cmd_len; i++)
		write_cmd[i] = cmd_buff[i];
	read_buf = &write_cmd[i];
	v_recv_len = rx_len;
	v_cmd_len = cmd_len;

	ER ercd = spi_eerom_transrecv(hw25->hspi, hw25->spi_ss, (uint8_t *)write_cmd, v_cmd_len, (uint8_t *)read_buf, v_recv_len);

	for (i = 0; i < v_recv_len; i++)
		rx_buff[i] = read_buf[i];

	free((void *)write_cmd);
#if SPI_WAIT_TIME == 0
	if (ercd == E_OK)
		ercd = spi_wait(hw25->hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

static ER
w25qxx_send_data(w25qxx_Handler_t *hw25, uint8_t *cmd_buff, uint8_t cmd_len, uint8_t *tx_buff, uint32_t tx_len)
{
	size_t i;
	uint32_t *buf;
	size_t v_send_len;

	buf = malloc((cmd_len + tx_len) * sizeof(uint32_t));
	if (buf == NULL)
	{
		syslog(LOG_ERROR, "malloc error");
		return E_NORES;
	}
	for (i = 0; i < cmd_len; i++)
		buf[i] = cmd_buff[i];
	for (i = 0; i < tx_len; i++)
		buf[cmd_len + i] = tx_buff[i];
	v_send_len = cmd_len + tx_len;

	ER ercd = spi_core_transmit(hw25->hspi, hw25->spi_ss, (uint8_t *)buf, v_send_len);

	free((void *)buf);
#if SPI_WAIT_TIME == 0
	if (ercd == E_OK)
		ercd = spi_wait(hw25->hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

ER
w25qxx_init(w25qxx_Handler_t *hw25)
{
	SPI_Init_t Init;
	SPI_Handle_t    *hspi;

	Init.WorkMode     = SPI_WORK_MODE_0;
	Init.FrameFormat  = SPI_FF_STANDARD;
	Init.DataSize     = 8;
	Init.Prescaler    = 25000000;
	Init.SignBit      = 0;
	Init.InstLength   = 0;
	Init.AddrLength   = 0;
	Init.WaitCycles   = 0;
	Init.IATransMode  = SPI_AITM_AS_FRAME_FORMAT;
	Init.SclkPin      = -1;
	Init.MosiPin      = -1;
	Init.MisoPin      = -1;
	Init.SsPin        = -1;
	Init.SsNo         = hw25->spi_ss;
	Init.TxDMAChannel = SPI4_DMA0_CH;
	Init.RxDMAChannel = SPI4_DMA1_CH;
	Init.semid        = SPI4TRN_SEM;
	Init.semlock      = SPI4LOCK_SEM;
	Init.semdmaid     = SPI4DMATX_SEM;
	hspi = spi_init(SPI4_PORTID, &Init);
	if (hspi == NULL)
	{
		syslog_0(LOG_ERROR, "SPI INIT ERROR");
		return E_SYS;
	}
	hw25->hspi = hspi;
//	(*hw25).hspi = hspi;

	return E_OK;
}

ER
w25qxx_read_id(w25qxx_Handler_t *hw25, uint8_t *manuf_id, uint8_t *device_id)
{
	uint8_t cmd[4] = {W25QXX_READ_ID, 0x00, 0x00, 0x00};
	uint8_t data[8] = {0};

	ER ercd = w25qxx_receive_data(hw25, cmd, 4, data, 2);
	if (ercd != E_OK)
		return ercd;
	*manuf_id = data[0];
	*device_id = data[1];
	return E_OK;
}

ER
w25qxx_write_enable(w25qxx_Handler_t *hw25)
{
	uint8_t cmd[1] = {WRITE_ENABLE};

	w25qxx_send_data(hw25, cmd, 1, 0, 0);
	return E_OK;
}

ER
w25qxx_write_status_reg(w25qxx_Handler_t *hw25, uint8_t reg1_data, uint8_t reg2_data)
{
	uint8_t cmd[3] = {WRITE_REG1, reg1_data, reg2_data};

	w25qxx_write_enable(hw25);
	w25qxx_send_data(hw25, cmd, 3, 0, 0);
	return E_OK;
}

ER
w25qxx_read_status_reg1(w25qxx_Handler_t *hw25, uint8_t *reg_data)
{
	uint8_t cmd[1] = {READ_REG1};
	uint8_t data[1] = {0};

	w25qxx_receive_data(hw25, cmd, 1, data, 1);
	*reg_data = data[0];
	return E_OK;
}

ER
w25qxx_read_status_reg2(w25qxx_Handler_t *hw25, uint8_t *reg_data)
{
	uint8_t cmd[1] = {READ_REG2};
	uint8_t data[1] = {0};

	w25qxx_receive_data(hw25, cmd, 1, data, 1);
	*reg_data = data[0];
	return E_OK;
}

ER
w25qxx_is_busy(w25qxx_Handler_t *hw25)
{
	uint8_t status = 0;

	w25qxx_read_status_reg1(hw25, &status);
	if (status & REG1_BUSY_MASK)
		return E_NORES;
	return E_OK;
}

ER
w25qxx_sector_erase(w25qxx_Handler_t *hw25, uint32_t addr)
{
	uint8_t cmd[4] = {SECTOR_ERASE};

	cmd[1] = (uint8_t)(addr >> 16);
	cmd[2] = (uint8_t)(addr >> 8);
	cmd[3] = (uint8_t)(addr);
	w25qxx_write_enable(hw25);
	w25qxx_send_data(hw25, cmd, 4, 0, 0);
	return E_OK;
}

ER
w25qxx_32k_block_erase(w25qxx_Handler_t *hw25, uint32_t addr)
{
	uint8_t cmd[4] = {BLOCK_32K_ERASE};

	cmd[1] = (uint8_t)(addr >> 16);
	cmd[2] = (uint8_t)(addr >> 8);
	cmd[3] = (uint8_t)(addr);
	w25qxx_write_enable(hw25);
	w25qxx_send_data(hw25, cmd, 4, 0, 0);
	return E_OK;
}

ER
w25qxx_64k_block_erase(w25qxx_Handler_t *hw25, uint32_t addr)
{
	uint8_t cmd[4] = {BLOCK_64K_ERASE};

	cmd[1] = (uint8_t)(addr >> 16);
	cmd[2] = (uint8_t)(addr >> 8);
	cmd[3] = (uint8_t)(addr);
	w25qxx_write_enable(hw25);
	w25qxx_send_data(hw25, cmd, 4, 0, 0);
	return E_OK;
}

ER
w25qxx_chip_erase(w25qxx_Handler_t *hw25)
{
	uint8_t cmd[1] = {CHIP_ERASE};

	w25qxx_write_enable(hw25);
	w25qxx_send_data(hw25, cmd, 1, 0, 0);
	return E_OK;
}

static ER
w25qxx_page_program(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf, uint32_t length)
{
	uint8_t cmd[4] = {PAGE_PROGRAM};

	cmd[1] = (uint8_t)(addr >> 16);
	cmd[2] = (uint8_t)(addr >> 8);
	cmd[3] = (uint8_t)(addr);
	w25qxx_write_enable(hw25);
	w25qxx_send_data(hw25, cmd, 4, data_buf, length);
	while (w25qxx_is_busy(hw25) == E_NORES)
		;
	return E_OK;
}

static ER
w25qxx_sector_program(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf)
{
	uint8_t index = 0;

	for (index = 0; index < w25qxx_FLASH_PAGE_NUM_PER_SECTOR; index++)
	{
		w25qxx_page_program(hw25, addr, data_buf, w25qxx_FLASH_PAGE_SIZE);
		addr += w25qxx_FLASH_PAGE_SIZE;
		data_buf += w25qxx_FLASH_PAGE_SIZE;
	}
	return E_OK;
}

ER
w25qxx_write_data(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf, uint32_t length)
{
	uint32_t sector_addr = 0;
	uint32_t sector_offset = 0;
	uint32_t sector_remain = 0;
	uint32_t write_len = 0;
	uint32_t index = 0;
	uint8_t *pread = NULL;
	uint8_t *pwrite = NULL;
	uint8_t swap_buf[w25qxx_FLASH_SECTOR_SIZE] = {0};

	while (length)
	{
		sector_addr = addr & (~(w25qxx_FLASH_SECTOR_SIZE - 1));
		sector_offset = addr & (w25qxx_FLASH_SECTOR_SIZE - 1);
		sector_remain = w25qxx_FLASH_SECTOR_SIZE - sector_offset;
		write_len = ((length < sector_remain) ? length : sector_remain);
		w25qxx_read_data(hw25, sector_addr, swap_buf, w25qxx_FLASH_SECTOR_SIZE, W25QXX_STANDARD);
		pread = swap_buf + sector_offset;
		pwrite = data_buf;
		for (index = 0; index < write_len; index++)
		{
			if ((*pwrite) != ((*pwrite) & (*pread)))
			{
				w25qxx_sector_erase(hw25, sector_addr);
				while (w25qxx_is_busy(hw25) == E_NORES)
					;
				break;
			}
			pwrite++;
			pread++;
		}
		if (write_len == w25qxx_FLASH_SECTOR_SIZE)
		{
			w25qxx_sector_program(hw25, sector_addr, data_buf);
		}
		else
		{
			pread = swap_buf + sector_offset;
			pwrite = data_buf;
			for (index = 0; index < write_len; index++)
				*pread++ = *pwrite++;
			w25qxx_sector_program(hw25, sector_addr, swap_buf);
		}
		length -= write_len;
		addr += write_len;
		data_buf += write_len;
	}
	return E_OK;
}

static ER
_w25qxx_read_data(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf, uint32_t length, w25qxx_read_t mode)
{
	uint32_t cmd[2] = {0};

	switch (mode) {
		case W25QXX_STANDARD:
			*(((uint8_t *)cmd) + 0) = READ_DATA;
			*(((uint8_t *)cmd) + 1) = (uint8_t)(addr >> 16);
			*(((uint8_t *)cmd) + 2) = (uint8_t)(addr >> 8);
			*(((uint8_t *)cmd) + 3) = (uint8_t)(addr >> 0);
			w25qxx_receive_data(hw25, (uint8_t *)cmd, 4, data_buf, length);
			break;
		case W25QXX_STANDARD_FAST:
			*(((uint8_t *)cmd) + 0) = FAST_READ;
			*(((uint8_t *)cmd) + 1) = (uint8_t)(addr >> 16);
			*(((uint8_t *)cmd) + 2) = (uint8_t)(addr >> 8);
			*(((uint8_t *)cmd) + 3) = (uint8_t)(addr >> 0);
			*(((uint8_t *)cmd) + 4) = 0xFF;
			w25qxx_receive_data(hw25, (uint8_t *)cmd, 5, data_buf, length);
			break;
		default:
			break;
	}
	return E_OK;
}

ER
w25qxx_read_data(w25qxx_Handler_t *hw25, uint32_t addr, uint8_t *data_buf, uint32_t length, w25qxx_read_t mode)
{
	uint32_t len = 0;

	while (length)
	{
		len = ((length >= 0x010000) ? 0x010000 : length);
		_w25qxx_read_data(hw25, addr, data_buf, len, mode);
		addr += len;
		data_buf += len;
		length -= len;
	}
	return E_OK;
}


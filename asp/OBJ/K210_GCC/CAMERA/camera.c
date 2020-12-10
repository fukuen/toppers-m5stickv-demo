/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: camera.c 2416 2020-12-10 08:06:20Z fukuen $
 */

/* 
 *  CAMERA �ǥ�ץ����
 */

#include <kernel.h>
#include <stdlib.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "device.h"
#include "pinmode.h"
#include "sipeed_st7789.h"
#include "sipeed_ov7740.h"
#include "kernel_cfg.h"
#include "camera.h"
#include "i2c.h"
#if defined(MAIXAMIGO) || defined(MAIXCUBE)
#include "maixamigo_axp173.h"
#else
#include "m5stickv_axp192.h"
#endif

/*
 *  �����ӥ�������Υ��顼�Υ�����
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

#ifndef SPI1DMATX_SEM
#define SPI1DMATX_SEM   0
#endif

static uint32_t heap_area[256*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*256*1024)
};

/*
 *  ���Ѥ����ѿ������
 */
LCD_Handler_t  LcdHandle;
LCD_DrawProp_t DrawProp;
DVP_Handle_t   DvpHandle;
OV7740_t       CameraHandle;

/*
 *  AXP192/AXP173 init
 */
#ifdef MAIXAMIGO
void init_axp173(void)
{
	ER ercd;
	I2C_Init_t i2c_initd;
	I2C_Handle_t *hi2c;
	AXP173_Handler_t  axp173Handle;

	i2c_initd.ClockSpeed      = 100000;
	i2c_initd.OwnAddress1     = 0;
	i2c_initd.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	i2c_initd.SclPin          = AXP173_SCL;
	i2c_initd.SdaPin          = AXP173_SDA;
	i2c_initd.semid           = I2CTRS_SEM;
	i2c_initd.semlock         = I2CLOC_SEM;
	syslog_2(LOG_NOTICE, "I2C SDAPIN(%d) SCLPIN(%d)", i2c_initd.SdaPin, i2c_initd.SclPin);

	if ((hi2c = i2c_init(I2C_PORTID, &i2c_initd)) == NULL) {
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## I2C ERROR(1) ##");
	}
	syslog_1(LOG_NOTICE, "AXP173 INITAILIZE(%d) !", I2C_PORTID);

	hi2c->writecallback = NULL;
	hi2c->readcallback  = NULL;
	hi2c->errorcallback = NULL;

	axp173Handle.hi2c  = hi2c;
	axp173Handle.saddr = AXP173_ADDR;
	if ((ercd = axp173_init(&axp173Handle)) != E_OK) {
		syslog_2(LOG_ERROR, "## AXP173 INIT ERROR(%d)[%08x] ##", ercd, hi2c->ErrorCode);
		slp_tsk();
	}
}
#else
void init_axp192(void)
{
	ER ercd;
	I2C_Init_t i2c_initd;
	I2C_Handle_t *hi2c;
	AXP192_Handler_t  axp192Handle;

	i2c_initd.ClockSpeed      = 100000;
	i2c_initd.OwnAddress1     = 0;
	i2c_initd.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	i2c_initd.SclPin          = AXP192_SCL;
	i2c_initd.SdaPin          = AXP192_SDA;
	i2c_initd.semid           = I2CTRS_SEM;
	i2c_initd.semlock         = I2CLOC_SEM;
	syslog_2(LOG_NOTICE, "I2C SDAPIN(%d) SCLPIN(%d)", i2c_initd.SdaPin, i2c_initd.SclPin);

	if ((hi2c = i2c_init(I2C_PORTID, &i2c_initd)) == NULL) {
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## I2C ERROR(1) ##");
	}
	syslog_1(LOG_NOTICE, "AXP192 INITAILIZE(%d) !", I2C_PORTID);

	hi2c->writecallback = NULL;
	hi2c->readcallback  = NULL;
	hi2c->errorcallback = NULL;

	axp192Handle.hi2c  = hi2c;
	axp192Handle.saddr = AXP192_ADDR;
	if ((ercd = axp192_init(&axp192Handle)) != E_OK) {
		syslog_2(LOG_ERROR, "## AXP192 INIT ERROR(%d)[%08x] ##", ercd, hi2c->ErrorCode);
		slp_tsk();
	}
}
#endif

/*
 *  �ᥤ�󥿥���
 */
void main_task(intptr_t exinf)
{
	SPI_Init_t Init;
	SPI_Handle_t    *hspi;
	LCD_Handler_t   *hlcd;
	OV7740_t        *hcmr;
	DVP_Handle_t    *hdvp;
	uint16_t        *lcd_buffer;
	ER_UINT	ercd;
	uint32_t count;
	unsigned long atmp;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Camera program starts (exinf = %d).", (int_t) exinf);

#if defined(MAIXAMIGO)
	syslog(LOG_NOTICE, "Complied with MaixAmigo.");
#elif defined(MAIXCUBE)
	syslog(LOG_NOTICE, "Compiled with MaixCube.");
#else
	syslog(LOG_NOTICE, "Compiled with M5StickV.");
#endif

	/*
	 *  ���ꥢ��ݡ��Ȥν����
	 *
	 *  �����ƥ����������Ʊ�����ꥢ��ݡ��Ȥ�Ȥ����ʤɡ����ꥢ��
	 *  �ݡ��Ȥ������ץ�Ѥߤξ��ˤϤ�����E_OBJ���顼�ˤʤ뤬���پ��
	 *  �ʤ���
	 */
	ercd = serial_opn_por(TASK_PORTID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORTID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));

#ifdef MAIXAMIGO
	init_axp173();
#else
	init_axp192();
#endif

    select_spi0_dvp_mode(1);

	hcmr = &CameraHandle;
	hcmr->frameSize = FRAMESIZE_QVGA;
	hcmr->pixFormat = PIXFORMAT_RGB565;
	ov7740_getResolition(hcmr, FRAMESIZE_QVGA);
	hcmr->_resetPoliraty  = ACTIVE_HIGH;
	hcmr->_pwdnPoliraty   = ACTIVE_HIGH;
	hcmr->_slaveAddr      = 0x00;
	hcmr->_dataBuffer     = NULL;
	hcmr->_aiBuffer       = NULL;

    // just support RGB565 and YUV442 on k210

    // Initialize the camera bus, 8bit reg
	hdvp = &DvpHandle;
	hcmr->hdvp = hdvp;
	hdvp->Init.Freq         = 22000000;
	hdvp->Init.Width        = hcmr->_width;
	hdvp->Init.Height       = hcmr->_height;
	hdvp->Init.Format       = DVP_FORMAT_YUY;
	hdvp->Init.BurstMode    = DVP_BURST_ENABLE;
	hdvp->Init.AutoMode     = DVP_AUTOMODE_DISABLE;
	hdvp->Init.GMMlen       = 4;

	hdvp->Init.num_sccb_reg = 8;
	hdvp->Init.CMosPClkPin  = 47;
	hdvp->Init.CMosXClkPin  = 46;
	hdvp->Init.CMosHRefPin  = 45;
	hdvp->Init.CMosPwDnPin  = 44;
	hdvp->Init.CMosVSyncPin = 43;
	hdvp->Init.CMosRstPin   = 42;
	hdvp->Init.SccbSClkPin  = 41;
	hdvp->Init.SccbSdaPin   = 40;
	hdvp->Init.IntNo        = INTNO_DVP;
	syslog_1(LOG_NOTICE, "## DvpHandle[%08x] ##", &DvpHandle);

	syslog_3(LOG_NOTICE, "## hcmr->_width(%d) hcmr->_height(%d) size[%08x] ##", hcmr->_width, hcmr->_height, (hcmr->_width * hcmr->_height * 2));
	hcmr->_dataBuffer = (uint32_t*)malloc(hcmr->_width * hcmr->_height * 2); //RGB565
    if(hcmr->_dataBuffer == NULL){
		hcmr->_width = 0;
		hcmr->_height = 0;
		syslog_0(LOG_ERROR, "Can't allocate _dataBuffer !");
		slp_tsk();
    }
	hcmr->_aiBuffer = (uint32_t*)malloc(hcmr->_width * hcmr->_height * 3 + 64 * 1024);   //RGB888
	if(hcmr->_aiBuffer == NULL){
		hcmr->_width = 0;
        hcmr->_height = 0;
		free(hcmr->_dataBuffer);
		hcmr->_dataBuffer = NULL;
		syslog_0(LOG_ERROR, "Can't allocate _aiBuffer !");
		slp_tsk();
	}
	syslog_2(LOG_NOTICE, "## hcmr->_dataBuffer[%08x] hcmr->_aiBuffer[%08x] ##", hcmr->_dataBuffer, hcmr->_aiBuffer);
	atmp = (unsigned long)hcmr->_aiBuffer;
	hdvp->Init.RedAddr    = (uint32_t)atmp;
	atmp = (unsigned long)(hcmr->_aiBuffer + hcmr->_width * hcmr->_height);
	hdvp->Init.GreenAddr  = (uint32_t)atmp;
	atmp = (unsigned long)(hcmr->_aiBuffer + hcmr->_width * hcmr->_height * 2);
	hdvp->Init.BlueAddr   = (uint32_t)atmp;
	atmp = (unsigned long)hcmr->_dataBuffer;
	hdvp->Init.RGBAddr    = (uint32_t)atmp;
    dvp_init(hdvp);

	if(ov7740_sensor_ov_detect(hcmr) == E_OK){
		syslog_0(LOG_NOTICE, "find ov sensor !");
	}
	else if(ov7740_sensro_gc_detect(hcmr) == E_OK){
		syslog_0(LOG_NOTICE, "find gc3028 !");
	}
	if(ov7740_reset(hcmr) != E_OK){
		syslog_0(LOG_ERROR, "ov7740 reset error !");
		slp_tsk();
	}
    if(ov7740_set_pixformat(hcmr) != E_OK){
		syslog_0(LOG_ERROR, "set pixformat error !");
        slp_tsk();
	}
    if(ov7740_set_framesize(hcmr) != E_OK){
		syslog_0(LOG_ERROR, "set frame size error !");
        slp_tsk();
	}
#ifdef MAIXAMIGO
	ov7740_choice(hcmr, 1);
#endif
	ov7740_setInvert(hcmr, true);
	syslog_1(LOG_NOTICE, "OV7740 id(%d)", ov7740_id(hcmr));

	Init.WorkMode     = SPI_WORK_MODE_0;
#if defined(MAIXAMIGO) || defined(MAIXCUBE)
	Init.FrameFormat  = SPI_FF_OCTAL;
//	Init.FrameFormat  = SPI_FF_STANDARD;
#else
	Init.FrameFormat  = SPI_FF_STANDARD;
#endif
	Init.DataSize     = 8;
#if defined(MAIXAMIGO) || defined(MAIXCUBE)
	Init.Prescaler    = 15000000;
#else
	Init.Prescaler    = 40000000;
#endif
	Init.SignBit      = 0;
	Init.InstLength   = 8;
	Init.AddrLength   = 0;
	Init.WaitCycles   = 0;
	Init.IATransMode  = SPI_AITM_AS_FRAME_FORMAT;
	Init.SclkPin      = SIPEED_ST7789_SCLK_PIN;
	Init.MosiPin      = SIPEED_ST7789_MOSI_PIN;
	Init.MisoPin      = SIPEED_ST7789_MISO_PIN;
	Init.SsPin        = SIPEED_ST7789_SS_PIN;
	Init.SsNo         = SIPEED_ST7789_SS;
	Init.TxDMAChannel = SIPEED_DMA_CH;
	Init.RxDMAChannel = -1;
	Init.semid        = SPI1TRN_SEM;
	Init.semlock      = SPI1LOCK_SEM;
	Init.semdmaid     = SPI1DMATX_SEM;
	hspi = spi_init(SPI_PORTID, &Init);
	if(hspi == NULL){
		syslog_0(LOG_ERROR, "SPI INIT ERROR");
		slp_tsk();
	}

	hlcd = &LcdHandle;
	hlcd->hspi    = hspi;
//	hlcd->spi_lock= SPI1LOCK_SEM;
	hlcd->dir     = DIR_YX_RLDU;
	hlcd->dcx_pin = SIPEED_ST7789_DCX_PIN;
	hlcd->rst_pin = SIPEED_ST7789_RST_PIN;
	hlcd->cs_sel  = SIPEED_ST7789_SS;
	hlcd->rst_no  = SIPEED_ST7789_RST_GPIONUM;
	hlcd->dcx_no  = SIPEED_ST7789_DCX_GPIONUM;
	DrawProp.hlcd = hlcd;
    lcd_init(hlcd);
	syslog_2(LOG_NOTICE, "width(%d) height(%d)", hlcd->_width, hlcd->_height);
	count = hcmr->_width * hcmr->_height;
	lcd_buffer = (uint16_t *)malloc(count * 2);
	if(lcd_buffer == NULL){
		syslog_0(LOG_ERROR, "no lcd buffer !");
		slp_tsk();
	}
	DrawProp.BackColor = ST7789_BLACK;
	DrawProp.TextColor = ST7789_WHITE;
	lcd_fillScreen(&DrawProp);

	if((ercd = ov7740_activate(hcmr, true)) != E_OK){
		syslog_2(LOG_NOTICE, "ov7740 activate error result(%d) id(%d) ##", ercd, ov7740_id(hcmr));
		slp_tsk();
	}

	for(;;){
		ercd = ov7740_snapshot(hcmr);
		if(ercd == E_OK){
			uint16_t *p = (uint16_t *)hcmr->_dataBuffer;
			uint32_t no;
			for (no = 0; no < count ; no += 2){
				lcd_buffer[no]   = SWAP_16(*(p + 1));
				lcd_buffer[no+1] = SWAP_16(*(p));
				p += 2;
			}
			lcd_drawPicture(hlcd, 0, 0, hcmr->_width, hcmr->_height, lcd_buffer);
		}
	}
	ov7740_activate(hcmr, false);

	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Camera program ends.");
	SVC_PERROR(ext_ker());
	assert(0);
}

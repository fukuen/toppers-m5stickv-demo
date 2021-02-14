/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2020-2021 by fukuen
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
 *  $Id: main.c 2416 2020-12-03 08:06:20Z fukuen $
 */

/* 
 *  FFT����ץ�ץ���������
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <sil.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include <target_syssvc.h>
#include <string.h>
#include <math.h>
#include "device.h"
#include "sipeed_st7789.h"
#include "i2s.h"
#include "fft.h"
#include "spi.h"
#include "sysctl.h"
#include "main.h"
#include "i2c.h"
#if defined(MAIXAMIGO) || defined(MAIXCUBE)
#include "maixamigo_axp173.h"
#else
#include "m5stickv_axp192.h"
#endif

#define SPK_SD_PIN    11
#define I2S_SCLK_PIN  15
#define I2S_WS_PIN    14
#define I2S_DA_PIN    17
#define MIC_SCLK_PIN  13
#define MIC_WS_PIN    10
#define MIC_DAT_PIN   12

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

#ifndef SPI1DMATX_SEM
#define SPI1DMATX_SEM   0
#endif

static uint32_t heap_area[2*512*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*2*512*1024)
};
/*
 *  ���Ѥ����ѿ������
 */
LCD_Handler_t  LcdHandle;
LCD_DrawProp_t DrawProp;
SPI_Init_t Init;
I2S_Init_t i2s_initd;
FFT_Init_t fft_initd;

#define FFT_N 512
#define FRAME_LENGTH 512
uint32_t i2s_rx_buf[FRAME_LENGTH * 2 * 2];
uint32_t g_index;
uint32_t i2s_rec_index;
uint8_t i2s_rec_flag;

uint16_t rx_buf[FRAME_LENGTH * 2];
int16_t s_tmp;


fft_data_t fft_in_data[FFT_N];
fft_data_t fft_out_data[FFT_N];
complex_hard_t data_hard[FFT_N] = {0};
float hard_power[FFT_N];

#define WIDTH 320
#define HEIGHT 240
uint16_t g_lcd_gram[WIDTH * HEIGHT] __attribute__((aligned(64)));

#define SAMPLE_RATE 16000

#define HIGH 0x1
#define LOW  0x0

#define INPUT           0x0
#define OUTPUT          0x3
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0X1

/*
 *  �����쥯�ȥǥ�����ԥ�����
*/
void
pinMode(uint8_t Pin, uint8_t dwMode){ 
	int gpionum = gpio_get_gpiohno(Pin, false);
	GPIO_Init_t init = {0};

	syslog_2(LOG_NOTICE, "## pinMode Pin(%d) gpionum(%d) ##", Pin, gpionum);
	if(gpionum >= 0){
		uint8_t function = FUNC_GPIOHS0 + gpionum;
		fpioa_set_function(Pin, function);
		switch(dwMode){
		case INPUT:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_NOPULL;
			break;
		case INPUT_PULLDOWN:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_PULLDOWN;
			break;
		case INPUT_PULLUP:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_PULLUP;
			break;
		case OUTPUT:
		default:
			init.mode = GPIO_MODE_OUTPUT;
			init.pull = GPIO_PULLDOWN;
			break;
		}
		gpio_setup(TADR_GPIOHS_BASE, &init, (uint8_t)gpionum);
	}
	return ;
}

/*
 *  �����쥯�ȥǥ�����ԥ����
 */
void
digitalWrite(uint8_t Pin, int dwVal){
	int8_t gpio_pin = gpio_get_gpiohno(Pin, false);

	if( gpio_pin >= 0){
		gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
	}
}

/*
 *  DMA������Хå�
 */
static void
readCallback(I2S_Handle_t *hi2s)
{
	if (g_index == 0) {
		g_index = FRAME_LENGTH * 2;
		i2s_receive_data(hi2s, &i2s_rx_buf[g_index], FRAME_LENGTH * 2);
		for(int i = 0; i < FRAME_LENGTH; i++) {
			rx_buf[i] = i2s_rx_buf[i * 2];
		}
	} else {
		g_index = 0;
		i2s_receive_data(hi2s, &i2s_rx_buf[g_index], FRAME_LENGTH * 2);
		for(int i = 0; i < FRAME_LENGTH; i++) {
			rx_buf[i] = i2s_rx_buf[i * 2 + FRAME_LENGTH * 2];
		}
	}
//	syslog_4(LOG_NOTICE, "%x %x %x %x ", i2s_rx_buf[0], i2s_rx_buf[2], i2s_rx_buf[4], i2s_rx_buf[6]);
	i2s_rec_flag = 1;
}

#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

void update_image_fft(float* hard_power, float pw_max, uint32_t* pImage, uint32_t color, uint32_t bkg_color)
{
	uint32_t bcolor= SWAP_16((bkg_color << 16)) | SWAP_16(bkg_color);
	uint32_t fcolor= SWAP_16((color << 16)) | SWAP_16(color);

	int  h[80];

	int i, x = 0;

	for (i = 0; i < 80; i++)
	{
		h[i]=120*(hard_power[i])/pw_max;

		if (h[i]>120)
			h[i] = 120;
		if (h[i]<0)
			h[i] = 0;
	}

	for (i = 0; i < 80; i++)  // 53* 38640/512 => ~4000Hz
	{
		x=i*2;
		for( int y=0; y<120; y++)
		{
			if( y<(120 - h[i+2]) )
			{
				pImage[x+(y+60)*160]=bcolor;
				pImage[x+1+(y+60)*160]=bcolor;
			}
			else
			{
				pImage[x+(y+60)*160]=fcolor;
				pImage[x+1+(y+60)*160]=fcolor;
			}
		}
	}
}

void FFT(FFT_Handle_t *hfft, int offset)
{
	for (int i = 0; i < FFT_N / 2; i++)
	{
		fft_in_data[i].I1 = 0;
		fft_in_data[i].R1 = rx_buf[2 * i + offset];
		fft_in_data[i].I2 = 0;
		fft_in_data[i].R2 = rx_buf[2 * i + 1 + offset];
	}
	fft_complex_uint16_dma(hfft, (uint64_t *)fft_in_data, (uint64_t *)fft_out_data);
	for (int i = 0; i < FFT_N / 2; i++)
	{
		data_hard[2 * i].imag = fft_out_data[i].I1;
		data_hard[2 * i].real = fft_out_data[i].R1;
		data_hard[2 * i + 1].imag = fft_out_data[i].I2;
		data_hard[2 * i + 1].real = fft_out_data[i].R2;
	}

	float pmax=10;
	for (int i = 0; i < FFT_N / 2; i++)
	{
		hard_power[i] = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);

		//Convert to dBFS
		hard_power[i] = 20*log(2*hard_power[i]/FFT_N);

		if( hard_power[i]>pmax)
			pmax = hard_power[i];
	}
}

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
	SPI_Handle_t  *hspi;
	LCD_Handler_t *hlcd;
	I2S_Handle_t  *hi2s_i;
	I2S_Handle_t  *hi2s_o;
	FFT_Handle_t  *hfft;
	ER_UINT	ercd;
	SYSTIM  tim;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Sample program starts (exinf = %d).", (int_t) exinf);

#ifdef MAIXAMIGO
	init_axp173();
#else
	init_axp192();
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

	syslog(LOG_NOTICE, "FFT DEMO START");

	select_spi0_dvp_mode(1);

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
		syslog_0(LOG_ERROR, "## SPI INIT ERROR ##");
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

	i2s_initd.RxTxMode = I2S_RECEIVER;
	i2s_initd.MclkPin  = -1;
	i2s_initd.SclkPin  = MIC_SCLK_PIN;
	i2s_initd.WsPin    = MIC_WS_PIN;
	i2s_initd.InD0Pin  = -1;
	i2s_initd.InD1Pin  = MIC_DAT_PIN;
	i2s_initd.InD2Pin  = -1;
	i2s_initd.InD3Pin  = -1;
	i2s_initd.OutD0Pin = -1;
	i2s_initd.OutD1Pin = -1;
	i2s_initd.OutD2Pin = -1;
	i2s_initd.OutD3Pin = -1;
	i2s_initd.RxChannelMask = 0x0C;
	i2s_initd.TxChannelMask = 0x00;
	i2s_initd.RxDMAChannel = I2S_DMA1_CH;
	i2s_initd.TxDMAChannel = -1;
	i2s_initd.semdmaid = I2SDMARX_SEM;
	i2s_initd.word_length = RESOLUTION_16_BIT;
	i2s_initd.word_select_size = SCLK_CYCLES_32;
	i2s_initd.trigger_level = TRIGGER_LEVEL_4;
	i2s_initd.word_mode = STANDARD_MODE;
	i2s_initd.sample_rate = SAMPLE_RATE;
	if((hi2s_i = i2s_init(I2S2_PORTID, &i2s_initd)) == NULL){
		syslog_0(LOG_ERROR, "## I2S INIT ERROR(2) ##");
		slp_tsk();
	}

	fft_initd.TxDMAChannel = FFT_DMATX_CH;
	fft_initd.RxDMAChannel = FFT_DMARX_CH;
	fft_initd.shift = FFT_FORWARD_SHIFT;
	fft_initd.direction = FFT_DIR_FORWARD;
	fft_initd.point_num = FFT_N;
	fft_initd.semdmaid = FFTDMARX_SEM;
	if((hfft = fft_init(&fft_initd)) == NULL){
		syslog_0(LOG_ERROR, "## FFT INIT ERROR ##");
		slp_tsk();
	}

	hi2s_o->hdmatx->xfercallback = NULL;
//	hi2s_i->hdmatx->xfercallback = NULL;
//	hi2s_i->readcallback = readCallback;
//	g_index = 0;
//	i2s_rec_index = 0;
//	i2s_rec_flag = 0;
//	i2s_receive_data(hi2s_i, i2s_rx_buf, FRAME_LENGTH * 2);

	// audio PA off
//	pinMode(SPK_DA, OUTPUT);
//	digitalWrite(SPK_DA, LOW);

	DrawProp.BackColor = ST7789_BLACK;
	DrawProp.TextColor = ST7789_RED;
	lcd_fillScreen(&DrawProp);
	dly_tsk(1000);

	while (1){
		i2s_receive_data(hi2s_i, i2s_rx_buf, FRAME_LENGTH * 2);
		for(int i = 0; i < FRAME_LENGTH; i++) {
			rx_buf[i] = i2s_rx_buf[i * 2] & 0xffff;
		}
		FFT(hfft, 0);
		update_image_fft(hard_power, 140 /*MAX range dBFS*/, (uint32_t *)g_lcd_gram, ST7789_BLUE, ST7789_BLACK);
		lcd_drawPicture(hlcd, 0, 0, WIDTH, HEIGHT, (uint16_t *)g_lcd_gram);
		i2s_rec_flag = 0;
		syslog_0(LOG_INFO, ".");
		sil_dly_nse(100);
	}

stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Sample program ends.");
//	SVC_PERROR(ext_ker());
}

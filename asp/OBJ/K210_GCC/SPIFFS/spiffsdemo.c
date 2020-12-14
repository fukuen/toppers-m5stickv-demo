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
 *  $Id: spiffsdemo.c 2416 2020-12-10 08:06:20Z fukuen $
 */

/* 
 *  SPIFFS �ǥ�ץ����
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
//#include "sipeed_ov7740.h"
#include "kernel_cfg.h"
//#include "camera.h"
#include "i2c.h"
#if defined(MAIXAMIGO) || defined(MAIXCUBE)
#include "maixamigo_axp173.h"
#else
#include "m5stickv_axp192.h"
#endif
#include "w25qxx.h"
#include <spiffs.h>
#include "spiffsdemo.h"
#include "tjpgd.h"

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

static uint32_t heap_area[2*256*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*2*256*1024)
};

/*
 *  ���Ѥ����ѿ������
 */
LCD_Handler_t  LcdHandle;
LCD_DrawProp_t DrawProp;


/*
 *  SPIFFS
 */
w25qxx_Handler_t hw25;
static spiffs fs;

int32_t spiffs_hal_write(uint32_t addr, uint32_t size, uint8_t *src)
{
	return (uint32_t)w25qxx_write_data(&hw25, addr, src, size);
}
int32_t spiffs_hal_erase(uint32_t addr, uint32_t size)
{
	return (uint32_t)w25qxx_sector_erase(&hw25, addr);
}
int32_t spiffs_hal_read(uint32_t addr, uint32_t size, uint8_t *dst)
{
	return (uint32_t)w25qxx_read_data(&hw25, addr, dst, size, W25QXX_STANDARD);
}

//#define LOG_PAGE_SIZE       SPIFFS_LOGICAL_PAGE_SIZE //256
#define LOG_PAGE_SIZE       0x1000

static u8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static u8_t spiffs_fds[512*4]; //32*4
static u8_t spiffs_cache_buf[(LOG_PAGE_SIZE+32)*4];

void my_spiffs_mount()
{
	spiffs_config cfg;
	cfg.phys_size = SPIFFS_SIZE; // use all spi flash
	cfg.phys_addr = SPIFFS_START_ADDR; // start spiffs at start of spi flash
	cfg.phys_erase_block = SPIFFS_EREASE_SIZE; // according to datasheet
	cfg.log_block_size = SPIFFS_LOGICAL_BLOCK_SIZE; // let us not complicate things
	cfg.log_page_size = SPIFFS_LOGICAL_PAGE_SIZE; // as we said
	
	cfg.hal_read_f = spiffs_hal_read;
	cfg.hal_write_f = spiffs_hal_write;
	cfg.hal_erase_f = spiffs_hal_erase;
	
	int res = SPIFFS_mount(&fs,
		&cfg,
		spiffs_work_buf,
		spiffs_fds,
		sizeof(spiffs_fds),
		spiffs_cache_buf,
		sizeof(spiffs_cache_buf),
		0);
	syslog(LOG_NOTICE, "mount res: %d\n", res);
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

typedef enum {
	JPEG_DIV_NONE,
	JPEG_DIV_2,
	JPEG_DIV_4,
	JPEG_DIV_8,
	JPEG_DIV_MAX
} jpeg_div_t;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t maxWidth;
	uint16_t maxHeight;
	uint16_t offX;
	uint16_t offY;
	jpeg_div_t scale;
	const void *src;
	size_t len;
	size_t index;
	LCD_Handler_t *lcd;
	uint16_t outWidth;
	uint16_t outHeight;
} jpg_file_decoder_t;

#define jpg_color(c)                                                            \
  (((uint16_t)(((uint8_t *)(c))[0] & 0xF8) << 8) |                             \
   ((uint16_t)(((uint8_t *)(c))[1] & 0xFC) << 3) |                             \
   ((((uint8_t *)(c))[2] & 0xF8) >> 3))

const char *jd_errors[] = {"Succeeded",
						   "Interrupted by output function",
						   "Device error or wrong termination of input stream",
						   "Insufficient memory pool for the image",
						   "Insufficient stream input buffer",
						   "Parameter error",
						   "Data format error",
						   "Right format but not supported",
						   "Not supported JPEG standard"};

static uint32_t jpg_read(JDEC *decoder, uint8_t *buf, uint32_t len)
{
	jpg_file_decoder_t *jpeg = (jpg_file_decoder_t *)decoder->device;
	if (buf) {
		memcpy(buf, (const uint8_t *)jpeg->src + jpeg->index, len);
	}
	jpeg->index += len;
	return len;
}

static uint32_t jpg_write(JDEC *decoder, void *bitmap, JRECT *rect)
{
	jpg_file_decoder_t *jpeg = (jpg_file_decoder_t *)decoder->device;
	uint16_t x = rect->left;
	uint16_t y = rect->top;
	uint16_t w = rect->right + 1 - x;
	uint16_t h = rect->bottom + 1 - y;
	uint16_t oL = 0, oR = 0;
	uint8_t *data = (uint8_t *)bitmap;

	if (rect->right < jpeg->offX) {
		return 1;
	}
	if (rect->left >= (jpeg->offX + jpeg->outWidth)) {
		return 1;
	}
	if (rect->bottom < jpeg->offY) {
		return 1;
	}
	if (rect->top >= (jpeg->offY + jpeg->outHeight)) {
		return 1;
	}
	if (rect->top < jpeg->offY) {
		uint16_t linesToSkip = jpeg->offY - rect->top;
		data += linesToSkip * w * 3;
		h -= linesToSkip;
		y += linesToSkip;
	}
	if (rect->bottom >= (jpeg->offY + jpeg->outHeight)) {
		uint16_t linesToSkip = (rect->bottom + 1) - (jpeg->offY + jpeg->outHeight);
		h -= linesToSkip;
	}
	if (rect->left < jpeg->offX) {
		oL = jpeg->offX - rect->left;
	}
	if (rect->right >= (jpeg->offX + jpeg->outWidth)) {
		oR = (rect->right + 1) - (jpeg->offX + jpeg->outWidth);
	}

	uint16_t pixBuf[32];
	uint8_t pixIndex = 0;
	uint16_t line;

	lcd_setAddrWindow(jpeg->lcd, x - jpeg->offX + jpeg->x + oL,
					   y - jpeg->offY + jpeg->y,
					   x - jpeg->offX + jpeg->x + oL + w - (oL + oR) - 1,
					   y - jpeg->offY + jpeg->y + h - 1);

	while (h--) {
		data += 3 * oL;
		line = w - (oL + oR);
		while (line--) {
			pixBuf[pixIndex++] = jpg_color(data);
			data += 3;
			if (pixIndex == 32) {
				lcd_writehalf(jpeg->lcd, pixBuf, 32);
				pixIndex = 0;
			}
		}
		data += 3 * oR;
	}
	if (pixIndex) {
		lcd_writehalf(jpeg->lcd, pixBuf, pixIndex);
	}
	return 1;
}

static ER jpg_decode(jpg_file_decoder_t *jpeg,
					  uint32_t (*reader)(JDEC *, uint8_t *, uint32_t)) {
	static uint8_t work[3100];
	JDEC decoder;

	JRESULT jres = jd_prepare(&decoder, reader, work, 3100, jpeg);
	if (jres != JDR_OK) {
		syslog(LOG_ERROR, "jd_prepare failed! %s", jd_errors[jres]);
		return E_OBJ;
	}

	uint16_t jpgWidth = decoder.width / (1 << (uint8_t)(jpeg->scale));
	uint16_t jpgHeight = decoder.height / (1 << (uint8_t)(jpeg->scale));

	if (jpeg->offX >= jpgWidth || jpeg->offY >= jpgHeight) {
		syslog(LOG_ERROR, "Offset Outside of JPEG size");
		return E_OBJ;
	}

	size_t jpgMaxWidth = jpgWidth - jpeg->offX;
	size_t jpgMaxHeight = jpgHeight - jpeg->offY;

	jpeg->outWidth =
		(jpgMaxWidth > jpeg->maxWidth) ? jpeg->maxWidth : jpgMaxWidth;
	jpeg->outHeight =
		(jpgMaxHeight > jpeg->maxHeight) ? jpeg->maxHeight : jpgMaxHeight;

	jres = jd_decomp(&decoder, jpg_write, (uint8_t)jpeg->scale);
	if (jres != JDR_OK) {
		syslog(LOG_ERROR, "jd_decomp failed! %s", jd_errors[jres]);
		return E_OBJ;
	}

	return E_OK;
}

/*
 *  �ᥤ�󥿥���
 */
void main_task(intptr_t exinf)
{
	SPI_Init_t Init;
	SPI_Handle_t    *hspi;
	LCD_Handler_t   *hlcd;
//	OV7740_t        *hcmr;
//	DVP_Handle_t    *hdvp;
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

	/*
	 *  AXP192/AXP173�ν����
	 */
#ifdef MAIXAMIGO
	init_axp173();
#else
	init_axp192();
#endif

	/*
	 *  LCD�ν����
	 */
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
	count = hlcd->_width * hlcd->_height;
	lcd_buffer = (uint16_t *)malloc(count * 2);
	if(lcd_buffer == NULL){
		syslog_0(LOG_ERROR, "no lcd buffer !");
		slp_tsk();
	}
	DrawProp.BackColor = ST7789_BLACK;
	DrawProp.TextColor = ST7789_WHITE;
	lcd_fillScreen(&DrawProp);


	/*
	 *  W25QXX����ν����
	 */
	hw25.spi_index = 3;
	hw25.spi_ss = 0;

	if((ercd = w25qxx_init(&hw25)) != E_OK){
		syslog_1(LOG_ERROR, "## W25QXX INIT ERROR(%d) ##", ercd);
		goto stop_task;
	}

	uint8_t manuf_id;
	uint8_t device_id;
	if((ercd = w25qxx_read_id(&hw25, &manuf_id, &device_id)) != E_OK){
		syslog_1(LOG_ERROR, "## W25QXX READ_ID ERROR(%d) ##", ercd);
		goto stop_task;
	}
	syslog_2(LOG_NOTICE, "## W25QXX MANUFACTURE(%d) DEVICE(%d) ##", manuf_id, device_id);


	/*
	 * SPIFFS�ե��������
	 */
	my_spiffs_mount();

	syslog(LOG_NOTICE, "## SPIFFS entries");
	spiffs_DIR dir;
	struct spiffs_dirent dirent;
	struct spiffs_dirent *pdirent = &dirent;
	struct spiffs_dirent e;
	struct spiffs_dirent *pe = &e;

	spiffs_DIR* result = SPIFFS_opendir(&fs, 0, &dir);
	if (!result) {
		printf("## SPIFFS OPEN ERROR(%d)\r\n", fs.err_code);
		goto stop_task;
	}

	while (SPIFFS_readdir(&dir, pdirent)) {
		printf("%s %d %d %d\n", dirent.name, dirent.size, dirent.obj_id, dirent.type);
	}

	SPIFFS_closedir(&dir);


	/*
	 * M5StickV jpeg����ɽ��
	 */
	uint8_t buf[40000];
	int buf_len;
	spiffs_stat stat;

	spiffs_file fd = SPIFFS_open(&fs, "/startup.jpg", SPIFFS_O_RDONLY, 0);
	if (fd < 0) {
		printf("## SPIFFS OPEN ERROR(%d)", SPIFFS_errno(&fs));
		goto stop_task;
	}

	int32_t ret = SPIFFS_fstat(&fs, fd, &stat);
	if (ret < 0) {
		printf("## SPIFFS STAT ERROR(%d)", SPIFFS_errno(&fs));
		goto stop_task;
	}
	printf("read size %d\n", stat.size);
	buf_len = stat.size;
	ret = SPIFFS_read(&fs, fd, buf, stat.size);
	if (ret < 0) {
		printf("## SPIFFS READ ERROR(%d)", SPIFFS_errno(&fs));
		goto stop_task;
	}

	ret = SPIFFS_close(&fs, fd);
	if (ret < 0) {
		printf("## SPIFFS CLOSE ERROR(%d)", SPIFFS_errno(&fs));
		goto stop_task;
	}

	jpg_file_decoder_t jpeg;

	jpeg.src = buf;
	jpeg.len = buf_len;
	jpeg.index = 0;
	jpeg.x = 40;
	jpeg.y = 60;
	jpeg.maxWidth = 240;
	jpeg.maxHeight = 135;
	jpeg.offX = 0;
	jpeg.offY = 0;
	jpeg.scale = 0;
	jpeg.lcd = hlcd;

	jpg_decode(&jpeg, jpg_read);


stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Camera program ends.");
	SVC_PERROR(ext_ker());
	assert(0);
}

/*
 *  TOPPERS BOOTSELECT app
 * 
 *  Copyright (C) 2020 by fukuen
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
 *  $Id: bootselect.c 2416 2020-12-08 08:06:20Z fukuen $
 */

/* 
 *  BOOTSELECTの本体
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
#include "device.h"
#include "pinmode.h"
#include "sipeed_st7789.h"
#ifdef MAIXAMIGO
#include "maixamigo_axp173.h"
#else
#include "m5stickv_axp192.h"
#endif
#include "w25qxx.h"
#include "bootselect.h"
#include "topamelogo.h"
#include <stdlib.h>
#include <stdio.h>

/*
 *  サービスコールのエラーのログ出力
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))


static uint32_t heap_area[256*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*256*1024)
};

LCD_Handler_t  LcdHandle;
LCD_DrawProp_t DrawProp;
LCD_Handler_t  *hlcd;

uint16_t *lcd_buffer;

#define WIDTH   280
#define HEIGHT  240
#define XOFFSET 40
#define YOFFSET 60

typedef struct {
	uint32_t ms;
	uint8_t pin;
	uint8_t pin_val;
	uint8_t invert;
	uint8_t state;
	uint8_t last_state;
	uint8_t changed;
	uint32_t time;
	uint32_t last_time;
	uint32_t last_change;
	uint32_t db_time;
	uint32_t press_time;
	uint8_t was_pressed;
} button_t;

#define BUTTON_A_PIN 36
#define BUTTON_B_PIN 37
#define DEBOUNCE_MS 50
button_t btn_a;
button_t btn_b;
#define BOOT_CONFIG_ADDR        0x00004000  // main boot config sector in flash at 52K
#define BOOT_CONFIG_ITEMS       8           // number of handled config entries

int pos_cursor = 0;
uint8_t buff[4096];

typedef struct {
	uint32_t entry_id;
	uint32_t app_start;
	uint32_t app_size;
	uint32_t app_crc;
	char app_name[16];
} app_entry_t;

app_entry_t app_entry[8];


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
 *  LCD初期化
 */
void init_lcd(void)
{
	SPI_Init_t Init;
	SPI_Handle_t    *hspi;
	uint32_t count;

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
	if (hspi == NULL) {
		syslog_0(LOG_ERROR, "SPI INIT ERROR");
		slp_tsk();
	}

	hlcd = &LcdHandle;
	hlcd->hspi    = hspi;
//	hlcd->spi_lock= SPI1LOCK_SEM;
	hlcd->dir     = DIR_YX_LRUD; //DIR_YX_RLDU;
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
	if (lcd_buffer == NULL) {
		syslog_0(LOG_ERROR, "no lcd buffer !");
		slp_tsk();
	}
}

void spc_dump(char *id, int rc, uint8_t *data, int len)
{
	int i;
	printf("[%s] = %d\n",id,rc);
	for (i=0;i<len;i++) {
		printf("%0x ",data[i]);
		if ( (i % 10) == 9) printf("\n");
	}
	printf("\n");
}

/*
 * Get uint32_t value from Flash
 * 8-bit Flash pionter is used,
 */
//-----------------------------------------
static uint32_t flash2uint32(uint32_t addr) {
	uint32_t val = buff[addr] << 24;
	val += buff[addr+1] << 16;
	val += buff[addr+2] << 8;
	val += buff[addr+3];
	return val;
}

/*
 *  Kbootメニューの存在チェック
 */
ER check_kboot(w25qxx_Handler_t *hw25)
{
	ER ercd = E_OK;

	hw25->spi_index = 3;
	hw25->spi_ss = 0;
	if ((ercd = w25qxx_init(hw25)) != E_OK) {
		syslog_1(LOG_ERROR, "## W25QXX INIT ERROR(%d) ##", ercd);
		return E_SYS;
	}

	uint8_t manuf_id;
	uint8_t device_id;
	if ((ercd = w25qxx_read_id(hw25, &manuf_id, &device_id)) != E_OK) {
		syslog_1(LOG_ERROR, "## W25QXX READ_ID ERROR(%d) ##", ercd);
		return E_SYS;
	}
	syslog_2(LOG_ERROR, "## W25QXX MANUFACTURE(%d) DEVICE(%d) ##", manuf_id, device_id);

	if ((ercd = w25qxx_read_data(hw25, 0x1000, buff, 0x20, W25QXX_STANDARD)) != E_OK) {
		syslog_1(LOG_ERROR, "## W25QXX READ_DATA ERROR(%d) ##", ercd);
		return E_OBJ;
	}

	if (buff[0] == 0x00 && buff[9] == 0x4b
		&& buff[10] == 0x62 && buff[11] == 0x6f && buff[12] == 0x6f
		&& buff[13] == 0x74 && buff[14] == 0x5f) {
		return E_OK;
	}

	return E_OBJ;
}

/*
 *  メニューエントリーの取得
 */
ER read_entry(w25qxx_Handler_t *hw25) {
	ER ercd = w25qxx_read_data(hw25, BOOT_CONFIG_ADDR, buff, BOOT_CONFIG_ITEMS * 0x20, W25QXX_STANDARD);
	if (ercd != E_OK)
		return ercd;
	for (int i = 0; i < BOOT_CONFIG_ITEMS; i++) {
		app_entry[i].entry_id = flash2uint32(i * 0x20);
		app_entry[i].app_start = flash2uint32(i * 0x20 + 4);
		app_entry[i].app_size = flash2uint32(i * 0x20 + 8);
		app_entry[i].app_crc = flash2uint32(i * 0x20 + 12);
		for (int j = 0; j < 16; j++) {
			app_entry[i].app_name[j] = buff[i * 0x20 + 16 + j];
		}
	}
	return E_OK;
}

/*
 *  フラグをフラッシュ書き込み
 */
ER write_active(w25qxx_Handler_t *hw25, int index, int active) {
	ER ercd = E_OK;
	ercd = w25qxx_read_data(hw25, BOOT_CONFIG_ADDR + index * 0x20 + 3, buff, 1, W25QXX_STANDARD);
	if (ercd != E_OK)
		return ercd;
	if (active == 0) {
		buff[0] = buff[0] & 0xfe;
	} else {
		buff[0] = buff[0] | 1;
	}
	ercd = w25qxx_write_data(hw25, BOOT_CONFIG_ADDR + index * 0x20 + 3, buff, 1);
	return ercd;
}

/*
 *  メニュー描画
 */
void draw_menu(void) {
	char num_string[3];
//	char name_string[17];
	DrawProp.BackColor = ST7789_BLACK;
	DrawProp.TextColor = ST7789_WHITE;
	lcd_fillScreen(&DrawProp);
	lcd_fillRect(hlcd, 0 + XOFFSET, YOFFSET, WIDTH, 8 + 4, ST7789_BLUE);
	DrawProp.TextColor = ST7789_WHITE;
	DrawProp.BackColor = ST7789_BLUE;
	DrawProp.pFont = &Font8;
	lcd_DisplayStringAt(&DrawProp, 1 + XOFFSET, 1 + YOFFSET, (uint8_t *)"BOOTSELECT for TOPPERS", LEFT_MODE);
	DrawProp.BackColor = ST7789_BLACK;
	for (int i = 0; i < BOOT_CONFIG_ITEMS; i++) {
		sprintf(num_string, "%d.", i + 1);
		DrawProp.TextColor = ST7789_YELLOW;
		lcd_DisplayStringAt(&DrawProp, XOFFSET, (i + 2) * 8 + YOFFSET, (uint8_t *)" ", LEFT_MODE);
		DrawProp.TextColor = ST7789_RED;
		if (app_entry[i].entry_id & 1) {
			lcd_DisplayStringAt(&DrawProp, 6 + XOFFSET, (i + 2) * 8 + YOFFSET, (uint8_t *)"*", LEFT_MODE);
		} else {
			lcd_DisplayStringAt(&DrawProp, 6 + XOFFSET, (i + 2) * 8 + YOFFSET, (uint8_t *)" ", LEFT_MODE);
		}
		DrawProp.TextColor = ST7789_WHITE;
		lcd_DisplayStringAt(&DrawProp, 12 + XOFFSET, (i + 2) * 8 + YOFFSET, (uint8_t *)num_string, LEFT_MODE);
//		memcpy(name_string, app_entry[i].app_name, 16);
		lcd_DisplayStringAt(&DrawProp, 22 + XOFFSET, (i + 2) * 8 + YOFFSET, (uint8_t *)app_entry[i].app_name, LEFT_MODE);
	}
	lcd_DisplayStringAt(&DrawProp, XOFFSET, 12 * 8 + YOFFSET, (uint8_t *)"BtnA: Toggle active", LEFT_MODE);
	lcd_DisplayStringAt(&DrawProp, XOFFSET, 13 * 8 + YOFFSET, (uint8_t *)"BtnB: Move cursor", LEFT_MODE);
	DrawProp.TextColor = ST7789_GREENYELLOW;
	lcd_DisplayStringAt(&DrawProp, XOFFSET, 14 * 8 + YOFFSET, (uint8_t *)"Copyright 2020 fukuen", LEFT_MODE);
}

/*
 *  カーソル描画
 */
void draw_cursor(void) {
	lcd_fillRect(hlcd, 0 + XOFFSET, 8 * 2 + YOFFSET, 6, 8 * 9, ST7789_BLACK);
	DrawProp.TextColor = ST7789_GREENYELLOW;
	lcd_DisplayStringAt(&DrawProp, 1 + XOFFSET, (pos_cursor + 2) * 8 + YOFFSET, (uint8_t *)">", LEFT_MODE);
}

/*
 *  フラグ・アクティブ/非アクティブ切り替え
 */
void toggle_active(w25qxx_Handler_t *hw25, int index) {
	lcd_fillRect(hlcd, 6 + XOFFSET, (index + 2) * 8 + YOFFSET, 6, 8, ST7789_BLACK);
	if ((app_entry[index].entry_id & 1) == 1) {
		app_entry[index].entry_id = app_entry[index].entry_id & 0xfffffffe;
		write_active(hw25, index, 0);
	} else {
		app_entry[index].entry_id = app_entry[index].entry_id | 1;
		write_active(hw25, index, 1);
		DrawProp.TextColor = ST7789_RED;
		lcd_DisplayStringAt(&DrawProp, 6 + XOFFSET, (index + 2) * 8 + YOFFSET, (uint8_t *)"*", LEFT_MODE);
	}
}

/*
 *  ミリ秒取得
 */
uint32_t millis(void)
{
	SYSTIM tim;
	get_tim(&tim);
	return (uint32_t)(tim);
}

/*
 *  ボタン初期化
 */
void init_button(button_t *button, uint8_t pin, uint8_t invert, uint8_t db_time)
{
	button->pin = pin;
	button->invert = invert;
	button->db_time = db_time;
	pinMode(pin, INPUT_PULLUP);
	button->state = digitalRead(pin);
	if (invert != 0) button->state = !button->state;
	button->time = millis();
	syslog(LOG_NOTICE, "time %x", button->time);
	button->last_state = button->state;
	button->changed = 0;
    button->last_time = button->time;
    button->last_change = button->time;
}

/*
 *  ボタン読み取り
 */
uint8_t read_button(button_t *button)
{
	static uint32_t ms;
	static uint8_t pin_val;

	ms = millis();
	pin_val = digitalRead(button->pin);
	if (button->invert != 0) pin_val = !pin_val;
	if (ms - button->last_change < button->db_time) {
		button->last_time = button->time;
		button->time = ms;
		button->changed = 0;
		return button->state;
	} else {
		button->last_time = button->time;
		button->time = ms;
		button->last_state = button->state;
		button->state = pin_val;
		if (button->state != button->last_state) {
			button->last_change = ms;
			button->changed = 1;
			if (button->state) { button->press_time = button->time; }
		} else {
			button->changed = 0;
		}
		button->was_pressed = (button->state && button->changed);
		return button->state;
	}
}


/*
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
//	SPI_Init_t Init;
//	SPI_Handle_t    *hspi;
//	LCD_Handler_t   *hlcd;
	w25qxx_Handler_t hw25;
	ER_UINT	ercd;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Bootselect program starts (exinf = %d).", (int_t) exinf);

#ifdef MAIXAMIGO
	init_axp173();
#else
	init_axp192();
#endif

	/*
	 *  シリアルポートの初期化
	 *
	 *  システムログタスクと同じシリアルポートを使う場合など，シリアル
	 *  ポートがオープン済みの場合にはここでE_OBJエラーになるが，支障は
	 *  ない．
	 */
	ercd = serial_opn_por(TASK_PORTID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORTID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));

	init_lcd();

	// splash screen
	DrawProp.BackColor = ST7789_WHITE;
	DrawProp.TextColor = ST7789_BLACK;
	lcd_fillScreen(&DrawProp);
	lcd_drawBitmap(hlcd, 100 + XOFFSET, 30 + YOFFSET, (uint8_t *)topamelogo);
	dly_tsk(1000);

	// check if kboot is installed
	ercd = check_kboot(&hw25);
	if (ercd != E_OK) {
		syslog(LOG_ERROR, "Kboot not installed");
		DrawProp.BackColor = ST7789_BLACK;
		DrawProp.TextColor = ST7789_YELLOW;
		DrawProp.pFont = &Font16;
		lcd_DisplayStringAt(&DrawProp, 50 + XOFFSET, 7 * 8 + YOFFSET, (uint8_t *)"Kboot not installed", LEFT_MODE);
		slp_tsk();
	}

	// initial screen
	read_entry(&hw25);
	draw_menu();
	draw_cursor();
	init_button(&btn_a, BUTTON_A_PIN, true, DEBOUNCE_MS);
	init_button(&btn_b, BUTTON_B_PIN, true, DEBOUNCE_MS);

	// loop
	while (1)
	{
		read_button(&btn_a);
		read_button(&btn_b);
		if (btn_a.was_pressed) {
			toggle_active(&hw25, pos_cursor);
		}
		if (btn_b.was_pressed) {
        	pos_cursor++;
			if (pos_cursor > BOOT_CONFIG_ITEMS - 1) {
				pos_cursor = 0;
			}
	      	draw_cursor();
		}
		dly_tsk(100);
    }

stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Bootselect program ends.");
//	SVC_PERROR(ext_ker());
}

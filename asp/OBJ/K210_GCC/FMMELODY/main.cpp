/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2021 by fukuen
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
 *  $Id: main.c 2416 2020-12-03 08:06:20Z fukuen $
 */

/***************************************/
/* FMmelody : FM synthesis version of  */
/* Arduino sample sketch "Melody"      */
/*                                     */
/* PWM analog wave output assigned for */
/* Digital 9 pin by default            */
/*                                     */
/* Serial 16-bit Non-oversampling      */
/* digital audio DAC supported         */
/*                                     */
/* 2009/6/26 created by PCM1723        */
/***************************************/

/* 
 *  FMmeldyプログラムの本体
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include <string.h>
#include "device.h"
#include "i2s.h"
#include "main.h"
#include "i2c.h"
#if defined(MAIXAMIGO) || defined(MAIXCUBE)
#include "maixamigo_axp173.h"
#else
#include "m5stickv_axp192.h"
#endif

#include <math.h>

#include "fmtg.h"
#include "notedefs.h"

#define TADR_GPIOHS_BASE  0x38001000UL
#define FUNC_GPIOHS0  24

#define SPK_SD_PIN    11
#define I2S_SCLK_PIN  15
#define I2S_WS_PIN    14
#define I2S_DA_PIN    17
#define MIC_SCLK_PIN  13
#define MIC_WS_PIN    10
#define MIC_DAT_PIN   12

#define SAMPLE_RATE 16000

I2S_Handle_t *hi2s;

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

static uint32_t heap_area[512*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*512*1024)
};

#define HIGH 0x1
#define LOW  0x0

#define INPUT           0x0
#define OUTPUT          0x3
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0X1

/*
*  ダイレクトデジタルピン設定
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
 *  ダイレクトデジタルピン出力
 */
void
digitalWrite(uint8_t Pin, int dwVal){
    int8_t gpio_pin = gpio_get_gpiohno(Pin, false);

    if( gpio_pin >= 0){
        gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
    }
}

int tick;
//int count = 0;

void tick_task(intptr_t exinf)
{
	tick = 1;
//	count++;
//	if (count % 1000 == 0) {
//		syslog_1(LOG_NOTICE, "count %d\n", count);
//	}
}

//
// sequence data
//
#define BPM (120)

int8_t seq_length  = 15; // the number of notes

int8_t notes[] = {M_C5, M_C5, M_G5, M_G5, 
                  M_A5, M_A5, M_G5, 
                  M_F5, M_F5, M_E5, M_E5, 
                  M_D5, M_D5, M_C5, 
                  M_REST};
                  
int8_t beats[] = {L_4,  L_4,  L_4,  L_4,
                  L_4,  L_4,  L_2, 
                  L_4,  L_4,  L_4,  L_4,
                  L_4,  L_4,  L_2,  
                  L_1};
//
// calculate unit duration for 96th note
//
int16_t unit_dur = (((4883/BPM)+1)>>1);                 
//
#ifdef USE_NOSDAC
  uint8_t led_pin =  8; // Idle LED
#else
  uint8_t led_pin = 13; // Idle LED
#endif
volatile int16_t dac_l, dac_r;  // DAC buffer
volatile uint8_t fsflag = 0;  // fs timing flag
FMop op[2]; // instantiate 2 FM operators
//
// instrument voice param array
//
#define NUM_INST (7)

//const PROGMEM prog_int8_t v_data[NUM_INST][6] = {
const int8_t v_data[NUM_INST][6] = {
//
// +---- OP0 ----+  OP1 oct
// FB MULT  TL  DR  DR  shift
  { 5,  1,  32,  1,  2,  0}, // Acoustic Piano
  { 7,  5,  44,  5,  2,  0}, // Electric Piano
  { 5,  9,  32,  2,  2,  0}, // Tubular Bells
  { 0,  8,  34,  8,  7,  0}, // Marimba
  { 7,  3,  32,  1,  2,  0}, // Jazz Guitar
  { 4,  1,  16,  1,  2, -2}, // Finger Bass
  { 4,  1,   8,  3,  2, -2}, // Slap Bass
}; // const PROGMEM prog_int8_t v_data[][]
//
// setup voice parameter for the program number
//
void program_change(uint8_t pno) 
{
  op[0].FB   = READ_BYTE(v_data[pno][0]);
  op[0].MULT = READ_BYTE(v_data[pno][1]);
  op[0].TL   = READ_BYTE(v_data[pno][2]);
  op[0].DR   = READ_BYTE(v_data[pno][3]);
  op[1].DR   = READ_BYTE(v_data[pno][4]);
} // void program_change()

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
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
	I2S_Init_t i2s_initd;
	ER_UINT	ercd;
	SYSTIM  tim;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Sample program starts (exinf = %d).", (int_t) exinf);

#ifdef MAIXAMIGO
	init_axp173();
#else
	init_axp192();
#endif

	// enable SPK
//	pinMode(SPK_SD, OUTPUT);
//	digitalWrite(SPK_SD, HIGH);

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

	syslog(LOG_NOTICE, "FMmelody test START(4)");

    select_spi0_dvp_mode(1);

	i2s_initd.RxTxMode = I2S_TRANSMITTER;
	i2s_initd.MclkPin  = -1;
	i2s_initd.SclkPin  = I2S_SCLK_PIN;
	i2s_initd.WsPin    = I2S_WS_PIN;
	i2s_initd.InD0Pin  = -1;
	i2s_initd.InD1Pin  = -1;
	i2s_initd.InD2Pin  = -1;
	i2s_initd.InD3Pin  = -1;
	i2s_initd.OutD0Pin = I2S_DA_PIN;
	i2s_initd.OutD1Pin = -1;
	i2s_initd.OutD2Pin = -1;
	i2s_initd.OutD3Pin = -1;
	i2s_initd.RxChannelMask = 0x00;
	i2s_initd.TxChannelMask = 0x03;
	i2s_initd.RxDMAChannel = -1;
	i2s_initd.TxDMAChannel = I2S_DMA_CH;
	i2s_initd.semdmaid = I2S0DMATX_SEM;
	i2s_initd.word_length = RESOLUTION_16_BIT;
	i2s_initd.word_select_size = SCLK_CYCLES_32;
	i2s_initd.trigger_level = TRIGGER_LEVEL_4;
	i2s_initd.word_mode = STANDARD_MODE;
	i2s_initd.sample_rate = SAMPLE_RATE;
	if((hi2s = i2s_init(I2S1_PORTID, &i2s_initd)) == NULL){
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## I2S ERROR(1) ##");
		slp_tsk();
	}

	program_change(0); // default instrument

	static int16_t ww  = 0; // wave work
	static uint8_t seq = 0; // sequencer index
	static uint8_t pno = 0; // program number
	static int8_t  nofs = 0; // note offset
	static int8_t  tick1ms = SEQTICK1ms-1; // 1ms tick counter
	static int16_t dur_cnt = 0; // duration counter
//
	while (1) {
		dac_l = ww;   // update DAC Lch
		fsflag = 0; // reset flag

		uint32_t temp[2];
		temp[0] = dac_l >> 8; // for silent
		temp[1] = 0;
		i2s_send_data(hi2s, &temp, 1);
		if (0 > (--tick1ms)) { // 1ms rate processing
			tick1ms = SEQTICK1ms-1;  // reset 1ms tick count
			op[0].eg_update(); // EG update for mod.
			op[1].eg_update(); // EG update for carr.
			if (0 > (--dur_cnt)){ // note duration over ?
				if (M_REST < notes[seq]) { // is it a note ? (skip if rest)
					if (0 == seq) { // voice load at top of sequence
						program_change(pno); // load new program
						nofs = 12 * READ_BYTE(v_data[pno][5]); // octave shift
						if (NUM_INST <= (++pno)) pno = 0; // rewind prog no. if last one
					} // if (0 == seq) ..
					for (uint8_t i = 0; i < 2; i++) { // GATE ON each OP for the note
						op[i].gate_on(notes[seq]+nofs, 127);
					} // for (i
				} // if (M_REST ...
				dur_cnt = beats[seq] * unit_dur - 1; // calculate unit_dur of this note
				if (seq_length <= (++seq)) seq = 0;  // rewind to top if the last note
			} // if (0 > (--dur_cnt)) ...
		} // if (0 > (--tick1ms)) ...
		// Operator calculation for series alghrithm
		ww = op[0].calc(0);  // modulator
		ww = op[1].calc((int32_t)ww << MOD_SHIFT); // carrier
	}

stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Sample program ends.");
//	SVC_PERROR(ext_ker());
}

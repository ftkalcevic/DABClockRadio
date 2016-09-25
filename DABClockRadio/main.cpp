/*
 * DABClockRadio.cpp
 *
 * Created: 24/09/2016 10:32:10 AM
 * Author : Frank Tkalcevic
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "time.h"
#include "ic2_7seg.h"

#define SLAVE_ADDR (0x70<<1)

SevenSeg< CSerialTxn< CSerialTxnImpl< CMegaI2CE, 64, 8> >, SLAVE_ADDR> sevenSeg_I2C;
IMPLEMENT_SERIAL_TXN_INTERRUPTS(E,sevenSeg_I2C)

//SevenSeg< CSerialTxn< CSerialTxnPolledImpl< CMegaI2CE > >, SLAVE_ADDR> sevenSeg_I2C;

#define INTERRUPTED

static void WaitForI2CIdle() { while ( (TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm) == 0 ); }
static void SendI2CStop() { TWIE.MASTER.STATUS = TWI_MASTER_CMD_STOP_gc; }

static void InitI2C()
{
#ifdef INTERRUPTED
	PR.PRPE &= ~PR_TWI_bm;
	sevenSeg_I2C.Init();
#else
	PR.PRPE &= ~PR_TWI_bm;

	//PORTE.PIN0CTRL =
	//PORTCFG.MPCMASK = 0x03; // Configure several PINxCTRL registers at the same time
	//PORTE.PIN0CTRL = (PORTE.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc; //Enable pull-up to get a defined level on the switches

	TWIE.CTRL = 0;
	TWIE.MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
	TWIE.MASTER.CTRLB = 0;
	TWIE.MASTER.CTRLC = 0;
	TWIE.MASTER.BAUD = 155; // 32MHz -> 400khz
//	TWIE.MASTER.BAUD = 5; // 1MHz -> 100khz
	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

	TWIE.MASTER.ADDR = SLAVE_ADDR;
	WaitForI2CIdle();
	TWIE.MASTER.DATA = 0x21;
	WaitForI2CIdle();
	SendI2CStop();

	//TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	//while ( (TWIE.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc );
	//	WaitForI2CIdle();
	TWIE.MASTER.ADDR = SLAVE_ADDR;
	WaitForI2CIdle();
	TWIE.MASTER.DATA = HT16K33_CMD_BRIGHTNESS | 4;
	WaitForI2CIdle();
	SendI2CStop();

	//TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	//while ( (TWIE.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc );
	//	WaitForI2CIdle();
	TWIE.MASTER.ADDR = SLAVE_ADDR;
	WaitForI2CIdle();
	TWIE.MASTER.DATA = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (0 << 1);
	WaitForI2CIdle();
	SendI2CStop();

	//TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	//while ( (TWIE.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) != TWI_MASTER_BUSSTATE_IDLE_gc );
	//	WaitForI2CIdle();
	TWIE.MASTER.ADDR = SLAVE_ADDR;
	WaitForI2CIdle();

	// decimal point 0x80
	TWIE.MASTER.DATA = 0;
	WaitForI2CIdle();
	for ( int i = 0; i < 10; i++ )
	{
		TWIE.MASTER.DATA = 0;
		WaitForI2CIdle();
	}
	SendI2CStop();
#endif
}



#ifdef INTERRUPTED
#else
static void DisplayUpdate( uint16_t *data, uint8_t len )
{
	TWIE.MASTER.ADDR = SLAVE_ADDR;
	WaitForI2CIdle();

	TWIE.MASTER.DATA = 0;
	WaitForI2CIdle();

	for ( int i = 0; i < len; i++ )
	{
		TWIE.MASTER.DATA = data[i] & 0xFF;
		WaitForI2CIdle();

		TWIE.MASTER.DATA = data[i] >> 8;
		WaitForI2CIdle();
	}
	SendI2CStop();
}
#endif // endif

static void Flash()
{

#ifdef INTERRUPTED
	static const uint8_t data[] = {SLAVE_ADDR, 4, 0 };
	sevenSeg_I2C.SendTxn(data, countof(data));
#endif
}


static void ShowTime( long nSeconds )
{
	struct tm t;

	gmtime_r((time_t*)&nSeconds, &t);

	//terminal.Send( "Tick ");
	//terminal.Send(t.tm_mday);
	//terminal.Send('/');
	//terminal.Send(t.tm_mon);
	//terminal.Send('/');
	//terminal.Send(t.tm_year+1900);
	//terminal.Send(' ');
	//
	//if (t.tm_hour < 10 )
	//terminal.Send(' ');
	//terminal.Send(t.tm_hour);
	//terminal.Send( ":");
	//if ( t.tm_min < 10 )
	//terminal.Send('0');
	//terminal.Send(t.tm_min);
	//terminal.Send( ":");
	//if ( t.tm_sec < 10 )
	//terminal.Send('0');
	//terminal.Send(t.tm_sec);
	//terminal.SendCRLF();

	static const uint8_t numbertable[] = {
		0x3F, /* 0 */
		0x06, /* 1 */
		0x5B, /* 2 */
		0x4F, /* 3 */
		0x66, /* 4 */
		0x6D, /* 5 */
		0x7D, /* 6 */
		0x07, /* 7 */
		0x7F, /* 8 */
		0x6F /* 9 */
	};

	bool pm = false;
	struct 
	{
		uint8_t addr;
		uint8_t row;
		uint16_t data[5];
	} data;
	data.addr = SLAVE_ADDR;
	data.row = 0;
	if ( t.tm_hour > 12 )
	{
		t.tm_hour -= 12;
		pm = true;
	}
	if ( t.tm_hour == 0 )
		t.tm_hour = 12;
	if ( t.tm_hour < 10 )
		data.data[0] = 0;
	else
	{
		data.data[0] = numbertable[1];
		t.tm_hour -= 10;
	}
	data.data[1] = numbertable[t.tm_hour];
	//data[2] = (nSeconds & 1) ? 0xFF : 0;
	data.data[2] = 0xFF;

	data.data[3] = numbertable[t.tm_min/10];
	data.data[4] = numbertable[t.tm_min%10];

	if ( pm )
		data.data[4] |= 0x80;
	
#ifdef INTERRUPTED
	sevenSeg_I2C.SendTxn( (uint8_t *)&data, sizeof(data) );
#else
	DisplayUpdate( data.data, 5 );
#endif

}




static volatile uint32_t ms;

ISR(TCC0_OVF_vect)
{
	ms++;
}

static void ioinit()
{
	// Turn of all peripherals
	PR.PRGEN = 0xFF;
	PR.PRPA = 0xFF;
	//PR.PRPB = 0xFF;
	PR.PRPC = 0xFF;
	PR.PRPD = 0xFF;
	PR.PRPE = 0xFF;
	PR.PRPF = 0xFF;

	// main clock to 32MHz
	OSC.CTRL = OSC_RC32MEN_bm;
	while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 )
		continue;

	_PROTECTED_WRITE(CLK_PSCTRL, 0);
	_PROTECTED_WRITE(CLK_CTRL, CLK_SCLKSEL_RC32M_gc);
	//CCP = CCP_IOREG_gc;
	//CLK.PSCTRL = 0;
	//CCP = CCP_IOREG_gc;
	//CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	// Enable dfll32m
	OSC.XOSCCTRL = OSC_XOSCSEL_32KHz_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while ( (OSC.STATUS & OSC_XOSCRDY_bm) == 0 )
		continue;
	OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;

	InitI2C();

	// TCC0 ms counter.
	PR.PRPC &= ~PR_TC0_bm;
	TCC0.CTRLA = TC_CLKSEL_DIV64_gc;
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCC0.CTRLC = 0;
	TCC0.CTRLD = 0;
	TCC0.CTRLE = 0;
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCC0.INTCTRLB = 0;
	TCC0.PER = 500;

	// Enable interrupt controller
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
}

int main(void)
{
	ioinit();
	sei();

#ifdef INTERRUPTED
	sevenSeg_I2C.Start();
#endif


	long n = 0;
    while (1) 
    {
		ShowTime(n);
		n+=60;
		_delay_ms(100);
    }
}



/*

70 21
70 E4
70 81
70 00 x 11
70 00 06 00 5B 00 FF 00 3F 00 3F 00

*/
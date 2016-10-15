/*
 * DABClockRadio.cpp
 *
 * Created: 24/09/2016 10:32:10 AM
 * Author : Frank Tkalcevic
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "..\images\font.h"

#include "time.h"
#include "ic2_7seg.h"
#include "serial.h"
#include "pictiva.h"
#include "monkeyboarddab.h"

//#include "..\..\images\consolas.h"


#define BTN_SNOOZE		0x0001
#define BTN_SLEEP		0x0002
#define BTN_RADIO_OFF	0x0004
#define BTN_RADIO_ON	0x0008
#define BTN_TIME_SET	0x0010
#define BTN_FWD_FAST	0x0020
#define BTN_FWD_SLOW	0x0040
#define BTN_ALARM1		0x0080
#define BTN_REV_FAST	0x0100
#define BTN_REV_SLOW	0x0200
#define BTN_ALARM2		0x0400
#define MAX_BUTTONS		11

#define SLAVE_ADDR (0x70<<1)

static SevenSeg< CSerialTxn< CSerialTxnImpl< CMegaI2CE, 64, 8> >, SLAVE_ADDR> sevenSeg_I2C;
IMPLEMENT_SERIAL_TXN_INTERRUPTS(E,sevenSeg_I2C)
static CMonkeyBoardDAB<CSerial< CSerialImpl<CMegaSerialE0,64,64> > > dab;
IMPLEMENT_SERIAL_INTERRUPTS( E0, dab )
static Pictiva display;

//SevenSeg< CSerialTxn< CSerialTxnPolledImpl< CMegaI2CE > >, SLAVE_ADDR> sevenSeg_I2C;

static CSerial< CSerialImpl<CMegaSerialC0,64,64>
#ifndef DEBUG
, false 
#endif
> terminal;
IMPLEMENT_SERIAL_INTERRUPTS( C0, terminal )

static uint32_t clock_secs = 0;
static bool bClockInitialised=false;
static uint32_t radioTimer;

static void ScrollText(bool bRedraw=false);



// Display mode - what is the display showing.
enum class DisplayMode: uint8_t
{
	Off,
	Playing,
	Tuner,
	Menu
} displayMode = DisplayMode::Off;


// Program Info
struct Programs
{
	const char *sShortName;
	const char *sLongName;
};
static Programs *programs;
static uint16_t nPrograms;
static uint16_t nProgramIdx;
static char *sProgramNames;
static uint16_t nLastPlayedProgram = 30; //4; //30;
static uint16_t nLastVolume = 0;	// What the volume is.
static uint8_t volumeSetting = 0;	// What to set it to.




static void InitI2C()
{
	PR.PRPE &= ~PR_TWI_bm;
	sevenSeg_I2C.Init();
}

static void ClearDisplay()
{
	sevenSeg_I2C.ClearDisplay();
}

static void FlashColon(bool bOn)
{
	// at the 1/2 second mark, turn off the colon (row 4)
	uint8_t data[] = {SLAVE_ADDR, 4, (uint8_t)(bOn?0xFF:0), 0 };
	sevenSeg_I2C.SendTxn(data, countof(data));
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
	
	sevenSeg_I2C.SendTxn( (uint8_t *)&data, sizeof(data) );
}

static void ShowDate()
{
	if ( displayMode == DisplayMode::Playing )
	{
		struct tm t;

		gmtime_r((time_t*)&clock_secs, &t);

	
		char s[16];	//Thu 20 Dec 2016
		memset( s, ' ', sizeof(s) );
		uint8_t n = 0;

		const char *WeekDay = "SunMonTueWedThuFriSat";
		s[n++] = WeekDay[t.tm_wday*3+0];
		s[n++] = WeekDay[t.tm_wday*3+1];
		s[n++] = WeekDay[t.tm_wday*3+2];
		n++;

		if ( t.tm_mday >= 10 )
			s[n++] = '0' + (t.tm_mday / 10 );
		s[n++] =  '0' + (t.tm_mday % 10 );
		n++;

		const char *Month = "JanFebMarAprMayJunJulAugSepOctNovDec";
		s[n++] = Month[t.tm_mon*3+0];
		s[n++] = Month[t.tm_mon*3+1];
		s[n++] = Month[t.tm_mon*3+2];
		n++;

		uint16_t y = t.tm_year + 1900;
		s[n++] = '0' + y/1000;
		s[n++] = '0' + (y%1000)/100;
		s[n++] = '0' + (y%100)/10;
		s[n++] = '0' + (y%10);
		s[n] = '\0';

		display.WriteText( &font_6x13, OP_SCREENW - 1 - (font_6x13.cols + 1)*n, 0, s );
	}
}

static void UpdateTime( struct tm &t )
{
	uint32_t seconds = mk_gmtime(&t);
	if ( abs((int32_t)seconds -  (int32_t)clock_secs) > 5 )
	{
		terminal.Send( "Setting time\r\n" );
		clock_secs = seconds;
		ShowTime(clock_secs);
		bClockInitialised = true;
	}
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

	// Enable 16MHz Ext Crystal
	OSC.XOSCCTRL = OSC_XOSCSEL_XTAL_256CLK_gc | OSC_FRQRANGE_12TO16_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	// Wait till stable.
	while ( (OSC.STATUS & OSC_XOSCRDY_bm) == 0 )
		continue;

	// PLL x2
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 2;
	OSC.CTRL |= OSC_PLLEN_bm;
	while ( (OSC.STATUS & OSC_PLLRDY_bm) == 0 )
		continue;

	// Select it as the new clock
	_PROTECTED_WRITE(CLK_PSCTRL, 0);
	_PROTECTED_WRITE(CLK_CTRL, CLK_SCLKSEL_PLL_gc);


	// RTC 
	PR.PRGEN &= ~PR_RTC_bm;
	CLK.RTCCTRL =  CLK_RTCSRC_TOSC_gc | CLK_RTCEN_bm;
	RTC.CTRL = RTC_PRESCALER_DIV256_gc;

	//// Enable dfll32m
	//OSC.XOSCCTRL = OSC_XOSCSEL_32KHz_gc;
	//OSC.CTRL |= OSC_XOSCEN_bm;
	////while ( (OSC.STATUS & OSC_XOSCRDY_bm) == 0 )
		////continue;
	//OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
	//DFLLRC32M.CTRL = DFLL_ENABLE_bm;

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

	// USART C0 - debug terminal
	PR.PRPC &= ~PR_USART0_bm;
	PORTC.DIRSET = PIN3_bm;	// TX is output, RX is input
	PORTC.OUTSET = PIN3_bm;	// set high

    // Encoder  - Using pins C0,C1 timer C1 and event channel 0
	PR.PRGEN  &= ~PR_EVSYS_bm;
	PR.PRPC &= ~PR_TC1_bm;

    PORTC.DIRCLR = PIN0_bm | PIN1_bm;
    PORTC.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    PORTC.PIN1CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    
    EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;
    EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
    
    TCC1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
    TCC1.PER = 0xFFFF;
    TCC1.CTRLA = TC_CLKSEL_DIV1_gc;

	// Key pins - b is always pull down input
	PORTCFG.MPCMASK = 0x3;
	PORTB.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	PORTB.DIRCLR = 3;

	// USART E0 - DAB
	PR.PRPE &= ~PR_USART0_bm;
	PORTE.DIRSET = PIN3_bm;	// TX is output, RX is input
	PORTE.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTE.OUTSET = PIN3_bm;	// set high

	display.Init();
	display.InitDisplay();

	// ADC (volume)
	PR.PRPA &= ~PR_ADC_bm;
	ADCA.CTRLA =  ADC_ENABLE_bm;
	ADCA.CTRLB = ADC_FREERUN_bm;
	ADCA.REFCTRL = ADC_REFSEL_INTVCC_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;

	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN10_gc;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;

	// Enable interrupt controller
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
}

static uint16_t PollPins( uint8_t pin, uint8_t next_pin )
{
	uint16_t n = (PORTA.IN & ~_BV(pin)) | ((PORTB.IN & 3) << 8);

	uint8_t nMask = 1 << next_pin;

	// Keypad keys A0-8 B0-1
	PORTCFG.MPCMASK = 0xFF;
	PORTA.PIN0CTRL = PORT_OPC_PULLDOWN_gc;

	PORTA.DIR = nMask;
	PORTA.OUTSET = nMask;

	return n;
}

#define DEBOUNCE_TIME	50	// ms
static uint16_t keydown = 0;

static uint16_t DebounceKeys(uint16_t keystate)
{
	static uint8_t button_timer[MAX_BUTTONS];
	static uint16_t pressing=0;
	static uint16_t pressed=0;
	uint16_t keychange = 0;

	uint16_t mask = 1;
	for ( int i = 0; i< MAX_BUTTONS; i++, mask <<= 1)
	{
		if ( pressing & mask )	
		{
			if ( keystate & mask )	// still pressing
			{
				button_timer[i]--;
				if ( button_timer[i] == 0 )
				{
					pressing &= ~mask;
					pressed |= mask;
					keychange |= mask;
					keydown |= mask;
				}
			}
			else
			{
				// Not pressing
				pressing &= ~mask;
			}
		}
		else if ( pressed & mask )
		{
			if ( ( keystate & mask ) == 0 )
			{
				pressed &= ~mask;
				keychange |= mask;
				keydown &= ~mask;
			}
		}
		else
		{
			if ( keystate & mask )
			{
				pressing |= mask;
				button_timer[i] = DEBOUNCE_TIME;
			}
		}
	}

	return keychange;
}

static uint16_t PollKeyboard()
{
	// A1 - S1-A2 S3-A4 S9-A5 S10-A6
	// A3 - S0-A2 S2-A4 S4-A0 S6-A5 S7-A7
	// A5 - S5-B0 S6-A3 S8-B1 S9-A1
	uint16_t keys = 0;
	uint8_t n1 = PollPins(1,3);
	uint8_t n2 = PollPins(3,5);
	uint16_t n3 = PollPins(5,1);
	if ( n1 & _BV(2) ) keys |= _BV(1);
	if ( n1 & _BV(4) ) keys |= _BV(3);
	if ( n1 & _BV(5) ) keys |= _BV(9);
	if ( n1 & _BV(6) ) keys |= _BV(10);

	if ( n2 & _BV(2) ) keys |= _BV(0);
	if ( n2 & _BV(4) ) keys |= _BV(2);
	if ( n2 & _BV(0) ) keys |= _BV(4);
	if ( n2 & _BV(5) ) keys |= _BV(6);
	if ( n2 & _BV(7) ) keys |= _BV(7);

	if ( n3 & _BV(8) ) keys |= _BV(5);
	if ( n3 & _BV(3) ) keys |= _BV(6);
	if ( n3 & _BV(9) ) keys |= _BV(8);
	if ( n3 & _BV(1) ) keys |= _BV(9);

	return DebounceKeys(keys);
}


//static void PrintDABDetails()
//{
	//bool bClockSet = false;
	//{
		//if ( dab.RTC_GetClockStatus(bClockSet))
		//{
			//terminal.Send("Clock Status: ");
			//terminal.Send(bClockSet?"Set\r\n":"Unset\r\n");
		//}
		//else
		//{
			//terminal.Send("Failed to get clock status\r\n");
		//}
	//}
//
	//if ( bClockSet )
	//{
		//uint8_t day, month, hour, minute, second;
		//uint16_t year;
		//if ( dab.RTC_GetClock(day,month,year,hour,minute,second))
		//{
			//terminal.Send("Clock: ");
			//terminal.Send( day );
			//terminal.Send( '/' );
			//terminal.Send( month );
			//terminal.Send( '/' );
			//terminal.Send( (int)year );
			//terminal.Send( ' ' );
			//terminal.Send( hour );
			//terminal.Send( ':' );
			//terminal.Send( minute );
			//terminal.Send( ':' );
			//terminal.Send( second );
			//terminal.SendCRLF();
//
			//struct tm t;
			//t.tm_year = year-1900;
			//t.tm_mon = month;
			//t.tm_mday = day;
			//t.tm_hour = hour;
			//t.tm_min = minute;
			//t.tm_sec = second;
//
			//UpdateTime( t );
		//}
		//else
		//{
			//terminal.Send("Failed to read clock\r\n");
		//}
	//}
//
	//{
		//uint16_t nPrograms;
		//if ( dab.STREAM_GetTotalProgram(nPrograms) )
		//{
			//terminal.Send( "Found ");
			//terminal.Send( (int)nPrograms );
			//terminal.Send( " DAB programs\r\n" );
		//}
		//else
		//{
			//terminal.Send( "Error reading DAB programs\r\n" );
		//}
	//}
//
	//DABPlayStatus nPlayStatus = DABPlayStatus::Stop;
	//{
		//if ( dab.STREAM_GetPlayStatus(nPlayStatus) )
		//{
			//terminal.Send( "DAB Play Status = ");
			//switch (nPlayStatus)
			//{
				//case DABPlayStatus::Playing: terminal.Send("Playing\r\n"); break;
				//case DABPlayStatus::Searching: terminal.Send("Searching\r\n"); break;
				//case DABPlayStatus::Tuning: terminal.Send("Tuning\r\n"); break;
				//case DABPlayStatus::Stop: terminal.Send("Stop\r\n"); break;
				//case DABPlayStatus::SortingChange: terminal.Send("Sorting Change\r\n"); break;
				//case DABPlayStatus::Reconfiguration: terminal.Send("Reconfiguration\r\n"); break;
				//default: terminal.Send((int)nPlayStatus); terminal.SendCRLF(); break;
			//}
		//}
		//else
		//{
			//terminal.Send( "Error reading DAB play status\r\n" );
		//}
	//}
//
	//{
		//DABPlayMode nPlayMode;
		//if ( dab.STREAM_GetPlayMode(nPlayMode) )
		//{
			//terminal.Send( "DAB Play Mode = ");
			//switch (nPlayMode)
			//{
				//case DABPlayMode::AM: terminal.Send("AM\r\n"); break;
				//case DABPlayMode::BEEPER: terminal.Send("BEEPER\r\n"); break;
				//case DABPlayMode::DAB: terminal.Send("DAB\r\n"); break;
				//case DABPlayMode::FM: terminal.Send("FM\r\n"); break;
				//case DABPlayMode::StreamStop: terminal.Send("Stream Stop\r\n"); break;
				//default: terminal.Send((int)nPlayMode); terminal.SendCRLF(); break;
			//}
		//}
		//else
		//{
			//terminal.Send( "Error reading DAB play status\r\n" );
		//}
	//}
//
	//{
		//uint32_t nIndex;
		//if ( dab.STREAM_GetPlayIndex(nIndex ) )
		//{
			//terminal.Send( "Play Index=" );
			//terminal.Send( (long)nIndex );
			//terminal.Send( "\r\n" );
		//}
		//else
		//{
			//terminal.Send( "Failed to read play index\r\n" );
		//}
	//}
//
	//if ( nPlayStatus != DABPlayStatus::Stop )
	//{
		//uint8_t nStrength; uint16_t nBitErrorRate;
		//if ( dab.STREAM_GetSignalStrength( nStrength, nBitErrorRate) )
		//{
			//terminal.Send( "Strength=" );
			//terminal.Send( (int)nStrength );
			//terminal.Send( ", Bit Error Rate=" );
			//terminal.Send( (int)nBitErrorRate );
			//terminal.Send( "\r\n" );
		//}
		//else
		//{
			//terminal.Send( "Failed to read signal strength\r\n" );
		//}
	//}
	//if ( nPlayStatus == DABPlayStatus::Playing )
	//{
		//char text[200];
		//if ( dab.STREAM_GetProgrammeText(text,sizeof(text)) )
		//{
			//terminal.Send("Program Text: ");
			//terminal.Send(text);
			//terminal.SendCRLF();
		//}
		//else
		//{
			//terminal.Send("Failed to read Program Text\r\n");
		//}
	//}
//
	//{
		//bool bEnabled;
		//if ( dab.RTC_GetSyncClockStatus(bEnabled))
		//{
			//terminal.Send("Clock Sync Status: ");
			//terminal.Send(bEnabled?"Enabled\r\n":"Disabled\r\n");
		//}
		//else
		//{
			//terminal.Send("Failed to get clock sync status\r\n");
		//}
	//}
//}

//static void PrintDABStations()
//{
	//uint16_t nPrograms;
	//if ( dab.STREAM_GetTotalProgram(nPrograms) )
	//{
		//terminal.Send( "Found ");
		//terminal.Send( (int)nPrograms );
		//terminal.Send( " DAB programs\r\n" );
	//}
	//else
	//{
		//terminal.Send( "Error reading DAB programs\r\n" );
	//}
//
	//for ( uint16_t i = 0; i < nPrograms; i++ )
	//{
		//char buf[120];
		//if ( dab.STREAM_GetProgrammeName( i, false, buf, sizeof(buf) ) )
		//{
			//terminal.Send( (int)i );
			//if ( i < 10 )
			//terminal.Send(' ');
			//terminal.Send(' ');
			//terminal.Send( buf );
			//for ( unsigned int j = 0; j < 9 - strlen(buf); j++)
			//terminal.Send(' ');
//
			//if ( dab.STREAM_GetProgrammeName( i, true, buf, sizeof(buf) ) )
			//{
				//terminal.Send( buf );
				//terminal.SendCRLF();
			//}
		//}
	//}
//}


static enum class RadioState: uint8_t
{
	Init,
	WaitForReply,
	Off,
	InitStartUpTasks,
	StartUpTasks,
	StartUpTasksPause,
	WaitForTaskReply,

	StartPlaying,
	InitStartTasks,
	Playing,
	PlayingNext
} radioState = RadioState::Init, successState, failState, taskDoneState;
static int8_t task;

void SetWaitForReply( RadioState success, RadioState failure)
{
	radioState = RadioState::WaitForReply;
	successState = success;
	failState = failure;
}


enum class TaskReturn: uint8_t
{
	Done,
	Fail,
	Continue
};

enum class TaskInit: uint8_t
{
	OK,
	Fail,
	Skip
};

#define MAX_TASKS	15
static struct StartupTaskFuncs
{
	TaskInit (*init)();
	TaskReturn (*check)(AsyncReturnCode ret);

} TaskList[MAX_TASKS];
uint8_t nTaskCount;


static TaskInit funcSetVolume0_init() 
{
	dab.STREAM_SetVolume_Async(0, ms );
	return TaskInit::OK;
} 

static TaskInit funcSetVolume_init()
{
	dab.STREAM_SetVolume_Async(nLastVolume, ms );
	return TaskInit::OK;
}


static TaskInit funcSetNotifications_init()
{
	dab.NOTIFY_SetNotification_Async(   (DABNotification)((uint16_t)DABNotification::ScanFinished |
										(uint16_t)DABNotification::NewFMText |
										(uint16_t)DABNotification::DABReconfig |
										(uint16_t)DABNotification::DABChanChange |
										(uint16_t)DABNotification::FMRDSGrpNotify |
										(uint16_t)DABNotification::NewDABText |
										(uint16_t)DABNotification::ScanFrequency), ms );
	return TaskInit::OK;
}

static TaskReturn ack_check(AsyncReturnCode ret) 
{
	if ( ret == AsyncReturnCode::ReplAck )
		return TaskReturn::Done;
	else
		return TaskReturn::Fail;
} 

static TaskInit funcEnableSyncClock_init() 
{
	dab.RTC_EnableSyncClock_Async(1, ms );
	return TaskInit::OK;
}

static TaskReturn funcGetSyncClockStatus_check(AsyncReturnCode ret)
{
	bool bSet = false;
	if ( ret == AsyncReturnCode::ReplAck )
	{
		bSet = dab.MsgBuffer()[0] != 0;
	}
	return bSet ? TaskReturn::Done : TaskReturn::Fail;
}

static TaskInit funcGetSyncClockStatus_init()
{
	dab.RTC_GetSyncClockStatus_Async( ms );
	return TaskInit::OK;
}

static TaskReturn funcGetClockStatus_check(AsyncReturnCode ret)
{
#ifdef SET_CLOCK_AT_POWERUP
	bool bEnabled = false;
	if ( ret == AsyncReturnCode::ReplAck )
	{
		bEnabled = dab.MsgBuffer()[0] != 0;
	}
	return bEnabled ? TaskReturn::Done : TaskReturn::Fail;
#else
	return TaskReturn::Done;
#endif
}

static TaskInit funcGetClockStatus_init()
{
	dab.RTC_GetClockStatus_Async( ms );
	return TaskInit::OK;
}

// TODO - call this every 5 minutes or so to verify the time.
static TaskInit funcGetClock_init()
{
	dab.RTC_GetClock_Async(ms);
	return TaskInit::OK;
}

static TaskReturn funcGetClock_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		uint8_t *input = dab.MsgBuffer();

		struct tm t;
		t.tm_year = input[6] + 2000 - 1900;
		t.tm_mon = input[5]-1;
		t.tm_mday = input[3];
		t.tm_hour = input[2];
		t.tm_min = input[1];
		t.tm_sec = input[0];

		UpdateTime( t );

		if ( radioState != RadioState::Off )
			ShowDate();

		return TaskReturn::Done;
	}
	else
		return TaskReturn::Fail;	
}

static TaskReturn funcGetClock_Optional_check(AsyncReturnCode ret)
{
	funcGetClock_check(ret);
	return TaskReturn::Done;
}

static TaskInit funcGetProgramCount_init()
{
	dab.STREAM_GetTotalProgram_Async(ms);
	return TaskInit::OK;
}

static TaskReturn funcGetProgramCount_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		nPrograms =  dab.MsgBuffer()[3] | ( dab.MsgBuffer()[2]<<8);	// use only the 2 low bytes.
		uint16_t program_space = sizeof(Programs)*nPrograms;
		memset( programs, 0, program_space );
		sProgramNames = (char *)programs + program_space;
		nProgramIdx = 0;
		return TaskReturn::Done;
	}
	else
		return TaskReturn::Fail;
}

static TaskInit funcGetPrograms_init()
{
	bool bFullName = programs[nProgramIdx].sLongName == NULL ? true: false;
	dab.STREAM_GetProgrammeName_Async(nProgramIdx, bFullName, sProgramNames, 256, ms);
	return TaskInit::OK;
}

static TaskReturn funcGetPrograms_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		bool bFullName = programs[nProgramIdx].sLongName == NULL ? true: false;
		const char *sText = sProgramNames;
		if ( bFullName )
			programs[nProgramIdx].sLongName = sProgramNames;
		else
			programs[nProgramIdx].sShortName = sProgramNames;

		// Remove trailing white space
		uint8_t i = dab.MsgLen();
		while ( i > 0 &&  (sProgramNames[i-1] == ' ' || sProgramNames[i-1] == 0 ) )
			i--;
		sProgramNames += i;
		*(sProgramNames++) = '\0';
		terminal.Send(nProgramIdx);
		terminal.Send(" ");
		terminal.Send(sText);
		terminal.SendCRLF();

		if ( bFullName ) // still need to do the short name
			return TaskReturn::Continue;

		nProgramIdx++;
		if ( nProgramIdx < nPrograms )
			return TaskReturn::Continue;
		else
			return TaskReturn::Done;
	}
	else
		return TaskReturn::Fail;
}

static TaskInit funcPlayProgramForTime_init()
{
	dab.STREAM_Play_Async(DABPlayMode::DAB,nLastPlayedProgram,ms);
	return TaskInit::OK;
}

static TaskInit funcPlayProgram_init()
{
	const char *sProgram = programs[nLastPlayedProgram].sLongName;

	terminal.Send("ProgramName: ");
	terminal.Send(sProgram);
	terminal.SendCRLF();
	if (  displayMode == DisplayMode::Playing )
		display.WriteText( &font_MSShell, 0, 0, sProgram );
	//display.WriteText( &font_MSShell, 0, 0, "RSN Racing&Sport" );
	
	dab.STREAM_Play_Async(DABPlayMode::DAB,nLastPlayedProgram,ms);
	return TaskInit::OK;
}

static TaskInit funcStopPlaying_init()
{
	dab.STREAM_Stop_Async(ms);
	return TaskInit::OK;
}

static TaskInit funcPowerOff_init()
{
	dab.PowerOff();
	display.displayPowerOff();
	return TaskInit::OK;
}

void InitStartUpTasks()
{
	task = 0;
	nTaskCount = 0;

	TaskList[nTaskCount++] = { funcPlayProgramForTime_init, ack_check };
	TaskList[nTaskCount++] = { funcSetVolume0_init, ack_check };
	TaskList[nTaskCount++] = { funcEnableSyncClock_init, ack_check };
	TaskList[nTaskCount++] = { funcGetProgramCount_init, funcGetProgramCount_check };
	TaskList[nTaskCount++] = { funcGetPrograms_init, funcGetPrograms_check };
	TaskList[nTaskCount++] = { funcGetSyncClockStatus_init, funcGetSyncClockStatus_check };
	TaskList[nTaskCount++] = { funcGetClockStatus_init, funcGetClockStatus_check };
#ifdef SET_CLOCK_AT_POWERUP
	TaskList[nTaskCount++] = { funcGetClock_init, funcGetClock_check };
#else
	bClockInitialised = true;
#endif
	TaskList[nTaskCount++] = { funcStopPlaying_init, ack_check };
	TaskList[nTaskCount++] = { funcPowerOff_init, NULL };
}

static TaskInit funcGetProgramText_init()
{
	dab.STREAM_GetProgrammeText_Async( sProgramNames, 256, ms);
	return TaskInit::OK;
}


static TaskReturn funcGetProgramText_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		// TODO : move unicode conversion into receiver.
		char *text = sProgramNames;
		*(text+dab.MsgLen()) = '\0';
		terminal.Send("ProgramText: ");
		terminal.Send(text);
		terminal.SendCRLF();
		//display.WriteText( &font_6x13, 0, 11, text );
		ScrollText(true);
		return TaskReturn::Done;
	}
	else if ( ret == AsyncReturnCode::ReplyNack )
	{
		// nothing there.
		terminal.Send("ProgramText: ");
		if ( dab.MsgBuffer()[0] == 0 )
			terminal.Send("No Text\r\n");
		else
			terminal.Send("No New Text\r\n");
		return TaskReturn::Done;
	}
	return TaskReturn::Fail;
}

static void InitStartTasks()
{
	task = 0;
	nTaskCount = 0;

	TaskList[nTaskCount++] = { funcPlayProgram_init, ack_check };
	TaskList[nTaskCount++] = { funcEnableSyncClock_init, ack_check };
	TaskList[nTaskCount++] = { funcSetVolume_init, ack_check };
	TaskList[nTaskCount++] = { funcSetNotifications_init, ack_check };
	TaskList[nTaskCount++] = { funcGetProgramText_init, funcGetProgramText_check };
}

static TaskReturn funcGetProgramTextNotify_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		// TODO : move unicode conversion into receiver.
		char *text = sProgramNames;
		*(text+dab.MsgLen()) = '\0';
		terminal.Send("ProgramText: ");
		terminal.Send(text);
		terminal.SendCRLF();
		//display.WriteText( &font_6x13, 0, 11, text );
		ScrollText(true);
		return TaskReturn::Done;
	}
	return TaskReturn::Fail;
}


static void InitGetProgramTextTasks()
{
	task = 0;
	nTaskCount = 0;

	TaskList[nTaskCount++] = { funcGetProgramText_init, funcGetProgramText_check };
	//PollTaskList[nTaskCount++] = { funcGetProgramText_init, funcGetProgramTextNotify_check };
}

static TaskInit funcGetPlayStatus_init() 
{ 
	dab.STREAM_GetPlayStatus_Async(ms); 
	return TaskInit::OK; 
}
static TaskReturn funcGetPlayStatus_check(AsyncReturnCode ret) 
{ 
	if ( ret == AsyncReturnCode::ReplAck )
	{
		terminal.Send("Play Status: ");
		terminal.Send(dab.MsgBuffer()[0]);
		terminal.SendCRLF();
		return TaskReturn::Done;
	}
	return TaskReturn::Fail;
}
static TaskInit funcGetPlayMode_init() 
{	
	dab.STREAM_GetPlayMode_Async(ms); 
	return TaskInit::OK; 
}
static TaskReturn funcGetPlayMode_check(AsyncReturnCode ret) 
{ 
	if ( ret == AsyncReturnCode::ReplAck )
	{
		terminal.Send("Play Mode: ");
		terminal.Send(dab.MsgBuffer()[0]);
		terminal.SendCRLF();
		return TaskReturn::Done;
	}
	return TaskReturn::Fail;
}
static TaskInit funcGetPlayIndex_init() 
{ 
	dab.STREAM_GetPlayIndex_Async(ms); 
	return TaskInit::OK; 
}
static TaskReturn funcGetPlayIndex_check(AsyncReturnCode ret) 
{ 
	if ( ret == AsyncReturnCode::ReplAck )
	{
		terminal.Send("Play Index: ");
		terminal.Send(dab.MsgBuffer()[3]);
		terminal.SendCRLF();
		return TaskReturn::Done;
	}
	return TaskReturn::Fail;
}
static TaskInit funcGetSignalStrength_init() 
{ 
	dab.STREAM_GetSignalStrength_Async(ms); 
	return TaskInit::OK; 
}
static TaskReturn funcGetSignalStrength_check(AsyncReturnCode ret) 
{ 
	if ( ret == AsyncReturnCode::ReplAck )
	{
		uint8_t strength = dab.MsgBuffer()[0];
		uint16_t errors = (dab.MsgBuffer()[1] << 8) | dab.MsgBuffer()[2];
		terminal.Send("Signal Strength: ");
		terminal.Send(strength);
		terminal.Send(" Errors: ");
		terminal.SendHex((int)errors);
		terminal.SendCRLF();
		return TaskReturn::Done;
	}
	else if ( ret == AsyncReturnCode::ReplyNack )
	{
		return TaskReturn::Done;
	}
	return TaskReturn::Fail;
}

static TaskInit funcUpdateVolume_init()
{
	if ( volumeSetting != nLastVolume )
	{
		if ( nLastVolume < volumeSetting )
			nLastVolume++;
		else if ( nLastVolume > volumeSetting )
			nLastVolume--;
		dab.STREAM_SetVolume_Async( nLastVolume, ms );

		terminal.Send("Volume=");
		terminal.Send(nLastVolume);
		terminal.SendCRLF();

		return TaskInit::OK;
	}
	return TaskInit::Skip;
}

TaskReturn funcUpdateVolume_check(AsyncReturnCode ret )
{
	return TaskReturn::Done;
}

void InitPollTasks(uint8_t nLevel)
{
	task = 0;
	nTaskCount = 0;

	// every second - radio play status, volume
	// every 15 seconds - play mode, play index, signal strength, program text
	// every 5 minutes - clock

	if ( nLevel >= 1 )
	{
		TaskList[nTaskCount++] = { funcGetPlayStatus_init, funcGetPlayStatus_check };
		TaskList[nTaskCount++] = { funcUpdateVolume_init, funcUpdateVolume_check };
	}
	if ( nLevel >= 2 )
	{
		TaskList[nTaskCount++] = { funcGetPlayMode_init, funcGetPlayMode_check };
		TaskList[nTaskCount++] = { funcGetPlayIndex_init, funcGetPlayIndex_check };
		TaskList[nTaskCount++] = { funcGetSignalStrength_init, funcGetSignalStrength_check };
		TaskList[nTaskCount++] = { funcGetProgramText_init, funcGetProgramText_check };
	}
	if ( nLevel >= 3 )
	{
		TaskList[nTaskCount++] = { funcEnableSyncClock_init, ack_check };
		TaskList[nTaskCount++] = { funcGetClock_init, funcGetClock_Optional_check };
	}
}

int8_t NextTask()
{
	task++;
	if ( task >= nTaskCount )
		task = -1;
	return task;
}

static uint32_t playPollTimer = 0;


void DoDAB()
{
	//{
		//static RadioState lastState = RadioState::Init;
		//if ( lastState != radioState )
		//{
			//terminal.Send((uint8_t)lastState);
			//terminal.Send("->");
			//terminal.Send((uint8_t)radioState);
			//terminal.SendCRLF();
			//lastState = radioState;
		//}
	//}
	AsyncReturnCode ret = dab.Async( ms );
	if ( ret != AsyncReturnCode::OK )
	{
		terminal.Send("ret="); terminal.Send((uint8_t)ret); terminal.SendCRLF();
	}
	if ( dab.isIdle() )
	{
		switch ( radioState )
		{
			case RadioState::Init:
				dab.SYSTEM_GetSysRdy_Async(ms,500);
				SetWaitForReply( RadioState::InitStartUpTasks, RadioState::Init );
				//SetWaitForReply( RadioState::Idle, RadioState::Init );
				break;

			case RadioState::WaitForReply:
				if ( ret == AsyncReturnCode::ReplAck )
					radioState = successState;
				else if ( ret != AsyncReturnCode::OK )
					radioState = failState;
				break;

			case RadioState::InitStartUpTasks:
				InitStartUpTasks();
				radioState = RadioState::StartUpTasks;
				taskDoneState = RadioState::Off;
				break;

			case RadioState::StartUpTasks:
			{
				// Next task
				TaskInit t = TaskList[task].init();
				if ( t == TaskInit::OK || t == TaskInit::Skip )
				{
					if ( t == TaskInit::Skip || TaskList[task].check == NULL )
					{
						task = NextTask();
						if ( task < 0 )
							radioState = taskDoneState;	// Done.
					}
					else
					{
						radioState = RadioState::WaitForTaskReply;
					}
				}
			}
				break;
				
			case RadioState::WaitForTaskReply:
				if ( ret != AsyncReturnCode::OK )
				{
					TaskReturn tret = TaskReturn::Done;
					if ( TaskList[task].check != NULL )
						tret = TaskList[task].check(ret);
					if ( tret == TaskReturn::Done )
					{
						task = NextTask();
						if ( task < 0 )
							radioState = taskDoneState;	// Done.
						else
							radioState = RadioState::StartUpTasks;
					}
					else if ( tret == TaskReturn::Continue )
					{
						radioState = RadioState::StartUpTasks;
					}
					else 
					{
						radioState = RadioState::StartUpTasksPause;
						radioTimer = ms + 250;
					}
				}
			break;

			case RadioState::StartUpTasksPause:
				if ( (int32_t)radioTimer - (int32_t)ms <= 0 )
					radioState = RadioState::StartUpTasks;
				break;

			case RadioState::StartPlaying:
				dab.SYSTEM_GetSysRdy_Async(ms,500);
				SetWaitForReply( RadioState::InitStartTasks, RadioState::StartPlaying );
				break;
								
			case RadioState::InitStartTasks:
				InitStartTasks();
				dab.Mute( false );
				radioState = RadioState::StartUpTasks;
				taskDoneState = RadioState::Playing;
				break;

			case RadioState::PlayingNext:
				radioState = RadioState::Playing;
				playPollTimer = ms + 1000;
				break;

			case RadioState::Playing:
				if ( ret == AsyncReturnCode::Notification )
				{
					// TODO - decide - do we continue with this?  Even after receiving the notification, programme text is not there.
					if ( dab.LastNotification() == DABNotificationType::NewProgrammeText )
					{
						InitGetProgramTextTasks();
						radioState = RadioState::StartUpTasks;
						taskDoneState = RadioState::Playing;
					}
					else
					{
						terminal.Send("Notification: ");
						terminal.Send( (uint8_t)ret);
						terminal.SendCRLF();
					}
				}
				else
				{

					// Execute Scheduled task lists
					// every second - radio play status, volume
					// every 15 seconds - play mode, play index, signal strength, program text
					// every 5 minutes - clock
					if ( (int32_t)playPollTimer - (int32_t)ms <= 0 )
					{
						const uint16_t LONG_TIME = 5*60;	// clock every 5 minutes.
						const uint16_t SHORT_TIME = 10;		// clock every 15 seconds.
						static uint16_t nTicker = LONG_TIME - 30;
						if ( nTicker == LONG_TIME )
						{
							InitPollTasks(3);
							nTicker = 0;
						}
						else if ( nTicker % SHORT_TIME == 0 )
							InitPollTasks(2);
						else
							InitPollTasks(1);

						nTicker++;
						radioState = RadioState::StartUpTasks;
						taskDoneState = RadioState::PlayingNext;
					}
				}
				break;

			case RadioState::Off:
				break;
		}
	}
}

static enum class ClockRadioState: uint8_t
{
	Off,
	Idle,
} clockRadioState = ClockRadioState::Off;


static uint16_t nTunerDisplayProgram;
static int16_t nTunerDelta;
const uint16_t SELECTOR_WIDTH = 60;

void InitTunerDisplay()
{
	display.clearScreen(true);
	nTunerDisplayProgram = nLastPlayedProgram;
	nTunerDelta = nLastPlayedProgram * 4;

	uint8_t y = 12 + 12/2;
	 
	// Put current station in the center of the screen. 4 rows?
	for ( int16_t n = -2; n <= 2; n++ )
	{
		int16_t nProg = nTunerDisplayProgram + n;
		if ( nProg >= 0 && nProg < nPrograms )
		{
			uint16_t len = strlen( programs[nProg].sLongName ) * (font_6x13.cols+1) - 1;
			display.WriteText( &font_6x13, OP_SCREENW/2-len/2,y+12*n, programs[nProg].sLongName );
		}
	}

	// Selection box
/////#define TUNER_BORDER
#ifdef TUNER_BORDER
	display.VLine( OP_SCREENW/2-SELECTOR_WIDTH, y-1, y+12-1, 0x3F );
	display.VLine( OP_SCREENW/2+SELECTOR_WIDTH, y-1, y+12-1, 0x3F );
	display.HLine( OP_SCREENW/2-SELECTOR_WIDTH,OP_SCREENW/2+SELECTOR_WIDTH, y-1, 0x3F );
	display.HLine( OP_SCREENW/2-SELECTOR_WIDTH,OP_SCREENW/2+SELECTOR_WIDTH, y+12-1, 0x3F );
#else
	display.SetRow(); display.SetColumn();
	display.SetFillMode( false, true );
	display.Copy( OP_SCREENW/2-SELECTOR_WIDTH,y-1, OP_SCREENW/2+SELECTOR_WIDTH,y+12-2, OP_SCREENW/2-SELECTOR_WIDTH,y-1, true );
	display.SetFillMode( false, false );
#endif	
}


void UpdateTunerDisplay(int8_t delta)
{
	nTunerDelta += delta;
	if ( nTunerDelta < 0 )
		nTunerDelta = 0;
	else if ( nTunerDelta > 4*(nPrograms-1) )
		nTunerDelta = 4*(nPrograms-1);

	if ( nTunerDelta/4 == nTunerDisplayProgram )
		return;

	nTunerDisplayProgram = nTunerDelta/4;

	display.ClearWindow(OP_SCREENW/2-SELECTOR_WIDTH,0,OP_SCREENW/2+SELECTOR_WIDTH,OP_SCREENH, true);
	
	uint8_t y = 12 + 12/2;
	// Put current station in the center of the screen. 4 rows?
	for ( int16_t n = -2; n <= 2; n++ )
	{
		int16_t nProg = nTunerDisplayProgram + n;
		if ( nProg >= 0 && nProg < nPrograms )
		{
			uint16_t len = strlen( programs[nProg].sLongName ) * (font_6x13.cols+1) - 1;
			display.WriteText( &font_6x13, OP_SCREENW/2-len/2,y+12*n, programs[nProg].sLongName );
		}
	}


	// Selection box
/////#define TUNER_BORDER
#ifdef TUNER_BORDER
	display.VLine( OP_SCREENW/2-SELECTOR_WIDTH, y-1, y+12-1, 0x3F );
	display.VLine( OP_SCREENW/2+SELECTOR_WIDTH, y-1, y+12-1, 0x3F );
	display.HLine( OP_SCREENW/2-SELECTOR_WIDTH,OP_SCREENW/2+SELECTOR_WIDTH, y-1, 0x3F );
	display.HLine( OP_SCREENW/2-SELECTOR_WIDTH,OP_SCREENW/2+SELECTOR_WIDTH, y+12-1, 0x3F );
#else
	display.SetRow(); display.SetColumn();
	display.SetFillMode( false, true );
	display.Copy( OP_SCREENW/2-SELECTOR_WIDTH,y-1, OP_SCREENW/2+SELECTOR_WIDTH,y+12-2, OP_SCREENW/2-SELECTOR_WIDTH,y-1, true );
	display.SetFillMode( false, false );
#endif	
}

static void Tuner( int16_t shift )
{
	switch ( displayMode )
	{
		case DisplayMode::Off:
			break;
		case DisplayMode::Menu:
			break;
		case DisplayMode::Playing:
			// Move from Playing display to Tuner (select program)
			InitTunerDisplay();
			displayMode = DisplayMode::Tuner;
			break;
		case DisplayMode::Tuner:
			UpdateTunerDisplay(shift);
			break;
		}
}

static void RadioPlay()
{
	if ( radioState == RadioState::Off )
	{
		dab.AsyncStart();
		displayMode = DisplayMode::Playing;
		display.InitDisplay();
		ShowDate();
		radioState = RadioState::StartPlaying;
		clockRadioState = ClockRadioState::Idle;
	}
}

static void RadioOff()
{
	if ( radioState != RadioState::Off )
	{
		dab.PowerOff();
		display.displayPowerOff();
		radioState = RadioState::Off;
		clockRadioState = ClockRadioState::Off;
	}
}

#define PROGRAM_TEXT_POS	36

static uint16_t textXShift = 0, textXShiftMax;
static int16_t xoffs;
static char sText[256];

static bool DrawProgramText(bool bNewText)
{
	bool bShifting = false;
	uint8_t len = strlen(sText);

	if ( bNewText )
	{
		textXShift = 0;
		uint16_t plen = len * (font_6x13.cols+1);
		if ( plen <= OP_SCREENW )
		{
			// Small enough to fit on the screen.
			xoffs = (OP_SCREENW - plen) / 2;
		}
		else 
		{
			xoffs = -1;
			textXShiftMax = plen - OP_SCREENW;
		}
	}
	if ( xoffs >= 0 )
	{
		if ( bNewText )
			display.WriteText( &font_6x13, xoffs, PROGRAM_TEXT_POS, sText);
	}
	else 
	{
		if ( textXShift < textXShiftMax )
		{
			div_t offs = div( textXShift, font_6x13.cols+1 );

			display.WriteText( &font_6x13, -offs.rem, PROGRAM_TEXT_POS, sText+offs.quot);
			textXShift++;
			bShifting = true;
		}
	}
	return bShifting;
}

static enum class ScrollState: uint8_t
{
	idle,
	shiftingOut,
	startNew,
	pause,
	scroll
} scrollState = ScrollState::idle, scrollNextState;
uint32_t scrollDelay;

static void ScrollText(bool bRedraw)
{
	static uint8_t nShiftCount = 0;
	static uint8_t last_ms8 = 0;
	uint8_t ms8 = *(uint8_t *)&ms;

	if ( bRedraw )
	{
		strncpy( sText, sProgramNames, sizeof(sText) );
		sText[sizeof(sText)-1] = '\0';
		nShiftCount = font_6x13.rows+1;
		scrollState = ScrollState::shiftingOut;
	}
	if ( ms8 != last_ms8 && displayMode == DisplayMode::Playing )
	{
		switch ( scrollState )
		{
			case ScrollState::idle:
				break;

			case ScrollState::shiftingOut:
				// Are we shifting the text away.
				if ( nShiftCount && (ms8 & 0x3f) == 0 )
				{
					uint16_t xs =0;
					uint8_t ys = PROGRAM_TEXT_POS-1+font_6x13.rows-(nShiftCount-1);
					uint16_t xe = OP_SCREENW-1;
					uint8_t ye = PROGRAM_TEXT_POS+font_6x13.rows;
					uint16_t xd=0;
					uint8_t yd = ys+1;

					display.Copy( xs,ys, xe,ye, xd,yd );
					nShiftCount--;
					if ( nShiftCount == 0 )
						scrollState = ScrollState::startNew;
				}
				break;

			case ScrollState::startNew:
				{
					uint16_t xe = OP_SCREENW-1;
					uint16_t xd=0;
					uint8_t yd = PROGRAM_TEXT_POS-1+font_6x13.rows;

					display.ClearWindow( xd,yd, xe,yd+1 );

					if ( DrawProgramText(true) )
					{
						scrollDelay = ms + 3000;
						scrollState = ScrollState::pause;
						scrollNextState = ScrollState::scroll;
					}
					else
						scrollState = ScrollState::idle;
				}
				break;

			case ScrollState::pause:
				if ( (int32_t)scrollDelay - (int32_t)ms < 0 )
				{
					scrollState = scrollNextState;
				}
				break;

			case ScrollState::scroll:
				if ( !DrawProgramText(false) )
				{
					nShiftCount = font_6x13.rows+1;
					scrollDelay = ms + 3000;
					scrollState = ScrollState::pause;
					scrollNextState = ScrollState::shiftingOut;
				}
				break;
		}
		last_ms8 = ms8;
	}

}


static void DoClockRadio(uint16_t key_changes, int16_t nEncoder )
{
	static int16_t nLastEncoder = 0;

	switch ( clockRadioState )
	{
		case ClockRadioState::Off:
			if ( key_changes & keydown & BTN_RADIO_ON ) 
			{
				RadioPlay();
			}
			break;

		case ClockRadioState::Idle:
			if ( key_changes ^ keydown )
			{
				if ( key_changes & keydown & BTN_RADIO_OFF )
				{
					RadioOff();
				}
				else if ( key_changes & keydown & BTN_SNOOZE )
				{
					//                      123456789012345678901234567890123456789012345678
					strcpy( sProgramNames, "This is a message that is less then 48 chars" );
					ScrollText(true);
				}
				else if ( key_changes & keydown & BTN_SLEEP )
				{
					//                      123456789012345678901234567890123456789012345678
					//strcpy( sProgramNames, "This is a message that is well over the 48 characters that can be displayed on the display in one go." );
					strcpy( sProgramNames, "This is a message." );
					ScrollText(true);
				}
				else if ( key_changes & keydown & BTN_TIME_SET )
				{
					//                      123456789012345678901234567890123456789012345678
					strcpy( sProgramNames, "Another long message; The quick brown fox jumps over the lazy dog." );
					ScrollText(true);
				}
			}
			else if ( nEncoder != nLastEncoder )
			{
				int16_t nEncoderChange = nEncoder - nLastEncoder;
				Tuner( nEncoderChange );
			}
			else
			{
				ScrollText();
			}
			break;
	}

	nLastEncoder = nEncoder;
}

void Spinner()
{
	// sin/cos x/y coordinates of the dots. (spaced so the whitespace of the character doesn't touch the neighbouring dots)
	static struct { uint8_t x; uint8_t y; } pos[] = {
			{122,20},
			{125,10},
			{132,3},
			{142,0},
			{152,3},
			{159,10},
			{162,20},
			{159,30},
			{152,37},
			{142,40},
			{132,37},
			{125,30},
	};
	static uint8_t lc = 0;
	lc = (lc-1) & 0x3F;
	uint8_t c = lc;
	for ( uint8_t i = 0; i < countof(pos); i++ )
	{
		display.WriteText( &font_6x13, pos[i].x, pos[i].y, "\x7f", c );
		c += 0x3F/12;
		c = c & 0x3f;
	}
}

static uint8_t ConvertVolume( int16_t v )
{
	uint16_t sample[] = 
	{
		//0,
		190,
		210,
		225,
		240,
		260,
		300,
		340,
		390,
		415,
		490,
		650,
		900,
		1260,
		1900,
		2700,
		2900,
		4096
	};


	uint8_t d = 0;
	while ( sample[d] < v )
		d++;

	return d;
}

int main(void)
{
	ioinit();

	// debug 115200
	int BSEL = 524;
	int BSCALE = -5;
	terminal.Init( MAKE_XBAUD( BSCALE, BSEL )  );

	// DBA chip is 57600
	BSEL = 540;
	BSCALE = -4;
	dab.Init( MAKE_XBAUD( BSCALE, BSEL )  );

	sei();

	terminal.Send( "\r\nDBA Clock Radio Starting\r\n");
	sevenSeg_I2C.Start();

	terminal.Send( "Resetting radio - ");
	dab.AsyncStart();
	display.setBrightness(15);

	programs = (Programs *)__malloc_heap_start;

	uint16_t last_tick = 0;
	bool bFlash = true;
	ShowTime(clock_secs);
    while (1) 
    {
		if ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) == 0 )
		{
			uint16_t rtc = RTC.CNT;
			uint16_t tick = rtc >> 2;
			if ( tick != last_tick )
			{
				clock_secs += (tick - last_tick);
				last_tick = tick;

				if ( !bClockInitialised )
				{
					ShowTime(clock_secs);
				}
				else
				{
					if ( clock_secs % 60 == 0 )
						ShowTime(clock_secs);
					else
						FlashColon(true);
				}
				bFlash = true;
				terminal.Send("tick\r\n");
			}
			else if ( bFlash && (rtc & 2) != 0 )
			{
				if ( !bClockInitialised )
					ClearDisplay();
				else
					FlashColon(false);
				bFlash = false;
			}

		}

		static int16_t nLastEncoder = -1;
		int16_t nEncoder = TCC1.CNT;
		if ( nEncoder != nLastEncoder )
		{
			nLastEncoder = nEncoder;
			terminal.Send( "Encoder: ");
			terminal.Send( (long)nEncoder );
			terminal.SendCRLF();
		}

		uint16_t key_changes;
		static uint8_t last_ms = -1;
		uint8_t ms8 = *(uint8_t *)&ms;
		if ( last_ms != ms8 )
		{
			key_changes = PollKeyboard();
			if ( key_changes )
			{
				terminal.SendHex((int)key_changes);
				terminal.SendCRLF();
			}
			last_ms = ms8;

			if ( !bClockInitialised && (ms8 % 20) == 0 )
				Spinner();
		}



		static uint8_t last_adc = 0;
		if ( last_adc != ms8 )
		{
			last_adc = ms8;
			static uint8_t n = 0;
			static uint16_t sum = 0;

			if ( ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm )
			{
				ADCA.CH0.INTFLAGS |= ADC_CH_CHIF_bm;
				sum += ADCA.CH0.RES;
				n++;
				if ( n == 16 )
				{
					volumeSetting = ConvertVolume(sum >> 4);
					n = 0;
					sum = 0;
				}
				ADCA.CH0.CTRL |= ADC_CH_START_bm;
			}
		}

		DoClockRadio(key_changes, nEncoder);
		DoDAB();

		//static int16_t t=0;
		//display.WriteText(&font_6x13, 258+t,0,"test1");
		//display.WriteText(&font_6x13, 259+t,11,"test2");
		//display.WriteText(&font_6x13, 260+t,23,"test3");
		//display.WriteText(&font_6x13, 261+t,35,"test4");
//
		//display.WriteText(&font_6x13,  0-t,0,"test1");
		//display.WriteText(&font_6x13, -1-t,11,"test2");
		//display.WriteText(&font_6x13, -2-t,23,"test3");
		//display.WriteText(&font_6x13, -3-t,35,"test4");
		//t++;
    }
}



/*

DAB
- Init 
	- EnableSyncClock.
	- Read programs
	- Set clock time
		- appears you can only get time when you are playing
		- Play a channel
		- Get time.
		- stop playing when we've got time.  We can abort this last step if we start playing.
- Radio On
	EnableSyncClock
	Play
	Volume
- Radio Off
	PowerOff
- Change channle
	- When playing, and encoder moves,
		- Show list of stations, encoder scrolls.  Stopping scrolling plays.

- Clock
	- Idle
	- Radio Play
	- Sleep
	- Alarm
	- Menu



-- TODO 
	- eeprom 
		- alarms
		- last station (short name)

	- fm support?

	- ldr for autobrightness.

	-- Menu - when off, encoder scrolls menu.  Use alarm1/2 buttons?
		-- alarm(s)
			- mon/tue/wed/thr/fri/sat/sun
				- time
				- repeating - one off.
		-- rescan DAB
			-- factory reset?
		-- different display layouts
		-- other?
		-- sleep time, snooze time.
	-- change station

	-- if program count = 0 dab scan (or just menu?)
*/
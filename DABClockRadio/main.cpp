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
static uint16_t nLastPlayedProgram = 30;
static uint16_t nLastVolume = 8;




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
} radioState = RadioState::Init, successState, failState, taskDoneState;
static int8_t task;

void SetWaitForReply( RadioState success, RadioState failure)
{
	radioState = RadioState::WaitForReply;
	successState = success;
	failState = failure;
}

enum class StartUpTasks: uint8_t
{
	PlayProgramForTime = 0,
	SetVolume0,
	EnableSyncClock,
	GetProgramCount,
	GetPrograms,
	GetSyncClockStatus,
	GetClockStatus,
	GetClock,
	StopPlaying,
	PowerOff,
	TasksMax
};

enum class StartTasks: uint8_t
{
	PlayProgram=0,
	EnableSyncClock,
	SetVolume,
	SetNotifications,
	GetProgramText,
	TasksMax
};


enum class GetProgramTextTasks: uint8_t
{
	GetProgramText=0,
	TasksMax
};

enum class PollTasks: uint8_t
{
	GetPlayStatus=0,
	GetPlayMode,
	GetPlayIndex,
	GetSignalStrength,
	//GetStereoMode,
	//GetStereo,
	//GetVolume,
	//GetProgrameType,
	//GetProgrameName,
	GetProgrameText,
	//GetSamplingRate,
	//GetDataRate,
	//GetSignalQuality,
	//GetFrequency,
	//GetEnsembleName,
	//GetTotalProgram,
	//IsActive,
	//GetServiceName,
	//GetSearchProgram,
	//GetPowerBar,
	//GetServCompType,
	//GetPreset,
	//GetSorter,
	//GetDrc,
	//GetBBEEQ,
	//GetHeadroom,
	//GetDLSCmd,
	//GetECC,
	//GetRdsPIcode,
	//GetServFollowingDABInfo,
	//GetRDSRawData.


	TasksMax
};

enum class TaskReturn: uint8_t
{
	Done,
	Fail,
	Continue
};
static struct StartupTaskFuncs
{
	bool (*init)();
	TaskReturn (*check)(AsyncReturnCode ret);

}	StartUpTaskList[(int)StartUpTasks::TasksMax], 
	StartTaskList[(int)StartTasks::TasksMax], 
	PollTaskList[(int)PollTasks::TasksMax], 
	GetProgramTextTaskList[(int)GetProgramTextTasks::TasksMax],  *pTasks;
uint8_t nTaskCount;


bool funcSetVolume0_init() 
{
	dab.STREAM_SetVolume_Async(0, ms );
	return true;
} 

bool funcSetVolume_init()
{
	dab.STREAM_SetVolume_Async(nLastVolume, ms );
	return true;
}


bool funcSetNotifications_init()
{
	dab.NOTIFY_SetNotification_Async(   (DABNotification)((uint16_t)DABNotification::ScanFinished |
										(uint16_t)DABNotification::NewFMText |
										(uint16_t)DABNotification::DABReconfig |
										(uint16_t)DABNotification::DABChanChange |
										(uint16_t)DABNotification::FMRDSGrpNotify |
										(uint16_t)DABNotification::NewDABText |
										(uint16_t)DABNotification::ScanFrequency), ms );
	return true;
}

static TaskReturn ack_check(AsyncReturnCode ret) 
{
	if ( ret == AsyncReturnCode::ReplAck )
		return TaskReturn::Done;
	else
		return TaskReturn::Fail;
} 

bool funcEnableSyncClock_init() 
{
	dab.RTC_EnableSyncClock_Async(1, ms );
	return true;
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

bool funcGetSyncClockStatus_init()
{
	dab.RTC_GetSyncClockStatus_Async( ms );
	return true;
}

static TaskReturn funcGetClockStatus_check(AsyncReturnCode ret)
{
	bool bEnabled = false;
	if ( ret == AsyncReturnCode::ReplAck )
	{
		bEnabled = dab.MsgBuffer()[0] != 0;
	}
	return bEnabled ? TaskReturn::Done : TaskReturn::Fail;
}

bool funcGetClockStatus_init()
{
	dab.RTC_GetClockStatus_Async( ms );
	return true;
}

// TODO - call this every 5 minutes or so to verify the time.
bool funcGetClock_init()
{
	dab.RTC_GetClock_Async(ms);
	return true;
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

		return TaskReturn::Done;
	}
	else
		return TaskReturn::Fail;	
}

bool funcGetProgramCount_init()
{
	dab.STREAM_GetTotalProgram_Async(ms);
	return true;
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

bool funcGetPrograms_init()
{
	bool bFullName = programs[nProgramIdx].sLongName == NULL ? true: false;
	dab.STREAM_GetProgrammeName_Async(nProgramIdx, bFullName, sProgramNames, 256, ms);
	return true;
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

		sProgramNames += dab.MsgLen();
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

bool funcPlayProgramForTime_init()
{
	dab.STREAM_Play_Async(DABPlayMode::DAB,nLastPlayedProgram,ms);
	return true;
}

bool funcPlayProgram_init()
{
	const char *sProgram = programs[nLastPlayedProgram].sLongName;

	terminal.Send("ProgramName: ");
	terminal.Send(sProgram);
	terminal.SendCRLF();
	//display.WriteText( &font_MSShell, 0, 0, sProgram );
	display.WriteText( &font_MSShell, 0, 0, "RSN Racing&Sport" );
	

	dab.STREAM_Play_Async(DABPlayMode::DAB,nLastPlayedProgram,ms);
	return true;
}


bool funcStopPlaying_init()
{
	dab.STREAM_Stop_Async(ms);
	return true;
}


bool funcPowerOff_init()
{
	dab.PowerOff();
	display.displayPowerOff();
	return true;
}

void InitStartUpTasks()
{
	memset( StartUpTaskList, 0, sizeof(StartUpTaskList) );
	nTaskCount = (uint8_t)StartUpTasks::TasksMax;
	pTasks = StartUpTaskList;
	task = 0;

	StartUpTaskList[(int)StartUpTasks::SetVolume0].init = funcSetVolume0_init; StartUpTaskList[(int)StartUpTasks::SetVolume0].check = ack_check;
	StartUpTaskList[(int)StartUpTasks::EnableSyncClock].init = funcEnableSyncClock_init; StartUpTaskList[(int)StartUpTasks::EnableSyncClock].check = ack_check;
	StartUpTaskList[(int)StartUpTasks::GetSyncClockStatus].init = funcGetSyncClockStatus_init; StartUpTaskList[(int)StartUpTasks::GetSyncClockStatus].check = funcGetSyncClockStatus_check;
	StartUpTaskList[(int)StartUpTasks::GetClockStatus].init = funcGetClockStatus_init; StartUpTaskList[(int)StartUpTasks::GetClockStatus].check = funcGetClockStatus_check;
	StartUpTaskList[(int)StartUpTasks::GetClock].init = funcGetClock_init; StartUpTaskList[(int)StartUpTasks::GetClock].check = funcGetClock_check;
	StartUpTaskList[(int)StartUpTasks::GetProgramCount].init = funcGetProgramCount_init; StartUpTaskList[(int)StartUpTasks::GetProgramCount].check = funcGetProgramCount_check;
	StartUpTaskList[(int)StartUpTasks::GetPrograms].init = funcGetPrograms_init; StartUpTaskList[(int)StartUpTasks::GetPrograms].check = funcGetPrograms_check;
	StartUpTaskList[(int)StartUpTasks::PlayProgramForTime].init = funcPlayProgramForTime_init; StartUpTaskList[(int)StartUpTasks::PlayProgramForTime].check = ack_check;
	StartUpTaskList[(int)StartUpTasks::StopPlaying].init = funcStopPlaying_init; StartUpTaskList[(int)StartUpTasks::StopPlaying].check = ack_check;
	StartUpTaskList[(int)StartUpTasks::PowerOff].init = funcPowerOff_init; //StartUpTaskList[PowerOff].check = NULL;
}

bool funcGetProgramText_init()
{
	dab.STREAM_GetProgrammeText_Async( sProgramNames, 256, ms);
	return true;
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
	memset( StartTaskList, 0, sizeof(StartTaskList) );
	nTaskCount = (uint8_t)StartTasks::TasksMax;
	pTasks = StartTaskList;
	task = 0;
	StartTaskList[(int)StartTasks::EnableSyncClock].init = funcEnableSyncClock_init; StartTaskList[(int)StartTasks::EnableSyncClock].check = ack_check;
	StartTaskList[(int)StartTasks::PlayProgram].init = funcPlayProgram_init; StartTaskList[(int)StartTasks::PlayProgram].check = ack_check;
	StartTaskList[(int)StartTasks::GetProgramText].init = funcGetProgramText_init; StartTaskList[(int)StartTasks::GetProgramText].check = funcGetProgramText_check;
	StartTaskList[(int)StartTasks::SetVolume].init = funcSetVolume_init; StartTaskList[(int)StartTasks::SetVolume].check = ack_check;
	StartTaskList[(int)StartTasks::SetNotifications].init = funcSetNotifications_init; StartTaskList[(int)StartTasks::SetNotifications].check = ack_check;
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
	memset( GetProgramTextTaskList, 0, sizeof(GetProgramTextTaskList) );
	nTaskCount = (uint8_t)GetProgramTextTasks::TasksMax;
	pTasks = GetProgramTextTaskList;
	task = 0;

//	GetProgramTextTaskList[(int)GetProgramTextTasks::GetProgramText].init = funcGetProgramText_init; GetProgramTextTaskList[(int)GetProgramTextTasks::GetProgramText].check = funcGetProgramText_check;
	GetProgramTextTaskList[(int)GetProgramTextTasks::GetProgramText].init = funcGetProgramText_init; GetProgramTextTaskList[(int)GetProgramTextTasks::GetProgramText].check = funcGetProgramTextNotify_check;
}

static bool funcGetPlayStatus_init() 
{ 
	dab.STREAM_GetPlayStatus_Async(ms); return true; 
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
static bool funcGetPlayMode_init() 
{	
	dab.STREAM_GetPlayMode_Async(ms); return true; 
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
static bool funcGetPlayIndex_init() 
{ 
	dab.STREAM_GetPlayIndex_Async(ms); return true; 
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
static bool funcGetSignalStrength_init() 
{ 
	dab.STREAM_GetSignalStrength_Async(ms); return true; 
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


void InitPollTasks()
{
	memset( PollTaskList, 0, sizeof(PollTaskList) );
	nTaskCount = (uint8_t)PollTasks::TasksMax;
	pTasks = PollTaskList;
	task = 0;

	PollTaskList[(int)PollTasks::GetPlayStatus].init = funcGetPlayStatus_init; PollTaskList[(int)PollTasks::GetPlayStatus].check = funcGetPlayStatus_check;
	PollTaskList[(int)PollTasks::GetPlayMode].init = funcGetPlayMode_init; PollTaskList[(int)PollTasks::GetPlayMode].check = funcGetPlayMode_check;
	PollTaskList[(int)PollTasks::GetPlayIndex].init = funcGetPlayIndex_init; PollTaskList[(int)PollTasks::GetPlayIndex].check = funcGetPlayIndex_check;
	PollTaskList[(int)PollTasks::GetSignalStrength].init = funcGetSignalStrength_init; PollTaskList[(int)PollTasks::GetSignalStrength].check = funcGetSignalStrength_check;
	PollTaskList[(int)PollTasks::GetProgrameText].init = funcGetProgramText_init; PollTaskList[(int)PollTasks::GetProgrameText].check = funcGetProgramText_check;
}

int8_t NextTask(bool bPassed)
{
	if ( bPassed )
		pTasks[task].init = NULL;
		
	for ( uint8_t i = 0; i < nTaskCount; i++ )
	{
		task++;
		if ( task >= nTaskCount )
			task = 0;
		if ( pTasks[task].init != NULL )
			return task;
	}
	return -1;
}

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
				// Next task
				if ( pTasks[task].init() )
				{
					if ( pTasks[task].check != NULL )
						radioState = RadioState::WaitForTaskReply;
					else
					{
						task = NextTask(true);
						if ( task < 0 )
							radioState = taskDoneState;	// Done.
					}
				}
				break;
				
			case RadioState::WaitForTaskReply:
			if ( ret != AsyncReturnCode::OK )
			{
				TaskReturn tret = TaskReturn::Done;
				if ( pTasks[task].check != NULL )
					tret = pTasks[task].check(ret);
				if ( tret == TaskReturn::Done )
				{
					task = NextTask(true);
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

			case RadioState::Playing:
				if ( ret == AsyncReturnCode::Notification )
				{
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
					static uint32_t pollTimer = 0;
					if ( (int32_t)pollTimer - (int32_t)ms <= 0 )
					{
						InitPollTasks();
						radioState = RadioState::StartUpTasks;
						taskDoneState = RadioState::Playing;
						pollTimer = ms + 15000;
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



static void RadioPlay()
{
	if ( radioState == RadioState::Off )
	{
		dab.AsyncStart();
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

static void DrawProgramText(bool bRedraw)
{
	uint8_t len = strlen(sProgramNames);
	if ( len > OP_SCREENW / (1+font_6x13.cols) )
	{
		len = OP_SCREENW / (1+font_6x13.cols);
		sProgramNames[len] = 0;
	}
	uint16_t plen = len * (1+font_6x13.cols);
	if ( plen <= OP_SCREENW && bRedraw )
	{
		display.WriteText( &font_6x13, (OP_SCREENW - plen)/2, PROGRAM_TEXT_POS, sProgramNames);
	}
	else
	{
		
	}
}

static void ScrollText(bool bRedraw)
{
	static uint8_t nShiftCount = 0;
	static uint8_t last_ms8 = 0;
	uint8_t ms8 = *(uint8_t *)&ms;

	if ( bRedraw )
	{
		nShiftCount = font_6x13.rows+1;
	}
	if ( ms8 != last_ms8 )
	{
		if ( nShiftCount && (ms8 & 0x3f) == 0 )
		{
			uint16_t xs =0;
			uint8_t ys = PROGRAM_TEXT_POS-1+font_6x13.rows-(nShiftCount-1);
			uint16_t xe = OP_SCREENW-1;
			uint8_t ye = PROGRAM_TEXT_POS+font_6x13.rows;
			uint16_t xd=0;
			uint8_t yd = ys+1;
			//terminal.Send("Copy ");
			//terminal.Send(xs); terminal.Send(",");
			//terminal.Send(ys); terminal.Send(" ");
			//terminal.Send(xe); terminal.Send(",");
			//terminal.Send(ye); terminal.Send(" ");
			//terminal.Send(xd); terminal.Send(",");
			//terminal.Send(yd); terminal.SendCRLF();

			display.Copy( xs,ys, xe,ye, xd,yd );
			nShiftCount--;
			if ( nShiftCount == 0 )
			{
				// todo don't do thsi as an array.
				_delay_ms(1);
				display.ClearWindow( xd,yd, xe,yd+1 );
				DrawProgramText(true);
			}
		}
		else if ( ms8 == 0 )
		{
			DrawProgramText(bRedraw);
		}
		last_ms8 = ms8;
	}

}

static void DoClockRadio(uint16_t key_changes)
{
	switch ( clockRadioState )
	{
		case ClockRadioState::Off:
			if ( key_changes & keydown & BTN_RADIO_ON ) 
			{
				RadioPlay();
			}
			break;

		case ClockRadioState::Idle:
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
			else
			{
				ScrollText();
			}
			break;
	}
}

void Spinner()
{
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
	for ( int8_t i = 0; i < countof(pos); i++ )
	{
		display.WriteText( &font_6x13, pos[i].x, pos[i].y, "\x7f", c );
		c += 0x3F/12;
		c = c & 0x3f;
	}
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
				//if ( clock_secs % 60 == 0 )
				//{
					//if ( (clock_secs / 60) & 1 )
						//display.displayPowerOff();
					//else
						//display.InitDisplay();
				//}
				
				//if ( clock_secs % 30 == 0 )
				//{
					//if ( radioState == RadioState::Idle && dab.isIdle() )
					//{
						//dab.RTC_EnableSyncClock(true);
						//dab.STREAM_Play( DABPlayMode::DAB, 4 );
//
						//PrintDABDetails();								
					//}
				//}
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

		static uint16_t nLastEncoder = -1;
		uint16_t nEncoder = TCC1.CNT;
		if ( nEncoder != nLastEncoder )
		{
			nLastEncoder = nEncoder;
			terminal.Send( "Encoder: ");
			terminal.Send( (long)nEncoder );
			terminal.SendCRLF();
		}

		//if ( ms % 50 == 0)
		//{
			//static uint8_t i = 0;
			//uint16_t i0 = i>>1;
			//uint16_t c = i0 | (i << 5) | (i0 << 11);
			//display.sendData(c >> 8);
			//display.sendData(c & 0xFF);
			//i+=2;
			//if ( i > 0x3F )
				//i = 0;
		//}

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

			//display.WriteText( &font_MSShell, 0, 0, "RSN Racing&Sport" );
			//display.WriteText( &font_6x13, 287-90, 0, "Mon 10 Oct 2016" );
			//display.WriteText( &font_6x13, 287-30, 12, "Line2" );
			//display.WriteText( &font_6x13, 0, 24, "Line3" );
			//display.WriteText( &font_6x13, 143-15, 24, "Line3" );
			//display.WriteText( &font_6x13, 287-30, 24, "Line3" );
			//display.WriteText( &font_6x13, 287-30, 36, "Line4" );
			//for (;;);
		}
		//if ( !dab.isIdle() )
		//{
			//static uint8_t i = 189;
			//if ( i != 0 )
			//{
				//display.WriteText( &font_6x13, i-1, 0, "Initialising..." );
				//i--;
			//}
		//}


		DoClockRadio(key_changes);
		DoDAB();
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

	- program text side scrolling

	- ldr for autobrightness.

	- volume - analog in 

	-- Menu - when off, encoder scrolls menu.  Use alarm1/2 buttons?
		-- alarm(s)
			- mon/tue/wed/thr/fri/sat/sun
				- time
				- repeating - one off.
		-- rescan DAB
			-- factory reset?
		-- different display layouts
		-- other?

	-- if program count = 0 dab scan (or just menu?)
*/
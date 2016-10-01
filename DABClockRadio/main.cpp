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

#include "..\images\font.h"

#include "time.h"
#include "ic2_7seg.h"
#include "serial.h"
#include "pictiva.h"
#include "monkeyboarddab.h"

//#include "..\..\images\consolas.h"



#define SLAVE_ADDR (0x70<<1)

static SevenSeg< CSerialTxn< CSerialTxnImpl< CMegaI2CE, 64, 8> >, SLAVE_ADDR> sevenSeg_I2C;
IMPLEMENT_SERIAL_TXN_INTERRUPTS(E,sevenSeg_I2C)
static CMonkeyBoardDAB<CSerial< CSerialImpl<CMegaSerialE0,64,64> > > dab;
IMPLEMENT_SERIAL_INTERRUPTS( E0, dab )
static Pictiva display;

//SevenSeg< CSerialTxn< CSerialTxnPolledImpl< CMegaI2CE > >, SLAVE_ADDR> sevenSeg_I2C;

static CSerial< CSerialImpl<CMegaSerialC0,64,64> > terminal;
IMPLEMENT_SERIAL_INTERRUPTS( C0, terminal )

static uint32_t clock_secs = 0;
static bool bClockInitialised=false;
static uint16_t nLastPlayedProgram = 4;
static uint32_t radioTimer;

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


static void PollKeyboard()
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

	terminal.SendHex((int)keys);
	terminal.SendCRLF();
}


static void PrintDABDetails()
{
	bool bClockSet = false;
	{
		if ( dab.RTC_GetClockStatus(bClockSet))
		{
			terminal.Send("Clock Status: ");
			terminal.Send(bClockSet?"Set\r\n":"Unset\r\n");
		}
		else
		{
			terminal.Send("Failed to get clock status\r\n");
		}
	}

	if ( bClockSet )
	{
		uint8_t day, month, hour, minute, second;
		uint16_t year;
		if ( dab.RTC_GetClock(day,month,year,hour,minute,second))
		{
			terminal.Send("Clock: ");
			terminal.Send( day );
			terminal.Send( '/' );
			terminal.Send( month );
			terminal.Send( '/' );
			terminal.Send( (int)year );
			terminal.Send( ' ' );
			terminal.Send( hour );
			terminal.Send( ':' );
			terminal.Send( minute );
			terminal.Send( ':' );
			terminal.Send( second );
			terminal.SendCRLF();

			struct tm t;
			t.tm_year = year-1900;
			t.tm_mon = month;
			t.tm_mday = day;
			t.tm_hour = hour;
			t.tm_min = minute;
			t.tm_sec = second;

			UpdateTime( t );
		}
		else
		{
			terminal.Send("Failed to read clock\r\n");
		}
	}

	{
		uint16_t nPrograms;
		if ( dab.STREAM_GetTotalProgram(nPrograms) )
		{
			terminal.Send( "Found ");
			terminal.Send( (int)nPrograms );
			terminal.Send( " DAB programs\r\n" );
		}
		else
		{
			terminal.Send( "Error reading DAB programs\r\n" );
		}
	}

	DABPlayStatus nPlayStatus = DABPlayStatus::Stop;
	{
		if ( dab.STREAM_GetPlayStatus(nPlayStatus) )
		{
			terminal.Send( "DAB Play Status = ");
			switch (nPlayStatus)
			{
				case DABPlayStatus::Playing: terminal.Send("Playing\r\n"); break;
				case DABPlayStatus::Searching: terminal.Send("Searching\r\n"); break;
				case DABPlayStatus::Tuning: terminal.Send("Tuning\r\n"); break;
				case DABPlayStatus::Stop: terminal.Send("Stop\r\n"); break;
				case DABPlayStatus::SortingChange: terminal.Send("Sorting Change\r\n"); break;
				case DABPlayStatus::Reconfiguration: terminal.Send("Reconfiguration\r\n"); break;
				default: terminal.Send((int)nPlayStatus); terminal.SendCRLF(); break;
			}
		}
		else
		{
			terminal.Send( "Error reading DAB play status\r\n" );
		}
	}

	{
		DABPlayMode nPlayMode;
		if ( dab.STREAM_GetPlayMode(nPlayMode) )
		{
			terminal.Send( "DAB Play Mode = ");
			switch (nPlayMode)
			{
				case DABPlayMode::AM: terminal.Send("AM\r\n"); break;
				case DABPlayMode::BEEPER: terminal.Send("BEEPER\r\n"); break;
				case DABPlayMode::DAB: terminal.Send("DAB\r\n"); break;
				case DABPlayMode::FM: terminal.Send("FM\r\n"); break;
				case DABPlayMode::StreamStop: terminal.Send("Stream Stop\r\n"); break;
				default: terminal.Send((int)nPlayMode); terminal.SendCRLF(); break;
			}
		}
		else
		{
			terminal.Send( "Error reading DAB play status\r\n" );
		}
	}

	{
		uint32_t nIndex;
		if ( dab.STREAM_GetPlayIndex(nIndex ) )
		{
			terminal.Send( "Play Index=" );
			terminal.Send( (long)nIndex );
			terminal.Send( "\r\n" );
		}
		else
		{
			terminal.Send( "Failed to read play index\r\n" );
		}
	}

	if ( nPlayStatus != DABPlayStatus::Stop )
	{
		uint8_t nStrength; uint16_t nBitErrorRate;
		if ( dab.STREAM_GetSignalStrength( nStrength, nBitErrorRate) )
		{
			terminal.Send( "Strength=" );
			terminal.Send( (int)nStrength );
			terminal.Send( ", Bit Error Rate=" );
			terminal.Send( (int)nBitErrorRate );
			terminal.Send( "\r\n" );
		}
		else
		{
			terminal.Send( "Failed to read signal strength\r\n" );
		}
	}
	if ( nPlayStatus == DABPlayStatus::Playing )
	{
		char text[200];
		if ( dab.STREAM_GetProgrammeText(text,sizeof(text)) )
		{
			terminal.Send("Program Text: ");
			terminal.Send(text);
			terminal.SendCRLF();
		}
		else
		{
			terminal.Send("Failed to read Program Text\r\n");
		}
	}

	{
		bool bEnabled;
		if ( dab.RTC_GetSyncClockStatus(bEnabled))
		{
			terminal.Send("Clock Sync Status: ");
			terminal.Send(bEnabled?"Enabled\r\n":"Disabled\r\n");
		}
		else
		{
			terminal.Send("Failed to get clock sync status\r\n");
		}
	}
}

static void PrintDABStations()
{
	uint16_t nPrograms;
	if ( dab.STREAM_GetTotalProgram(nPrograms) )
	{
		terminal.Send( "Found ");
		terminal.Send( (int)nPrograms );
		terminal.Send( " DAB programs\r\n" );
	}
	else
	{
		terminal.Send( "Error reading DAB programs\r\n" );
	}

	for ( uint16_t i = 0; i < nPrograms; i++ )
	{
		char buf[120];
		if ( dab.STREAM_GetProgrammeName( i, false, buf, sizeof(buf) ) )
		{
			terminal.Send( (int)i );
			if ( i < 10 )
			terminal.Send(' ');
			terminal.Send(' ');
			terminal.Send( buf );
			for ( unsigned int j = 0; j < 9 - strlen(buf); j++)
			terminal.Send(' ');

			if ( dab.STREAM_GetProgrammeName( i, true, buf, sizeof(buf) ) )
			{
				terminal.Send( buf );
				terminal.SendCRLF();
			}
		}
	}
}


static enum class RadioState: uint8_t
{
	Init,
	WaitForReply,
	Idle,
	InitStartUpTasks,
	StartUpTasks,
	StartUpTasksPause,
	WaitForTaskReply
} radioState = RadioState::Init, successState, failState;
static int8_t task;

void SetWaitForReply( RadioState success, RadioState failure)
{
	radioState = RadioState::WaitForReply;
	successState = success;
	failState = failure;
}

enum StartUpTasks
{
	SetVolume0 = 0,
	EnableSyncClock,
	PlayProgramForTime,
	GetProgramCount,
	GetPrograms,
	GetSyncClockStatus,
	GetClockStatus,
	GetClock,
	StopPlaying,
	PowerOff,
	StartUpTasksMax
};

struct StartupTaskFuncs
{
	bool (*init)();
	bool (*check)(AsyncReturnCode ret);

} StartUpTaskList[StartUpTasksMax];

bool funcSetVolume0_init() 
{
	dab.STREAM_SetVolume_Async(0, ms );
	return true;
} 

bool ack_check(AsyncReturnCode ret) 
{
	if ( ret == AsyncReturnCode::ReplAck )
		return true;
	else
		return false;
} 

bool funcEnableSyncClock_init() 
{
	dab.RTC_EnableSyncClock_Async(1, ms );
	return true;
}

bool funcGetSyncClockStatus_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		bool bSet = dab.MsgBuffer()[0] != 0;
		return bSet;
	}
	else
	return false;
}

bool funcGetSyncClockStatus_init()
{
	dab.RTC_GetSyncClockStatus_Async( ms );
	return true;
}

bool funcGetClockStatus_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		bool bEnabled = dab.MsgBuffer()[0] != 0;
		return bEnabled;
	}
	else
		return false;
}

bool funcGetClockStatus_init()
{
	dab.RTC_GetClockStatus_Async( ms );
	return true;
}


bool funcGetClock_init()
{
	dab.RTC_GetClock_Async(ms);
	return true;
}

bool funcGetClock_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		uint8_t *input = dab.MsgBuffer();

		struct tm t;
		t.tm_year = input[6] + 2000 - 1900;
		t.tm_mon = input[5];
		t.tm_mday = input[3];
		t.tm_hour = input[2];
		t.tm_min = input[1];
		t.tm_sec = input[0];

		UpdateTime( t );

		return true;
	}
	else
		return false;	
}

static uint16_t nPrograms;
static uint16_t nProgramIdx;

bool funcGetProgramCount_init()
{
	dab.STREAM_GetTotalProgram_Async(ms);
	return true;
}

bool funcGetProgramCount_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		nPrograms =  dab.MsgBuffer()[3] | ( dab.MsgBuffer()[2]<<8);	// use only the 2 low bytes.
		nProgramIdx = 0;
		return true;
	}
	else
		return false;
}

char namebuf[20];
bool funcGetPrograms_init()
{
	dab.STREAM_GetProgrammeName_Async(nProgramIdx, true, namebuf,sizeof(namebuf),ms);
	return true;
}

bool funcGetPrograms_check(AsyncReturnCode ret)
{
	if ( ret == AsyncReturnCode::ReplAck )
	{
		for ( uint16_t i = 1; i < dab.MsgLen(); i+=2 )
			terminal.Send( namebuf[i] );
		terminal.SendCRLF();

		nProgramIdx++;
		if ( nProgramIdx < nPrograms )
			return false;
		else
			return true;
	}
	else
		return false;
}

bool funcPlayProgramForTime_init()
{
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
	return true;
}

void InitStartUpTasks()
{
	memset( StartUpTaskList, 0, sizeof(StartUpTaskList) );
	StartUpTaskList[SetVolume0].init = funcSetVolume0_init; StartUpTaskList[SetVolume0].check = ack_check;
	StartUpTaskList[EnableSyncClock].init = funcEnableSyncClock_init; StartUpTaskList[EnableSyncClock].check = ack_check;
	StartUpTaskList[GetSyncClockStatus].init = funcGetSyncClockStatus_init; StartUpTaskList[GetSyncClockStatus].check = funcGetSyncClockStatus_check;
	StartUpTaskList[GetClockStatus].init = funcGetClockStatus_init; StartUpTaskList[GetClockStatus].check = funcGetClockStatus_check;
	StartUpTaskList[GetClock].init = funcGetClock_init; StartUpTaskList[GetClock].check = funcGetClock_check;
	StartUpTaskList[GetProgramCount].init = funcGetProgramCount_init; StartUpTaskList[GetProgramCount].check = funcGetProgramCount_check;
	StartUpTaskList[GetPrograms].init = funcGetPrograms_init; StartUpTaskList[GetPrograms].check = funcGetPrograms_check;
	StartUpTaskList[PlayProgramForTime].init = funcPlayProgramForTime_init; StartUpTaskList[PlayProgramForTime].check = ack_check;
	StartUpTaskList[StopPlaying].init = funcStopPlaying_init; StartUpTaskList[StopPlaying].check = ack_check;
	StartUpTaskList[PowerOff].init = funcPowerOff_init; //StartUpTaskList[PowerOff].check = NULL;
}

int8_t NextTask(bool bPassed)
{
	if ( bPassed )
		StartUpTaskList[task].init = NULL;

	for ( uint8_t i = 0; i < StartUpTasksMax; i++ )
	{
		task++;
		if ( task >= StartUpTasksMax )
			task = 0;
		if ( StartUpTaskList[task].init != NULL )
			return task;
	}
	return -1;
}

void DoDAB()
{
	AsyncReturnCode ret = dab.Async( ms );
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
				task = 0;
				radioState = RadioState::StartUpTasks;
				// fall through

			case RadioState::StartUpTasks:
				// Next task
				if ( StartUpTaskList[task].init() )
					radioState = RadioState::WaitForTaskReply;
				break;
				
			case RadioState::WaitForTaskReply:
			{
				if ( StartUpTaskList[task].check == NULL || StartUpTaskList[task].check(ret) )
				{
					task = NextTask(true);
					if ( task < 0 )
						radioState = RadioState::Idle;	// Done.
					else
						radioState = RadioState::StartUpTasks;
				}
				else
				{
					radioState = RadioState::StartUpTasksPause;
					radioTimer = ms + 500;
				}
				break;
			}

			case RadioState::StartUpTasksPause:
				if ( (int32_t)(ms - radioTimer) <= 0 )
					radioState = RadioState::StartUpTasks;
				break;

			case RadioState::Idle:
				break;
		}
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

	//for ( uint16_t i=0; i < img_size; i++ )
		//display.sendData( pgm_read_byte(img+i) );

	display.SetColumn(0);
	display.SetRow(0);

	//display.sendData(0);
	//display.sendData(0);
	//display.sendData(0);
	//display.SetRow(0,10-1);
	//display.SetColumn(0,3-1);
	//for ( int i = 0; i < 10; i++)
	//for ( int j = 0; j < 3; j++)
	//{
		//uint16_t n = i%3 == 0 ? 0x8000 : i%3 == 1 ? 0x0400 : 0x0010;
		////uint16_t n = 0x0400;
		//display.sendData(n >> 8);
		//display.sendData(n & 0xFF);
	//}
	//display.sendData(0xFF);

//	display.displayPowerOff();
//	display.WriteText( &font_6x13, 3, 1, "|" );
	display.WriteText( &font_6x13, 0, 0, "Initialising..." );
	display.WriteText( &font_6x13, 1, 11, "Initialising..." );
	for ( int i = 188; i >= 0; i--)
		display.WriteText( &font_6x13, i, 22, "Initialising..." );


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
				PollKeyboard();
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

		DoDAB();
    }
}



/*

Start up
- EnableSyncClock.
- Read programs
- Set clock time
	- appears you can only get time when you are playing
	- Play a channel
	- Get time.
	- stop playing when we've got time.  We can abort this last step if we start playing.

*/
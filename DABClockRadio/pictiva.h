#pragma once

// Pins
// SCLK - PC7 - SPIC
// MOSI	- PC5 - SPIC
// D/~C	- PD1
// ~RST	- PD0
// ~SS	- PC4

#define	PICTIVA_SPI			SPIC
#define	PICTIVA_D_C			PIN1_bm
#define	PICTIVA_D_C_PORT	PORTD
#define	PICTIVA_RST			PIN0_bm
#define	PICTIVA_RST_PORT	PORTD
#define	PICTIVA_SS			PIN4_bm
#define	PICTIVA_SS_PORT		PORTC


// Code initially from https://github.com/sumotoy/OLED_pictivaWide

const static uint8_t _CMD_SETCOLUMN = 		0x15;//ok
const static uint8_t _CMD_SETROW =    		0x75;//ok
const static uint8_t _CMD_SETCRTSA =    	0x81;//set contrast color A
const static uint8_t _CMD_SETCRTSB =    	0x82;//set contrast color B
const static uint8_t _CMD_SETCRTSC =    	0x83;//set contrast color C
const static uint8_t _CMD_MASTCURR =    	0x87;//master current control
const static uint8_t _CMD_SETREMAP = 		0xA0;//ok
const static uint8_t _CMD_STARTLINE = 		0xA1;//ok
const static uint8_t _CMD_VERTSCROLL = 		0xA2;//ok
const static uint8_t _CMD_NORMALDISPLAY = 	0xA4;//ok
const static uint8_t _CMD_DISPLAYALLON =  	0xA5;//ok
const static uint8_t _CMD_DISPLAYALLOFF = 	0xA6;//ok
const static uint8_t _CMD_INVERTDISPLAY = 	0xA7;//ok
const static uint8_t _CMD_DISPLAYOFFSET = 	0xA2;//ok

const static uint8_t _CMD_MUXRATIO =       	0xA8;//changed adrs
const static uint8_t _CMD_MASTCONF = 		0xAD;//master configuration
const static uint8_t _CMD_DISPLAYOFF = 		0xAE;//ok
const static uint8_t _CMD_DISPLAYON =     	0xAF;//ok
const static uint8_t _CMD_POWSAVE =     	0xB0;//set power save
const static uint8_t _CMD_PHASEADJ =     	0xB1;//phase 1,2 period adj
const static uint8_t _CMD_CLOCKDIV = 		0xB3;//ok
const static uint8_t _CMD_SETGRAY = 		0xB8;//ok
const static uint8_t _CMD_LINGRYTABLE = 	0xB9;//enable linear gray scale table
const static uint8_t _CMD_VPALEVELA = 		0xBB;//level for color A
const static uint8_t _CMD_VPALEVELB = 		0xBC;//level for color B
const static uint8_t _CMD_VPALEVELC = 		0xBD;//level for color C
const static uint8_t _CMD_VCOMH = 			0xBE;//ok
const static uint8_t _CMD_NOP = 			0xE3;//ok
//graphic commands
const static uint8_t _CMD_DRAW_LINE = 		0x21;//
const static uint8_t _CMD_DRAW_RECT = 		0x22;//
const static uint8_t _CMD_DRAW_COPY = 		0x23;//
const static uint8_t _CMD_DRAW_DIMW = 		0x24;//
const static uint8_t _CMD_DRAW_CLRW = 		0x25;//
const static uint8_t _CMD_FILL_MODE = 		0x26;//
#define OP_SCREENW 	288
#define OP_SCREENH 	48
#define OP_HDWMAXW 	95
#define OP_HDWMAXH 	47
#define OP_HRDWMUX	48//63
#define OP_OFFSET 	-1//0x30

class Pictiva
{
private:
	void _send( uint8_t b )
	{
		PICTIVA_SPI.STATUS |= SPI_IF_bm;
		uint8_t t = PICTIVA_SPI.DATA;

		PICTIVA_SPI.DATA = b;
		while ( (PICTIVA_SPI.STATUS & SPI_IF_bm) == 0 )
			continue;
	}

	void send( uint8_t cmd, bool isCmd = false ) 
	{
		if (isCmd)
		{
			PICTIVA_D_C_PORT.OUTCLR = PICTIVA_D_C;
		}
		else
		{
			PICTIVA_D_C_PORT.OUTSET = PICTIVA_D_C;
		}

		_send( cmd );
	}

public:
	enum class DispMode {ON=0,OFF,NORMAL,INVERTED,ALL_ON,ALL_OFF };

	void Init()
	{
		PORTC.DIRSET = PIN7_bm;		// CLK
		PORTC.DIRSET = PIN5_bm;		// MOSI
		PICTIVA_D_C_PORT.DIRSET = PICTIVA_D_C;		// D/C#
		PICTIVA_RST_PORT.DIRSET = PICTIVA_RST;		// RES#
		PICTIVA_SS_PORT.DIRSET = PICTIVA_SS;		// CS#

		PICTIVA_RST_PORT.OUTCLR = PICTIVA_RST;
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;

		PR.PRPC &= ~PR_SPI_bm;	// enable SPIC

		PICTIVA_SPI.INTCTRL = 0;
		PICTIVA_SPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_CLK2X_bm;
	}


	void InitDisplay()
	{
		PICTIVA_RST_PORT.OUTCLR = PICTIVA_RST;
		_delay_ms(50);
		PICTIVA_RST_PORT.OUTSET = PICTIVA_RST;
		_delay_ms(500);

		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		setContrast(0xff);
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		setBrightness(4);
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;


		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send(_CMD_SETREMAP,true);
		send(0b01110010,true);//0b00110011
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;

		
		//A[0]=0, Horizontal address increment (POR)
		//A[0]=1, Vertical address increment
		//A[1]=0, Column address 0 is mapped to SEG0 (POR)
		//A[1]=1, Column address 95 is mapped to SEG0
		//A[4]=0, Scan from COM 0 to COM [N �1]
		//A[4]=1, Scan from COM [N-1] to COM0. Where N is the Multiplex ratio.
		//A[5]=0, Disable COM Split Odd Even (POR)
		//A[5]=1, Enable COM Split Odd Even
		//A[7:6]=00; 256 color format = 01; 65k color format(POR)
			//0:Vertical address increment			ok
			//1:Column address 95 is mapped to SEG0	ok
			//2:nc
			//3:nc
			//4:Scan from COM [N-1] to COM0. Where N is the Multiplex ratio.
			//5:Disable COM Split Odd Even (POR)
			//6.7:65k color format
			//
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send(_CMD_DISPLAYOFFSET,true);
		send(OP_OFFSET,true);
		//_opw_setAddress(0,63,0,95);

		send(_CMD_MUXRATIO,true); 		send(OP_HRDWMUX,true);//63

		send(_CMD_MASTCONF,true); 		send(142,true);

		clearScreen();
	
		displayMode(DispMode::ON);
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
		_delay_ms(100);
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		displayMode(DispMode::NORMAL);
		//setFont(&arial_x2);
		//setCursor(0,0);

		send( _CMD_SETCOLUMN, true );
		send( 0, true );
		send( 95, true );
		send( _CMD_SETROW, true );
		send( 0, true );
		send( 47, true );

		//for ( int i = 0; i < 10000; i++)
		//{
			//send(0xff);		send(0xff);
			//send(0xff);		send(0xff);
			//send(0xff);		send(0xff);
		//}
	}

	//OK 0...255
	void setContrast(uint8_t contrast)
	{
		send(_CMD_SETCRTSA,true);
		send(contrast,true);
		send(_CMD_SETCRTSB,true);
		send(contrast,true);
		send(_CMD_SETCRTSC,true);
		send(contrast,true);
	}

	//ok (0x0...0x15)(0..21)
	void setBrightness(uint8_t val)
	{
		if (val > 15) val = 15;
		send(_CMD_MASTCURR,true);
		send(val,true);//1000xxxx
	}

	void clearScreen(void)
	{
		send(_CMD_DRAW_CLRW,true);
		send(0,true);
		send(0,true);
		send(OP_HDWMAXW,true);
		send(63,true);
		
		_delay_us(400);//increase if there's image flickering
		//undocumented bug, after this command the SSD0332 needs some time to stabilize
	}

	void displayPowerOff()
	{
		PICTIVA_RST_PORT.OUTCLR = PICTIVA_RST; 
	}

	void displayMode(enum DispMode m) 
	{
		switch(m)
		{
			case DispMode::ON:
				send(_CMD_DISPLAYON,true);
				break;
			case DispMode::OFF:
				send(_CMD_DISPLAYOFF,true);
				break;
			case DispMode::NORMAL:
			default:
				send(_CMD_NORMALDISPLAY,true);
				break;
			case DispMode::INVERTED:
				send(_CMD_INVERTDISPLAY,true);
				break;
			case DispMode::ALL_ON:
				send(_CMD_DISPLAYALLON,true);
				break;
			case DispMode::ALL_OFF:
				send(_CMD_DISPLAYALLOFF,true);
				break;
		}
	}
	void sendData(uint8_t b)
	{
		send(b);
	}
	void SetColumn(uint8_t c)
	{
		send( _CMD_SETCOLUMN, true );
		send( 0, true );
		send( 95, true );
		
	}
	void SetRow(uint8_t r)
	{
		send( _CMD_SETROW, true );
		send( 0, true );
		send( 47, true );

	}
};
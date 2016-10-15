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
//		PICTIVA_SPI.STATUS |= SPI_IF_bm;
//		uint8_t t = PICTIVA_SPI.DATA;

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

	void _sendData( uint16_t cmd )
	{
		PICTIVA_D_C_PORT.OUTSET = PICTIVA_D_C;

		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		_send( cmd >> 8 );
		_send( cmd & 0xFF );
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
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
		//A[4]=0, Scan from COM 0 to COM [N –1]
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

		clearScreen(true);
	
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
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;

		send(_CMD_SETCRTSA,true);
		send(contrast,true);
		send(_CMD_SETCRTSB,true);
		send(contrast,true);
		send(_CMD_SETCRTSC,true);
		send(contrast,true);

		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}

	//ok (0x0...0x15)(0..21)
	void setBrightness(uint8_t val)
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;

		if (val > 15) val = 15;
		send(_CMD_MASTCURR,true);
		send(val,true);//1000xxxx

		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}

	void clearScreen(bool bWait=false)
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;

		send(_CMD_DRAW_CLRW,true);
		send(0,true);
		send(0,true);
		send(OP_HDWMAXW,true);
		send(63,true);

		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
		
		if ( bWait )
		{
			_delay_us(400); // Wait for the clearscreen to complete.
		}
	}

	void displayPowerOff()
	{
		PICTIVA_RST_PORT.OUTCLR = PICTIVA_RST; 
	}

	void displayMode(enum DispMode m) 
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;

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

		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}

	void sendData(uint16_t *buffer, uint16_t len)
	{
		PICTIVA_D_C_PORT.OUTSET = PICTIVA_D_C;
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		while ( len )
		{
			_send(*((uint8_t *)buffer+1));
			_send(*((uint8_t *)buffer));
			buffer++;
			len--;
		}
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}
	void sendData(uint8_t b)
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send(b);
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}
	void SetColumn(uint8_t start=0, uint8_t end = 95)
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send( _CMD_SETCOLUMN, true );
		send( start, true );
		send( end, true );
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}
	void SetRow(uint8_t start=0, uint8_t end=47)
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send( _CMD_SETROW, true );
		send( start, true );
		send( end, true );
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}
	void SetFillMode(bool bRectFill, bool bInvertCopy)
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send( _CMD_FILL_MODE, true );
		send( (bRectFill ? 0x1 : 0 ) | (bInvertCopy ? 0x10 : 0), true );
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;
	}


	void PutPixel( uint8_t c, uint8_t nPos )
	{
		static uint16_t pixel;
		switch ( nPos )
		{
			case 0:	
				pixel = (c>>1) << 11;
				break;
			case 1:
				pixel |= c << 5;
				break;
			case 2:
				pixel |= c>>1;
				_sendData(pixel);
				break;
		}
	}

	// Write a line of text.  No clipping or boundary checking.  Monospace.
	void WriteText( const FontStruct * font, int16_t x, int8_t y, const char *s, uint8_t colour = 0x3F )
	{
		// On the screen?
		if ( y + font->rows < 0 || y > OP_SCREENH )
			return;

		uint8_t slen = strlen(s);
		uint16_t plen = slen * (font->cols + 1);

		// Because each pixel is made up of 3 dots, we need to shift the physical start/end to 3 pixel boundaries.
		div_t start = div(x,3);
		div_t end = div(x+plen,3);

		// bounding rectangle
		int8_t sy = y;
		int8_t ey = y+font->rows-1;
		if ( sy < 0 )
			sy = 0;
		if ( ey >= OP_SCREENH )
			ey = OP_SCREENH - 1;
		SetRow( sy, ey );

		int16_t xe = end.quot + (end.rem ? 1 : 0) - 1;
		if ( xe >= OP_SCREENW/3 )
			xe = OP_SCREENW/3 - 1;
		int16_t xs = start.quot;
		if ( xs < 0 )
			xs = 0;
		SetColumn( xs, xe );

		// output the data into the block, row by row, letting the graphics controller shift lines.
		int16_t startx = (start.quot + (start.rem < 0 ? -1 : 0 ))*3;
		if ( start.rem < 0 )
			start.rem += 3;
		for ( uint8_t row = 0; row < font->rows; row++, y++ )
		{
			if ( y >=0 && y < OP_SCREENH )
			{
				const char *ptr = s;
				uint8_t p = 0;
				int16_t px = startx;

				// Leading pixels
				for ( ; p < start.rem; p++, px++ )
					if ( px >= 0 )
						PutPixel( 0, p );

				// text
				while ( *ptr && px < OP_SCREENW )
				{
					uint8_t bits = pgm_read_byte( font->data + (*ptr - font->first_char) * font->rows + row );
					for ( uint8_t c = 0; c < font->cols && px < OP_SCREENW; c++ )
					{
						if ( px >= 0 )
							PutPixel( (bits & _BV(7-c)) ? colour : 0, p );
						p++;
						px++;
						if (p==3) p = 0;
					}
					if ( px < OP_SCREENW )
					{
						if ( px >= 0 )
							PutPixel( 0, p );
						px++;
						p++;
						if (p==3) p = 0;
					}
					ptr++;
				}

				// trailing pixels
				for ( int8_t i = end.rem; i >= 0 && px < OP_SCREENW; i-- )
				{
					if ( px >= 0 )
						PutPixel( 0, p );
					px++;
					p++;
				}
			}
		}
	}

	typedef void (*PutPixelFunc)( uint8_t c, uint8_t p);
	typedef void (*SetRowFunc)(uint8_t start, uint8_t end);
	typedef void (*SetColFunc)(uint8_t start, uint8_t end);

	// Write a line of text.  No clipping or boundary checking.  Monospace.
	template< PutPixelFunc funcPutPixel, SetRowFunc funcSetRow, SetColFunc funcSetCol, const int Width, const int Height> void WriteText( const FontStruct * font, int16_t x, int8_t y, const char *s, uint8_t colour = 0x3F )
	{
		// On the screen?
		if ( y + font->rows < 0 || y > Width )
			return;

		uint8_t slen = strlen(s);
		uint16_t plen = slen * (font->cols + 1);

		// Because each pixel is made up of 3 dots, we need to shift the physical start/end to 3 pixel boundaries.
		div_t start = div(x,3);
		div_t end = div(x+plen,3);

		// bounding rectangle
		int8_t sy = y;
		int8_t ey = y+font->rows-1;
		if ( sy < 0 )
			sy = 0;
		if ( ey >= Height )
			ey = Height - 1;
		funcSetRow( sy, ey );

		int16_t xe = end.quot + (end.rem ? 1 : 0) - 1;
		if ( xe >= Width/3 )
			xe = Width/3 - 1;
		int16_t xs = start.quot;
		if ( xs < 0 )
			xs = 0;
		funcSetCol( xs, xe );

		// output the data into the block, row by row, letting the graphics controller shift lines.
		int16_t startx = (start.quot + (start.rem < 0 ? -1 : 0 ))*3;
		if ( start.rem < 0 )
			start.rem += 3;
		for ( uint8_t row = 0; row < font->rows; row++, y++ )
		{
			if ( y >=0 && y < Height )
			{
				const char *ptr = s;
				uint8_t p = 0;
				int16_t px = startx;

				// Leading pixels
				for ( ; p < start.rem; p++, px++ )
				if ( px >= 0 )
					funcPutPixel( 0, p );

				// text
				while ( *ptr && px < Width )
				{
					uint8_t bits = pgm_read_byte( font->data + (*ptr - font->first_char) * font->rows + row );
					for ( uint8_t c = 0; c < font->cols && px < OP_SCREENW; c++ )
					{
						if ( px >= 0 )
							funcPutPixel( (bits & _BV(7-c)) ? colour : 0, p );
						p++;
						px++;
						if (p==3) p = 0;
					}
					if ( px < Width )
					{
						if ( px >= 0 )
							funcPutPixel( 0, p );
						px++;
						p++;
						if (p==3) p = 0;
					}
					ptr++;
				}

				// trailing pixels
				for ( int8_t i = end.rem; i >= 0 && px < Width; i-- )
				{
					if ( px >= 0 )
						funcPutPixel( 0, p );
					px++;
					p++;
				}
			}
		}
	}

	void WriteText( const PropFontStruct * font, uint16_t x, uint8_t y, const char *s, uint8_t colour = 0x3F )
	{
		uint8_t slen = strlen(s);

		// pixel length
		uint16_t plen = 0;
		for ( uint8_t i = 0; i < slen; i++ )
		{
			const PropCharStruct *p = font->chars;
			p += (s[i] - font->first_char);
			uint8_t width = pgm_read_byte( &(p->width) );
			plen += width + 1;
		}

		if ( x + plen > OP_SCREENW )
			plen = OP_SCREENW - x;

		// Because each pixel is made up of 3 dots, we need to shift the physical start/end to 3 pixel boundaries.
		div_t start = div(x,3);
		div_t end = div(x+plen,3);

		// bounding rectangle
		SetRow( y, y+font->rows-1 );
		SetColumn( start.quot, end.quot + (end.rem ? 1 : 0) - 1 );

		// output the data into the block, row by row, letting the graphics controller shift lines.
		for ( uint8_t row = 0; row < font->rows; row++ )
		{
			const char *ptr = s;
			uint8_t p = 0;
			uint16_t px = x;

			// Leading pixels
			for ( ; p < start.rem; p++ )
			{
				PutPixel( 0, p );
				px++;
			}

			// text
			while ( *ptr && px < OP_SCREENW )
			{
				const PropCharStruct *pchar = font->chars;
				pchar += (*ptr - font->first_char);
			 
				uint8_t width = pgm_read_byte( &(pchar->width) );
				const uint8_t * PROGMEM data = (const uint8_t PROGMEM *)pgm_read_ptr( &(pchar->data) );

				for ( uint8_t c = 0; c < width && px < OP_SCREENW; c++ )
				{
					uint8_t colour = pgm_read_byte( data + width * row + c );
					PutPixel( colour, p );
					px++;
					p++;
					if (p==3) p = 0;
				}
				if ( px < OP_SCREENW )
				{
					PutPixel( 0, p );
					px++;
					p++;
					if (p==3) p = 0;
				}
				ptr++;
			}

			// trailing pixels
			for ( int8_t i = end.rem; i >= 0 && px < OP_SCREENW; i-- )
			{
				PutPixel( 0, p );
				p++;
				px++;
			}
		}
	}

	void Copy( uint16_t xs, uint8_t ys, uint16_t xe,uint8_t ye,  uint16_t xd, uint8_t yd, bool bWait = false )
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send( _CMD_DRAW_COPY, true );
		send( xs / 3, true );
		send( ys, true );
		send( xe / 3, true );
		send( ye, true );
		send( xd / 3, true );
		send( yd, true );
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;

		if ( bWait )
		{
			// As there is not feedback from the SSD0332, we need to wait for the copy to finish.  The more pixels the long the wait.
			uint16_t w;
			w = (xe - xs)>>1;	// Actually 3/2 (3 pixels per 2 bytes) but this just an estimate.
			w *= (ye - ys);
			w /= 64;
			while ( w > 0 )
			{
				_delay_us(5);	// argument must be a constant so the delay routine is calculated at compile time.  Otherwise it does floating point math.	
				w--;
			}
		}
	}
	void ClearWindow( uint16_t xs, uint8_t ys, uint16_t xe,uint8_t ye, bool bWait = false )
	{
		PICTIVA_SS_PORT.OUTCLR = PICTIVA_SS;
		send( _CMD_DRAW_CLRW, true );
		send( xs / 3, true );
		send( ys, true );
		send( xe / 3, true );
		send( ye, true );
		PICTIVA_SS_PORT.OUTSET = PICTIVA_SS;

		if ( bWait )
		{
			// As there is not feedback from the SSD0332, we need to wait for the clear to finish.  The more pixels the long the wait.
			uint16_t w;
			w = (xe - xs)>>1;	// Actually 3/2 (3 pixels per 2 bytes) but this just an estimate.
			w *= (ye - ys);
			w /= 64;
			while ( w > 0 )
			{
				_delay_us(2);	// argument must be a constant so the delay routine is calculated at compile time.  Otherwise it does floating point math.
				w--;
			}
		}
	}

	void HLine( int16_t x1, int16_t x2, int8_t y, uint8_t c)
	{
		if ( y < 0 || y >= OP_SCREENH )
			return;

		if ( x1 > x2 )
		{
			uint16_t tmp = x1;
			x1 = x2;
			x2 = tmp;
		}

		if ( x1 < 0 )
			x1 = 0;
		if ( x2 >= OP_SCREENW )
			x2 = OP_SCREENW-1;

		// Round to 3 pixels
		div_t start = div(x1,3);
		div_t end = div(x2,3);

		SetRow( y, y );
		SetColumn( start.quot, end.quot + (end.rem ? 1 : 0) - 1 );

		uint8_t p = 0;

		// Leading pixels
		for ( ; p < start.rem; p++ )
		{
			PutPixel( 0, p );
		}

		for ( int16_t x = x1; x <= x2; x++ )
		{
			PutPixel( c, p );
			p++;
			if ( p == 3 ) p = 0;
		}
		// trailing pixels
		for ( int8_t i = end.rem; i >= 0; i-- )
		{
			PutPixel( 0, p );
			p++;
		}
	}

	uint16_t PixelData( uint8_t c1, uint8_t c2, uint8_t c3 )
	{
		uint16_t pixel;
		pixel = (c1>>1) << 11;
		pixel |= c2 << 5;
		pixel |= c3>>1;
		return pixel;
	}

	void VLine( uint16_t x, uint8_t y1, uint8_t y2, uint8_t c)
	{
		if ( x < 0 || x >= OP_SCREENW )
			return;

		if ( y1 > y2 )
		{
			int8_t tmp = y1;
			y1 = y2;
			y2 = tmp;
		}

		if ( y1 < 0 )
			y1 = 0;
		if ( y2 >= OP_SCREENH )
			y2 = OP_SCREENH-1;

		// We always write 2 bytes, 3 pixels.  Find which one is on.
		div_t pixel = div(x,3);

		uint16_t data;
		if ( pixel.rem == 0 )
			data = PixelData( c, 0, 0 );
		else if ( pixel.rem == 1 )
			data = PixelData( 0, c, 0 );
		else
			data = PixelData( 0, 0, c );

		SetRow( y1, y2-1 );
		SetColumn( pixel.quot, pixel.quot );

		for ( int8_t y = y1; y <= y2; y++ )
		{
			_sendData( data );
		}
	}

};
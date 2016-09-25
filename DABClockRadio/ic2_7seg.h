/*
 * ic2_7seg.h
 *
 * Created: 24/09/2016 10:38:37 AM
 *  Author: Frank Tkalcevic
 */ 


#pragma  once

#include "i2c.h"

#define countof(x)	(sizeof(x)/sizeof((x)[0]))


#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3
#define HT16K33_CMD_BRIGHTNESS 0xE0



template <class TSerialTxnImpl, uint8_t SLAVE_ADDR>
class SevenSeg: public TSerialTxnImpl
{
public:
	void Init()
	{
		TSerialTxnImpl::Init();
	}
	void Start()
	{
		static const uint8_t SetUp[] = { SLAVE_ADDR, 0x21 };
		TSerialTxnImpl::SendTxn( SetUp, countof(SetUp) );

		static const uint8_t Brightness[] = { SLAVE_ADDR, HT16K33_CMD_BRIGHTNESS | 4 };
		TSerialTxnImpl::SendTxn( Brightness, countof(Brightness) );

		static const uint8_t Display[] = { SLAVE_ADDR, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (0 << 1) };
		TSerialTxnImpl::SendTxn( Display, countof(Display) );

		TSerialTxnImpl::BeginTxn( SLAVE_ADDR );
		TSerialTxnImpl::SendData( 0 );	// Row select
		for ( int i = 0; i < 10; i++ )	
			TSerialTxnImpl::SendData( 0 );	// Data;
		TSerialTxnImpl::EndTxn();

	}

};
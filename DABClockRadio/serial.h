/*
 * Serial.h
 *
 * Created: 8/02/2015 8:20:06 AM
 *  Author: Frank Tkalcevic
 */ 


#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <avr/pgmspace.h>
#include <stdlib.h>

//#define _SFR_ASM_COMPAT 1


#define REG_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define REG_WORD(mem_addr) (*(volatile uint16_t *)(mem_addr))
#define REG_DWORD(mem_addr) (*(volatile uint32_t *)(mem_addr))

#define MAKE_XBAUD( BSCALE, BSEL )		(((BSCALE) << 12) | ((BSEL) & 0x0FFF))
	
#define _VECTOR(N) __vector_ ## N


template< uint16_t UCSRA, uint16_t UCSRB, uint16_t UCSRC, uint16_t UDR, uint16_t UBRR, uint8_t UDRIE, uint8_t RXCIE, uint8_t TXEN, uint8_t RXEN, uint8_t UCSZ0, uint8_t UCSZ1 >
class CMegaSerial
{
protected:
	static void EnableTransmit() { REG_BYTE(UCSRB) |= _BV(UDRIE); } /* Enable UDRE and TXCinterrupt */
	static void TransmitByte( uint8_t b ) 	{ REG_BYTE(UDR) = b; }
	static void UDRIE_OFF() { REG_BYTE(UCSRB) &= ~_BV(UDRIE); }         /* Disable UDRE interrupt */
	static uint8_t ReceiveByte( ) 	{ return REG_BYTE(UDR); }
	static void Init() 
	{
		REG_BYTE(UCSRA) = 0;
		REG_BYTE(UCSRB) = 0;
		REG_BYTE(UCSRC) = 0;
	}
	static void SetBaud(uint16_t nBaudSetting, uint8_t /*bits*/, bool /*parity*/, uint8_t /*stop*/ )
	{
		REG_WORD(UBRR) = nBaudSetting;
		REG_BYTE(UCSRC) |= _BV(UCSZ1) | _BV(UCSZ0);	// 8,n,1
	}
	
	static void EnableTx()
	{
		REG_BYTE(UCSRB) |=  _BV(TXEN);
	}

	static void EnableRx()
	{
		REG_BYTE(UCSRB) |=  _BV(RXCIE) | _BV(RXEN);
	}
};

#if __AVR_ARCH__ >= 100
// xmega

template< uint16_t _USART >
class CXMegaSerial
{
	protected:
	void EnableTransmit() { ((USART_t *)_USART)->CTRLA = (((USART_t *)_USART)->CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc; } /* Enable UDRE interrupt */
	void TransmitByte( uint8_t b ) 	{ ((USART_t *)_USART)->DATA = b; }
	void UDRIE_OFF() { ((USART_t *)_USART)->CTRLA &= ~USART_DREINTLVL_gm;  }         /* Disable UDRE interrupt */
	uint8_t ReceiveByte( ) 	{ return ((USART_t *)_USART)->DATA; }
	void Init()
	{
		USART_t *usart = ((USART_t *)_USART);
		usart->CTRLA = 0;
		usart->CTRLB = 0;
		usart->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_bm /* 2 stop bit */ | USART_CHSIZE_8BIT_gc;
#if defined(__AVR_ATxmega32E5__)
		usart->CTRLD = 0;
#endif
	}
	void SetBaud(uint16_t nBaudSetting, uint8_t /*bits*/, bool /*parity*/, uint8_t /*stop*/ )
	{
		USART_t *usart = ((USART_t *)_USART);
		usart->BAUDCTRLA = nBaudSetting & 0xFF;
		usart->BAUDCTRLB = nBaudSetting >> 8;
	}
	
	void EnableTx()
	{
		USART_t *usart = ((USART_t *)_USART);
		usart->CTRLB |= USART_TXEN_bm;
	}

	void EnableRx()
	{
		USART_t *usart = ((USART_t *)_USART);
		usart->CTRLB |= USART_RXEN_bm;
		usart->CTRLA = (usart->CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
	}
};
#endif

	#ifdef _SFR_MEM8
		#undef _SFR_MEM8
	#endif
	#ifdef _SFR_MEM16
		#undef _SFR_MEM16
	#endif

	#ifdef _SFR_IO8
		#undef _SFR_IO8
	#endif
	#ifdef _SFR_IO16
		#undef _SFR_IO16
	#endif

	#define _SFR_MEM8(mem_addr) (mem_addr)
	#define _SFR_MEM16(mem_addr) (mem_addr)
	#define _SFR_IO8(io_addr) ((io_addr) + __SFR_OFFSET)
	#define _SFR_IO16(io_addr) ((io_addr) + __SFR_OFFSET)

#if __AVR_ARCH__ >= 100
// xmega
	typedef CXMegaSerial<USARTC0_DATA>		CMegaSerialC0;
	typedef CXMegaSerial<USARTD0_DATA>		CMegaSerialD0;
	typedef CXMegaSerial<USARTE0_DATA>		CMegaSerialE0;
	
	#define IMPLEMENT_SERIAL_INTERRUPTS( UartNo, SerialInstance )			ISR(USART##UartNo##_DRE_vect) { SerialInstance.ISR_USART_UDRE_vect(); }		\
																			ISR(USART##UartNo##_RXC_vect) { SerialInstance.ISR_USART_RX_vect(); }

	#define IMPLEMENT_SERIAL_INTERRUPTS_SINGLE( SerialInstance )			ISR(USART_DRE_vect) { SerialInstance.ISR_USART_UDRE_vect(); }		\
																			ISR(USART_RXC_vect) { SerialInstance.ISR_USART_RX_vect(); }

#else
	typedef CMegaSerial<UCSR0A,UCSR0B,UCSR0C,UDR0,UBRR0,UDRIE0,RXCIE0,TXEN0,RXEN0,UCSZ00,UCSZ01>		CMegaSerial0;
	#ifdef UCSR1A
	typedef CMegaSerial<UCSR1A,UCSR1B,UCSR1C,UDR1,UBRR1,UDRIE1,RXCIE1,TXEN1,RXEN1,UCSZ10,UCSZ11>		CMegaSerial1;
	#endif
	
	#define IMPLEMENT_SERIAL_INTERRUPTS( UartNo, SerialInstance )			ISR(USART##UartNo##_UDRE_vect) { SerialInstance.USART_UDRE_vect(); }		\
																			ISR(USART##UartNo##_RX_vect) { SerialInstance.USART_RX_vect(); }

#endif



template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE>
class CSerialImpl: protected CLowLevel
{
protected:
	uint8_t UART_TxBuf[UART_TX_BUFFER_SIZE];
	volatile uint8_t UART_TxHead;
	volatile uint8_t UART_TxTail;	

	uint8_t UART_RxBuf[UART_RX_BUFFER_SIZE];
	volatile uint8_t UART_RxHead;
	volatile uint8_t UART_RxTail;

public:
	void ISR_USART_UDRE_vect() 
	{
		//_delay_us(100);
		uint8_t tmptail;

		/* Check if all data is transmitted */
		if ( UART_TxHead != UART_TxTail )
		{
			/* Calculate buffer index */
			tmptail = ( UART_TxTail + 1 ) & (UART_TX_BUFFER_SIZE-1);
			UART_TxTail = tmptail;      /* Store new index */

			CLowLevel::TransmitByte( UART_TxBuf[tmptail] );  /* Start transmition */
		}
		else
		{
			CLowLevel::UDRIE_OFF();
		}
	}	
	
	void ISR_USART_RX_vect()
	{
		uint8_t data;
		uint8_t tmphead;

		data = CLowLevel::ReceiveByte();                 /* Read the received data */
		
		/* Calculate buffer index */
		tmphead = ( UART_RxHead + 1 ) & (UART_RX_BUFFER_SIZE-1);
		UART_RxHead = tmphead;      /* Store new index */

		if ( tmphead == UART_RxTail )
		{
			/* ERROR! Receive buffer overflow */
		}

		UART_RxBuf[tmphead] = data; /* Store received data in buffer */
	}	
	
protected:
	void InitBuffers()
	{
		CLowLevel::Init();
		UART_TxTail = 0;
		UART_TxHead = 0;
		UART_RxTail = 0;
		UART_RxHead = 0;
	}

	void SendChar( uint8_t data )
	{
		unsigned char tmphead;
		
		/* Calculate buffer index */
		tmphead = ( CSerialImpl::UART_TxHead + 1 ) & (UART_TX_BUFFER_SIZE-1); /* Wait for free space in buffer */
		while ( tmphead == UART_TxTail );
	
		UART_TxBuf[tmphead] = data;           /* Store data in buffer */
		UART_TxHead = tmphead;                /* Store new index */

		CSerialImpl::EnableTransmit();
	}

	uint8_t ReadByte( void )
	{
		uint8_t tmptail;
	
		while ( UART_RxHead == UART_RxTail )  /* Wait for incoming data */
		;
		tmptail = ( UART_RxTail + 1 ) & (UART_TX_BUFFER_SIZE-1);/* Calculate buffer index */
	
		UART_RxTail = tmptail;                /* Store new index */
	
		return UART_RxBuf[tmptail];           /* Return data */
	}

	bool IsDataAvailable( void )
	{
		return ( UART_RxHead != UART_RxTail ); /* Return 0 (FALSE) if the receive buffer is empty */
	}

	void InitSerial( uint16_t nBaudSetting )
	{
		CLowLevel::SetBaud(nBaudSetting, 8, false, 1 );
		CLowLevel::EnableTx();
		CLowLevel::EnableRx();
	}
};

//template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE> uint8_t CSerialImpl<CLowLevel, UART_TX_BUFFER_SIZE, UART_RX_BUFFER_SIZE>::UART_TxBuf[UART_TX_BUFFER_SIZE];
//template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE> volatile uint8_t CSerialImpl<CLowLevel, UART_TX_BUFFER_SIZE, UART_RX_BUFFER_SIZE>::UART_TxHead;
//template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE> volatile uint8_t CSerialImpl<CLowLevel, UART_TX_BUFFER_SIZE, UART_RX_BUFFER_SIZE>::UART_TxTail;
//template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE> uint8_t CSerialImpl<CLowLevel, UART_TX_BUFFER_SIZE, UART_RX_BUFFER_SIZE>::UART_RxBuf[UART_RX_BUFFER_SIZE];
//template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE> volatile uint8_t CSerialImpl<CLowLevel, UART_TX_BUFFER_SIZE, UART_RX_BUFFER_SIZE>::UART_RxHead;
//template<class CLowLevel, uint8_t UART_TX_BUFFER_SIZE, uint8_t UART_RX_BUFFER_SIZE> volatile uint8_t CSerialImpl<CLowLevel, UART_TX_BUFFER_SIZE, UART_RX_BUFFER_SIZE>::UART_RxTail;

template<class CSerialImpl, bool bEnabled=true>
class CSerial: public CSerialImpl
{
public:
	void Init( unsigned short nBaudSetting )
	{
		if ( bEnabled )
		{
			CSerialImpl::InitBuffers();
			CSerialImpl::InitSerial( nBaudSetting );
		}
	}
	
	//void RS232SetParity( enum EParity eParity )
	//{
	//register byte oldUCSRC;
	//cli();
	//oldUCSRC = UCSR0C;
	//oldUCSRC = UCSR0C;
	//sei();
	//
	//switch ( eParity )
	//{
	//case eOdd:
	//UCSR0C = oldUCSRC | _BV(UPM01) | _BV(UPM00);
	//break;
	//case eEven:
	//UCSR0C = (oldUCSRC & ~ (_BV(UPM00))) | _BV(UPM01);
	//break;
	//case eNone:
	//default:
	//UCSR0C = (oldUCSRC & ~ (_BV(UPM01) | _BV(UPM00)));
	//break;
	//}
	//}
	//
	//void RS232SetStopBits( byte nBits )
	//{
	//byte oldUCSRC;
	//cli();
	//oldUCSRC = UCSR0C;
	//oldUCSRC = UCSR0C;
	//sei();
	//
	//switch ( nBits )
	//{
	//case 2:
	//UCSR0C = oldUCSRC | _BV(USBS0);
	//break;
	//case 1:
	//default:
	//UCSR0C = (oldUCSRC & ~ (_BV(USBS0)));
	//break;
	//}
	//}
	//
	//void RS232SetCharSize( byte nBits )
	//{
	//byte oldUCSRC;
	//cli();
	//oldUCSRC = UCSR0C;
	//oldUCSRC = UCSR0C;
	//sei();
	//
	//switch ( nBits )
	//{
	//case 5:
	//UCSR0C = (oldUCSRC & ~(_BV(UCSZ01) | _BV(UCSZ00)));
	//UCSR0B = UCSR0B & ~(_BV(UCSZ02));
	//break;
	//case 6:
	//UCSR0C = (oldUCSRC & ~(_BV(UCSZ01)))  | _BV(UCSZ00);
	//UCSR0B = UCSR0B & ~(_BV(UCSZ02));
	//break;
	//case 7:
	//UCSR0C = (oldUCSRC & ~(_BV(UCSZ00))) | _BV(UCSZ01);
	//UCSR0B = UCSR0B & ~(_BV(UCSZ02));
	//break;
	//case 8:
	//default:
	//UCSR0C = oldUCSRC | _BV(UCSZ01) | _BV(UCSZ00);
	//UCSR0B = UCSR0B & ~(_BV(UCSZ02));
	//break;
	//case 9:
	//UCSR0C = oldUCSRC | _BV(UCSZ01) | _BV(UCSZ00);
	//UCSR0B = UCSR0B | _BV(UCSZ02);
	//break;
	//}
	//}
	//
	//void RS232SetDoubleSpeedMode( bool bEnable )
	//{
	//if ( bEnable )
	//UCSR0A |= _BV(U2X0);
	//else
	//UCSR0A &= ~_BV(U2X0);
	//}

	void Send( char c )
	{
		if ( bEnabled )
		{
			CSerialImpl::SendChar( c );
		}
	}


	void Send( const char *str )
	{
		if ( bEnabled )
		{
			while ( *str != 0 )
			{
				CSerialImpl::SendChar( *str );
				str++;
			}
		}
	}

	void Send( const char *str, uint16_t len )
	{
		if ( bEnabled )
		{
			for ( uint16_t i = 0; i < len; i++ )
				CSerialImpl::SendChar( str[i] );
		}
	}

	void Send( const uint8_t *str, uint16_t len )
	{
		if ( bEnabled )
		{
			for ( uint16_t i = 0; i < len; i++ )
				CSerialImpl::SendChar( str[i] );	
		}
	}

	void Send_P( PGM_P s  )
	{
		if ( bEnabled )
		{
			char b;
			goto ReadByte;
			while ( b != 0 )
			{
				CSerialImpl::SendChar(b );
				s++;
			ReadByte:
				b = pgm_read_byte( s );
			}
		}
	}


	void Send( int v )
	{
		if ( bEnabled )
		{
			char s[7];
			itoa( v, s, 10 );
			Send( s );
		}
	}

	void Send( long v )
	{
		if ( bEnabled )
		{
			char s[11];
			ltoa( v, s, 10 );
			Send( s );
		}
	}

	void SendHex( uint8_t v )
	{
		if ( bEnabled )
		{
			uint8_t b = v >> 4;
			if ( b < 10 )
				CSerialImpl::SendChar( '0' + b );
			else
				CSerialImpl::SendChar( 'A' + b - 10 );
			b = v & 0xF;
			if ( b < 10 )
				CSerialImpl::SendChar( '0' + b );
			else
				CSerialImpl::SendChar( 'A' + b - 10 );
		}
	}

	void SendHex( int v )
	{
		if ( bEnabled )
		{
			SendHex( v >> 8 );
			SendHex( v & 0xFF );
		}
	}

	void SendCRLF( void )
	{
		if ( bEnabled )
		{
			CSerialImpl::SendChar( '\r' );
			CSerialImpl::SendChar( '\n' );
		}
	}


	uint8_t ReadByte( void )
	{
		if ( bEnabled )
			return CSerialImpl::ReadByte();
		else
			return 0;
	}

	bool IsDataAvailable( void )
	{
		if ( bEnabled )
			return CSerialImpl::IsDataAvailable();
		else
			return false;
	}

};

	#ifdef _SFR_MEM8
		#undef _SFR_MEM8
	#endif
	#ifdef _SFR_MEM16
		#undef _SFR_MEM16
	#endif

	#ifdef _SFR_IO8
		#undef _SFR_IO8
	#endif
	#ifdef _SFR_IO16
		#undef _SFR_IO16
	#endif

	#define _SFR_MEM8(mem_addr) _MMIO_BYTE(mem_addr)
	#define _SFR_MEM16(mem_addr) _MMIO_WORD(mem_addr)
	#define _SFR_IO8(io_addr) _MMIO_BYTE((io_addr) + __SFR_OFFSET)
	#define _SFR_IO16(io_addr) _MMIO_WORD((io_addr) + __SFR_OFFSET)

#endif /* SERIAL_H_ */

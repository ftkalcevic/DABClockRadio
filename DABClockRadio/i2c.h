#pragma once


#include <avr/pgmspace.h>
#include <stdlib.h>

//#define _SFR_ASM_COMPAT 1



#if __AVR_ARCH__ >= 100
// xmega

template< uint16_t _TWI, uint32_t nFrequency >
class CXMegaI2CMaster
{
protected:
	void SendAddress( uint8_t Addr ) { ((TWI_t *)_TWI)->MASTER.ADDR = Addr; }
	void SendData( uint8_t Data ) { ((TWI_t *)_TWI)->MASTER.DATA = Data; }
	void SendStop() { ((TWI_t *)_TWI)->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc; }
	void ClearWIF() { ((TWI_t *)_TWI)->MASTER.STATUS |= TWI_MASTER_WIF_bm; }
	void WaitForReady() { while ( (((TWI_t *)_TWI)->MASTER.STATUS & TWI_MASTER_WIF_bm) == 0 ); }
	void EnableInterrupts() { ((TWI_t *)_TWI)->MASTER.CTRLA |= TWI_MASTER_WIEN_bm; }
	TWI_MASTER_BUSSTATE_enum GetStatus() { return (TWI_MASTER_BUSSTATE_enum)(((TWI_t *)_TWI)->MASTER.STATUS & TWI_MASTER_BUSSTATE_gm); }

	// Bus master write only 
	void Init()
	{
		((TWI_t *)_TWI)->CTRL = 0;
		((TWI_t *)_TWI)->MASTER.CTRLA = TWI_MASTER_ENABLE_bm | TWI_MASTER_INTLVL_MED_gc;
		((TWI_t *)_TWI)->MASTER.CTRLB = 0;
		((TWI_t *)_TWI)->MASTER.CTRLC = 0;
		((TWI_t *)_TWI)->MASTER.BAUD = F_CPU/(2*nFrequency)-5;
		((TWI_t *)_TWI)->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
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
typedef CXMegaI2CMaster<TWIE_CTRL,400000>		CMegaI2CE;
typedef CXMegaI2CMaster<TWIC_CTRL,400000>		CMegaI2CC;

#define IMPLEMENT_SERIAL_TXN_INTERRUPTS( TWINo, TWIInstance )			ISR(TWI##TWINo##_TWIM_vect) { TWIInstance.MASTER_vect(); }

#endif



template<class CLowLevel, uint8_t TX_BUFFER_SIZE, uint8_t TXN_BUFFER_SIZE>
class CSerialTxnImpl: protected CLowLevel
{
protected:
	uint8_t TxBuf[TX_BUFFER_SIZE];
	volatile uint8_t TxHead;
	volatile uint8_t TxTail;

	uint8_t TxnBuf[TXN_BUFFER_SIZE];
	volatile uint8_t TxnHead;
	volatile uint8_t TxnTail;
	volatile uint8_t TxnLen;
	uint8_t NewTxnLen;

public:
	void MASTER_vect()
	{
		TxnLen--;
		if ( TxnLen == 0 )
		{
			TxnLen = PopTxn();
			if ( TxnLen == 0 )	// Nothing waiting
			{
				CLowLevel::SendStop();
				return;
			}
			else
				CLowLevel::SendAddress( PopData() );
		}
		else
			CLowLevel::SendData( PopData() );
	}

	
protected:
	void InitBuffers()
	{
		CLowLevel::Init();
		CLowLevel::EnableInterrupts();
		TxTail = 0;
		TxHead = 0;
		TxnTail = 0;
		TxnHead = 0;
		TxnLen = 0;
	}

private:
	uint8_t PopTxn()
	{
		/* Check if all data is transmitted */
		if ( TxnHead != TxnTail )
		{
			uint8_t tmptail;

			/* Calculate buffer index */
			tmptail = ( TxnTail + 1 ) & (TXN_BUFFER_SIZE-1);
			TxnTail = tmptail;      /* Store new index */

			return TxnBuf[tmptail]; 
		}
		else
		{
			return 0;
		}
	}

	uint8_t PopData()
	{
		/* Check if all data is transmitted */
		if ( TxHead != TxTail )
		{
			/* Calculate buffer index */
			uint8_t tmptail = ( TxTail + 1 ) & (TX_BUFFER_SIZE-1);
			TxTail = tmptail;      /* Store new index */

			return TxBuf[tmptail];
		}
		return 0;
	}

	void PushData( uint8_t data )
	{
		unsigned char tmphead;
			
		/* Calculate buffer index */
		tmphead = ( TxHead + 1 ) & (TX_BUFFER_SIZE-1);		/* Wait for free space in buffer */
		while ( tmphead == TxTail );
			
		TxBuf[tmphead] = data;		/* Store data in buffer */
		TxHead = tmphead;				/* Store new index */
	}

	void PushTxn( uint8_t len )
	{
		unsigned char tmphead;

		/* Calculate buffer index */
		tmphead = ( TxnHead + 1 ) & (TXN_BUFFER_SIZE-1);		/* Wait for free space in buffer */
		while ( tmphead == TxnTail );

		TxnBuf[tmphead] = len;		/* Store data in buffer */
		TxnHead = tmphead;				/* Store new index */
	}

protected:
	void SendTxn( const uint8_t *buf, uint8_t len )
	{
		// Store data
		for ( uint8_t i = 0; i < len; i++ )
		{
			PushData( buf[i] );
		}

		// Store txn len
		PushTxn(len);

		if ( TxnLen == 0  )	// not busy.
		{
			TxnLen = PopTxn();
			CLowLevel::SendAddress( PopData() );
		}
	}

	void BeginTxn( uint8_t addr ) 
	{ 
		NewTxnLen = 1;
		PushData( addr );
	}
	void SendData( uint8_t data ) 
	{ 
		NewTxnLen++;
		PushData( data );
	}
	void EndTxn() 
	{
		PushTxn(NewTxnLen);
		if ( TxnLen == 0  )	// not busy.
		{
			TxnLen = PopTxn();
			CLowLevel::SendAddress( PopData() );
		}
	}

};





template<class CLowLevel>
class CSerialTxnPolledImpl: protected CLowLevel
{
protected:
	void InitBuffers()
	{
		CLowLevel::Init();
	}

protected:
	void SendTxn( const uint8_t *buf, uint8_t len )
	{
		CLowLevel::SendAddress( buf[0] );
		CLowLevel::WaitForReady();
		// Store data
		for ( uint8_t i = 1; i < len; i++ )
		{
			CLowLevel::SendData( buf[i] );
			CLowLevel::WaitForReady();
		}
		CLowLevel::SendStop();
	}

	void BeginTxn( uint8_t addr ) 
	{ 
		CLowLevel::SendAddress( addr );
	}
	void SendData( uint8_t data ) 
	{ 
		CLowLevel::WaitForReady();
		CLowLevel::SendData( data );
	}
	void EndTxn() 
	{
		CLowLevel::WaitForReady();
		CLowLevel::SendStop();
	}

};


template<class CSerialTxnImpl>
class CSerialTxn: public CSerialTxnImpl
{
	public:
	void Init()
	{
		CSerialTxnImpl::InitBuffers();
	}

	void SendTxn( const uint8_t *buf, uint8_t len )
	{
		CSerialTxnImpl::SendTxn( buf, len );
	}

	void BeginTxn( uint8_t addr ) { CSerialTxnImpl::BeginTxn( addr );}
	void SendData( uint8_t data ) { CSerialTxnImpl::SendData( data );}
	void EndTxn() { CSerialTxnImpl::EndTxn();}

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

#pragma once


#define	DBA_MUTE			PIN4_bm
#define	DBA_MUTE_PORT		PORTE
#define	DBA_SHUTDOWN		PIN5_bm
#define	DBA_SHUTDOWN_PORT	PORTE
#define	DBA_RESET			PIN5_bm
#define	DBA_RESET_PORT		PORTD


enum class DABCmdType: uint8_t
{
	System = 0,
	Stream = 1,
	_RTC = 2,
	MOT = 3,
	Notification = 7
};


enum class DABReponseType: uint8_t
{
	Ack = 0,
};

enum class DABResponse: uint8_t
{
	Ack = 1,
	Nak = 2,
};

enum class DABSystemCmd: uint8_t
{
	GetSysRdy = 0x00,
	Reset = 0x01,
	GetMCUVersion = 0x02,
	GetBootVersion = 0x03,
	GetASPVersion = 0x04,
	GetAllVersion = 0x05,
	SetAudioOutputType = 0x06
};

enum class DABStreamCmd: uint8_t
{
	Play = 0x00,
	Stop = 0x01,
	Search = 0x02,
	AutoSearch = 0x03,
	StopSearch = 0x04,
	GetPlayStatus = 0x05,
	GetPlayMode = 0x06,
	GetPlayIndex = 0x07,
	GetSignalStrength = 0x08,
	SetStereoMode = 0x09,
	GetStereoMode = 0x0A,
	GetStereo = 0x0B,
	SetVolume = 0x0C,
	GetVolume = 0x0D,
	GetProgrammeType = 0x0E,
	GetProgrammeName = 0x0F,
	GetProgrammeText = 0x10,
	GetSamplingRate = 0x11,
	GetDataRate = 0x12,
	GetSignalQuality = 0x13,
	GetFrequency = 0x14,
	GetEnsembleName = 0x15,
	GetTotalProgramme = 0x16,
	IsActive = 0x17,
	SetFMSeekThdLevel_Deprecated = 0x18,
	GetFMSeekThdLevel_Deprecated = 0x19,
	GetServiceName  = 0x1A,
	GetSearchProgram = 0x1B,
	SetPowerBar = 0x1C,
	GetPowerBar = 0x1D,
	GetServCompType = 0x1E,
	SetBBEEQ = 0x1F,
	SetHeadroom = 0x20,
	SetPreset = 0x21,
	GetPreset = 0x22,
	GetProgrameInfo = 0x23,
	GetSorter = 0x24,
	SetSorter = 0x25,
	GetDrc = 0x26,
	SetDrc = 0x27,
	GetBBEEQ = 0x28,
	GetHeadroom = 0x29,
	GetDLSCmd = 0x2A,
	PruneStation = 0x2B,
	DirectTuneProgram_Deprecated = 0x2C,
	GetECC = 0x2D,
	GetRdsPIcode = 0x2E,
	GetServFollowingDABInfo = 0x2F,
	SetFMStereoThdLevel_Deprecated = 0x30,
	GetFMSteroThdLevel_Deprecated = 0x31,
	GetRDSRawData = 0x32,
	GetServFollowingFMInfo = 0x34,
	SetFMSeekThreshold = 0x35,
	GetFMSeekThreshold = 0x36,
	SetFMSteroThreshold = 0x37,
	GetFMSteroThreshold = 0x38,
};

enum class DABRTCCmd: uint8_t
{
	SetClock = 0x00,
	GetClock = 0x01,
	EnableSyncClock = 0x02,
	GetSyncClockStatus = 0x03,
	GetClockStatus = 0x04,
};

enum class DABNotificationCmd: uint8_t
{
	SetNotification = 0x00,
	GetNotification = 0x01
};

enum class DABPlayStatus: uint8_t
{
	Playing = 0,
	Searching = 1,
	Tuning = 2, 
	Stop = 3,
	SortingChange = 4,
	Reconfiguration = 5
};

enum class DABPlayMode: uint8_t
{
	DAB = 0,
	FM = 1,
	BEEPER = 3,
	AM = 4,
	StreamStop = 0xFF
};


enum class DABNotification: uint16_t
{
	ScanFinished	= 0x01,
	NewFMText		= 0x02,
	DABReconfig		= 0x04,
	DABChanChange	= 0x08,
	FMRDSGrpNotify	= 0x10,
	NewDABText		= 0x20,
	ScanFrequency	= 0x40
};

enum class AsyncReturnCode: uint8_t
{
	OK,
	ReplyTimedOut,
	ReplAck,
	ReplyNack,
	Error
};


template <class TSerialImpl>
class CMonkeyBoardDAB: public TSerialImpl
{
private:
	const uint8_t START_BYTE= 0xFE;
	const uint8_t END_BYTE = 0xFD;
	uint8_t serialNumber;

	bool GoodHeader(uint8_t* input, uint16_t dwBytes, uint8_t sn)
	{
		if ( input[0] == START_BYTE && 
			 input[3] == sn &&
			 input[dwBytes-1] == END_BYTE )
			return true;
		else
			return false;
	}

	uint8_t SendCommand( uint8_t cmdType, uint8_t cmd, uint8_t *data, uint16_t len )
	{
		uint8_t sn = serialNumber++;
		TSerialImpl::SendChar( START_BYTE );
		TSerialImpl::SendChar( cmdType );
		TSerialImpl::SendChar( cmd );
		TSerialImpl::SendChar( sn );
		TSerialImpl::SendChar( (uint8_t)((len>>8) & 0xFF) );
		TSerialImpl::SendChar( (uint8_t)(len & 0xFF) );
		for ( uint16_t i = 0; i < len; i++ )
			TSerialImpl::SendChar( data[i] );
		TSerialImpl::SendChar( END_BYTE );
		return sn;
	}

	
	uint8_t SendCommand( DABSystemCmd cmd, uint8_t *data = nullptr, uint16_t len = 0 ) { return SendCommand( (uint8_t)DABCmdType::System, (uint8_t)cmd, data, len ); }
	uint8_t SendCommand( DABStreamCmd cmd, uint8_t *data = nullptr, uint16_t len = 0 ) { return SendCommand( (uint8_t)DABCmdType::Stream, (uint8_t)cmd, data, len ); }
	uint8_t SendCommand( DABRTCCmd cmd, uint8_t *data = nullptr, uint16_t len = 0 ) { return SendCommand( (uint8_t)DABCmdType::_RTC, (uint8_t)cmd, data, len ); }
	uint8_t SendCommand( DABNotificationCmd cmd, uint8_t *data = nullptr, uint16_t len = 0 ) { return SendCommand( (uint8_t)DABCmdType::Notification, (uint8_t)cmd, data, len ); }

	bool WaitForReply( uint8_t cmd, uint8_t sn, uint8_t *buf, uint16_t len, uint8_t timeout_ms = 250 )
	{
		uint16_t n = 0;
		uint8_t timeout = 0;
		while ( n < len )
		{
			if ( TSerialImpl::IsDataAvailable() )
			{
				buf[n++] = TSerialImpl::ReadByte();
				timeout = 0;
			}
			else
			{
				if ( timeout >= timeout_ms )
					return false;

				_delay_ms(1);
				timeout++;
			}
		}
		return true;
	}

	bool WaitForReply( DABSystemCmd cmd, uint8_t sn, uint8_t *buf, uint16_t len, uint8_t timeout_ms = 250 ) { return WaitForReply((uint8_t)cmd, sn, buf, len, timeout_ms ); }
	bool WaitForReply( DABStreamCmd cmd, uint8_t sn, uint8_t *buf, uint16_t len, uint8_t timeout_ms = 250 ) { return WaitForReply((uint8_t)cmd, sn, buf, len, timeout_ms ); }
	bool WaitForReply( DABRTCCmd cmd, uint8_t sn, uint8_t *buf, uint16_t len, uint8_t timeout_ms = 250 ) { return WaitForReply((uint8_t)cmd, sn, buf, len, timeout_ms ); }
	bool WaitForReply( DABNotificationCmd cmd, uint8_t sn, uint8_t *buf, uint16_t len, uint8_t timeout_ms = 250 ) { return WaitForReply((uint8_t)cmd, sn, buf, len, timeout_ms ); }

	bool RetreiveText( DABStreamCmd cmd, uint8_t sn, char *buf, int len, uint8_t timeout_ms =250 )
	{
		bool bReadingHeader=true;
		uint8_t header[6];
		uint8_t nIndex = 0;
		uint8_t timeout = 0;
		uint16_t nDataLength;
		while ( timeout < timeout_ms )
		{
			if ( TSerialImpl::IsDataAvailable() )
			{
				timeout = 0;
				if ( bReadingHeader )
				{
					header[nIndex++] = TSerialImpl::ReadByte();
					if ( nIndex == sizeof(header) )
					{
						if ( header[0] == START_BYTE &&
							 header[1] == (uint8_t)DABCmdType::Stream &&
							 header[2] == (uint8_t)cmd &&
							 header[3] == sn )
						{
							bReadingHeader = false;
							nIndex = 0;
							nDataLength = (header[4] << 8) | header[5];
						}
						else if ( header[0] == START_BYTE &&
								  header[1] == 0 &&
								  header[2] == (uint8_t)DABResponse::Nak &&
								  header[3] == sn )
						{
							uint8_t b = TSerialImpl::ReadByte();
							b = b;
							return ErrorCleanup();
						}
						else
						{
							return ErrorCleanup();
						}
					}
				}
				else
				{
					uint8_t b = TSerialImpl::ReadByte();
					nIndex++;
					if ( (nIndex & 1) == 0 )	// Unicode, so only take alternate characters
					{
						*(buf++) = b;
						len--;
						if ( len == 1 )	// No more space in our buffer
						{
							*(buf) = 0;
							ErrorCleanup();
							return true;
						}
					}
					if ( nIndex == nDataLength+1 )	// +1 for END_BYTE
						return true;
				}
			}
			else
			{
				_delay_ms(1);
				timeout++;
			}
		}
		// timed out
		return ErrorCleanup();
	}


	bool ErrorCleanup()
	{
		uint8_t timeout = 0;
		while ( timeout < 2 )
		{
			if ( TSerialImpl::IsDataAvailable() )
			{
				TSerialImpl::ReadByte();
				timeout = 0;
			}
			else
			{
				_delay_ms(1);
				timeout++;
			}
		}
		return false;
	}
public:
	CMonkeyBoardDAB()
	{
		serialNumber = 0;
	}

	void SYSTEM_GetSysRdy_Async(const uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t sn = SendCommand( DABSystemCmd::GetSysRdy );
		WaitForReply_Async( DABSystemCmd::GetSysRdy, sn, clock+timeout );
	}

	bool SYSTEM_GetSysRdy()
	{
		uint8_t input[7];

		uint8_t sn = SendCommand( DABSystemCmd::GetSysRdy )	;

		if ( WaitForReply(DABSystemCmd::GetSysRdy,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}


	bool HardResetRadio(void)
	{
		DABOff();

		_delay_ms(100);

		DABPowerOn();

		_delay_ms(100);
		
		DABEnable();

		//_delay_ms(800);
		// Wait for escape
		for ( uint8_t t = 0; t < 200; t++ )
		{
			while ( TSerialImpl::IsDataAvailable() )
			{
				uint8_t b = TSerialImpl::ReadByte();
				if ( b == 0x1B )
					return true;
				//TSerialImpl::Send(0x55);
			}
			//TSerialImpl::Send('?');
			_delay_ms(50);
		}

		return false;
	}

	void STREAM_GetTotalProgram_Async(const uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetTotalProgramme );
		WaitForReply_Async( DABStreamCmd::GetTotalProgramme, sn, clock+timeout, 4 );
	}

	
	bool STREAM_GetTotalProgram(uint16_t &nPrograms)
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetTotalProgramme );

		uint8_t input[11];
		if ( WaitForReply(DABStreamCmd::GetTotalProgramme,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABStreamCmd::GetTotalProgramme )
		{
			nPrograms = input[9] | (input[8]<<8);
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	bool STREAM_GetPlayStatus( DABPlayStatus &status )
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetPlayStatus );

		uint8_t input[8];
		if ( WaitForReply(DABStreamCmd::GetPlayStatus,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABStreamCmd::GetPlayStatus )
		{
			status = (DABPlayStatus)input[6];
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	bool STREAM_GetPlayMode( DABPlayMode &mode )
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetPlayMode );

		uint8_t input[8];
		if ( WaitForReply(DABStreamCmd::GetPlayMode,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABStreamCmd::GetPlayMode )
		{
			mode = (DABPlayMode)input[6];
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	
	bool STREAM_GetPlayIndex(uint32_t &nIndex )
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetPlayIndex );

		uint8_t input[11];
		if ( WaitForReply(DABStreamCmd::GetPlayIndex,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABStreamCmd::GetPlayIndex )
		{
			nIndex = (uint32_t)input[9] | ((uint32_t)input[8]<<8)  | ((uint32_t)input[7]<<16)  | ((uint32_t)input[6]<<24) ;
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}
	
	bool STREAM_GetSignalStrength(uint8_t &nStrength, uint16_t &nBitErrorRate )
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetSignalStrength );

		uint8_t input[10];
		if ( WaitForReply(DABStreamCmd::GetSignalStrength,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABStreamCmd::GetSignalStrength )
		{
			nStrength = input[6];
			nBitErrorRate = (uint16_t)input[8] | ((uint16_t)input[7]<<8);
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void STREAM_Play_Async( DABPlayMode mode, uint32_t program, uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t out[5];
		out[0] = (uint8_t)mode;
		out[1] = (program >>24) & 0xFF;
		out[2] = (program >>16) & 0xFF;
		out[3] = (program >>8) & 0xFF;
		out[4] = (program) & 0xFF;
		uint8_t sn = SendCommand( DABStreamCmd::Play, out, sizeof(out) );
		WaitForReply_Async( DABStreamCmd::Play, sn, clock+timeout );
	}

	bool STREAM_Play( DABPlayMode mode, uint32_t program )
	{
		uint8_t out[5];
		out[0] = (uint8_t)mode;
		out[1] = (program >>24) & 0xFF;
		out[2] = (program >>16) & 0xFF;
		out[3] = (program >>8) & 0xFF;
		out[4] = (program) & 0xFF;
		uint8_t sn = SendCommand( DABStreamCmd::Play, out, sizeof(out) );

		uint8_t input[7];
		if ( WaitForReply(DABStreamCmd::Play,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void STREAM_Stop_Async( uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t sn = SendCommand( DABStreamCmd::Stop );
		WaitForReply_Async( DABStreamCmd::Stop, sn, clock+timeout );
	}

	bool STREAM_Stop()
	{
		uint8_t sn = SendCommand( DABStreamCmd::Stop );

		uint8_t input[7];
		if ( WaitForReply(DABStreamCmd::Stop,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void STREAM_GetProgrammeName_Async( uint32_t nIndex, bool bFullName, char *buf, int len, uint32_t clock, uint16_t timeout=250 )
	{
		uint8_t out[5];
		out[0] = (nIndex >>24) & 0xFF;
		out[1] = (nIndex >>16) & 0xFF;
		out[2] = (nIndex >>8) & 0xFF;
		out[3] = (nIndex) & 0xFF;
		out[4] = bFullName ? 1 :0;
		uint8_t sn = SendCommand( DABStreamCmd::GetProgrammeName, out, sizeof(out) );

		WaitForReply_Async( DABStreamCmd::GetProgrammeName, sn, clock+timeout, -1, (uint8_t *)buf, len );
	}

	bool STREAM_GetProgrammeName( uint32_t nIndex, bool bFullName, char *buf, int len )
	{
		uint8_t out[5];
		out[0] = (nIndex >>24) & 0xFF;
		out[1] = (nIndex >>16) & 0xFF;
		out[2] = (nIndex >>8) & 0xFF;
		out[3] = (nIndex) & 0xFF;
		out[4] = bFullName ? 1 :0;
		uint8_t sn = SendCommand( DABStreamCmd::GetProgrammeName, out, sizeof(out) );

		return RetreiveText( DABStreamCmd::GetProgrammeName, sn, buf, len );
	}

	bool STREAM_GetProgrammeText(char *buf, int len )
	{
		uint8_t sn = SendCommand( DABStreamCmd::GetProgrammeText );

		return RetreiveText( DABStreamCmd::GetProgrammeText, sn, buf, len );	
	}

	void STREAM_SetVolume_Async(uint8_t volume, const uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t out[1];
		out[0] = volume;
		uint8_t sn = SendCommand( DABStreamCmd::SetVolume, out, sizeof(out) );
		WaitForReply_Async(DABStreamCmd::SetVolume,sn, clock+timeout);
	}

	bool STREAM_SetVolume( uint8_t volume )
	{
		uint8_t out[1];
		out[0] = volume;
		uint8_t sn = SendCommand( DABStreamCmd::SetVolume, out, sizeof(out) );

		uint8_t input[7];
		if ( WaitForReply(DABStreamCmd::SetVolume,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void RTC_GetClock_Async( uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t sn = SendCommand( DABRTCCmd::GetClock );
		WaitForReply_Async( DABRTCCmd::GetClock, sn, clock+timeout, 7 );
	}

	bool RTC_GetClock( uint8_t &day, uint8_t &month, uint16_t &year, uint8_t &hour, uint8_t &minute, uint8_t &second  )
	{
		uint8_t sn = SendCommand( DABRTCCmd::GetClock );

		uint8_t input[14];
		if ( WaitForReply(DABRTCCmd::GetClock,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			input[2] == (uint8_t)DABResponse::Ack )
		{
			day = input[9];
			month = input[11];
			year = input[12] + 2000;
			hour = input[8];
			minute = input[7];
			second = input[6];
			
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void RTC_GetClockStatus_Async( uint32_t clock, uint16_t timeout = 250 )
	{
		uint8_t sn = SendCommand( DABRTCCmd::GetClockStatus );
		WaitForReply_Async( DABRTCCmd::GetClockStatus, sn, clock+timeout, 1 );
	}
	bool RTC_GetClockStatus( bool &bSet )
	{
		uint8_t sn = SendCommand( DABRTCCmd::GetClockStatus );

		uint8_t input[8];
		if ( WaitForReply(DABRTCCmd::GetClockStatus,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABRTCCmd::GetClockStatus )
		{
			bSet = input[6] != 0;
			
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void RTC_GetSyncClockStatus_Async( uint32_t clock, uint16_t timeout=250 )
	{
		uint8_t sn = SendCommand( DABRTCCmd::GetSyncClockStatus );
		WaitForReply_Async( DABRTCCmd::GetSyncClockStatus, sn, clock+timeout, 1 );
	}
	bool RTC_GetSyncClockStatus( bool &bEnabled )
	{
		uint8_t sn = SendCommand( DABRTCCmd::GetSyncClockStatus );

		uint8_t input[8];
		if ( WaitForReply(DABRTCCmd::GetSyncClockStatus,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABRTCCmd::GetSyncClockStatus )
		{
			bEnabled = input[6] != 0;
			
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	void RTC_EnableSyncClock_Async( bool bEnable, uint32_t clock, uint16_t timeout=250 )
	{
		uint8_t output = bEnable ? 1 : 0;
		uint8_t sn = SendCommand( DABRTCCmd::EnableSyncClock, &output, 1 );
		WaitForReply_Async( DABRTCCmd::EnableSyncClock, sn, clock+timeout );
	}

	bool RTC_EnableSyncClock( bool bEnable )
	{
		uint8_t output = bEnable ? 1 : 0;
		uint8_t sn = SendCommand( DABRTCCmd::EnableSyncClock, &output, 1 );

		uint8_t input[7];
		if ( WaitForReply(DABRTCCmd::EnableSyncClock,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	bool NOTIFY_SetNotification( DABNotification n )
	{
		uint8_t data[2];
		data[0] = (uint16_t)n >> 8;
		data[1] = (uint16_t)n & 0xFF;
		uint8_t sn = SendCommand( DABNotificationCmd::SetNotification, data, sizeof(data) );

		uint8_t input[7];
		if ( WaitForReply(DABNotificationCmd::SetNotification,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}

	bool NOTIFY_GetNotification( DABNotification &n )
	{
		uint8_t sn = SendCommand( DABNotificationCmd::GetNotification );

		uint8_t input[9];
		if ( WaitForReply(DABNotificationCmd::SetNotification,sn,input,sizeof(input),250) &&
			 GoodHeader(input,sizeof(input),sn) &&
			 input[2] == (uint8_t)DABResponse::Ack )
		{
			n = (input[6] << 8) + input[7];
			return true;
		}
		else
		{
			return ErrorCleanup();
		}
	}
	//SetClock = 0x00,
	// = 0x02,
private:
	void DABOff()
	{
		DBA_RESET_PORT.DIRSET = DBA_RESET;
		DBA_RESET_PORT.OUTCLR = DBA_RESET;	// Reset

		DBA_SHUTDOWN_PORT.DIRSET = DBA_SHUTDOWN;
		DBA_SHUTDOWN_PORT.OUTCLR = DBA_SHUTDOWN;	// Power off
	}

	void DABPowerOn()
	{
		DBA_SHUTDOWN_PORT.OUTSET = DBA_SHUTDOWN;	// Power on
	}

	void DABEnable()
	{
		DBA_RESET_PORT.OUTSET = DBA_RESET; // Reset off
	}

	enum class DABState: uint8_t
	{
		Initialise,
		PowerOn,
		Enable,
		TimedWait,
		WaitForStart,
		WaitForReply,
		WaitForIdle,
		Idle,
		Off
	} state, nextState;
	uint32_t timer;
	int8_t	wait_for_msgLen;
	uint8_t wait_byte_count;
	uint8_t wait_header[10];	// a bit bigger than the 6 for a header as we use this a small msg buffer.
	uint8_t *wait_output_buffer;
	uint8_t wait_output_buffer_len;
	uint16_t wait_body_length;
	uint16_t wait_body_count;

	void WaitForState( DABState nextState, uint32_t time )
	{
		this->nextState = nextState;
		this->timer = time;
		this->state = DABState::TimedWait;
	}

	void WaitForReply_Async( DABStreamCmd cmd, uint8_t sn, uint32_t timeout, int8_t msgLen=0, uint8_t *output_buffer = NULL, uint8_t output_buffer_len = 0 )
	{
		WaitForReply_Async( (uint8_t)DABCmdType::Stream, (uint8_t)cmd, sn, timeout, msgLen, output_buffer, output_buffer_len );
	}
	void WaitForReply_Async( DABRTCCmd cmd, uint8_t sn, uint32_t timeout, int8_t msgLen=0, uint8_t *output_buffer = NULL, uint8_t output_buffer_len = 0 )
	{
		WaitForReply_Async( (uint8_t)DABCmdType::_RTC, (uint8_t)cmd, sn, timeout, msgLen, output_buffer, output_buffer_len );
	}
	void WaitForReply_Async( DABSystemCmd cmd, uint8_t sn, uint32_t timeout, int8_t msgLen=0, uint8_t *output_buffer = NULL, uint8_t output_buffer_len = 0 )
	{
		WaitForReply_Async( (uint8_t)DABCmdType::System, (uint8_t)cmd, sn, timeout, msgLen, output_buffer, output_buffer_len );
	}
	void WaitForReply_Async( uint8_t nCmdType, uint8_t nCmd, uint8_t sn, uint32_t timeout, int8_t msgLen=0, uint8_t *output_buffer = NULL, uint8_t output_buffer_len = 0 )
	{
		if ( this->state != DABState::Idle )
		{
			// do something.
			return;
		}
		this->state = DABState::WaitForReply;
		this->wait_header[0] = START_BYTE;
		if ( msgLen == 0 )
		{
			this->wait_header[1] = (uint8_t)DABReponseType::Ack;
			this->wait_header[2] = (uint8_t)DABResponse::Ack;
		}
		else
		{
			this->wait_header[1] = nCmdType;
			this->wait_header[2] = nCmd;
		}
		this->wait_header[3] = sn;
		this->wait_for_msgLen = msgLen;
		this->wait_byte_count = 0;
		this->timer = timeout;
		this->wait_output_buffer = output_buffer;
		this->wait_output_buffer_len = output_buffer_len;
	}

	AsyncReturnCode WaitForIdle(uint32_t clock)
	{
		state = DABState::WaitForIdle;
		timer = clock + WAIT_FOR_IDLE_TIME;	// 20 ms for the RX buffer to clear.
		return AsyncReturnCode::Error;
	}

	const uint8_t WAIT_FOR_IDLE_TIME = 20;

public:
	uint8_t *MsgBuffer() { return wait_header; }
	uint16_t MsgLen() { return wait_body_count < wait_body_length ? wait_byte_count : wait_body_length; }

	bool isIdle() { return state == DABState::Idle; }

	void PowerOff()
	{
		DABOff();
		state = DABState::Off;
	}	

	void AsyncStart()
	{
		state = DABState::Initialise;
	}

	AsyncReturnCode Async( const uint32_t clock )
	{
		AsyncReturnCode ret = AsyncReturnCode::OK;

		switch ( state )
		{
			case DABState::Initialise:
				DABOff();
				WaitForState( DABState::PowerOn, clock+100 );
				break;

			case DABState::PowerOn:
				DABPowerOn();
				WaitForState( DABState::Enable, clock+100 );
				break;
		
			case DABState::Enable:
				DABEnable();
				state = DABState::WaitForStart;
				timer = clock + 3000;
				break;

			case DABState::WaitForStart:
				if ( (int32_t)(timer - clock) <= 0 )
				{
					state = DABState::Idle;
					break;
				}

				while ( TSerialImpl::IsDataAvailable() )
				{
					uint8_t b = TSerialImpl::ReadByte();
					if ( b == 0x1B )
					{
						state = DABState::Idle;
						break;
					}
				}
				break;

			case DABState::WaitForReply:
				if ( (int32_t)(timer - clock) <= 0 )
				{
					state = DABState::Idle;
					ret = AsyncReturnCode::ReplyTimedOut;
					break;
				}

				while ( TSerialImpl::IsDataAvailable() )
				{
					uint8_t b = TSerialImpl::ReadByte();
					if ( wait_byte_count < 4 )
					{
						// Read and validate header.
						if ( wait_header[wait_byte_count++] != b )
						{
							ret = WaitForIdle(clock);
							break;
						}
					}
					else if ( wait_byte_count < 6 )
					{
						// Read the length
						wait_header[wait_byte_count++] = b;
						if ( wait_byte_count == 6 )
						{
							wait_body_length = (wait_header[4] << 8) | wait_header[5];
							if ( wait_for_msgLen > 0 &&  (uint8_t)wait_for_msgLen != wait_body_length )
							{
								ret = WaitForIdle(clock);
								break;
							}
							wait_body_count = 0;
						}
					}
					else
					{
						if ( wait_output_buffer )
						{
							if ( wait_body_count < wait_body_length )
								wait_output_buffer[wait_body_count] = b;
						}
						else
						{
							if ( wait_body_count < sizeof(wait_header) )
								wait_header[wait_body_count] = b;
						}
						wait_body_count++;
						if ( wait_body_count == wait_body_length + 1)
						{
							if ( b != END_BYTE )
							{
								ret = WaitForIdle(clock);
								break;
							}
							else
							{
								ret = AsyncReturnCode::ReplAck;
								state = DABState::Idle;
							}
						}
					}
				}
				break;

			case DABState::WaitForIdle:
				if ( (int32_t)(timer - clock) <= 0 )
				{
					state = DABState::Idle;
					break;
				}

				while ( TSerialImpl::IsDataAvailable() )
				{
					TSerialImpl::ReadByte();
					timer = clock + WAIT_FOR_IDLE_TIME;
				}
				break;
								
			case DABState::TimedWait:
				if ( (int32_t)(timer - clock) <= 0 )
					state = nextState;
				break;

			case DABState::Idle:
				while ( TSerialImpl::IsDataAvailable() )
				{
					uint8_t b = TSerialImpl::ReadByte();
					b=b;
				}
				break;

			case DABState::Off:
				break;
		}
		return ret;
	}

};


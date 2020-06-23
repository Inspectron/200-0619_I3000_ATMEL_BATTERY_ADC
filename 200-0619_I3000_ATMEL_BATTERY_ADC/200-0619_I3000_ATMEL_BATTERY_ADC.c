/*
 * _200_0588__FWARE_ROCAM4_METABO.c
 *
 * Created: 02/19/2020
 *  Author: tnewman
 *
 *  Revision History:
 *
 *  Original project was branched from 200-0588 ROCAM4 Metabo
 *
 *  1.0 - Initial release
 *
 */

#include "config.h"
#include "usiTwiSlave.h"

int main()                             __attribute__ ((section (".bootloader")));
void writePacket(tBootPacket *pPacket) __attribute__ ((section (".bootloader")));
int application(void)                  __attribute__ ((section (".staticapp")));

FUSES =
{
	.low      = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),
	.high     = (FUSE_SPIEN & FUSE_BODLEVEL1),
	.extended = EFUSE_DEFAULT,
};


volatile uint16  gTimer0OverflowCnt	= 0;
volatile uint8   gnBtyesinBuf		= 0;
volatile uint8   gADC_channel		= 2;
volatile uint32  gnWrites			= 0;

uint8 gTWIRegisters[] = {
	twiEND,          // num reg                           0
	0x00,	         //twiMasterBusy                      1
	0x10,            //twiFWVersion                       2
	twiStatusReady,  // twiStatus                         3
	0x00,	         //twiExternalBatteryReadingHighByte  4	
	0x00,	         //twiExternalBatteryReadingLowByte   5	
	0x00,	         //twiInternalBatteryReadingHighByte  6
	0x00,            //twiInternalBatteryReadingLowByte   7
	0x00,	         //twiBatteryTempHighByte             8
	0x00,	         //twiBatteryTempLowByte              9
	0x00,            //twiExternalBatteryPresent          10
        // Download control
        0x00,            // twiDownload                       11
        0x00,            // twiPacketDownload                 12
        0x00             // twiFailureTest                    13 
};

#if 0
ISR( TIM1_OVF_vect )
{
	gTimer0OverflowCnt++;
	gTXReady = true;

	// set the read flag for reading the Equalizer LOCK and battery temperature every 1 second
	if( (gTimer0OverflowCnt % 2) == 0)
	{
		gReadEqualizerLock = true;
	}
}
#endif

ISR ( ADC_vect )
{
	switch (gADC_channel)
	{
		case EXTERNAL_BATTERY_VOLTAGE:
			gTWIRegisters[twiExternalBatteryReadingLowByte]  = ADCL;
			gTWIRegisters[twiExternalBatteryReadingHighByte] = ADCH;
			
			if(gTWIRegisters[twiExternalBatteryReadingLowByte] > EXTERNAL_THRESHOLD) {
				gTWIRegisters[twiExternalBatteryPresent] = true;
			} else {
				gTWIRegisters[twiExternalBatteryPresent] = false;
			}
			break;
		case INTERNAL_SUPPLY_VOLTAGE:
			gTWIRegisters[twiInternalBatteryReadingLowByte]  = ADCL;
			gTWIRegisters[twiInternalBatteryReadingHighByte] = ADCH;
			break;
		case EXTERNAL_BATTERY_TEMP:
			gTWIRegisters[twiBatteryTempLowByte]  = ADCL;
			gTWIRegisters[twiBatteryTempHighByte] = ADCH;
			break;
	}

	gADC_channel++;
	if(gADC_channel > 2) gADC_channel = 0;
	ADMUX   = (ADMUX & 0xC0) | gADC_channel;		// select the next ADC channel to read

}

void DoTWISlaveWrite(void)
{
	enum eTWIRegisters	addr;
	unsigned char		data;

	addr = usiTwiReceiveByte();
	data = usiTwiReceiveByte();
}

int application(void)
{

	DDRA  = 0x20;														// 0010 0000 Make PORTA PIN5 output

	PORTB  |= (1 << LED_SELECT);
	DDRB   |= (1 << LED_SELECT);

	DIDR0  |= (1 << ADC0_BIT) | (1 << ADC1_BIT) | (1 << ADC2_BIT);		// turn digital input buffers OFF
	ADMUX  |= ADC2_BIT | (1 << REFS1);									// set ADC2 (Battery Temperature) as the default ADC on power-up and user internal adc ref of 1.1VDC
	ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) |				// set ADC pre-scaler to 128
			  (1 << ADATE) | (1 << ADIE)  | (1 << ADEN);				// enable auto trigger and enable to adc complete interrupt and enable the adc interface
	ADCSRB |= (1 << ADTS2) | (1 << ADTS1);								// set the adc trigger to timer1 overflow

	TCCR1A  = 0x00;
	TCCR1B  = 0x03;														// set the time1 clock source to prescaler of 64.  At 8Mhz yields an overflow every 0.524 seconds;
	TCNT1   = 0x0000;													// Initialize the timer to zero
	//TIMSK1 |= (1 << TOIE1);												// turn on the timer0 overflow interrupt

	// Initialize the I2C interfaces
	usiTwiSlaveInit();

	_delay_ms(350);

	// Set the Global interrupt flag
	sei();

    while(1)
    {
		asm("nop;");

		if(usiTwiDataInReceiveBuffer())
		{
			if(gTWISlaveWriteRequest)
			{
				gnBtyesinBuf = 0;
				gTWISlaveWriteRequest = false;
				gTWIRegisters[twiMasterBusy] = twiStatusBusy;
				DoTWISlaveWrite();
				gTWIRegisters[twiMasterBusy] = twiStatusReady;
			}
		}
    }
}

void writePacket(tBootPacket *pPacket)
{
	uint16          i;
	uint8           sreg;
	uint16          w;
	uint8           checksum = 0;
	static uint8    packets = 0;
	static boolean  checksumfail = false;
	static uint8    writebuf[SPM_PAGESIZE];
	static uint16   writeAddr;

	if(gTWIRegisters[twiFailureTest] == 1)
	{
		gTWIRegisters[twiStatus] = twiStatusBadChecksum;
		return;
	}

	if(packets == 0)
	{
		writeAddr = (pPacket->destAddr >> 8) | (pPacket->destAddr << 8);
	}

	checksum += pPacket->nBytes;
	checksum += (pPacket->destAddr >> 8);
	checksum += (pPacket->destAddr & 0x00FF);

	for (i = 0; i < 16; i++)
	{
		writebuf[(packets << 4) + i] =  pPacket->data[i];
		checksum += pPacket->data[i];
	}

	checksum = ~checksum + 1;

	if (checksum != pPacket->checksum)
	{
		checksumfail |= true;
	}

	if (++packets == 4)
	{
		if(gTWIRegisters[twiFailureTest] == 2)
		{
			gTWIRegisters[twiStatus] = twiStatusBadReadback;
			return;
		}

		packets = 0;

		// Disable interrupts.
		sreg = SREG;
		cli();

		if (!checksumfail)
		{
			asm("nop;");
			eeprom_busy_wait ();
			boot_page_erase (writeAddr);
			boot_spm_busy_wait ();          // Wait until the memory is erased.

			asm("nop;");

			for (i = 0; i < SPM_PAGESIZE; i += 2)
			{
				w = (writebuf[i+1] << 8) | writebuf[i];
				boot_page_fill (writeAddr + i, w);
			}

			boot_page_write (writeAddr);    // Store buffer in flash page.
			boot_spm_busy_wait();           // Wait until the memory is written.
			gnWrites++;

			for (i = 0; i < SPM_PAGESIZE; i += 2)
			{
				w = pgm_read_word(writeAddr + i);

				if((w & 0xff) != writebuf[i] || (w >> 8) != writebuf[i + 1])
				{
					gTWIRegisters[twiStatus] = twiStatusBadReadback;
					break;
				}
			}

			if(i >= SPM_PAGESIZE)
			{
				gTWIRegisters[twiStatus] = twiStatusGoodChecksum;
			}
		}

		else
		{
			gTWIRegisters[twiStatus] = twiStatusBadChecksum;
		}

		// Re-enable interrupts (if they were ever enabled).
		SREG = sreg;

		asm("nop;");
	}
}

int main() /* bootload in disguise */
{
	tBootPacket packet;
	uint8 *pPacket = (uint8 *)&packet;
	uint8 bufIdx = 0;
	uint8 regAddr;
	uint8 regData;

	usiTwiSlaveInit();
	sei();

	if (!(PINA & (1 << PINA7)))
	{
		gTWIRegisters[twiDownload] = twiapplication;
		application();
	}

	else
	{
		gTWIRegisters[twiDownload] = twiacceptdata;

		for(;;)
		{
			if (usiTwiDataInReceiveBuffer() && gTWISlaveWriteRequest)
			{
				gTWISlaveWriteRequest = false;

				regAddr = usiTwiReceiveByte();
				regData = usiTwiReceiveByte();

				if(regAddr == twiPacketDownload)
				{
					*pPacket = regData;
					pPacket++;
					bufIdx++;

					if(bufIdx == 20)
					{
						bufIdx = 0;
						pPacket = (uint8 *)&packet;
						writePacket(&packet);
						asm("nop;");
					}
				}

				if(regAddr == twiFailureTest)
				{
					gTWIRegisters[twiFailureTest] = regData;
				}
			}
		}
	}

	return 0;
}

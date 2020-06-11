/*
 * config.h
 *
 * Created: 9/8/2014 11:03:57 AM
 *  Author: tnewman
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <util/delay.h>

typedef char           char8;   //!<  8 bit character

typedef signed char    sint8;   //!<  8 bit signed integer
typedef signed short   sint16;  //!< 16 bit signed integer
typedef signed long    sint32;  //!< 32 bit signed integer;

typedef unsigned char  uint8;   //!<  8 bit unsigned integer
typedef unsigned short uint16;  //!< 16 bit unsigned integer
typedef unsigned long  uint32;  //!< 32 bit unsigned integer

typedef float          float32; //!< 32 bit floating point
typedef long double    float64; //!< 64 bit floating point

typedef uint8          boolean;	//!<  8 bit value to be used for true/false

#define bit_set(x, y)    x |= (1 << (y))
#define bit_clear(x, y)  x &= ~(1 << (y))
#define bit_test(x, y)	 (((x) & (1 << (y))) != 0)

#define true  1
#define false 0

#define EQUALIZER_LOCK				(bit_test(PINA, PINA7) == 1)
#define EQUALIZER_PORT				PORTA
#define EQUALIZER_FREEZE			PORTA3
#define EQUALIZER_INVERT			PORTA5

#define	SIXTYFIVE_DETECT     		(bit_test(PINA, PINA3) == 1)

#define LED_SELECT					PINB0

#define EXTERNAL_BATTERY_VOLTAGE	ADC0_BIT
#define INTERNAL_SUPPLY_VOLTAGE     ADC1_BIT
#define EXTERNAL_BATTERY_TEMP		ADC2_BIT
#define EXTERNAL_THRESHOLD			785

enum eTWIstatus
{
	twiStatusReady,
	twiStatusBusy,
	twiStatusError,
	twiStatusPicError,
	twiStatusClockError,
	twiStatusGoodChecksum,
	twiStatusBadChecksum,
	twiStatusBadReadback
};

enum eTWIDownload
{
	twiapplication,
	twiacceptdata,
};

enum eTWIRegisters
{	// Register Name						R/W			    Notes
    twiNReg,                                //  R           total # of registers available (same as twiEnd, at the bottom of this enum definition)
	twiMasterBusy,							//  R			one of enum eTWIstatus
	twiFWVersion,							//  R
    twiStatus,                              //  R       enum eTWIstatus
	twiExternalBatteryReadingHighByte,		//  R			A/D voltage reading from battery
	twiExternalBatteryReadingLowByte,		//  R			A/D voltage reading from battery
	twiInternalBatteryReadingHighByte,		//  R			A/D voltage reading from battery
	twiInternalBatteryReadingLowByte,		//  R			A/D voltage reading from battery
	twiBatteryTempHighByte,					//  R			A/D temperature reading from battery
	twiBatteryTempLowByte,					//  R			A/D temperature reading from battery
	twiExternalBatteryPresent,				//  R           0 = not present; 1 = present

    // Download control
    twiDownload,                            //  R           0x00 = normal operation, 0x01 = ready to accept download data
    twiPacketDownload,                      //  R/W         0x00 = no packet pending; 0x01 = packet data byte is eminent
    twiFailureTest,                         //  R/W         0x00 = no failures, 0x01 = simulate CRC error, 0x02 = simulate read back error

	twiEND
};

struct sBootPacket
{
	uint8  nBytes;
	uint16 destAddr;
	uint8  data[16];
	uint8  checksum;
};
typedef struct sBootPacket tBootPacket;

extern unsigned char gTWIRegisters[twiEND];
extern volatile unsigned char gnBtyesinBuf;


#endif /* CONFIG_H_ */
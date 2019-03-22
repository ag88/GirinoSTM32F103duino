//-----------------------------------------------------------------------------
// Girino.h
//-----------------------------------------------------------------------------
// Copyright 2012 Cristiano Lino Fontana
//
// ported to stm32duino libmaple core
// additions Copyright 2019 Andrew Goh
//
// This file is part of Girino.
//
//	Girino is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	Girino is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with Girino.  If not, see <http://www.gnu.org/licenses/>.
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <Arduino.h>

//-----------------------------------------------------------------------------
// Defines and Typedefs
//-----------------------------------------------------------------------------

#define DEBUG		0

#define ADCBUFFERSIZE	1280

// PA0 ADC1 channel 0
// take note of the pin map - see comments for initADC()
#define ADC_CHANNEL		0

#define errorPin	LED_BUILTIN
#define thresholdPin	3

#define BAUDRATE	115200	// Baud rate of UART in bps
#define COMMANDDELAY	10	// ms to wait for the filling of Serial buffer
#define COMBUFFERSIZE	3	// Size of buffer for incoming numbers

// voltage translations stm to girino select only one of this
// note girino maps the range to 0 to 255 - -2.5 to 2.5v, so 0v is around 128
// stm32 adc maps the range range 0 to 4096 - 0v to 3.3v
//
// use higher order 8 bits from 12 bits of adc (voltage scale looks 'incorrect')
#define VTRANS8BIT
// use only higher order 7 bits and translate 0 to 127, so 255 ~ 3.3v (coarse) !
//#define VTRANS7BIT
// use only higher order 7 bits and translate 0 to 127, make 2.5 to 3.3v
// translations so 255 ~ 2.5v (coarse, slow and possibly inaccurate)
// but voltage scale looks 'correct' !
//#define VTRANS7BIT3325

#if DEBUG == 1
	#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
	#define dshow(expression) Serial.println( expression )
#else
	#define dprint(expression) 
	#define dshow(expression)
#endif

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void initPins(void);
void initADC(int8_t channel);
void initAnalogComparator(void);

void startADC( void );
void stopADC( void );
void startAnalogComparator( void );
void stopAnalogComparator( void );

void setADCPrescaler( uint8_t prescaler );
void setVoltageReference( uint8_t reference );
void setTriggerEvent( uint8_t event );
void setSamprate(int samprate);
void triggered(void);
uint8_t vstmtogirino(uint16_t volt);
uint16_t vgirinotostm(uint8_t volt);

void error (void);
// Fills the given buffer with bufferSize chars from a Serial object
void fillBuffer( \
	char *buffer, \
	byte bufferSize, \
	USBSerial *serial = &Serial);
void printStatus(void);
void printData(uint8_t *buffer, int offset, int size);

enum {
	TRG_TOGGLE = 0,
	TRG_FALLINGEDGE = 2,
	TRG_RISINGEDGE = 3
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern volatile  boolean wait;
extern          uint16_t waitDuration;
extern volatile uint16_t stopIndex;
extern volatile uint16_t ADCCounter;
extern          uint8_t ADCBuffer[ADCBUFFERSIZE];
extern volatile  boolean freeze;

extern           uint8_t prescaler;
extern           uint8_t triggerEvent;
extern           uint16_t threshold;

extern              char commandBuffer[COMBUFFERSIZE+1];



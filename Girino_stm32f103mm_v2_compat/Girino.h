//-----------------------------------------------------------------------------
// Girino.h
//-----------------------------------------------------------------------------
// ported to stm32duino libmaple core
// implementation nearly completely re-written
// different from original release and specifically taylored
// to stm32f103c{8,b} hardware, only the protocol is left intact
// Copyright 2019 Andrew Goh
//
// Original:
// Copyright 2012 Cristiano Lino Fontana
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

#define BAUDRATE	115200	// Baud rate of UART in bps
#define CWAIT	10	// ms timeout waiting for parameter over serial
#define PLEN	3	// length of the parameter

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void initPins(void);
void initADC(int8_t channel);
void initAnalogComparator(void);
void initAWDtriggers(void);

void startADC( void );
void stopADC( void );

void setADCPrescaler( uint8_t prescaler );
void setVoltageReference( uint8_t reference );
void setTriggerEvent( uint8_t event );
void setSamprate(int samprate);
void triggered(void);
uint8_t vstmtogirino(uint16_t volt);
uint16_t vgirinotostm(uint8_t volt);


void error (void);
char* readParam( uint8_t length, uint8_t cwait);
void printStatus(void);
void printData(void);

enum {
	TRG_TOGGLE = 0,
	TRG_FALLINGEDGE = 2,
	TRG_RISINGEDGE = 3
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern volatile  boolean btriggered;
extern volatile  boolean bframedone;
extern          uint16_t waitDuration;
extern volatile uint16_t stopIndex;
extern          uint16_t ADCBuffer[ADCBUFFERSIZE];


extern 			uint8_t errorPin;
extern			boolean bLedOn;
#define LEDON   bLedOn?HIGH:LOW
#define LEDOFF  bLedOn?LOW:HIGH

extern           uint8_t prescaler;
extern           uint8_t triggerEvent;
extern           uint16_t threshold;

extern              char Param[5];



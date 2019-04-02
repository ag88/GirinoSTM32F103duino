//-----------------------------------------------------------------------------
// Girino.ino
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
#include "Girino.h"
#include "CAdcMgr.h"
#include "util.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

		  uint8_t errorPin;
		  boolean bLedOn;

          uint8_t prescaler;
         uint16_t samprate;

             char Param[5];

          CAdcMgr AdcMgr;

//-----------------------------------------------------------------------------
// Main routines
//-----------------------------------------------------------------------------
//
// The setup function initializes registers.
//
void setup (void) {		// Setup of the microcontroller

	// Open serial port with a baud rate of BAUDRATE b/s
	Serial.begin(BAUDRATE);

	// setup timer 2 to generate test signals, comment this if not required
	initTesttimer();

	// Clear buffers
	memset( (void *)Param, 0, sizeof(Param) );

	prescaler = 128;
	samprate = 100;

	// Activate interrupts
	interrupts();

	errorPin = LED_BUILTIN;
	bLedOn = HIGH;
	initPins();

	AdcMgr.initADC(ADC_CHANNEL);

	while(!Serial.isConnected());

	Serial.println("Girino stm32f103mm ready");

}

void loop (void) {

	// If frame is done, then it is time to send the buffer to the serial port
	AdcMgr.doframedone();

	if ( Serial.available() ) {
		// Read the incoming byte
		char c = Serial.read();
		// Parse character
		switch (c) {
			case 's':			// 's' for starting ADC conversions
				//Serial.println("ADC conversions started");
				AdcMgr.startConv();
				break;

			case 'S':			// 'S' for stopping ADC conversions
				//Serial.println("ADC conversions stopped");
				AdcMgr.stopConv();
				break;

			case 'p':			// 'p' for new prescaler setting
				{
				readParam(PLEN, CWAIT);

				// Convert buffer to integer
				uint8_t newP = atoi( Param );

				// Display moving status indicator
				Serial.print("Setting prescaler to: ");
				Serial.println(newP);

				prescaler = newP;
				setADCPrescaler(newP);
				}
				break;

			case 'P':
				{
				readParam(PLEN, CWAIT);

				// Convert buffer to integer
				int s1 = atoi( Param );

				// Display moving status indicator
				Serial.print("Setting prescaler1 to: ");
				Serial.println(s1);

				samprate = s1;
				AdcMgr.setSamprate(samprate * 1000);
				}
				break;

			case 'r':			// 'r' for new voltage reference setting
			case 'R': {
				readParam(PLEN, CWAIT);

				// Convert buffer to integer
				uint8_t newR = atoi( Param );

				// Display moving status indicator
				Serial.print("Setting voltage reference to: ");
				Serial.println(newR);

				//setVoltageReference(newR);
				}
				break;

			case 'e':			// 'e' for new trigger event setting
			case 'E': {
				readParam(PLEN, CWAIT);

				// Convert buffer to integer
				uint8_t newE = atoi( Param );

				// Display moving status indicator
				Serial.print("Setting trigger event to: ");
				Serial.println(newE);

				//triggerEvent = newE;
				//setTriggerEvent(newE);
				AdcMgr.setTriggerEvent(newE);
				}
				break;

			case 'w':			// 'w' for new wait setting
			case 'W': {
				readParam(4, CWAIT);

				// Convert buffer to integer
				uint16_t newW = atoi( Param );

				// Display moving status indicator
				Serial.print("Setting waitDuration to: ");
				Serial.println(newW);

				AdcMgr.setWaitDur(newW);

				}
				break;

			case 't':			// 'w' for new threshold setting
			case 'T': {
				readParam(PLEN, CWAIT);

				// Convert buffer to integer
				uint8_t newT = atoi( Param );

				// Display moving status indicator
				Serial.print("Setting threshold to: ");
				Serial.println(newT);

				AdcMgr.setThreshold(vgirinotostm(newT));

				}
				break;

			case 'd':			// 'd' for display status
			case 'D':
				printStatus();
				break;

			case 'z':			// 'd' for display status
				printMem();
				break;

			default:
				// Display error message
				Serial.print("ERROR: Command not found, it was: ");
				Serial.println(c);
				error();
		}
	}

}

void initPins() {
	//check for blue pill
	gpio_set_mode(GPIOC,13,GPIO_INPUT_FLOATING);
	if(gpio_read_bit(GPIOC,13)) { //bluepill
		errorPin = 14; //PC13
		bLedOn = LOW;
		pinMode(errorPin, OUTPUT_OPEN_DRAIN);
	} else { //maple mini
		errorPin = 33;
		bLedOn = HIGH;
		pinMode(errorPin, OUTPUT);
	}
	digitalWrite(errorPin,LEDOFF);

}


void initTesttimer(void) {

	Timer2.pause();
	Timer2.setPrescaleFactor(1); //36mhz
	timer_oc_set_mode(TIMER2,TIMER_CH2,TIMER_OC_MODE_PWM_1,0);
	timer_cc_enable(TIMER2, TIMER_CH2);

	Timer2.setPeriod(1000); // 1khz
	Timer2.setCompare(TIMER_CH2,Timer2.getOverflow()/2);

	//Timer 2 Channel 2 timer output is on PA1
	//setup pin PA1 for alt function output
	gpio_set_mode(GPIOA, 1, GPIO_AF_OUTPUT_PP);

	//Timer2.attachCompare2Interrupt(timer2handle);

	// start the timer
	Timer2.refresh();
	Timer2.resume();

}



#define UNOSYSCLK 16000000

void setADCPrescaler( uint8_t prescaler ) {
	// These bits determine the division factor between the system clock
	// frequency and the input clock to the ADC.
	// prescaler: 2, 4, 8, 16, 32, 64, 128 (default)
	prescaler = (prescaler == 0 ? 128 : prescaler);

	int adclk = UNOSYSCLK / prescaler;
	int samplerate = adclk / 13; //13 clock cycles per conversion on uno

	AdcMgr.setSamprate(samplerate);
}


//-----------------------------------------------------------------------------
void error (void) {
	digitalWrite( errorPin, HIGH );
	delay( 500 );
	digitalWrite( errorPin, LOW );
	delay( 250 );
	digitalWrite( errorPin, HIGH );
	delay( 500 );
	digitalWrite( errorPin, LOW );
}


char* readParam( uint8_t length, uint8_t cwait) {

	memset( (void *)Param, '\0', 5);

	uint8_t i = 0;
	while(cwait > 0 && i < length) {
		while(Serial.available()) {
			uint8_t c = Serial.read();
			Param[i++] = c;
		}
		delay(1);
		cwait--;
	}

	return Param;

}


void printStatus( void )
{
	Serial.print("Buffer size: ");
	Serial.println(ADCBUFFERSIZE);
	Serial.print("Baud rate: ");
	Serial.println(BAUDRATE);
	Serial.print("Prescaler: ");
	Serial.println(prescaler);
	Serial.print("Prescale1: ");
	Serial.println(samprate);
	Serial.print("Voltage reference:");
	Serial.println(" 0");
	Serial.print("Trigger event: ");
	Serial.println(AdcMgr.triggerEvent);
	Serial.print("Wait duration: ");
	Serial.println(AdcMgr.m_samplecounts);
	Serial.print("Threshold: ");
	Serial.println(vstmtogirino(AdcMgr.threshold));
}

void printMem(void) {
    Serial.print("Freestack:");
    Serial.println(FreeStack());
}


uint8_t vstmtogirino(uint16_t volt) {
	int vret = volt >> 4; // divide by 4096 / 256 = 16
	return vret & 0xff;
}

uint16_t vgirinotostm(uint8_t volt) {
    int vret = volt << 4; // multiply by 4096 / 256 = 16
	return vret & 0xffff;
}



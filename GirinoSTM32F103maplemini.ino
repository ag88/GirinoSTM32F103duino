//-----------------------------------------------------------------------------
// Girino.ino
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

#include "Girino.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

volatile  boolean wait;
         uint16_t waitDuration;
volatile uint16_t stopIndex;
volatile uint16_t ADCCounter;
         uint8_t ADCBuffer[ADCBUFFERSIZE];
volatile  boolean freeze;

          uint8_t prescaler;
          uint8_t triggerEvent;
          uint16_t threshold;


char commandBuffer[COMBUFFERSIZE+1];


void initTesttimer(void);
void initAdctimer(void);

void adchandle(void);

//-----------------------------------------------------------------------------
// Main routines
//-----------------------------------------------------------------------------
//
// The setup function initializes registers.
//
void setup (void) {		// Setup of the microcontroller
	// Open serial port with a baud rate of BAUDRATE b/s
	Serial.begin(BAUDRATE);

	dshow("# setup()");

	// setup timer 2 to generate test signals, comment this if not required
	initTesttimer();

	//Adc timer is used to drive the ADC sampling
	initAdctimer();

	// Clear buffers
	memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) );
	memset( (void *)commandBuffer, 0, sizeof(commandBuffer) );
	ADCCounter = 0;
	wait = false;
	waitDuration = ADCBUFFERSIZE - 32;
	stopIndex = -1;
	freeze = false;

	prescaler = 128;
	triggerEvent = 3;

	threshold = 127 * 4096 / 256;

	// Activate interrupts
	interrupts();

	//initPins();
	initADC();

	//initAnalogComparator();

	while(!Serial.isConnected());

	Serial.println("Girino ready");
	//printStatus();
}

void loop (void) {
	dprint(ADCCounter);
	dprint(stopIndex);
	dprint(wait);
	dprint(freeze);
	#if DEBUG == 1
	Serial.println( ADCSRA, BIN );
	Serial.println( ADCSRB, BIN );
	#endif

	// If freeze flag is set, then it is time to send the buffer to the serial port
	if ( freeze )
	{
		dshow("# Frozen");

		// Send buffer through serial port in the right order
		//Serial.print("Buffer: ");
		//Serial.write( ADCBuffer, ADCBUFFERSIZE );
		//Serial.print("End of Buffer");
		printData(ADCBuffer, ADCCounter, ADCBUFFERSIZE);
		//Serial.write( (uint8_t *)ADCBuffer + ADCCounter, ADCBUFFERSIZE - ADCCounter );
		//Serial.write( (uint8_t *)ADCBuffer, ADCCounter );


		// Turn off errorPin
		//digitalWrite( errorPin, LOW );

		wait = false;
		freeze = false;

		// Clear buffer
		//memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) );

		//startADC();
		// Let the ADC fill the buffer a little bit
		//delay(1);
		//startAnalogComparator();

		#if DEBUG == 1
		delay(3000);
		#endif
	}

	if ( Serial.available() > 0 ) {
		// Read the incoming byte
		char theChar = Serial.read();
			// Parse character
		switch (theChar) {
			case 's':			// 's' for starting ADC conversions
				//Serial.println("ADC conversions started");

				// Clear buffer
				memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) );

				startADC();
				// Let the ADC fill the buffer a little bit
				//delay(1);
				//startAnalogComparator();
				break;
			case 'S':			// 'S' for stopping ADC conversions
				//Serial.println("ADC conversions stopped");
				//stopAnalogComparator();
				stopADC();
				break;
			case 'p':			// 'p' for new prescaler setting
			case 'P': {
				// Wait for COMMANDDELAY ms to be sure that the Serial buffer is filled
				delay(COMMANDDELAY);

				fillBuffer( commandBuffer, COMBUFFERSIZE );

				// Convert buffer to integer
				uint8_t newP = atoi( commandBuffer );

				// Display moving status indicator
				Serial.print("Setting prescaler to: ");
				Serial.println(newP);

				prescaler = newP;
				setADCPrescaler(newP);
				}
				break;

			case 'r':			// 'r' for new voltage reference setting
			case 'R': {
				// Wait for COMMANDDELAY ms to be sure that the Serial buffer is filled
				delay(COMMANDDELAY);

				fillBuffer( commandBuffer, COMBUFFERSIZE );

				// Convert buffer to integer
				uint8_t newR = atoi( commandBuffer );

				// Display moving status indicator
				Serial.print("Setting voltage reference to: ");
				Serial.println(newR);

				setVoltageReference(newR);
				}
				break;

			case 'e':			// 'e' for new trigger event setting
			case 'E': {
				// Wait for COMMANDDELAY ms to be sure that the Serial buffer is filled
				delay(COMMANDDELAY);

				fillBuffer( commandBuffer, COMBUFFERSIZE );

				// Convert buffer to integer
				uint8_t newE = atoi( commandBuffer );

				// Display moving status indicator
				Serial.print("Setting trigger event to: ");
				Serial.println(newE);

				triggerEvent = newE;
				setTriggerEvent(newE);
				}
				break;

			case 'w':			// 'w' for new wait setting
			case 'W': {
				// Wait for COMMANDDELAY ms to be sure that the Serial buffer is filled
				delay(COMMANDDELAY);

				fillBuffer( commandBuffer, COMBUFFERSIZE );

				// Convert buffer to integer
				uint8_t newW = atoi( commandBuffer );

				// Display moving status indicator
				Serial.print("Setting waitDuration to: ");
				Serial.println(newW);

				waitDuration = newW;
				}
				break;

			case 't':			// 'w' for new threshold setting
			case 'T': {
				// Wait for COMMANDDELAY ms to be sure that the Serial buffer is filled
				delay(COMMANDDELAY);

				fillBuffer( commandBuffer, COMBUFFERSIZE );

				// Convert buffer to integer
				uint8_t newT = atoi( commandBuffer );

				// Display moving status indicator
				Serial.print("Setting threshold to: ");
				Serial.println(newT);

				threshold = newT * 4096 / 256;

				}
				break;

			case 'd':			// 'd' for display status
			case 'D':
				printStatus();
				break;

			default:
				// Display error message
				Serial.print("ERROR: Command not found, it was: ");
				Serial.println(theChar);
				error();
		}
	}
}

void printData(uint8_t *buffer, int offset, int size) {
	//Serial.write( (uint8_t *)ADCBuffer + ADCCounter, ADCBUFFERSIZE - ADCCounter );
	//Serial.write( (uint8_t *)ADCBuffer, ADCCounter );
	int i;
	for(i=offset; i<size ; i++)
		Serial.write(*(buffer + i));
	for(i=0; i<offset; i++)
		Serial.write(*(buffer + i));

}


void initTesttimer(void) {
	Timer2.pause();
	Timer2.setPeriod(1000); // 1khz
	Timer2.setChannel1Mode(TIMER_OUTPUT_COMPARE);
	Timer2.setCompare(TIMER_CH2,1);
	// start the timer
	Timer2.refresh();
	Timer2.resume();
	//timer2.attachCompare1Interrupt(handler2);
}

void initAdctimer(void) {
	Timer1.pause();
	Timer1.setPrescaleFactor(1); //72mhz
	Timer1.setPeriod(500); // 500us, 2khz
	Timer1.setChannel1Mode(TIMER_OUTPUT_COMPARE);
	TIMER1->regs.adv->EGR |= 0b00000010; //generate CC1 event
	Timer1.attachCompare1Interrupt(adchandle);
	//Timer1.setCompare(TIMER_CH1,1);
	//Timer1.refresh();
	//Timer1.resume();

}

void adchandle(void) {

	uint16_t data = adc_read(ADC1, 1);

	if(!wait) {
		switch (triggerEvent) {
			case TRG_FALLINGEDGE:
				if (data < threshold)
					triggered();
				break;
			case TRG_TOGGLE:
				if (data > threshold)
					triggerEvent = TRG_FALLINGEDGE;
				else
					triggerEvent = TRG_RISINGEDGE;
				break;
			case TRG_RISINGEDGE:
			default:
				if (data > threshold)
					triggered();
				break;
		}

	}

	// When ADCL is read, the ADC Data Register is not updated until ADCH
	// is read. Consequently, if the result is left adjusted and no more
	// than 8-bit precision is required, it is sufficient to read ADCH.
	// Otherwise, ADCL must be read first, then ADCH.
	ADCBuffer[ADCCounter] = (data >> 4) & 0xff;

	ADCCounter = ( ADCCounter + 1 ) % ADCBUFFERSIZE;

	if ( wait )
	{
		if ( stopIndex == ADCCounter )
		{
			// Freeze situation
			// Disable ADC and stop Free Running Conversion Mode
			stopADC();

			freeze = true;
		}
	}

}

//called when trigger fires
void triggered()
{
	// Disable Analog Comparator interrupt

	// Turn on errorPin
	//digitalWrite( BOARD_LED_PIN, HIGH );

	wait = true;
	stopIndex = ( ADCCounter + waitDuration ) % ADCBUFFERSIZE;

}



void initADC(void) {

	//adc_init(ADC1);
	//this sample rate only determines how long stm32 samples each inputs,
	//it does not affect the sample rate which is driven by the timer
	//this is using ADC_SMPR_7_5 adc clocks per sample (it takes 14 adc cycles for one conversion)
	//if this is too slow, change it to ADC_SMPR_1_5, sampling quality may become worse
	adc_set_sample_rate(ADC1,ADC_SMPR_7_5);
	adc_calibrate(ADC1);
	adc_enable(ADC1);

}


void startADC(void) {
	wait = false;
	//start timer 1 which triggers the ADC conversion
	Timer1.refresh();
	//Timer1.setCompare(1,Timer1.getOverflow());
	Timer1.resume();
}


void stopADC(void) {
	//stop timer 1 which triggers the ADC conversion
	Timer1.pause();
}

#define UNOSYSCLK 16000000

void setADCPrescaler( uint8_t prescaler ) {
	dshow("# setADCPrescaler()");
	dprint(Prescaler);
	// These bits determine the division factor between the system clock
	// frequency and the input clock to the ADC.
	// prescaler: 2, 4, 8, 16, 32, 64, 128 (default)
	prescaler = (prescaler == 0 ? 128 : prescaler);

	int adclk = UNOSYSCLK / prescaler;
	int samprate = adclk / 13; //13 clock cycles per conversion on uno
	setSamprate(samprate);
}

/*
 * ref: RM0008 stm32f103 ref manual chapter 11 ADC
 *
 * stm32f103 max adc clock is 14 mhz
 * it takes 14 adc clocks for 1 conversion - 1 msps max
 * there are 2 adcs on stm32f103c{8,B}, current setup is for 1 adc
 *
 * however, as sysclock is set at 72 mhz
 * available adc prescalers are 2, 4, 6, 8.
 * 6 is the best fitting prescaler which gives 12 mhz adc clock
 * however, it still takes 14 adc clocks for 1 conversion ~ 857100 samp per sec max
 *
 * use Timer1 to trigger the adc conversion, hence, the settings here are for Timer1
 */

#define MAXSAMPRATE 857100
#define MAX_RELOAD ((1 << 16) - 1)

void setSamprate(int samprate) {
	samprate = (samprate > MAXSAMPRATE ? MAXSAMPRATE : samprate );
	samprate = (samprate == 0 ? 1000: samprate ); // set 1ksps default

    uint32 period_cyc = F_CPU / samprate;
    uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
    uint16 overflow = (uint16)((period_cyc + (prescaler / 2)) / prescaler);
    Timer1.setPrescaleFactor(prescaler);
    Timer1.setOverflow(overflow);
}

void setTriggerEvent( uint8_t TriggerEvent )
{
	dshow("# setTriggerEvent()");
	dprint(event);
	//	TriggerEvent:
	//	0	Toggle
	//	2	Falling edge
	//	3	Rising edge (default
	triggerEvent = TriggerEvent;
}

void setVoltageReference( uint8_t reference ) {
	dshow("# setVoltageReference()");
	dprint(reference);
	// These bits select the voltage reference for the ADC. If these bits
	// are changed during a conversion, the change will not go in effect
	// until this conversion is complete (ADIF in ADCSRA is set). The
	// internal voltage reference options may not be used if an external
	// reference voltage is being applied to the AREF pin.
	//	Voltage reference
	//	0	AREF, Internal Vref turned off
	//	1	AVCC with external capacitor at AREF pin (default)
	//	2	Internal 1.1V Voltage Reference with external (1.2v on stm32f103)
	//			capacitor at AREF pin

	// atmega ref manual:
	// The reference voltage for the ADC (V REF) indicates the conversion range
	// for the ADC. Single ended channels that exceed VREF will result in codes
	// close to 0x3FF. VREF can be selected as either AVCC,
	// internal 1.1V reference, or external AREF pin.

	// on stm32f103c{8,B}
	// VREF+ = VDD = 3.3v, VREF- = VSS = GND (0v)
	// hence the full range is 0 gnd - 4096 (3.3v)
	// there is no sub-selection of range

    // vrefint 1.2v is mainly to calibrate the adc as vref int can be read
	// to get the adc value. this would be more accurate in case VREF- and VREF+
	// isn't at gnd and 3.3v, hence the range is smaller and can be more accurately
	// computed using 4096 * 1.2  / adc_vrefint_value
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

//-----------------------------------------------------------------------------
// fillBuffer
//-----------------------------------------------------------------------------
// Fills the given buffer with bufferSize chars from a Serial object

void fillBuffer( char *buffer, byte bufferSize, USBSerial *serial )
{
	// Clean buffer
	memset( (void *)buffer, '\0', sizeof(char) * bufferSize );

	dprint(serial->available());

	byte limit = ( bufferSize < serial->available() ) ? bufferSize : serial->available();

	dprint(serial->available());
	dprint(bufferSize);
	dprint(limit);

	// Fill buffer
	for ( byte i = 0; i < limit; i++ ) {
		dprint(serial->available());
		dprint(i);
		dprint(buffer);

		buffer[i] = serial->read();
	}
}

void printStatus( void )
{
	Serial.print("Buffer size: ");
	Serial.println(ADCBUFFERSIZE);
	Serial.print("Baud rate: ");
	Serial.println(BAUDRATE);
	Serial.print("Wait duration: ");
	Serial.println(waitDuration);
	Serial.print("Prescaler: ");
	Serial.println(prescaler);
	Serial.print("Trigger event: ");
	Serial.println(triggerEvent);
	Serial.print("Threshold: ");
	Serial.println(threshold/16);
	//Serial.print("adctrigger:");
	//Serial.println(badc?"yes":"no");
}





/*
 * TODO replace with MIT lic
 */
#include <Arduino.h>
#include "CAdcMgr.h"
#include "WordConv.h"
#include "util.h"

/* global variables and defines */
#define DEBUG		0
#define LEDON  	LOW
#define LEDOFF  HIGH
#define IDENT   "STM32DAQF4v1.0"


/*
 * Girino parameters and variables
 */
//BAUD rate is not really needed as stm32duino use usb-serial
#define BAUDRATE	115200
uint8_t prescaler = 128;


/*
 * Global Variables
 */

int fled = 0;
uint8_t errorPin = LED_BUILTIN;

#define PARAMLEN 20
char Param[PARAMLEN];
#define CTIMEOUT	10000	//ms timeout waiting for command/param


void initPins(uint8_t pin);
void setADCPrescaler( uint8_t prescaler );
void initTesttimer(void);

char* readParam();
int readParamInt();
int procParam(void (*callback)(uint16_t), const char *response);

void handlePrescaler(uint16_t p);
void handleSamprate(uint16_t samprate);
void handleTrigger(uint16_t p);
void handleWait(uint16_t p);
void handleThreshold(uint16_t p);
void handleNoop(uint16_t p);
void handleFieldsize(uint16_t p);

void dumpcmdG(void);
void printMem(void);
void sleep(uint16_t ms);

CAdcMgr AdcMgr;

void setup(void) {

	//usb-serial
	Serial.begin();

	initPins(errorPin);
	digitalWrite(errorPin, LEDON);

	// Activate interrupts
	interrupts();

	// setup timer 2 to generate test signals, comment this if not required
	initTesttimer();

	// Clear buffers
	memset(Param, 0, PARAMLEN);

	AdcMgr.initADC(ADC_CHANNEL);

	while (!Serial.isConnected());

	Serial.print(IDENT);
	Serial.println(" ready");

}

void loop(void) {

	// If frame is done, then it is time to send the buffer to the serial port
	AdcMgr.doframedone();
	// note error handling codes are present but are commented out
	// it use a different protocol which sent a code byte before sending results
	// to use that edits need to be done in
	// 1) AdcMgr.doframedone() to sent a code byte (value zero) before data
	// 2) AdcMgr.initADC() to setup the overflow isr handler
	// 3) uncomment the next line
	// AdcMgr.doerr();

	if ( Serial.available()) {
		digitalWrite(errorPin, LEDON);
		// Read the incoming byte
		char c = Serial.read();
		// Parse character
		switch (c) {
		case 's': // 's' start ADC conversions
			// note: it is not necessary to issue 'S' (stop) after 's'
			//       it fills and returns a fixed buffer sized of byte
			//       data.
			//Serial.println("ADC conversions started");
			AdcMgr.startConv();
			break;

		case 'S': // 'S' stop ADC conversions
			//Serial.println("ADC conversions stopped");
			AdcMgr.stopConv();
			break;

		case 'p': // 'pnnn' prescalar settings (nnn ascii integer)
			procParam(handlePrescaler, "Setting prescaler to: ");
			break;

		case 'A': // 'Annnn' set sample rates where nnnn is the sample rates in khz
			/* note that while the parameter takes nnnn (4 ascii integer digits) as
			 * the sample rate. sample rates are not arbitrary continuous sample rates
			 * there are only specific fixed ranges and fixed rates achievable
			 * for more info see AdcMgr.setSamprate
			 */
			procParam(handleSamprate, "Setting samplerate to: ");
			break;

		case 'r': // 'rnnn' voltage reference setting
			//note not implemented, provided only for compatibility
			procParam(handleNoop, "Setting voltage reference to: ");
			break;

		case 'e':	// 'ennn' set trigger event setting, nnn ascii integer
			/* TriggerEvent:
			 *  0	Toggle
			 *	2	Falling edge
			 *	3	Rising edge (default
			 *  4   None
			 */
			procParam(handleTrigger, "Setting trigger event to: ");
			break;

		case 'W':	// 'fnnn' set word size 8 / 12 / 16 bits
			procParam(handleFieldsize, "Setting word size to: ");
			break;

		case 'w': // 'wnnnn' wait setting, nnnn ascii integer
			/* 'wait' settings specify the number of samples to return
			 * it is zero padded in the front, the 's' (start acquisition)
			 * command always returns a fixed ADCBUFFERSIZE number of samples (bytes)
			 *
			 * samples returned = ADCBUFFERSIZE - w
			 * max length is buffer_size
			 */
			procParam(handleWait, "Setting waitDuration to: ");
			break;

		case 't':	// 'tnnn' threshold setting
			procParam(handleThreshold, "Setting threshold to: ");
			break;

		case 'd': // 'd' Girino dump command, displays a list of parameter values
			dumpcmdG();
			break;

		case 'I': // 'I' returns identity
			Serial.println(IDENT);
			break;

		case 'z': // 'z' displays free stack
			printMem();
			break;

		case 'Z': // reset
			//  it does not 'reset' everything, it merely reset some variables/parameters
			AdcMgr.reset();
			break;

		default:
			// Display error message
			Serial.print("ERROR: Command not found, it was: ");
			Serial.println(c);
			error();
		}
		digitalWrite(errorPin, LEDOFF);
	}

	sleep(1);
}

int procParam(void (*callback)(uint16_t), const char *response) {
	uint16_t param = readParamInt();

	callback(param);

	//this return message is required for compatibility
	Serial.print(response);
	Serial.println(param);

	return 0;
}

char* readParam() {

	memset(Param,0,PARAMLEN);

	uint16_t i=0;
	uint32_t timeout = millis() + CTIMEOUT;
	while (millis() < timeout) {
		if (Serial.available()) {
			uint8_t c = Serial.read();
			Serial.print((char) c);
			if (c == 0x08 || c == 0x7F) {
				Param[--i] = 0;
			} else if (c == '\r' || c == '\n') {
				return (char*) &Param;
			} else if (i < PARAMLEN)
				Param[i++] = c;
		}
		sleep(1);
	}
	return (char*) &Param;
}

int readParamInt() {
	// Convert buffer to integer
	return atoi(readParam());
}



void handlePrescaler(uint16_t p) {
	setADCPrescaler((uint8_t) p);
}

void handleSamprate(uint16_t samprate) {
	AdcMgr.setSamprate(samprate * 1000);
}

void handleTrigger(uint16_t p) {
	AdcMgr.setTriggerEvent((uint8_t) p);
}

void handleWait(uint16_t p) {
	AdcMgr.setWaitDur(p);
}

void handleThreshold(uint16_t p) {
	//AdcMgr.setThreshold(vgirinotostm((uint8_t) p));
	AdcMgr.setThreshold(wconv.fromWord(p));
}

void handleFieldsize(uint16_t p) {
	wconv.setWordSize(p);
}

void handleNoop(uint16_t p) {
}

void initPins(uint8_t pin) {
	pinMode(pin, OUTPUT_OPEN_DRAIN); //PC13
}

void timer_trig() {
	fled = fled ? 0 : 1;
	digitalWrite(errorPin, fled);
	TIMER2->regs.gen->SR = 0; //clear interrupt flags
}

void initTesttimer(void) {

	//Timer2.init();
	Timer2.pause();
	//Timer2.setPrescaleFactor(1); //36mhz
	//enable preload
	TIMER2->regs.gen->CR1 |= TIMER_CR1_ARPE;
	timer_set_mode(TIMER2, TIMER_CH2, TIMER_PWM);
	Timer2.setPeriod(1000); // 1khz
	Timer2.setCompare(TIMER_CH2, Timer2.getOverflow() / 2);

	//Timer 2 Channel 2 timer output is on PA1
	//setup pin PA1 for alt function output
	gpio_set_mode(PA1, GPIO_AF_OUTPUT_PP);
	//select afio function timer2
	gpio_set_af_mode(PA1, GPIO_AFMODE_TIM1_2);

	//Timer2.attachInterrupt(TIMER_CH2, timer_trig);
	// start the timer
	Timer2.refresh();
	Timer2.resume();

}

#define UNOSYSCLK 16000000

void setADCPrescaler(uint8_t prescaler) {
	// These bits determine the division factor between the system clock
	// frequency and the input clock to the ADC.
	// prescaler: 2, 4, 8, 16, 32, 64, 128 (default)
	prescaler = (prescaler == 0 ? 128 : prescaler);

	int adclk = UNOSYSCLK / prescaler;
	int samplerate = adclk / 13; //13 clock cycles per conversion on uno

	AdcMgr.setSamprate(samplerate);
}

//-----------------------------------------------------------------------------
void error(void) {
	digitalWrite(errorPin, HIGH);
	delay(500);
	digitalWrite(errorPin, LOW);
	delay(250);
	digitalWrite(errorPin, HIGH);
	delay(500);
	digitalWrite(errorPin, LOW);
}


void dumpcmdG(void) {
	Serial.print("Word size: ");
	Serial.println(wconv.getWordSize());
	Serial.print("Buffer size: ");
	Serial.println(ADCBUFFERSIZE);
	Serial.print("Baud rate: ");
	Serial.println(BAUDRATE);
	Serial.print("Prescaler: ");
	Serial.println(prescaler);
	/*
	 Serial.print("Voltage reference:");
	 Serial.println(" 0");
	 */
	Serial.print("Trigger event: ");
	Serial.println(AdcMgr.triggerEvent);
	Serial.print("Wait duration: ");
	Serial.println(AdcMgr.m_samplecounts);
	Serial.print("Threshold: ");
	//Serial.println(vstmtogirino(AdcMgr.threshold));
	Serial.println(wconv.toWord(AdcMgr.threshold));
}

void printMem(void) {
	Serial.print("Freestack:");
	Serial.println(FreeStack());
}

void sleep(uint16_t ms) {
	for(uint16_t i=0; i<ms;i++)
		asm("wfi");
}



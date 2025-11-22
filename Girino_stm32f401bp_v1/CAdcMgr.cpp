/*
 * TODO: place MIT license
 */
#include <Arduino.h>
#include <adc.h>
#include <libmaple/dma.h>
#include "CAdcMgr.h"
#include "WordConv.h"

/*
 * ADC and DMA buffers
 */
uint16_t ADCBuffer[ADCBUFFERSIZE];

/*
 * class implementations
 */

CAdcMgr *CAdcMgr::s_me = NULL;

CAdcMgr::CAdcMgr() {
	s_me = this;
	reset();
}

CAdcMgr::~CAdcMgr() {
}

CAdcMgr* CAdcMgr::getInstance() {
	return s_me;
}


void CAdcMgr::reset() {
	stopConv();
	m_channel = ADC_CHANNEL;
	m_adcops = AdcOps::Range500;
	m_samplecounts = BufferSize - 32;
	m_state = RunState::INIT;
	bfirsttrig = false;
	btriggered = false;
	triggerEvent = Trig::TRG_TOGGLE;
	//threshold = vgirinotostm(127);
	threshold = 0x7ff;
	m_errnum = 0;
}

// ISR callbacks
// note these are static methods
void CAdcMgr::isradctimer(void) {
	//start conversion when called
	//clear status registers
	ADC1->regs->SR = 0;
	//ADC1->regs->CR2 |= ADC_CR2_SWSTART;
	adc_start_convert(ADC1);
}

void CAdcMgr::isrtrigger(void) {
	CAdcMgr::getInstance()->triggered();
}

void CAdcMgr::isrdma(void) {
	CAdcMgr::getInstance()->dmacmplthandle();
}

void CAdcMgr::isrovr(void) {
	CAdcMgr::getInstance()->ovrthandle();
}

// conversion routines and methods

/*
 * start conversion
 */
void CAdcMgr::startConv() {
	btriggered = false;
	m_state = RunState::INIT;

	adc_disable(ADC1);
	//disable dma during the initial triggering phase
	adc_dma_disable(ADC1);
	//clear overflow
	ADC1->regs->SR &= ~ ADC_SR_OVR;
	//adc_disable(ADC2);

	clearBuf();

	adc_clear_continuous(ADC1);
	adc_enable(ADC1);


	if(m_adcops == AdcOps::Fixed2400 ||
	   m_adcops == AdcOps::Fixed1333 ||
	   m_adcops == AdcOps::Fixed900 ||
	   m_adcops == AdcOps::Fixed667 ) {
		//setup dma
		//initDMA(m_samplecounts,1);
		dmaSetsize(m_samplecounts);

		// set adc in continuous mode,
		//adc_cont_enable(ADC1);
		adc_set_continuous(ADC1);

		//clear status flags
		ADC1->regs->SR = 0;


		if( triggerEvent == Trig::TRG_NONE) {
			btriggered = true;
			//disable analog watch dog
			adc_awd_disable(ADC1);

			//start adc dma recording
			adc_dma_enable(ADC1);
		} else {
			//setup the triggers using analog watch dog
			initAWDtriggers();
		}

		//start conversion
		//ADC1->regs->CR2 |= ADC_CR2_SWSTART;
		adc_start_convert(ADC1);

		m_state = RunState::CONV;

/*	dual interleaved mode is not available on F401
    } else if (m_adcops == AdcOps::Fixed2400 ||
			m_adcops == AdcOps::Fixed933 ) {
		//ADC1,ADC2 dual fast interleave mode
		adc_enable(ADC2);
		//setup dma
		//initDMA(m_samplecounts/2,2);
		//32 bit words
		startDMA(m_samplecounts/2);

		// set adc in continuous mode,
		//adc_cont_enable(ADC1);
		adc_set_continuous(ADC1);

		//start conversion
		ADC1->regs->CR2 |= ADC_CR2_SWSTART;
*/
	} else { //AdcOps::Range500
		//setup dma
		//initDMA(m_samplecounts,1);
		dmaSetsize(m_samplecounts);


		if( triggerEvent == Trig::TRG_NONE) {
			btriggered = true;
			//disable analog watch dog
			adc_awd_disable(ADC1);

			//start adc dma recording
			adc_dma_enable(ADC1);
		} else {
			//setup the triggers using analog watch dog
			initAWDtriggers();
		}

		//clear status flags
		ADC1->regs->SR = 0;

		//start timer 1 which triggers the ADC conversion
		Timer1.refresh();
		Timer1.resume();

		m_state = RunState::CONV;

	}
}

void CAdcMgr::stopConv() {

	//stop timer 1 which triggers the ADC conversion
	Timer1.pause();
	//adc continuous mode, we simply clear ADON
	adc_disable(ADC1);
	//adc_disable(ADC2);
}




//called when trigger fires
void CAdcMgr::triggered() {
	// Turn on errorPin
	//digitalWrite( BOARD_LED_PIN, HIGH );

	if(!bfirsttrig) {
		bfirsttrig = true;
		if (triggerEvent == Trig::TRG_RISINGEDGE) {
			setawdhighthres(threshold);
			return;
		} else if (triggerEvent == Trig::TRG_FALLINGEDGE) {
			setawdlowthres(threshold);
			return;
		}
	} // else TRG_TOGGLE

	btriggered = true;
	//disable analog watch dog
	adc_awd_disable(ADC1);

	//start adc dma recording
	adc_dma_enable(ADC1);
}


void CAdcMgr::dmacmplthandle(void) {

	if(m_state == RunState::CONV) {
		stopConv();
		m_state = RunState::DONE;
	}
	dma_clear_isr_bits(DMA2,DMA_STREAM0);

	return;
}

void CAdcMgr::ovrthandle(void) {
	stopConv();
	m_errnum = 10;
	m_state = RunState::ERR;
}

void CAdcMgr::doframedone(void) {

	// If frame is done, then it is time to send the buffer to the serial port
	if ( m_state == RunState::DONE )
	{
		//this is for a different protocol in which a code byte is sent before data
		//code 0 is for data
		//the purpose of which is to enable error handling
		//code 1 is for error
		//Serial.write((uint8_t) 0);

		printData();

		btriggered = false;
		m_state = RunState::INIT;

		// Clear buffer
		clearBuf();
	}

}


void CAdcMgr::printData(void) {
	uint16_t *buffer = ADCBuffer;
	int blanksize = BufferSize - m_samplecounts;

	/*
	//pad with zeros
	for(int i=0; i< blanksize ; i++) {
		Serial.write((uint8_t) 0);
	}

	buffer = ADCBuffer;
	for(int i=0; i<m_samplecounts ; i++) {
		Serial.write(vstmtogirino(*(buffer + i)));
	}
	*/
	wconv.printWordbuf(buffer, m_samplecounts, blanksize);

}

void CAdcMgr::doerr() {
	if(m_state == RunState::ERR) {
		error();
		//this is for a different protocol in which a byte is sent before data
		//code one is an error
		Serial.write((uint8_t) 1);
		Serial.write((uint8_t) m_errnum);
		clearBuf();
		m_state = RunState::INIT;
	}
}


/**
 * @brief initialise the ADC channel and setup the pin
 *
 * @param int8_t channel
 * 		  take note of the ADC channel to pin map
 * 		  see specs on datasheet
 * 		  https://www.st.com/en/microcontrollers-microprocessors/stm32f103cb.html
 * 		  channel 0-7 : PA0-PA7
 * 		  channel 8-9 : PB0-PB1
 * 		  channel 10-15 : PC0-PC5
 *
 */
void CAdcMgr::initADC(int8_t channel) {

	m_state = RunState::INIT;
	//powerup initialization
	//adc_init(ADC1);

	// note this sketch requires sysclk to be 72 mhz
	// note accordingly to F401 datasheet
	// max ADC clock (typ) 30 mhz, (max) 36 mhz
	// pclk 72 mhz / 2 = 36mhz adc clock

	// Tconv = sampling time + 12 cycles
	//       = 3 + 12 = 15 cycles
	// samples per sec: 36 / 15 = 2.4 msps

	adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
	adc_set_sampling_time(ADC1, ADC_SMPR_3);

	//F4 don't have calibrate !
	//adc_calibrate(ADC1);

	configADCpin(channel);

	//setup dma
	dma_init(DMA2); //clock DMA
	// dma end transfer interrupt
	dma_attach_interrupt(DMA2, DMA_STREAM0, isrdma);
	// analog watch dog interrupt
	adc_attach_interrupt(ADC1, ADC_AWD, isrtrigger);
	// overrun interrupt
	//adc_attach_interrupt(ADC1, ADC_OVR, isrovr);


	//Adc timer is used to drive the ADC sampling
	initAdctimer();

	setops(m_adcops);

}


void CAdcMgr::setops(AdcOps ops) {

	m_state = RunState::INIT;

	// configure 'reset status'
	adc_disable(ADC1);
	//adc_disable(ADC2);
	adc_dma_disable(ADC1);
	adc_awd_disable(ADC1);
	adc_clear_continuous(ADC1);
	//clear dual mode register
	//adc_clear_dual(ADC1);
	// adc clock = 72 mhz / 2 = 36mhz
	// Tconv = sample time + 12 cycle
	//       = 3 + 12 = 15 cycles
	// msps = 36 / 15 = 2.4 msps
	adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
	adc_set_sampling_time(ADC1, ADC_SMPR_3);


	// clear buffer
	clearBuf();

	/* dual interleave is not available on F401
	if(ops == AdcOps::Fixed1400 ||
	   ops == AdcOps::Fixed933) {

		  if(ops == AdcOps::Fixed933) {
			  // adccclock = 84 mhz / 6 = 14mhz
			  // Tconv = sample time + 12 cycle
			  //       = 3 + 12 = 15 cycles
			  // msps = 14 / 15 = 933.33 ksps
			  adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);
		  }

		  //note that continuous mode for both ADC1 and ADC2
		  //needs to be set at acquisition start
		  adc_set_sampling_time(ADC1, ADC_SMPR_3);
		  //F401 has only 1 ADC
		  //adc_set_sampling_time(ADC2, ADC_SMPR_3);

		  adc_set_reg_seqlen(ADC1, 1);
		  ADC1->regs->SQR3 = m_channel;

		  //adc_set_reg_seqlen(ADC2, 1);
		  //ADC2->regs->SQR3 = m_channel;
		  //adc_set_dual(ADC1,ADC_DUAL_FASTINT); // fast interleave mode

		  //32 bit words
		  initDMA(2);

		  m_adcops = ops;

	} else */
	if(ops == AdcOps::Fixed2400 ||
	   ops == AdcOps::Fixed1333 ||
	   ops == AdcOps::Fixed900 ||
	   ops == AdcOps::Fixed667 ) {
		if(ops == AdcOps::Fixed1333) {
			adc_set_sampling_time(ADC1, ADC_SMPR_15);
		} else if (ops == AdcOps::Fixed900) {
			adc_set_sampling_time(ADC1, ADC_SMPR_28);
		} else if (ops == AdcOps::Fixed667) {
			adc_set_prescaler(ADC_PRE_PCLK2_DIV_4);
			adc_set_sampling_time(ADC1, ADC_SMPR_15);
		}// else Fixed2400

		//setup the analog watch dog
		adc_awd_set_high_limit(ADC1, 0x0fff);
		adc_awd_set_low_limit(ADC1, 0);
		adc_awd_enable_channel(ADC1, m_channel);
		adc_awd_enable(ADC1);

		//16 bit words
		initDMA(1);

		m_adcops = ops;

	} else { // Range500

		//this sample time only determines how long stm32 samples each inputs,
		//it does not affect the sample rate which is driven by the timer
		//adc clock = 72 / 2 = 36 mhz
		//this is using ADC_SMPR_15 adc clocks per sample
		//it takes sample_time + 12 adc cycles for one conversion
		//so 36 mhz (adc clk) / (28 + 12) ~ 900 khz
		//if this is too slow, change it to ADC_SMPR_3 or 12 or 28  , sampling quality may become worse
		adc_set_sampling_time(ADC1, ADC_SMPR_28);

		//setup the analog watch dog
		adc_awd_set_high_limit(ADC1, 0x0fff);
		adc_awd_set_low_limit(ADC1, 0);
		adc_awd_enable_channel(ADC1, m_channel);
		adc_awd_enable(ADC1);

		adc_enable(ADC1);

		//16 bit words
		initDMA(1);

		m_adcops = ops;
	}

}


void CAdcMgr::dmaSetsize(uint16_t dmacount) {
	dma_disable(DMA2, DMA_STREAM0);
    dma_set_num_transfers(DMA2, DMA_STREAM0, dmacount);
    dma_clear_isr_bits(DMA2,DMA_STREAM0);
    dma_enable(DMA2, DMA_STREAM0); // Enable the channel and start the transfer.
}


void CAdcMgr::initDMA(uint8_t words) {
	dma_xfer_size xfer_size;
	uint32_t dmaflags;
    const void *memory_address;

	dma_disable(DMA2, DMA_STREAM0);
	dma_clear_isr_bits(DMA2,DMA_STREAM0);

	if(words == 2) {
		xfer_size = DMA_SIZE_32BITS;
	} else {
		xfer_size = DMA_SIZE_16BITS;
	}

	//single mode, enable the transfer complete interrupt handler
	dmaflags = (DMA_MINC_MODE | DMA_TRNS_CMPLT);
	memory_address = ADCBuffer;
//		(DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT);

    dma_setup_transfer(DMA2,
    		DMA_STREAM0,
			DMA_CH0,
			xfer_size,
			&ADC1->regs->DR,
			memory_address,
			NULL,
			dmaflags);// Receive buffer DMA
    dma_enable(DMA2, DMA_STREAM0); // Enable the channel and start the transfer.
}

void CAdcMgr::clearBuf(void) {
	memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) );
}


void CAdcMgr::initAdctimer(void) {

	//Timer1.init();
	Timer1.pause();
	Timer1.setPrescaleFactor(1); //72mhz
	Timer1.setPeriod(500); // 500us, 2khz
	//Timer1.setMode(1,TIMER_OUTPUT_COMPARE);
	Timer1.setMode(1,TIMER_PWM);
	Timer1.attachInterrupt(1, isradctimer);
	//turn off timer output
	TIMER1->regs.adv->BDTR &= ~ TIMER_BDTR_MOE;
	//clear status register
	TIMER1->regs.adv->SR = 0;

}

void CAdcMgr::configADCpin(int8_t channel) {
	m_channel = channel;

	//setup channel for adc
	adc_set_reg_seqlen(ADC1, 1);
	ADC1->regs->SQR1 = 0;
	ADC1->regs->SQR2 = 0;
	ADC1->regs->SQR3 = channel & 0x1f;

	//configure the input pin
	switch (channel) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		//gpio_set_mode(GPIOA, channel, GPIO_INPUT_ANALOG);
		gpio_set_mode(channel, GPIO_INPUT_ANALOG);
		break;
	case 8:
	case 9:
		//gpio_set_mode(GPIOB, channel - 8, GPIO_INPUT_ANALOG);
		gpio_set_mode(16 + channel - 8, GPIO_INPUT_ANALOG);
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		//gpio_set_mode(GPIOC, channel - 10, GPIO_INPUT_ANALOG);
		gpio_set_mode(32 + channel - 10, GPIO_INPUT_ANALOG);
		break;
	default:
		break;
	}
}


//setup the triggers using analog watch dog
void CAdcMgr::initAWDtriggers(void) {
	adc_awd_disable(ADC1);
	if(triggerEvent == Trig::TRG_TOGGLE) {
		uint16_t data = adc_read(ADC1,m_channel);
		if (data < threshold) //rising edge trigger
			setawdhighthres(threshold);
		else //falling edge
			setawdlowthres(threshold);
		bfirsttrig = true;
	} else if (triggerEvent == Trig::TRG_FALLINGEDGE) {
		setawdhighthres(threshold);
		bfirsttrig = false;
	} else  if (triggerEvent == Trig::TRG_RISINGEDGE) {
		setawdlowthres(threshold);
		bfirsttrig = false;
	} else { //NONE
		bfirsttrig = true;
		btriggered = true;
	}

	adc_awd_enable(ADC1);
}

void CAdcMgr::setawdlowthres(uint16_t threshold) {
	adc_awd_set_high_limit(ADC1, 0x0fff);
	adc_awd_set_low_limit(ADC1, threshold);
}


void CAdcMgr::setawdhighthres(uint16_t threshold) {
	adc_awd_set_high_limit(ADC1, threshold);
	adc_awd_set_low_limit(ADC1, 0);
}

#define MAX_RELOAD ((1 << 16) - 1)

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
void CAdcMgr::setSamprate(int samplerate) {
	samplerate = (samplerate == 0 ? 1000: samplerate ); // set 1ksps default

	if (samplerate > 1333000) {
		setops(AdcOps::Fixed2400);
	} else if (samplerate > 900000) {
		setops(AdcOps::Fixed1333);
	} else if (samplerate > 667000) {
		setops(AdcOps::Fixed900);
	} else if (samplerate > 500000) {
		setops(AdcOps::Fixed667);
	} else {
		setops(AdcOps::Range500);

		//uint32 period_cyc = F_CPU / samplerate;
		uint32 period_cyc = CYCLES_PER_MICROSECOND * 1000000 / samplerate;
		uint16 prescaler = (uint16) (period_cyc / MAX_RELOAD + 1);
		uint16 overflow = (uint16) ((period_cyc + (prescaler / 2)) / prescaler);
		Timer1.setPrescaleFactor(prescaler);
		Timer1.setOverflow(overflow);

	}
}

void CAdcMgr::setWaitDur(uint16_t waitdur) {
	m_samplecounts = waitdur < BufferSize ? waitdur : BufferSize;
}

//	TriggerEvent:
//	0	Toggle
//	2	Falling edge
//	3	Rising edge (default
//  4   None
void CAdcMgr::setTriggerEvent(uint16_t trigEvt) {
	triggerEvent = (Trig) trigEvt;

}

void CAdcMgr::setThreshold(uint16_t thres) {
	//this->threshold = threshold;
	threshold = thres;
}

void CAdcMgr::setCheannel(uint8_t channel) {
	m_channel = channel;
}




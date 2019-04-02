//-----------------------------------------------------------------------------
// Girino.ino
//-----------------------------------------------------------------------------
// ported to stm32duino libmaple core
// implementation nearly completely re-written
// different from original release and specifically taylored
// to stm32f103c{8,b} hardware, only the protocol is left intact
//
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
#include "util_adc.h"
#include <libmaple/dma.h>
#include "CAdcMgr.h"

//-----------------------------------------------------------------------------
// ADC and DMA buffers
//-----------------------------------------------------------------------------
uint16_t ADCBuffer[ADCBUFFERSIZE];

//-----------------------------------------------------------------------------
// global variables
//-----------------------------------------------------------------------------


CAdcMgr *CAdcMgr::s_me = NULL;

CAdcMgr::CAdcMgr() {
	m_channel = ADC_CHANNEL;
	m_adcops = AdcOps::Range500;
	m_samplecounts = BufferSize - 32;
	triggerEvent = TRG_TOGGLE;
	threshold = vgirinotostm(127);

	s_me = this;
}

CAdcMgr::~CAdcMgr() {
}

void CAdcMgr::startConv() {
	btriggered = false;

	adc_disable(ADC1);
	adc_disable(ADC2);
	adc_dma_disable(ADC1);

	clearBuf();

	adc_cont_disable(ADC1);
	adc_enable(ADC1);

	//setup the triggers using analog watch dog
	initAWDtriggers();

	//disable dma during the initial triggering phase
	adc_dma_disable(ADC1);


	if(m_adcops == AdcOps::Fixed600 ||
	   m_adcops == AdcOps::Fixed857) {
		//setup dma
		initDMA(m_samplecounts,1);

		// set adc in continuous mode,
		adc_cont_enable(ADC1);
		//start conversion
		ADC1->regs->CR2 |= ADC_CR2_SWSTART;

	} else if (m_adcops == AdcOps::Fixed1714 ||
			m_adcops == AdcOps::Fixed1285 ) {
		//ADC1,ADC2 dual fast interleave mode
		adc_enable(ADC2);
		//setup dma
		initDMA(m_samplecounts/2,2);

		// set adc in continuous mode,
		adc_cont_enable(ADC1);
		adc_cont_enable(ADC2);
		//start conversion
		ADC1->regs->CR2 |= ADC_CR2_SWSTART;

	} else { //AdcOps::Range500
		//setup dma
		initDMA(m_samplecounts,1);

		//start timer 1 which triggers the ADC conversion
		Timer1.refresh();
		Timer1.resume();
	}
}

void CAdcMgr::stopConv() {
	if(m_adcops == AdcOps::Range500) {
		//stop timer 1 which triggers the ADC conversion
		Timer1.pause();
	} else {
		//adc continuous mode, we simply clear ADON
		adc_disable(ADC1);
		adc_disable(ADC2);
	}
}

void CAdcMgr::isradctimer(void) {
	s_me->adctimerhandle();
}

void CAdcMgr::adctimerhandle(void) {
	//start conversion when called
	ADC1->regs->CR2 |= ADC_CR2_SWSTART;
}


void CAdcMgr::isrtrigger(void) {
	s_me->triggered();
}


//called when trigger fires
void CAdcMgr::triggered() {
	// Turn on errorPin
	//digitalWrite( BOARD_LED_PIN, HIGH );

	if(!bfirsttrig) {
		bfirsttrig = true;
		if (triggerEvent == TRG_RISINGEDGE) {
			setawdhighthres(threshold);
			return;
		} else if (triggerEvent == TRG_FALLINGEDGE) {
			setawdlowthres(threshold);
			return;
		}
	} // else TRG_TOGGLE

	btriggered = true;
	//disable analog watch dog
	disable_awd(ADC1);

	//start adc dma recording
	adc_dma_enable(ADC1);
}

void CAdcMgr::isrdma(void) {
	s_me->dmacmplthandle();
}

void CAdcMgr::dmacmplthandle(void) {

	stopConv();
	bframedone = true;
	dma_clear_isr_bits(DMA1,DMA_CH1);

	return;
}

void CAdcMgr::doframedone(void) {

	// If frame is done, then it is time to send the buffer to the serial port
	if ( bframedone )
	{
		printData();

		btriggered = false;
		bframedone = false;

		// Clear buffer
		clearBuf();
	}

}


void CAdcMgr::printData(void) {
	uint16_t *buffer = ADCBuffer;
	int blanksize = BufferSize - m_samplecounts;

	//pad with zeros
	for(int i=0; i< blanksize ; i++) {
		Serial.write((uint8_t) 0);
	}

	buffer = ADCBuffer;
	for(int i=0; i<m_samplecounts ; i++) {
		Serial.write(vstmtogirino(*(buffer + i)));
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

	//powerup initialization
	//adc_init(ADC1);
	// 72 mhz / 6 = 12mhz adc clock, 14 adc clocks per conversion
	adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);
	adc_set_sample_rate(ADC1, ADC_SMPR_7_5);
	adc_calibrate(ADC1);

	configADCpin(channel);

	//setup dma
	dma_init(DMA1); //clock DMA
	// dma end transfer interrupt
	dma_attach_interrupt(DMA1, DMA_CH1, isrdma);
	// analog watch dog interrupt
	adc_attach_interrupt(ADC1, ADC_AWD, isrtrigger);

	//Adc timer is used to drive the ADC sampling
	initAdctimer();

	setops(m_adcops);

}


void CAdcMgr::setops(AdcOps ops) {

	// configure 'reset status'
	adc_disable(ADC1);
	adc_disable(ADC2);
	adc_dma_disable(ADC1);
	disable_awd(ADC1);
	dma_disable(DMA1, DMA_CH1);
	adc_cont_disable(ADC1);
	//clear dual mode register
	adc_clear_dual(ADC1);
	// 72 mhz / 6 = 12mhz adc clock, 14 adc clocks per conversion
	adc_set_prescaler(ADC_PRE_PCLK2_DIV_6);


	// clear buffer
	clearBuf();

	if(ops == AdcOps::Fixed1714 ||
	   ops == AdcOps::Fixed1285) {

		  if(ops == AdcOps::Fixed1285) {
			  // 72 mhz / 8 = 9mhz adc clock, 14 adc clocks per conversion
			  // hence 1285714 samples per sec
			  adc_set_prescaler(ADC_PRE_PCLK2_DIV_8);
		  }

		  //note that continuous mode for both ADC1 and ADC2
		  //needs to be set at acquisition start
		  adc_set_sample_rate(ADC1, ADC_SMPR_1_5);
		  adc_set_sample_rate(ADC2, ADC_SMPR_1_5);

		  adc_set_reg_seqlen(ADC1, 1);
		  ADC1->regs->SQR3 = m_channel;

		  adc_set_reg_seqlen(ADC2, 1);
		  ADC2->regs->SQR3 = m_channel;
		  adc_set_dual(ADC1,ADC_DUAL_FASTINT); // fast interleave mode


		  m_adcops = ops;

	} else if(ops == AdcOps::Fixed600 ||
	          ops == AdcOps::Fixed857) {
		if(ops == AdcOps::Fixed857)
			adc_set_sample_rate(ADC1, ADC_SMPR_1_5);
		else //AdcOps::Fixed857
			adc_set_sample_rate(ADC1, ADC_SMPR_7_5);

		//setup the analog watch dog
		set_awd_high_limit(ADC1, 0x0fff);
		set_awd_low_limit(ADC1, 0);
		set_awd_channel(ADC1, m_channel);
		enable_awd(ADC1);

		m_adcops = ops;

	} else { // Range500

		//this sample time only determines how long stm32 samples each inputs,
		//it does not affect the sample rate which is driven by the timer
		//this is using ADC_SMPR_7_5 adc clocks per sample
		//it takes sample_time + 12.5 adc cycles for one conversion
		//so 12 mhz (adc clk) / (7.5 + 12.5) ~ 600 ksps
		//if this is too slow, change it to ADC_SMPR_1_5, sampling quality may become worse
		adc_set_sample_rate(ADC1, ADC_SMPR_7_5);

		//setup the analog watch dog
		set_awd_high_limit(ADC1, 0x0fff);
		set_awd_low_limit(ADC1, 0);
		set_awd_channel(ADC1, m_channel);
		enable_awd(ADC1);

		adc_enable(ADC1);

		m_adcops = ops;
	}

}


void CAdcMgr::initDMA(uint16_t dmacount, uint8_t words) {
	dma_xfer_size xfer_size;
	uint32_t dmaflags;
    volatile void *memory_address;

	dma_disable(DMA1, DMA_CH1);
	dma_clear_isr_bits(DMA1,DMA_CH1);

	if(words == 2) {
		xfer_size = DMA_SIZE_32BITS;
	} else {
		xfer_size = DMA_SIZE_16BITS;
	}

	//single mode, enable the transfer complete interrupt handler
	dmaflags = (DMA_MINC_MODE | DMA_TRNS_CMPLT);
	memory_address = ADCBuffer;
//		(DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT);

    dma_setup_transfer(DMA1,
    		DMA_CH1,
			&ADC1->regs->DR,
			xfer_size,
			memory_address,
			xfer_size,
			dmaflags);// Receive buffer DMA
    dma_set_num_transfers(DMA1, DMA_CH1, dmacount);
    dma_clear_isr_bits(DMA1,DMA_CH1);
    dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
}


void CAdcMgr::clearBuf(void) {
	memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) );
}


void CAdcMgr::initAdctimer(void) {
	Timer1.pause();
	Timer1.setPrescaleFactor(1); //72mhz
	Timer1.setPeriod(500); // 500us, 2khz
	Timer1.setChannel1Mode(TIMER_OUTPUT_COMPARE);
	Timer1.attachCompare1Interrupt(isradctimer);
	//Timer1.attachCompare1Interrupt(adctimerhandle);
	//Timer1.setCompare(TIMER_CH1,1);
	//Timer1.refresh();
	//Timer1.resume();
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
		gpio_set_mode(GPIOA, channel, GPIO_INPUT_ANALOG);
		break;
	case 8:
	case 9:
		gpio_set_mode(GPIOB, channel - 8, GPIO_INPUT_ANALOG);
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		gpio_set_mode(GPIOC, channel - 10, GPIO_INPUT_ANALOG);
		break;
	default:
		break;
	}
}


//setup the triggers using analog watch dog
void CAdcMgr::initAWDtriggers(void) {
	disable_awd(ADC1);
	if(triggerEvent == TRG_TOGGLE) {
		uint16_t data = adc_read(ADC1,m_channel);
		if (data < threshold) //rising edge trigger
			setawdhighthres(threshold);
		else //falling edge
			setawdlowthres(threshold);
		bfirsttrig = true;
	} else if (triggerEvent == TRG_FALLINGEDGE) {
		setawdhighthres(threshold);
		bfirsttrig = false;
	} else { // TRG_RISINGEDGE
		setawdlowthres(threshold);
		bfirsttrig = false;
	}
	enable_awd(ADC1);
}

void CAdcMgr::setawdlowthres(uint16_t threshold) {
	set_awd_high_limit(ADC1,0x0fff);
	set_awd_low_limit(ADC1,threshold);
}


void CAdcMgr::setawdhighthres(uint16_t threshold) {
	set_awd_high_limit(ADC1,threshold);
	set_awd_low_limit(ADC1,0);
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

	if (samplerate > 1300000) {
		setops(AdcOps::Fixed1714);
	} else if (samplerate > 900000) {
		setops(AdcOps::Fixed1285);
	} else if (samplerate > 650000) {
		setops(AdcOps::Fixed857);
	} else if (samplerate > 530000) {
		setops(AdcOps::Fixed600);
	} else {
		setops(AdcOps::Range500);

		uint32 period_cyc = F_CPU / samplerate;
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
void CAdcMgr::setTriggerEvent(uint16_t trigEvt) {
	triggerEvent = trigEvt;

}

void CAdcMgr::setThreshold(uint16_t thres) {
	//this->threshold = threshold;
	threshold = thres;
}

void CAdcMgr::setCheannel(uint8_t channel) {
	m_channel = channel;
}




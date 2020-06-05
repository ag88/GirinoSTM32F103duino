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
#ifndef ADCMGR_H_
#define ADCMGR_H_
#include "Girino.h"


//-----------------------------------------------------------------------------
// ADC and DMA buffers
//-----------------------------------------------------------------------------
extern uint16_t ADCBuffer[ADCBUFFERSIZE];


//-----------------------------------------------------------------------------
// global variables
//-----------------------------------------------------------------------------

enum AdcOps {
	Range500 = 1, // 0 - 500 ksps range timer driven
	Fixed600, // 600 ksps, ADC1 continuous mode, sample time 7.5 adc clocks
	Fixed857, // 857 ksps, ADC1 continuous mode, sample time 1.5 adc clocks
	Fixed1714, // 1.71 msps dual fast interleaved mode
	Fixed1285 // 1.285 msps dual fast interleaved mode
};


class CAdcMgr {
public:
	const uint16_t BufferSize = ADCBUFFERSIZE;

	AdcOps m_adcops;
	boolean bframedone = false;
	boolean btriggered;
	uint8_t triggerEvent;
	uint16_t threshold;
	uint8_t m_channel;
	uint16_t m_samplecounts;

	CAdcMgr();
	virtual ~CAdcMgr();

	void startConv();
	void stopConv();

	static void isradctimer(void);
	static void isrtrigger(void);
	static void isrdma(void);
	static CAdcMgr* s_me;

	void adctimerhandle(void);
	void triggered();
	void dmacmplthandle(void);

	void doframedone(void);
	void printData(void);

	void initADC(int8_t channel);
	void setops(AdcOps ops);
	void initDMA(uint16_t dmacount, uint8_t words);
	void clearBuf(void);
	void initAdctimer(void);
	void configADCpin(int8_t channel);

	void setSamprate(int samprate);
	void setWaitDur(uint16_t waitdur);
	void setThreshold(uint16_t thres);
	void setTriggerEvent(uint16_t trigEvt);
	void setCheannel(uint8_t channel);


private:
	boolean bfirsttrig;
	void initAWDtriggers(void);
	void setawdhighthres(uint16_t threshold);
	void setawdlowthres(uint16_t threshold);


};

#endif /* ADCMGR_H_ */

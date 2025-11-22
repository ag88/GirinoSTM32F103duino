/*
 * TODO: place MIT license
 */
#ifndef ADCMGR_H_
#define ADCMGR_H_

// note that this sketch requires the mcu to run at 72 mhz due to the ADC divisors

/*
 * ADC and DMA buffers
 */
// PA0 ADC1 channel 0
// take note of the pin map - see comments for initADC()
#define ADC_CHANNEL		0

#define ADCBUFFERSIZE	1280
extern uint16_t ADCBuffer[ADCBUFFERSIZE];

/*
 * global variables and defines
 */
enum Trig {
	TRG_TOGGLE = 0,
	TRG_FALLINGEDGE = 2,
	TRG_RISINGEDGE = 3,
	TRG_NONE = 4
};

enum AdcOps {
	Range500 = 1, // 0 - 500 ksps range timer driven, has watch dog
	Fixed2400, // 2.4 msps, adc clock 36 mhz, sample time 3 adc clocks, has wd
	Fixed1333, // 1.333 msps, adc clock 36 mhz, sample time 15 adc clocks, has wd
	Fixed900, // 900 ksps, adc clock 36 mhz, sample time 28 adc clocks, has wd
	Fixed667 // 666.67 ksps, adc clock 18 mhz, sample time 15 adc clocks, has wd
};

enum RunState {
	INIT = 0,
	CONV,
	DONE,
	ERR
};

/*
 * global shared functions
 */
void error (void);

// call back for timer trigger unused
extern void timer_trig();

class CAdcMgr {
public:
	const uint16_t BufferSize = ADCBUFFERSIZE;

	AdcOps m_adcops;
	RunState m_state;
	bool btriggered;
	Trig triggerEvent;
	uint16_t threshold;
	uint8_t m_channel;
	uint16_t m_samplecounts;
	uint16_t m_errnum;

	CAdcMgr();
	virtual ~CAdcMgr();

	void startConv();
	void stopConv();

	static CAdcMgr* s_me;
	static CAdcMgr* getInstance();

	// irq call back functions
	static void isrtrigger(void);
	static void isrovr(void);
	static void isreoc(void);
	static void isradctimer(void);
	static void isrdma(void);


	void triggered();
	void dmacmplthandle(void);
	void ovrthandle(void);

	void doframedone(void);
	void printData(void);
	void doerr(void);

	void initADC(int8_t channel);
	void setops(AdcOps ops);
	void initDMA(uint8_t words);
	void dmaSetsize(uint16_t dmacount);

	void clearBuf(void);
	void initAdctimer(void);
	void configADCpin(int8_t channel);

	void setSamprate(int samprate);
	void setWaitDur(uint16_t waitdur);
	void setThreshold(uint16_t thres);
	void setTriggerEvent(uint16_t trigEvt);
	void setCheannel(uint8_t channel);
	void reset();


private:
	bool bfirsttrig;
	void initAWDtriggers(void);
	void setawdhighthres(uint16_t threshold);
	void setawdlowthres(uint16_t threshold);


};

#endif /* ADCMGR_H_ */

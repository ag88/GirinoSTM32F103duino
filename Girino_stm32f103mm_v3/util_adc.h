/*
 *  this file comes from Arduino_STM32/STM32F1/libraries/STM32ADC/
 *  name=STM32ADC
 *  version=1.0
 *  author=Cardoso, bubulindo, stevstrong, victorpv, roger ... many for libmaple adc codes
 *  email=
 *  sentence=Analog to Digital Converter Driver
 *  paragraph=ADC for STM32F1
 *  url=
 *  architectures=STM32F1
 *  maintainer=
 *  category=Timing
 */

#ifndef __UTIL_ADC_H
#define __UTIL_ADC_H

#include <libmaple/adc.h>

#ifdef __cplusplus
extern "C"{
#endif

#define ADC_DUAL_INDEP 0x0 //independent mode
#define ADC_DUAL_REGSIMU 0x6 //regular simultaneous mode
#define ADC_DUAL_FASTINT 0x7 //fast interleave mode
#define ADC_DUAL_SLOWINT 0x8 //fast interleave mode


void start_single_convert(adc_dev* dev, uint8 channel);

void start_continuous_convert(adc_dev* dev, uint8 channel);

void enable_adc_irq(adc_dev* dev);

void enable_internal_reading(adc_dev *dev);

void internalRead(adc_dev *dev, uint8 channel);

void enable_awd_irq( adc_dev * dev);

void set_awd_low_limit( adc_dev * dev, uint32 limit);

void set_awd_high_limit( adc_dev * dev, uint32 limit);

void enable_awd( adc_dev * dev);

void disable_awd( adc_dev * dev);

void set_awd_channel( adc_dev * dev, uint8 awd_channel);

void adc_set_reg_seq_channel( adc_dev * dev, unsigned char *channels, unsigned char len);

void set_continuous( adc_dev * dev);

uint8 poll_adc_convert(adc_dev *dev);

void adc_dma_enable(adc_dev * dev);

void adc_dma_disable(adc_dev * dev);

void adc_cont_enable(adc_dev * dev);

void adc_cont_disable(adc_dev * dev);

void adc_set_dual(adc_dev * dev, uint8_t mode);

void adc_clear_dual(adc_dev * dev);


#ifdef __cplusplus
} // extern "C"
#endif

#endif //__UTIL_ADC_H

#include <Arduino.h>
#include "WordConv.h"

WordConv wconv;

WordConv::WordConv() {
	wsize = WSIZE::BITS8;
}

WordConv::WordConv(WSIZE wsize) {
	this->wsize = wsize;
}

uint16_t WordConv::toWord(uint16_t data) {
	switch(wsize) {
	case WSIZE::BITS16:
		return data;
	case WSIZE::BITS12:
		return data & 0x0fff;
	case WSIZE::BITS8:
	default:
		int vret = data >> 4; // take high order 8 bits
		return vret & 0xff;
	}
}

void WordConv::printWordbuf(uint16_t *buffer, uint16_t count, uint16_t blanksize) {

	//pad with zeros
	bstate = Bstate::Init;
	for(int i=0; i< blanksize ; i++) {
		if(wsize == WSIZE::BITS16) {
			Serial.write((uint8_t) 0);
			Serial.write((uint8_t) 0);
		} else if (wsize == WSIZE::BITS12) {
			if(bstate == Bstate::Init || bstate == Bstate::First) {
				bstate = Bstate::Second;
			} else if (bstate == Bstate::Second) {
				Serial.write((uint8_t) 0);
				Serial.write((uint8_t) 0);
				Serial.write((uint8_t) 0);
				bstate = Bstate::First;
			}
		} else { //8bits
			Serial.write((uint8_t) 0);
		}
	}

	bstate = Bstate::Init;
	wordbuf = 0;

	for(int i=0; i<count ; i++) {
		uint16_t word = toWord(*(buffer + i));
		if(wsize == WSIZE::BITS16) {
			Serial.write(word & 0xff);
			Serial.write((word >> 8) & 0xff);
		} else if (wsize == WSIZE::BITS12) {
			if(bstate == Bstate::Init || bstate == Bstate::First) {
				wordbuf = word;
				bstate = Bstate::Second;
			} else if (bstate == Bstate::Second) {
				wordbuf |= word << 12;
				Serial.write(wordbuf & 0xff);
				Serial.write((wordbuf >> 8 )&0xff);
				Serial.write((wordbuf >> 16 )&0xff);
				wordbuf = 0;
				bstate = Bstate::First;
			}
		} else { //8 bits
			Serial.write(word);
		}
	}

}

uint16_t WordConv::fromWord(uint16_t data) {
	switch(wsize) {
	case WSIZE::BITS16:
		return data;
	case WSIZE::BITS12:
		return data & 0x0fff;
	case WSIZE::BITS8:
	default:
		int vret = data << 4; // use high order 8 bits
		return vret & 0xffff;
	}
}

void WordConv::setWordSize(uint16_t size) {
	switch(size) {
	case 16:
		wsize = WSIZE::BITS16;
		break;
	case 12:
		wsize = WSIZE::BITS12;
		break;
	case 8:
	default:
		wsize = WSIZE::BITS8;
		break;
	}
}

uint8_t WordConv::getWordSize() {
	switch(wsize) {
	case WSIZE::BITS16:
		return 16;
	case WSIZE::BITS12:
		return 12;
	case WSIZE::BITS8:
	default:
		return 8;
	}
}

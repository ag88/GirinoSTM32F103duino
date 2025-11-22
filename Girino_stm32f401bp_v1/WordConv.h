#ifndef WORDCONV_H_
#define WORDCONV_H_

enum WSIZE {
	BITS8 = 0,
	BITS12,
	BITS16
};

class WordConv {
public:
	WSIZE wsize;
	WordConv();
	WordConv(WSIZE wsize);

	uint16_t toWord(uint16_t data);

	void printWordbuf(uint16_t *buffer, uint16_t count, uint16_t blanksize);

	uint16_t fromWord(uint16_t word);

	void setWordSize(uint16_t size);

	uint8_t getWordSize();

private:
	enum Bstate {
		Init,
		First,
		Second,
		End
	};
	Bstate bstate;
	uint32_t wordbuf;

};

extern WordConv wconv;

#endif /* WORDCONV_H_ */

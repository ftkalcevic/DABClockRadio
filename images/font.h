/*
 * font.h
 *
 * Created: 1/10/2016 11:38:50 AM
 *  Author: Frank Tkalcevic
 */ 


#ifndef FONT_H_
#define FONT_H_

struct FontStruct
{
	uint8_t first_char;
	uint8_t last_char;
	uint8_t rows;
	uint8_t cols;
	const uint8_t * PROGMEM data;
};

extern const struct FontStruct font_6x13;


#endif /* FONT_H_ */
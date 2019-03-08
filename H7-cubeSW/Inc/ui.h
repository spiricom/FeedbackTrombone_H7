/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UI_H
#define __UI_H

uint8_t oled_buffer[32];
extern GFX theGFX;

typedef enum _OLEDLine
{
	FirstLine = 0,
	SecondLine,
	BothLines,
	NilLine
} OLEDLine;

void OLEDwriteString(uint8_t* myCharArray, uint8_t arrayLength, uint8_t startCursor, OLEDLine line);

void OLEDwriteLine(uint8_t* myCharArray, uint8_t arrayLength, OLEDLine line);

void OLEDwriteInt(uint32_t myNumber, uint8_t numDigits, uint8_t startCursor, OLEDLine line);

void OLEDwriteIntLine(uint32_t myNumber, uint8_t numDigits, OLEDLine line);

void OLEDwritePitch(float midi, uint8_t startCursor, OLEDLine line);

void OLEDwritePitchLine(float midi, OLEDLine line);

void OLEDwriteFixedFloat(float input, uint8_t numDigits, uint8_t numDecimal, uint8_t startCursor, OLEDLine line);

void OLEDwriteFixedFloatLine(float input, uint8_t numDigits, uint8_t numDecimal, OLEDLine line);
#endif /* __UI_H */

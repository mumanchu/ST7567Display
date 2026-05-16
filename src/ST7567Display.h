#pragma once

/////////////////////////////////////////////////////////////////////
// Display driver for 12864 LCD displays 
// with Sitronix ST7567 controller
// Copyright (C) mumanchu and muman.ch, 2026.05.03
// All rights reversed
// https://github/mumanchu
/*
Pixels are written to a memory buffer (gdram), then transferred to 
the LCD display by calling updateDisplay(). For speed, only the 
changed pixels are sent to the display, according to the 'changes
rectangle'.

Read the commented code for details.

Power save mode is not supported. It's assumed the device is plugged
into a continuous power source.

ST7567 DATA SHEET
=================
https://github.com/mumanchu/mumanchu/blob/main/assets/ST7567S.pdf

TEXT FONTS
==========
To use the "Classic Code Page 437, 5 x 8 font" character set, just
define this symbol before including this file. The character set 
data is in "fontCP437.h".

	#define LCD12864_CP437
	#include "ST7567Display.h"

Adafruit GFX fonts are also supported. These can be downloaded 
from here as '.h' files:
https://github.com/adafruit/Adafruit-GFX-Library/tree/master/Fonts

It's also possible to create a font from any TrueType font using 
the 'fontconvert.exe' program. The source code for this is here:
https://github.com/adafruit/Adafruit-GFX-Library/tree/master/fontconvert

Or you can download a version compiled for Windows from here (some 
changes were needed to make it run on Windows):
https://muman.ch/muman/fontconvert.exe

I2C VERSION
===========
This library uses SPI, but it's easy to modify for I2C by editing 
these two methods:
	bool begin(TwoWire* wire, ...)
	void sendBytes(...)
*/

#include <SPI.h>

// Adafruit GFX font support
#include "gfxfont.h"


#ifdef DEBUG
// These macros validate the pixel range when debugging
// they catch a lot of runtime errors during development!
#define ASSERTXY(x, y) if (x >= xPixels || y >= yPixels) { \
	LOGERROR("invalid x,y"); return; }		// returns nothing
#define ASSERTXY0(x, y) if (x >= xPixels || y >= yPixels) { \
	LOGERROR("invalid x,y"); return 0; }	// returns 0
#else
#define ASSERTXY(x, y)
#define ASSERTXY2(x, y)
#endif


class ST7567Display
{
	// LCD size, 128x64
	static const uint xPixels = 128;
	static const uint yPixels = 64;

	// SPI stuff
	SPIClass* spi;			// SPI channel
	SPISettings spiSettings; // SPI settings for this device

	// LCD pins
	uint csPin;				// SPI chip select
	uint a0Pin;				// 0 = command, 1 = data
	//uint rstPin;			// LCD reset

	// Font sizes, set up by setFont()
	const GFXfont* gfxFont;	// current Adafruit GFX font, see setFont()
	uint fontHeight;		// max. pixel height of tallest character
	uint fontAscender;		// max. pixels above the base line
	uint fontDescender;		// max. pixels below the base line

	// Graphics Data Ram (GDRam) image
	// updates are made in this buffer before writing to the display
	// yPixels / 8 = page
	byte gdram[(yPixels / 8) * xPixels];

	// Current display brightness, 0..63 (0..0x3f)
	uint currentBrightness;

	// gdram address = gdram + ((y / 8) * 128) + x
	#define GDRAM_ADDRESS(x, y) (gdram + ((y & 0xf8) << 4) + x)
	#define GDRAM_BIT(y)        (1 << (y & 7))

	// Changes rectangle for changes-only display refresh
	uint changesRectXmin, changesRectYmin;
	uint changesRectXmax, changesRectYmax;

public:
	bool begin(SPIClass* spi, uint csPin, uint a0Pin, uint rstPin = PNUM_NOT_DEFINED);
	void softwareReset();
	void setBrightness(uint brightness);
	inline uint getBrightness();
	void displayOn(bool on = true);
	void allPixelsOn(bool on = true);
	void invertDisplay(bool invert = true);
	void clearDisplay();
	void updateDisplay();
	inline void writeGDram();
	inline void clearGDram();

	enum PIXEL_STATE { PIXEL_SET, PIXEL_CLEAR, PIXEL_INVERT };

	inline void setPixel(uint x, uint y);
	inline void clearPixel(uint x, uint y);
	inline void invertPixel(uint x, uint y);
	inline void setPixel(uint x, uint y, PIXEL_STATE state);
	inline bool getPixel(uint x, uint y);

	void drawHorizontalLine(uint x, uint y, uint length);
	void drawVerticalLine(uint x, uint y, uint length);
	void drawLine(uint x0, uint y0, uint x1, uint y1);
	void drawRectangle(uint x, uint y, uint width, uint height);
	void fillRectangle(uint x, uint y, uint width, uint height, 
		PIXEL_STATE state = PIXEL_SET);
	void drawCircle(uint x, uint y, uint radius);
	void drawTriangle(uint x0, uint y0, uint x1, uint y1, uint x2, uint y2);
	void drawBitmap(const void* bitmap, uint bmpWidth, uint x, uint y, 
		uint width, uint height);

	void setFont(const GFXfont* gfxFont);
	const GFXfont* getFont() { return gfxFont; }
	uint drawChar(char ch, uint x, uint y);
	uint drawText(const char* s, uint x, uint y);
	bool getTextBounds(const char* s, uint* width, uint* height);
	uint drawCP437Char(char ch, uint x, uint y);
	uint drawCP437Text(const char* s, uint x, uint y);

	void dumpGDRam();

protected:
	inline void updateChangesRect(uint x, uint y);
	inline void clearChangesRect();
	void writeGDramRectangle(uint x, uint y, uint width, uint height);

	inline void sendAddress(uint x, uint y);
	inline void sendCommand(byte cmd);
	inline void sendCommands(const byte* cmds, uint length);
	inline void sendData(const byte* data, uint length);
	void sendBytes(const byte* data, uint length);
};


// Call this AFTER calling SPI.begin(), SPI may be shared by other devices
// NOTE: This sets the brightness to default (32), you may need to call
// setBrightness() for your display, my display needs 'setBrightness(42);'
bool ST7567Display::begin(SPIClass* spi, uint csPin, uint a0Pin, uint rstPin)
{
	this->spi = spi;
	this->csPin = csPin;
	this->a0Pin = a0Pin;

	// settings for this SPI device
	spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);

	// all pins are outputs
	pinMode(csPin, OUTPUT);
	digitalWrite(csPin, 1);
	pinMode(a0Pin, OUTPUT);
	digitalWrite(a0Pin, 0);			// 0=command, 1=data

	// hardware reset to initialize the ST7567 to default settings
	// (it' will normally have been's probably already been reset on power up)
	if (rstPin != PNUM_NOT_DEFINED) {
		pinMode(rstPin, OUTPUT);
		digitalWrite(rstPin, 1);	// RSTB := 1
		delay(1);
		digitalWrite(rstPin, 0);
		delay(10);
		digitalWrite(rstPin, 1);
	}
	currentBrightness = 32;			// reset sets default brightness (EV)

	// send multiple initialization commands, p42
	// most are already set to defaults by hardwareReset()
	static const byte initCommands[] =
	{
		0x2c,		// booster on
		0x2e,		// booster + regulator on
		0x2f,		// power control, booster + regulator + follower on
		0xa4,		// cancel all pixels on (just in case)
		0xc8		// COM direction = reverse, else the display is upside-down
	};
	for (int i = 0; i < sizeof(initCommands); ++i)
		sendCommand(initCommands[i]);

	clearDisplay();
	displayOn();

	return true;
}

// Resets the Start Line, Column Address, Page Address settings, p32
// Note: COM direction (vertical display direction) and EV are re-initialized
// so the display is the right way up and has the desired brightness
void ST7567Display::softwareReset()
{
	// software reset
	sendCommand(0xe2);

	// re-initialize important settings
	sendCommand(0xc8);		// COM direction (vertical display direction)
	sendCommand(0xa4);		// cancel all pixels on
	setBrightness(currentBrightness);
}

// Set the brightness level, 0..63 (0..0x3f)
// this often needs a different setting for each display
// the data sheet calls this the "electronic volume" EV level
void ST7567Display::setBrightness(uint brightness)
{
	if (brightness > 63)
		brightness = 63;
	currentBrightness = brightness;
	byte cmds[2] = { 0x81, (byte)brightness };
	sendCommands(cmds, 2);
}

// Return the current brightness level for brightness adjustment
inline uint ST7567Display::getBrightness()
{
	return currentBrightness;
}

void ST7567Display::displayOn(bool on)
{
	sendCommand(on ? 0xaf : 0xae);
}

void ST7567Display::allPixelsOn(bool on)
{
	sendCommand(on ? 0xa5 : 0xa4);
}

void ST7567Display::invertDisplay(bool invert)
{
	sendCommand(invert ? 0xa7 : 0xa6);
}

void ST7567Display::clearDisplay()
{
	clearGDram();
	writeGDram();
}

// Update the display with only the parts that have changed
void ST7567Display::updateDisplay()
{
	// do nothing is if nothing has changed
	if (changesRectXmax == 0 && changesRectYmax == 0)
		return;

	// write the changed 'pages' (byte rows) to the display
	writeGDramRectangle(changesRectXmin, changesRectYmin,
		changesRectXmax - changesRectXmin + 1,		// width
		changesRectYmax - changesRectYmin + 1);		// height
}

// Write the entire graphics data ram buffer to the display
void ST7567Display::writeGDram()
{
	writeGDramRectangle(0, 0, xPixels, yPixels);
}

// Clear graphics data ram
void ST7567Display::clearGDram()
{
	memset(gdram, 0, sizeof(gdram));

	// entire display must be updated
	changesRectXmin = 0;
	changesRectYmin = 0;
	changesRectXmax = xPixels - 1;
	changesRectYmax = yPixels - 1;
}


/////////////////////////////////////////////////////////////////////
// Pixel Operations

void ST7567Display::setPixel(uint x, uint y, PIXEL_STATE state)
{
	ASSERTXY(x, y);
	byte* ptr = GDRAM_ADDRESS(x, y);
	byte bit = GDRAM_BIT(y);
	byte b = *ptr;

	switch (state) {
	case PIXEL_SET:
		b |= bit;
		break;
	case PIXEL_CLEAR:
		b &= ~bit;
		break;
	case PIXEL_INVERT:
		b ^= bit;
		break;
	}
	*ptr = b;
}

void ST7567Display::setPixel(uint x, uint y)
{
	ASSERTXY(x, y);
	*GDRAM_ADDRESS(x, y) |= GDRAM_BIT(y);
}

void ST7567Display::clearPixel(uint x, uint y)
{
	ASSERTXY(x, y);
	*GDRAM_ADDRESS(x, y) &= ~GDRAM_BIT(y);
}

void ST7567Display::invertPixel(uint x, uint y)
{
	ASSERTXY(x, y);
	*GDRAM_ADDRESS(x, y) ^= GDRAM_BIT(y);
}

bool ST7567Display::getPixel(uint x, uint y)
{
	ASSERTXY0(x, y);
	return (*GDRAM_ADDRESS(x, y) & GDRAM_BIT(y)) != 0;
}


/////////////////////////////////////////////////////////////////////
// Shape Drawing Operations

void ST7567Display::drawHorizontalLine(uint x, uint y, uint length)
{
	ASSERTXY(x + length - 1, y);
	updateChangesRect(x, y);
	updateChangesRect(x + length - 1, y);

	byte* p = GDRAM_ADDRESS(x, y);
	uint bit = GDRAM_BIT(y);
	while (length--)
		*p++ |= bit;
}

void ST7567Display::drawVerticalLine(uint x, uint y, uint length)
{
	ASSERTXY(x, y + length - 1);
	updateChangesRect(x, y);
	updateChangesRect(x, y + length - 1);

	for (int dy = 0; dy < length; ++dy)
		setPixel(x, y++);
}

void ST7567Display::drawLine(uint x0, uint y0, uint x1, uint y1)
{
	ASSERTXY(x0, y0);
	ASSERTXY(x1, y1);
	updateChangesRect(x0, y0);
	updateChangesRect(x1, y1);

	int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
	int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;
	int err = dx - dy;

	while (1) {
		setPixel(x0, y0);
		if (x0 == x1 && y0 == y1)
			break;
		int e2 = err + err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}
}

void ST7567Display::drawRectangle(uint x, uint y, uint width, uint height)
{
	ASSERTXY(x, y);
	ASSERTXY(x + width - 1, y + height - 1);

	// these call updateChangesRect()
	drawHorizontalLine(x, y, width);				// top
	drawHorizontalLine(x, y + height - 1, width);	// bottom
	drawVerticalLine(x, y, height);					// left
	drawVerticalLine(x + width - 1, y, height);		// right
}

void ST7567Display::fillRectangle(uint x, uint y, uint width, uint height, PIXEL_STATE state)
{
	ASSERTXY(x, y);
	ASSERTXY(x + width - 1, y + height - 1);
	updateChangesRect(x, y);
	updateChangesRect(x + width - 1, y + height - 1);

	for (uint iy = 0; iy < height; ++iy) {
		uint y0 = y + iy;
		for (uint ix = 0; ix < width; ++ix) {
			uint x0 = x + ix;
			setPixel(x0, y0, state);
		}
	}
}

// x,y is the centre of the circle
void ST7567Display::drawCircle(uint x, uint y, uint radius)
{
	ASSERTXY(x + radius - 1, y + radius - 1);
	ASSERTXY(x - radius + 1, y - radius + 1);
	updateChangesRect(x - radius, y - radius);
	updateChangesRect(x + radius, y + radius);

	int f = 1 - (int)radius;
	int ddx = 1;
	int ddy = -2 * (int)radius;
	int x0 = 0;
	int y0 = radius;

	setPixel(x, y + radius);
	setPixel(x, y - radius);
	setPixel(x + radius, y);
	setPixel(x - radius, y);

	while (x0 < y0) {
		if (f >= 0) {
			y0--;
			ddy += 2;
			f += ddy;
		}
		x0++;
		ddx += 2;
		f += ddx;
		setPixel(x + x0, y + y0);
		setPixel(x - x0, y + y0);
		setPixel(x + x0, y - y0);
		setPixel(x - x0, y - y0);
		setPixel(x + y0, y + x0);
		setPixel(x - y0, y + x0);
		setPixel(x + y0, y - x0);
		setPixel(x - y0, y - x0);
	}
}

void ST7567Display::drawTriangle(uint x0, uint y0, uint x1, uint y1, uint x2, uint y2)
{
	ASSERTXY(x0, y0);
	ASSERTXY(x1, y1);
	ASSERTXY(x2, y2);

	// these call updateChangesRect()
	drawLine(x0, y0, x1, y1);
	drawLine(x1, y1, x2, y2);
	drawLine(x2, y2, x0, y0);
}

// Draw a bitmap, 8, 16 or 32 bits in width, height is up to 64 bits
// This is NOT a '.bmp' file format, it's an array of 8, 16 or 32-bit binary values
// bitmapWidth : 8 (byte array), 16 (uint16_t array) or 32 (uint32_t array)
// The full width or height of the bitmap does not need to be displayed.
// e.g. you can display the 4-bit left-hand column of an 8-bit array
void ST7567Display::drawBitmap(const void* bitmap, uint bitmapWidth,
	uint x, uint y, uint width, uint height)
{
	ASSERTXY(x, y);
	ASSERTXY(x + width - 1, y + height - 1);
	updateChangesRect(x, y);
	updateChangesRect(x + width - 1, y + height - 1);

	uint startxpos = x + width - 1;
	byte* p = (byte*)bitmap;

	for (uint iy = 0; iy < height; ++iy) {
		uint ypos = y + iy;
		uint xpos = startxpos;
		
		// get 8, 16 or 32 bits
		uint32_t b;
		switch(bitmapWidth) {
		case 8:
			b = *p++;
			break;
		case 16:
			b = *(uint16_t*)p;
			p += 2;
			break;
		case 32:
			b = *(uint32_t*)p;
			p += 4;
			break;
		default:
			return;
		}
		uint32_t mask = 1;

		// write the bits
		for (uint ix = 0; ix < width; ++ix) {
			if (b & mask)
				setPixel(xpos, ypos);
			else
				clearPixel(xpos, ypos);
			--xpos;
			mask <<= 1;
		}
	}
}


/////////////////////////////////////////////////////////////////////
// Text operations using Adafruit GFX Library Fonts
// 
// #include the Adafruit GFX font file, then call setFont(GFXFont* gfxFont);
// Create new font files with fontconverter.exe
// 
// NOTE! The 'y' position is the 'cursor position' - the base line of 
// the font character. This is because the height of each character is
// not the same, and variable-length descenders will go below this line.
// 
// The font's 'yAdvance' value (the y offset between each line) is often
// too big, so the lines are spaced too far apart. This can be reduced 
// to get more lines on the display, e.g. 5 instead of 4. But there's 
// no universal way to do this - you'll need to set the 'yAdvance' and 
// initial 'y' cursor position according to the font size.
// 
// Line feed '\n' and carriage return '\r' are not handled. In graphics
// mode these are drawn as characters. Use the x and y coordinates to 
// control the text position.
//
// To draw a character it inverts the display bits (the '1' bits). This
// looks best on an all-white or all-black background. It also prevents
// empty space overwriting the background.

// Calculate some properties of the font
void ST7567Display::setFont(const GFXfont* gfxFont)
{
	this->gfxFont = gfxFont;
	int numChars = gfxFont->last - gfxFont->first + 1;

	int height = 0;
	int descender = 0;
	for (int i = 0; i < numChars; ++i) {
		GFXglyph* glyph = gfxFont->glyph + i;
		if (glyph->height > height)
			height = glyph->height;
		int d = glyph->height + glyph->yOffset;
		if (d > descender)
			descender = d;
	}
	fontHeight = height;
	fontDescender = descender;
	fontAscender = height - descender + 1;
}

uint ST7567Display::drawChar(char ch, uint x, uint y)
{
	ASSERTXY0(x, y);
	if (gfxFont == NULL || ch < gfxFont->first || ch > gfxFont->last)
		return 0;

	GFXglyph* glyph = gfxFont->glyph + (ch - gfxFont->first);
	byte* bitmap = gfxFont->bitmap + glyph->bitmapOffset;
	uint width = glyph->width;
	uint height = glyph->height;

	updateChangesRect(x, y);
	x += glyph->xOffset;
	y += glyph->yOffset;
	updateChangesRect(x, y);

	uint bit = 0;
	uint bits = 0;
	for (uint yy = 0; yy < height; yy++) {
		for (uint xx = 0; xx < width; xx++) {
			if (!(bit & 7))
				bits = *bitmap++;
			// only update the '1' bits
			if (bits & 0x80)
				invertPixel(x + xx, y + yy);
			++bit;
			bits <<= 1;
		}
	}
	return glyph->xAdvance;
}

uint ST7567Display::drawText(const char* s, uint x, uint y)
{
	while (*s) {
		uint dx = drawChar(*s++, x, y);
		x += dx;
		if (x >= xPixels)
			break;
	}
	return x;
}

// Get the size of the text according to the current font
// Returns false if setFont() not called or the text contains 
// characters not in the font
// NOTE: Do not use this for the default CP437 font, instead use:
// height = 7 or 8 (7=without or 8=with line spacing)
// width = strlen(s) * 6
bool ST7567Display::getTextBounds(const char* s, uint* width, uint* height)
{
	*width = 0;
	*height = 0;
	// must call setFont()
	if (gfxFont == NULL)
		return false;
	uint w = 0;
	while (*s) {
		char ch = *s++;
		if (ch < gfxFont->first || ch > gfxFont->last)
			return false;		// character not in the font
		GFXglyph* glyph = gfxFont->glyph + (ch - gfxFont->first);
		w += glyph->xAdvance;
	}
	*width = w;
	*height = fontHeight;
	return true;
}


/////////////////////////////////////////////////////////////////////
#ifdef LCD12864_CP437

// Classic "Code Page 437" 5 x 8 font
// character spacing 6 pixels, line spacing 8 pixels
// For line drawing characters 179..218 use 5 pixel character spacing
// (no space between characters)
// https://en.wikipedia.org/wiki/Code_page_437

#include "fontCP437.h"

uint ST7567Display::drawCP437Char(char ch, uint x, uint y)
{
	ASSERTXY0(x, y);
	ASSERTXY0(x + 5, y + 8);
	updateChangesRect(x, y);
	updateChangesRect(x + 5, y + 8);

	const byte* pfont = fontCP437 + ch * 5;

	// 5 x 8 bitmap
	for (uint i = 0; i < 5; ++i) {
		uint bits = *pfont++;
		for (uint j = 0; j < 8; ++j) {
			// only update the '1' bits
			if (bits & 1)
				invertPixel(x + i, y + j);
			bits >>= 1;
		}
	}
	return 6;
}

uint ST7567Display::drawCP437Text(const char* s, uint x, uint y)
{
	while (*s) {
		x += drawCP437Char(*s++, x, y);
		if (x >= xPixels)
			break;
	}
	return x;
}

#endif


/////////////////////////////////////////////////////////////////////
// Internal Methods

#ifdef DEBUG
// Dump graphics data ram to Serial, 'X' = pixel, '.' = no pixel
// This is useful when debugging new code, you can see all the pixels.
// A big port monitor window is needed, I use the old-but-good 'PuTTY'.
void ST7567Display::dumpGDRam()
{
	for (uint y = 0; y < yPixels; ++y) {
		Serial.printf("%u ", y >> 3);
		for (uint x = 0; x < xPixels; ++x) {
			byte* p = GDRAM_ADDRESS(x, y);
			uint bit = GDRAM_BIT(y);
			Serial.print(*p & bit ? 'X' : '.');
		}
		Serial.println();
	}
	Serial.flush();
}
#endif


// Update changes rectangle, for efficient display update
void ST7567Display::updateChangesRect(uint x, uint y)
{
	ASSERTXY(x, y);
	if (x < changesRectXmin)
		changesRectXmin = x;
	else if (x > changesRectXmax)
		changesRectXmax = x;
	if (y < changesRectYmin)
		changesRectYmin = y;
	else if (y > changesRectYmax)
		changesRectYmax = y;
}

// Clear = no changes
void ST7567Display::clearChangesRect()
{
	changesRectXmin = xPixels;
	changesRectXmax = 0;
	changesRectYmin = yPixels;
	changesRectYmax = 0;
}


// Update a rectangular part of the display from the RAM buffer
// it writes the 'pages' (byte rows) which incorporate the supplied rectangle
// the 'changes rectangle' is cleared (set to 'no changes')
void ST7567Display::writeGDramRectangle(uint x, uint y, uint width, uint height)
{
	ASSERTXY(x, y);
	ASSERTXY(x + width - 1, y + height - 1);

	// send each 'page' (each row of bytes)
	uint firstY = y & 0xf8;
	uint lastY = (y + height - 1) & 0xf8;
	for (int y0 = firstY; y0 <= lastY; y0 += 8) {
		sendAddress(x, y0);
		sendData(GDRAM_ADDRESS(x, y0), width);
	}

	// nothing changed now
	clearChangesRect();
}

// Set address for write data
void ST7567Display::sendAddress(uint x, uint y)
{
	ASSERTXY(x, y);
	// page address, column address MS 4-bits, LS 4-bits
	byte cmds[3] = { 0xb0 + (y >> 3), 0x10 + (x >> 4), x & 0x0f };
	sendCommands(cmds, 3);
}

// Send a single byte command
void ST7567Display::sendCommand(byte cmd)
{
	digitalWrite(a0Pin, 0);		// 0 = command, 1 = data
	sendBytes(&cmd, 1);
}

// Send multiple commands
void ST7567Display::sendCommands(const byte* cmds, uint length)
{
	digitalWrite(a0Pin, 0);		// 0 = command, 1 = data
	sendBytes(cmds, length);
}

// Send one or more data bytes
void ST7567Display::sendData(const byte* data, uint length)
{
	digitalWrite(a0Pin, 1);		// 0 = command, 1 = data
	sendBytes(data, length);
}

// SPI version
// to use I2C, change only this and add 'begin(TwoWire* wire, ...)'
void ST7567Display::sendBytes(const byte* data, uint length)
{
	digitalWrite(csPin, 0);
	spi->beginTransaction(spiSettings);
	spi->transfer((void*)data, length, true);
	spi->endTransaction();
	digitalWrite(csPin, 1);
}


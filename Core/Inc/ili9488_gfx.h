/*
 * ili9488_gfx.h
 *
 *  Created on: Jan 7, 2022
 *      Author: timagr615
 */

#ifndef INC_ILI9488_GFX_H_
#define INC_ILI9488_GFX_H_

#include "stm32f4xx_hal.h"
#include "font.h"

#define HORIZONTAL_IMAGE	0
#define VERTICAL_IMAGE		1

//adafruit GFX strutures

// Font data stored PER GLYPH
typedef struct {
  uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
  uint8_t width;         ///< Bitmap dimensions in pixels
  uint8_t height;        ///< Bitmap dimensions in pixels
  uint8_t xAdvance;      ///< Distance to advance cursor (x axis)
  int8_t xOffset;        ///< X dist from cursor pos to UL corner
  int8_t yOffset;        ///< Y dist from cursor pos to UL corner
} GFXglyph;

// Data stored for FONT AS A WHOLE
typedef struct {
  uint8_t *bitmap;  ///< Glyph bitmaps, concatenated
  GFXglyph *glyph;  ///< Glyph array
  uint16_t first;   ///< ASCII extents (first char)
  uint16_t last;    ///< ASCII extents (last char)
  uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;


GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c);
uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont);

void ILI9341_set_adafruit_font(const GFXfont *font_pointer);
void ILI9341_Draw_Hollow_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour);
void ILI9341_Draw_Filled_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour);
void ILI9341_Draw_Hollow_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour);
void ILI9341_Draw_Filled_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour);
//void ILI9341_Draw_Char(char Character, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
//void ILI9341_Draw_Text(const char* Text, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_TextFont(const char* Text, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_Font_Background(const char* Text, uint16_t X, uint16_t Y, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_OnText_Font_Background(const char* Text, uint16_t X, uint16_t Y, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_Filled_Rectangle_Size_Text(uint16_t X0, uint16_t Y0, uint16_t Size_X, uint16_t Size_Y, uint16_t Colour);

void ILI9341_DrawChar(char ch, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor);
void ILI9341_DrawText(const char* str, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor);
void ILI9341_DrawNumber(char ch, const uint16_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor, uint8_t size);
void ILI9341_DrawNumberText(const char* str, const uint16_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor, uint8_t size);


//USING CONVERTER: http://www.digole.com/tools/PicturetoC_Hex_converter.php
//65K colour (2Bytes / Pixel)
void ILI9341_Draw_Image(const uint8_t *Image_Array, uint16_t X, uint16_t Y, uint16_t Size_X, uint16_t Size_Y);
void ILI9341_Draw_CharFont(unsigned char c, int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);

#endif /* INC_ILI9488_GFX_H_ */

/*
 * ili9488_gfx.c
 *
 *  Created on: Jan 7, 2022
 *      Author: timagr615
 */

#include "stdlib.h"
#include "string.h"

#include "main.h"

#include "ili9488.h"
#include "ili9488_gfx.h"
//#include "5x5font.h"

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr))

#define LCD_WIDTH 480
#define LCD_HEIGHT 320

extern uint8_t SPI1_TX_completed_flag;

const GFXfont *gfxFont = NULL;
int32_t max_font_height;
int32_t min_font_height;


GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c)
{
  return gfxFont->glyph + c;
}

uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont)
{
  return gfxFont->bitmap;
}

uint16_t byte_reverse(uint16_t in_color)
{
    uint16_t out_color = (in_color << 8) | (in_color >> 8);
    return out_color;
}


void ILI9341_set_adafruit_font(const GFXfont *font_pointer)
{
	gfxFont = font_pointer;

	int32_t min_y = -9999;
	int32_t max_y = 9999;

    for(int i = 0; i < 95; i++)
    {
        //find the highest pixel in font
        if(gfxFont->glyph[i].yOffset < max_y)
        	max_y = gfxFont->glyph[i].yOffset;

        //find the lowest pixel in font
        if(gfxFont->glyph[i].height + gfxFont->glyph[i].yOffset > min_y)
            min_y = gfxFont->glyph[i].height + gfxFont->glyph[i].yOffset;
    }
    max_font_height = min_y;
    min_font_height = max_y;

}


/*Draw hollow circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Hollow_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{
	int x = Radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (Radius << 1);

    while (x >= y)
    {
        ILI9341_Draw_Pixel(X + x, Y + y, Colour);
        ILI9341_Draw_Pixel(X + y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - x, Y + y, Colour);
        ILI9341_Draw_Pixel(X - x, Y - y, Colour);
        ILI9341_Draw_Pixel(X - y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + x, Y - y, Colour);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        if (err > 0)
        {
            x--;
            dx += 2;
            err += (-Radius << 1) + dx;
        }
    }
}

/*Draw filled circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Filled_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{

		int x = Radius;
    int y = 0;
    int xChange = 1 - (Radius << 1);
    int yChange = 0;
    int radiusError = 0;

    while (x >= y)
    {
        for (int i = X - x; i <= X + x; i++)
        {
            ILI9341_Draw_Pixel(i, Y + y,Colour);
            ILI9341_Draw_Pixel(i, Y - y,Colour);
        }
        for (int i = X - y; i <= X + y; i++)
        {
            ILI9341_Draw_Pixel(i, Y + x,Colour);
            ILI9341_Draw_Pixel(i, Y - x,Colour);
        }

        y++;
        radiusError += yChange;
        yChange += 2;
        if (((radiusError << 1) + xChange) > 0)
        {
            x--;
            radiusError += xChange;
            xChange += 2;
        }
    }
		//Really slow implementation, will require future overhaul
		//TODO:	https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles
}

/*Draw a hollow rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Hollow_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	float 		Calc_Negative = 0;

	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;

	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;


	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;
	}
	else
	{
		X_length = X0 - X1;
	}
	ILI9341_Draw_Horizontal_Line(X0, Y0, X_length, Colour);
	ILI9341_Draw_Horizontal_Line(X0, Y1, X_length, Colour);



	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;
	}
	else
	{
		Y_length = Y0 - Y1;
	}
	ILI9341_Draw_Vertical_Line(X0, Y0, Y_length, Colour);
	ILI9341_Draw_Vertical_Line(X1, Y0, Y_length, Colour);

	if((X_length > 0)||(Y_length > 0))
	{
		ILI9341_Draw_Pixel(X1, Y1, Colour);
	}

}

/*Draw a filled rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Filled_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	int32_t 	Calc_Negative = 0;

	uint16_t X0_true = 0;
	uint16_t Y0_true = 0;

	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;

	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;


	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;
		X0_true = X0;
	}
	else
	{
		X_length = X0 - X1;
		X0_true = X1;
	}

	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;
		Y0_true = Y0;
	}
	else
	{
		Y_length = Y0 - Y1;
		Y0_true = Y1;
	}

	ILI9341_Draw_Rectangle(X0_true, Y0_true, X_length, Y_length, Colour);
}

void ILI9341_DrawChar(char ch, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor)
{
	if ((ch < 31) || (ch > 127)) return;

	uint8_t fOffset, fWidth, fHeight, fBPL;
	uint8_t *tempChar;

	fOffset = font[0];
	fWidth = font[1];
	fHeight = font[2];
	fBPL = font[3];

	tempChar = (uint8_t*)&font[((ch - 0x20) * fOffset) + 4]; /* Current Character = Meta + (Character Index * Offset) */

	/* Clear background first */
	ILI9341_Draw_Rectangle(X, Y, fWidth, fHeight, bgcolor);

	for (int j=0; j < fHeight; j++)
	{
		for (int i=0; i < fWidth; i++)
		{
			uint8_t z =  tempChar[fBPL * i + ((j & 0xF8) >> 3) + 1]; /* (j & 0xF8) >> 3, increase one by 8-bits */
			uint8_t b = 1 << (j & 0x07);
			if (( z & b ) != 0x00)
			{
				ILI9341_Draw_Pixel(X+i, Y+j, color);
			}
		}
	}
}

void ILI9341_DrawText(const char* str, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor)
{
	uint8_t charWidth;			/* Width of character */
	uint8_t fOffset = font[0];	/* Offset of character */
	uint8_t fWidth = font[1];	/* Width of font */

	while (*str)
	{
		ILI9341_DrawChar(*str, font, X, Y, color, bgcolor);

		/* Check character width and calculate proper position */
		uint8_t *tempChar = (uint8_t*)&font[((*str - 0x20) * fOffset) + 4];
		charWidth = tempChar[0];

		if(charWidth + 2 < fWidth)
		{
			/* If character width is smaller than font width */
			X += (charWidth + 2);
		}
		else
		{
			X += fWidth;
		}

		str++;
	}
}

/*void ILI9341_Draw_Char(char Character, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	uint8_t function_char;
	uint8_t i, j;

	function_char = Character;

	if (function_char < ' ')
	{
		Character = 0;
	}
	else
	{
		function_char -= 32;
	}

	char temp[CHAR_WIDTH];
	for (uint8_t k = 0; k < CHAR_WIDTH; k++)
	{
		temp[k] = font[function_char][k];
	}

	// Draw pixels
	ILI9341_Draw_Rectangle(X, Y, CHAR_WIDTH * Size, CHAR_HEIGHT * Size, Background_Colour);
	for (j = 0; j < CHAR_WIDTH; j++)
	{
		for (i = 0; i < CHAR_HEIGHT; i++)
		{
			if (temp[j] & (1 << i))
			{
				if (Size == 1)
				{
					ILI9341_Draw_Pixel(X + j, Y + i, Colour);
				}
				else
				{
					ILI9341_Draw_Rectangle(X + (j * Size), Y + (i * Size), Size, Size, Colour);
				}
			}
		}
	}
}*/

/*Draws an array of characters (fonts imported from fonts.h) at X,Y location with specified font color, size and Background color*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
/*void ILI9341_Draw_Text(const char* Text, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text)
    {
        ILI9341_Draw_Char(*Text++, X, Y, Colour, Size, Background_Colour);
        X += CHAR_WIDTH*Size;
    }
}*/

void ILI9341_Draw_CharFont(unsigned char c, int16_t x, int16_t y, uint16_t color, uint16_t background, uint8_t size_x, uint8_t size_y)
{
	if(gfxFont == NULL)
		return;
    // Character is assumed previously filtered by write() to eliminate
    // newlines, returns, non-printable characters, etc.  Calling
    // drawChar() directly with 'bad' characters of font may cause mayhem!

    c -= (uint8_t)pgm_read_byte(&gfxFont->first);      //convert input char to corresponding byte from font array
    GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);  //get pointer of glyph corresponding to char
    uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);    //get pointer of char bitmap

    background = byte_reverse(background);

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t width = pgm_read_byte(&glyph->width);
    uint8_t height = pgm_read_byte(&glyph->height);

    //int8_t x_offset = pgm_read_byte(&glyph->xOffset);
    int8_t y_offset = pgm_read_byte(&glyph->yOffset);

    uint16_t *tmp_char_bitmap = malloc(2*width*height);   //allocate dynamic array for bitmap

    //decide for background color or font color
    uint8_t bit = 0;
    uint8_t bits = 0;
    uint8_t y_pos = 0;
    uint8_t x_pos = 0;

    for(y_pos = 0; y_pos < height; y_pos++)
    {
    	for(x_pos = 0; x_pos < width; x_pos++)
    	{
			if (!(bit++ & 7))
			{
				bits = pgm_read_byte(&bitmap[bo++]);
			}
			if (bits & 0x80)
			{
				tmp_char_bitmap[y_pos*width+x_pos] = color;
			}
			else
			{
				tmp_char_bitmap[y_pos*width+x_pos] = background;
			}
			bits <<= 1;
    	}
    }
    ILI9341_Draw_Image((uint8_t*)tmp_char_bitmap, x, y+y_offset, width, height);       //draw character as a bitmap
    free(tmp_char_bitmap);                                                             //free allocated memory
}

void ILI9341_Draw_Font_Background(const char* Text, uint16_t X, uint16_t Y, uint16_t Size, uint16_t Background_Colour)
{
	int32_t length = 0;
	int32_t min_y = -9999;
	int32_t max_y = 9999;

    while (*Text)
    {
        length = length + gfxFont->glyph[*Text-32].width  +  gfxFont->glyph[*Text-32].xOffset;

        //find the highest pixel in string
        if(gfxFont->glyph[*Text-32].yOffset < max_y)
        	max_y = gfxFont->glyph[*Text-32].yOffset;

        //find the lowest pixel in string
        if(gfxFont->glyph[*Text-32].height + gfxFont->glyph[*Text-32].yOffset > min_y)
            min_y = gfxFont->glyph[*Text-32].height + gfxFont->glyph[*Text-32].yOffset;
        Text++;
    }
    ILI9341_Draw_Filled_Rectangle_Coord(X, Y+max_y, X+length, Y+min_y, Background_Colour);
}

void ILI9341_Draw_OnText_Font_Background(const char* Text, uint16_t X, uint16_t Y, uint16_t Size, uint16_t Background_Colour)
{
	uint16_t length= 0;

    while (*Text)
    {
        length = length + gfxFont->glyph[*Text-32].xAdvance;
        Text++;
    }
    ILI9341_Draw_Filled_Rectangle_Coord(X, Y+min_font_height, X+length, Y+max_font_height, Background_Colour);
}


void ILI9341_Draw_TextFont(const char* Text, uint16_t X, uint16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text)
    {
        ILI9341_Draw_CharFont(*Text, X, Y, Colour, Background_Colour, Size, Size);
        X = X + gfxFont->glyph[*Text-32].xAdvance;
        Text++;
    }
}


// Вывод символа
void ILI9341_DrawNumber(char ch, const uint16_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor, uint8_t size)
{
	if ((ch < 31) || (ch > 127)) return;

	uint16_t fOffset, fWidth, fHeight, fBPL;
	uint16_t *tempChar;

	fOffset = font[0];
	fWidth = font[1];
	fHeight = font[2];
	fBPL = font[3];

	tempChar = (uint16_t*)&font[((ch - 0x20) * fOffset) + 4]; /* Current Character = Meta + (Character Index * Offset) */

	/* Clear background first */
	ILI9341_Draw_Rectangle(X, Y, fWidth*size, fHeight*size, bgcolor);

	for (int j=0; j < fHeight; j++)
	{
		for (int i=0; i < fWidth; i++)
		{
			uint16_t z =  tempChar[fBPL * i + ((j & 0xF8) >> 3) + 1]; /* (j & 0xF8) >> 3, increase one by 8-bits */
			uint16_t b = 1 << (j & 0x07);
			if (( z & b ) != 0x00)
			{
				if (size == 1)
				{
					ILI9341_Draw_Pixel(X+i, Y+j, color);
				}
				else{
					ILI9341_Draw_Rectangle(X+i*size, Y+j*size, size, size, color);
				}

			}

		}
	}
}

void ILI9341_DrawNumberText(const char* str, const uint16_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor, uint8_t size)
{
	//char str1[20];
	//strcpy(str1, str);
	uint16_t charWidth;			/* Width of character */
	uint16_t fOffset = font[0];	/* Offset of character */
	uint16_t fWidth = font[1];	/* Width of font */
	uint16_t fHeight = font[2];
	//uint16_t lenStr = sizeof(&str)/sizeof(char);
	//uint8_t lenStr = strlen(str1)-1;
	//ILI9341_Draw_Rectangle(X, Y, fWidth*size*lenStr, fHeight*size, bgcolor);

	while (*str)
	{
		ILI9341_DrawNumber(*str, font, X, Y, color, bgcolor, size);

		/* Check character width and calculate proper position */
		uint16_t *tempChar = (uint16_t*)&font[((*str - 0x20) * fOffset) + 4];
		charWidth = tempChar[0];

		if(charWidth + 2 < fWidth)
		{
			/* If character width is smaller than font width */
			X += (charWidth*size + 2);
		}
		else
		{
			X += fWidth*size;
		}

		str++;
	}
}

/*
 * \brief Write bitmap of specified size to specified location on screen.
 *
 * Bitmap is uint16_t array, where one value corresponds to one pixel. However, display expects to receive low byte first
 * and MCU sends high byte first. For effective DMA utilization bitmaps have bytes reversed. Using online bitmaps converters
 * will result in reversed colors.
 *
 * \param[in] *Image_Array Pointer to image array
 * \param[in] X desired X location of top left corner of bitmap on screen
 * \param[in] Y desired Y location of top left corner of bitmap on screen
 * \param[in] Size_X Width of image
 * \param[in] Size_Y Height of image
 */
void ILI9341_Draw_Image(const uint8_t *Image_Array, uint16_t X, uint16_t Y, uint16_t Size_X, uint16_t Size_Y)
{
	if ((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT))
		return;
	if ((X + Size_X - 1) >= LCD_WIDTH)
	{
		Size_X = LCD_WIDTH - X;
	}
	if ((Y + Size_Y - 1) >= LCD_HEIGHT)
	{
		Size_Y = LCD_HEIGHT - Y;
	}
	uint32_t Size = Size_X*Size_Y*2;
	ILI9341_Set_Address(X, Y, X + Size_X - 1, Y + Size_Y - 1);

	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);


	int x;
	for(x = 0; x < Size/0xFFFF; x++)
	{
		SPI1_TX_completed_flag = 0;
		HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (unsigned char*) (Image_Array+(x*0xFFFF)), 0xFFFF);
		while (SPI1_TX_completed_flag == 0);
	}

	if(Size % 0xFFFF > 0)
	{
		SPI1_TX_completed_flag = 0;
		HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (unsigned char*) (Image_Array+(x*0xFFFF)), Size % 0xFFFF);
		while (SPI1_TX_completed_flag == 0);
	}

}




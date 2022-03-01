/*
 * ili9488.c
 *
 *  Created on: Dec 26, 2021
 *      Author: timagr615
 */

#include "main.h"
#include "ili9488.h"
//#include "stdlib.h"
extern UART_HandleTypeDef huart1;

/* Global Variables ------------------------------------------------------------------*/
volatile uint16_t LCD_HEIGHT = ILI9488_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9488_SCREEN_WIDTH;

extern uint8_t SPI1_TX_completed_flag;
//extern static void MX_GPIO_Init(void);
//extern static void MX_SPI1_Init(void);

void ILI9341_SPI_Init(void)
{
	/////// ??????????? Нужны ли две функции ниже? ////////////////
	//MX_SPI1_Init();
	//MX_GPIO_Init();
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
}

void ILI9341_SPI_Send(uint8_t SPI_Data)
{
	SPI1_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(HSPI_INSTANCE, &SPI_Data, 1);
	while (SPI1_TX_completed_flag == 0);
}

void ILI9341_Write_Command(uint8_t Command)
{
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
	ILI9341_SPI_Send(Command);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

}

/* Send Data (char) to LCD */
void ILI9341_Write_Data(uint8_t Data)
{
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	ILI9341_SPI_Send(Data);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* Set Address - Location block - to draw into */
void ILI9341_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
	ILI9341_Write_Command(0x2A);
	ILI9341_Write_Data(X1 >> 8);
	ILI9341_Write_Data(X1);
	ILI9341_Write_Data(X2 >> 8);
	ILI9341_Write_Data(X2);

	ILI9341_Write_Command(0x2B);
	ILI9341_Write_Data(Y1 >> 8);
	ILI9341_Write_Data(Y1);
	ILI9341_Write_Data(Y2 >> 8);
	ILI9341_Write_Data(Y2);

	ILI9341_Write_Command(0x2C);
}

/*HARDWARE RESET*/
void ILI9341_Reset(void)
{
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
}

/*Set rotation of the screen - changes x0 and y0*/
void ILI9341_Set_Rotation(uint8_t Rotation)
{
	uint8_t screen_rotation = Rotation;
	ILI9341_Write_Command(0x36);
	HAL_Delay(1);
	switch(screen_rotation)
	{
		case SCREEN_VERTICAL_1:
			ILI9341_Write_Data(0x40 | 0x08);
			LCD_WIDTH = ILI9488_SCREEN_HEIGHT;
			LCD_HEIGHT = ILI9488_SCREEN_WIDTH;
			break;
		case SCREEN_HORIZONTAL_1:
			ILI9341_Write_Data(0x20 | 0x08);
			LCD_WIDTH  = ILI9488_SCREEN_WIDTH;
			LCD_HEIGHT = ILI9488_SCREEN_HEIGHT;
			break;
		case SCREEN_VERTICAL_2:
			ILI9341_Write_Data(0x80 | 0x08);
			LCD_WIDTH  = ILI9488_SCREEN_HEIGHT;
			LCD_HEIGHT = ILI9488_SCREEN_WIDTH;
			break;
		case SCREEN_HORIZONTAL_2:
			ILI9341_Write_Data(0x40 | 0x80 | 0x20 | 0x08);
			LCD_WIDTH  = ILI9488_SCREEN_WIDTH;
			LCD_HEIGHT = ILI9488_SCREEN_HEIGHT;
			break;
		default:
			//EXIT IF SCREEN ROTATION NOT VALID!
			break;
	}
}

/*Enable LCD display*/
void ILI9341_Enable(void)
{
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
}

/*Enable LCD led*/
/*void ILI9488_LedEnable(void)
{
	HAL_GPIO_WritePin(LCD_LED_PORT, LCD_LED_PIN, GPIO_PIN_SET);
}*/

/*Disable LCD led*/
/*void ILI9488_LedDisable(void)
{
	HAL_GPIO_WritePin(LCD_LED_PORT, LCD_LED_PIN, GPIO_PIN_RESET);
}*/

/*Initialize and turn On the LCD display*/
void ILI9341_Init(void)
{
	ILI9341_Enable();
	ILI9341_SPI_Init();
	ILI9341_Reset();
	// ---
	ILI9341_Write_Command(0xE0);
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x03);
	ILI9341_Write_Data(0x09);
	ILI9341_Write_Data(0x08);
	ILI9341_Write_Data(0x16);
	ILI9341_Write_Data(0x0A);
	ILI9341_Write_Data(0x3F);
	ILI9341_Write_Data(0x78);
	ILI9341_Write_Data(0x4C);
	ILI9341_Write_Data(0x09);
	ILI9341_Write_Data(0x0A);
	ILI9341_Write_Data(0x08);
	ILI9341_Write_Data(0x16);
	ILI9341_Write_Data(0x1A);
	ILI9341_Write_Data(0x0F);

	ILI9341_Write_Command(0XE1);
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x16);
	ILI9341_Write_Data(0x19);
	ILI9341_Write_Data(0x03);
	ILI9341_Write_Data(0x0F);
	ILI9341_Write_Data(0x05);
	ILI9341_Write_Data(0x32);
	ILI9341_Write_Data(0x45);
	ILI9341_Write_Data(0x46);
	ILI9341_Write_Data(0x04);
	ILI9341_Write_Data(0x0E);
	ILI9341_Write_Data(0x0D);
	ILI9341_Write_Data(0x35);
	ILI9341_Write_Data(0x37);
	ILI9341_Write_Data(0x0F);

	ILI9341_Write_Command(0XC0);      //Power Control 1
	ILI9341_Write_Data(0x17);    //Vreg1out
	ILI9341_Write_Data(0x15);    //Verg2out

	ILI9341_Write_Command(0xC1);      //Power Control 2
	ILI9341_Write_Data(0x41);    //VGH,VGL

	ILI9341_Write_Command(0xC5);      //Power Control 3
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x12);    //Vcom
	ILI9341_Write_Data(0x80);

 	ILI9341_Write_Command(0x36);      //Memory Access
	ILI9341_Write_Data(0x48);

	ILI9341_Write_Command(0x3A);      // Interface Pixel Format
	ILI9341_Write_Data(0x66); 	  //18 bit

	ILI9341_Write_Command(0XB0);      // Interface Mode Control
	ILI9341_Write_Data(0x80);     			 //SDO NOT USE

	ILI9341_Write_Command(0xB1);      //Frame rate
	ILI9341_Write_Data(0xA0);    //60Hz

	ILI9341_Write_Command(0xB4);      //Display Inversion Control
	ILI9341_Write_Data(0x02);    //2-dot

	ILI9341_Write_Command(0XB6); //Display Function Control  RGB/MCU Interface Control

	ILI9341_Write_Data(0x02);    //MCU
	ILI9341_Write_Data(0x02);    //Source,Gate scan dieection

	ILI9341_Write_Command(0XE9);      // Set Image Functio
	ILI9341_Write_Data(0x00);    // Disable 24 bit data

	ILI9341_Write_Command(0xF7);      // Adjust Control
	ILI9341_Write_Data(0xA9);
	ILI9341_Write_Data(0x51);
	ILI9341_Write_Data(0x2C);
	ILI9341_Write_Data(0x82);    // D7 stream, loose

	ILI9341_Write_Command(0x11);    //Exit Sleep

	HAL_Delay(120);

	ILI9341_Write_Command(0x29);    //Display on

	//STARTING ROTATION
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
}

void ILI9341_Draw_Colour(uint16_t Colour)
{
//SENDS COLOUR
	uint8_t data[3];
	data[0] = (((Colour & 0xF800) >> 11) * 255) / 31; // r
	data[1] = (((Colour & 0x07E0) >> 5) * 255) / 63;  // g
	data[2] = ((Colour & 0x001F) * 255) / 31;

	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);

	SPI1_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(HSPI_INSTANCE, data, 3);
	while (SPI1_TX_completed_flag == 0)

	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

//INTERNAL FUNCTION OF LIBRARY
/*Sends block colour information to LCD*/
void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size)
{
//SENDS COLOUR
	uint8_t r = (Colour & 0xF800) >> 11;
	uint8_t g = (Colour & 0x07E0) >> 5;
	uint8_t b = Colour & 0x001F;

	r = (r * 255) / 31;
	g = (g * 255) / 63;
	b = (b * 255) / 31;

	uint32_t Buffer_Size = 0;
	if ((Size * 3) < BURST_MAX_SIZE)
	{
		Buffer_Size = Size*3;
	}
	else
	{
		Buffer_Size = BURST_MAX_SIZE;
	}

	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);

	//unsigned char MSB_color = Colour >> 8;

	unsigned char burst_buffer[Buffer_Size];
	for (uint32_t j = 0; j < Buffer_Size; j += 3)
	{
		burst_buffer[j] = r;
		burst_buffer[j + 1] = g;
		burst_buffer[j + 2] = b;
	}

	uint32_t Sending_Size = Size * 3;
	if (Size*3 == Buffer_Size)
	{
		Sending_Size = Buffer_Size;
	}
	uint32_t Sending_in_Block = Sending_Size / Buffer_Size;
	uint32_t Remainder_from_block = Sending_Size % Buffer_Size;
	//uint8_t str[100];
	//uint32_t timer = HAL_GetTick();
	if (Sending_in_Block != 0)
	{
		for (uint32_t j = 0; j < (Sending_in_Block); j++)
		{
			SPI1_TX_completed_flag = 0;
			HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (unsigned char*) burst_buffer, Buffer_Size);

			while (SPI1_TX_completed_flag == 0);
		}
	}
	//sprintf(str, "%u \n\r", HAL_GetTick()-timer);
	//HAL_UART_Transmit(&huart1, str, strlen((char *)str), 0xFFFF);

//REMAINDER!

	if (Remainder_from_block > 0)
	{
		SPI1_TX_completed_flag = 0;
		HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (unsigned char*) burst_buffer, Remainder_from_block);
		while (SPI1_TX_completed_flag == 0);
	}
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}


//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
void ILI9341_Fill_Screen(uint16_t Colour)
{
	ILI9341_Set_Address(0, 0, LCD_WIDTH, LCD_HEIGHT);
	ILI9341_Draw_Colour_Burst(Colour, LCD_WIDTH * LCD_HEIGHT);
}


void ILI9341_Draw_Pixel(uint16_t X, uint16_t Y, uint16_t Colour)
{
	if ((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT))
		return;	//OUT OF BOUNDS!

//ADDRESS
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2A);
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

//XDATA
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	unsigned char Temp_Buffer[4] =
	{ X >> 8, X, (X + 1) >> 8, (X + 1) };

	SPI1_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(HSPI_INSTANCE, Temp_Buffer, 4);
	while (SPI1_TX_completed_flag == 0);

	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

//ADDRESS
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2B);
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

//YDATA
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	unsigned char Temp_Buffer1[4] =
	{ Y >> 8, Y, (Y + 1) >> 8, (Y + 1) };

	SPI1_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(HSPI_INSTANCE, Temp_Buffer1, 4);
	while (SPI1_TX_completed_flag == 0);

	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

//ADDRESS
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2C);
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

//COLOUR
	uint8_t r = (Colour & 0xF800) >> 11;
		uint8_t g = (Colour & 0x07E0) >> 5;
		uint8_t b = Colour & 0x001F;

		r = (r * 255) / 31;
		g = (g * 255) / 63;
		b = (b * 255) / 31;
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	unsigned char Temp_Buffer2[3] =
	{ r, g, b };

	SPI1_TX_completed_flag = 0;
	HAL_SPI_Transmit_DMA(HSPI_INSTANCE, Temp_Buffer2, 3);
	while (SPI1_TX_completed_flag == 0);

	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

}

void ILI9341_Draw_Rectangle(uint16_t X, uint16_t Y, uint16_t Width,	uint16_t Height, uint16_t Colour)
{
	if ((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT))
		return;
	if ((X + Width - 1) >= LCD_WIDTH)
	{
		Width = LCD_WIDTH - X;
	}
	if ((Y + Height - 1) >= LCD_HEIGHT)
	{
		Height = LCD_HEIGHT - Y;
	}
	ILI9341_Set_Address(X, Y, X + Width - 1, Y + Height - 1);
	ILI9341_Draw_Colour_Burst(Colour, Height * Width);
}

void ILI9341_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour)
{
	if ((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT))
		return;
	if ((X + Width - 1) >= LCD_WIDTH)
	{
		Width = LCD_WIDTH - X;
	}
	ILI9341_Set_Address(X, Y, X + Width - 1, Y);
	ILI9341_Draw_Colour_Burst(Colour, Width);
}

void ILI9341_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour)
{
	if ((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT))
		return;
	if ((Y + Height - 1) >= LCD_HEIGHT)
	{
		Height = LCD_HEIGHT - Y;
	}
	ILI9341_Set_Address(X, Y, X, Y + Height - 1);
	ILI9341_Draw_Colour_Burst(Colour, Height);
}


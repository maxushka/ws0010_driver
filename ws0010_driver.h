/**
 * @file  			ws0010_driver.h
 * @description 	Library of OLED display management functions 
 *                  based on the WS0010 controller 
 *                  for microcontrollers STM32F4 series.
 * @author			M.Beletsky
 * @date 			2019-11-11
 */

#ifndef _WS0010_DRIVER_H
#define _WS0010_DRIVER_H

//----------------------------------------------------------------------
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

//----------------------------------------------------------------------

/**
 * -------------------------------
 * @defgroup WS0010_Commands
 * -------------------------------
 */

// Control commands
#define LCD_CLEAR_DISPLAY 			0x01 // Clears the display
#define LCD_RETURN_HOME 			0x02 // Returns the cursor to the home position
#define LCD_FUNCTION_SET 			0x28
#define LCD_SET_CGRAM_ADDR 			0x40 // Set a specific character RAM address, use to add custom characters
#define LCD_SET_DDRAM_ADDR 			0x80 // Set a specific DDRAM address

// flags for display entry mode
#define LCD_ENTRY_MODE_SET 			0x04 // Use with entry mode flags to change how the text is inserted
#define LCD_ENTRY_RIGHT 			0x00 // Right-to-left text
#define LCD_ENTRY_LEFT 				0x02 // Left-to-right text
#define LCD_ENTRY_SHIFT_INCREMENT 	0x01 // 'Right justify' text
#define LCD_ENTRY_SHIFT_DECREMENT 	0x00 // 'Left justify' text

// flags for display on/off control
#define LCD_DISPLAY_CONTROL 		0x08 // Use to turn the display on/off and toggle cursor/blink.
#define LCD_DISPLAY_ON 				0x04 // Turn the display on
#define LCD_DISPLAY_OFF 			0x00 // Turn the display off
#define LCD_CURSOR_ON 				0x02 // Turn the cursor visible
#define LCD_CURSOR_OFF 				0x00 // Turn the cursor invisible
#define LCD_BLINK_ON 				0x01 // Turn on cursor blinking
#define LCD_BLINK_OFF 				0x00 // Turn off cursor blinking

// flags for display/cursor shift
#define LCD_CURSOR_SHIFT 			0x10 // Use with LCD_DISPLAY_MOVE to scroll the display without chaning the RAM
#define LCD_DISPLAY_MOVE 			0x08 // Use with LCD_CURSOR_SHIFT to scroll the display without chaning the RAM.
#define LCD_CURSOR_MOVE 			0x00 // Move cursor
#define LCD_MOVE_RIGHT 				0x04 // Scroll the display right
#define LCD_MOVE_LEFT 				0x00 // Scroll the display left

// Power and display modes
#define LCD_POWER_ON 				0x07 // Use with LCD_CURSOR_SHIFT to turn on the power
#define LCD_POWER_OFF 				0x03 // Use with LCD_CURSOR_SHIFT to turn off the power
#define LCD_GRAPHIC_MODE 			0x0B // Use with LCD_CURSOR_SHIFT to go in graphic mode (don't forget power)
#define LCD_CHARACTER_MODE 			0x03 // Use with LCD_CURSOR_SHIFT to go in graphic mode (don't forget power)

// flags for function set
#define LCD_8BIT_MODE   			0x10
#define LCD_4BIT_MODE   			0x00
#define LCD_JAPANESE    			0x00
#define LCD_EUROPEAN_I  			0x01
#define LCD_RUSSIAN     			0x02
#define LCD_EUROPEAN_II 			0x03

//----------------------------------------------------------------------

typedef struct
{
	GPIO_TypeDef *Data_Port;
	uint16_t Pin_DB0;
	uint16_t Pin_DB1;
	uint16_t Pin_DB2;
	uint16_t Pin_DB3;
	uint16_t Pin_DB4;
	uint16_t Pin_DB5;
	uint16_t Pin_DB6;
	uint16_t Pin_DB7;

	GPIO_TypeDef *RW_Port;
	uint16_t RW_Pin;

	GPIO_TypeDef *EN_Port;
	uint16_t EN_Pin;

	GPIO_TypeDef *RS_Port;
	uint16_t RS_Pin;

} WS0010_PlatformInit_Typedef;

typedef enum
{
	LOW = 0,
	HIGH = 1	
} WS0010_State_Typedef;

typedef enum
{
	MODE_WRITE = 0,
	MODE_READ = 1
} WS0010_RW_Mode_Typedef;

typedef enum
{
	MODE_DATA = 1,
	MODE_CONTROL = 0
} WS0010_RS_Mode_Typedef;

typedef enum
{
	EN = 0,
	RUS = 1
} TypeChar_Typedef;

//----------------------------------------------------------------------

extern volatile uint32_t _ws0010_timeout;

//----------------------------------------------------------------------

#define ws0010_start_timer()		{TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); TIM_Cmd(TIM3, ENABLE);}
#define ws0010_stop_timer()			{TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE); TIM_Cmd(TIM3, DISABLE);}

#ifdef USE_HAL
	#define ws0010_write_bit(port, pin, state)		HAL_GPIO_WritePin(port, pin, state)
	#define ws0010_write_port(port, data) 			HAL_GPIO_Write(port, data)
	#define ws0010_read_bit(port)					HAL_GPIO_ReadPin(port)
#else
	#define ws0010_write_bit(port, pin, state)		GPIO_WriteBit(port, pin, state)
	#define ws0010_write_port(port, data) 			GPIO_Write(port, data)
	#define ws0010_read_port(port)					GPIO_ReadInputData(port)
#endif

#define WS0010_DelayUs()			for(int i = 0; i < 5000; i++)

//----------------------------------------------------------------------

void WS0010_PlatformInit(WS0010_PlatformInit_Typedef *pltf);
void WS0010_Delay(uint32_t ticks);
void WS0010_SetMode_RW(WS0010_PlatformInit_Typedef *pltf, WS0010_RW_Mode_Typedef mode);
void WS0010_SetMode_RS(WS0010_PlatformInit_Typedef *pltf, WS0010_RS_Mode_Typedef mode);
void WS0010_Pins_Data_Write(WS0010_PlatformInit_Typedef *pltf, uint8_t data);
uint8_t WS0010_Pins_Data_Read(WS0010_PlatformInit_Typedef *pltf);
void WS0010_Pin_Enable_Write(WS0010_PlatformInit_Typedef *pltf, WS0010_State_Typedef state);
void WS0010_Init(WS0010_PlatformInit_Typedef *pltf);
void WS0010_WriteControl(WS0010_PlatformInit_Typedef *pltf, unsigned char cmd);
void WS0010_WriteData(WS0010_PlatformInit_Typedef *pltf, unsigned char data);
void WS0010_SendByte(WS0010_PlatformInit_Typedef *pltf, unsigned char byte);
void WS0010_SendNibble(WS0010_PlatformInit_Typedef *pltf, unsigned char nibble);
void WS0010_IsReady(WS0010_PlatformInit_Typedef *pltf);
void WS0010_Position(WS0010_PlatformInit_Typedef *pltf, uint8_t column, uint8_t row);
void WS0010_Sleep(WS0010_PlatformInit_Typedef *pltf);
void WS0010_Wakeup(WS0010_PlatformInit_Typedef *pltf);
void WS0010_PrintString(WS0010_PlatformInit_Typedef *pltf, char *string, TypeChar_Typedef type);
void WS0010_PrintByte(WS0010_PlatformInit_Typedef *pltf, unsigned char byte);
char NibbleToAscii(char nibble);

//----------------------------------------------------------------------

#endif //_WS0010_DRIVER_H

/**************************** END OF FILE ****************************/

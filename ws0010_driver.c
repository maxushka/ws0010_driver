/**
 * @file  			ws0010_driver.c
 * @description 	Library of OLED display management functions 
 *                  based on the WS0010 controller 
 *                  for microcontrollers STM32F4 series.
 * @author			M.Beletsky
 * @date 			2019-11-11
 */

#include "ws0010_driver.h"

// Offset for each row in DDRAM
unsigned char row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
// Extern variable for delay
volatile uint32_t _ws0010_timeout = 0;

/**
 * The function initializes the display control I/O ports
 * Before initializing the platform, enable clocking of the bus 
 * on which the display pins are located.
 * For the delay functions, the TIM3 timer is used. 
 * If you are already using this timer, reassign it to another 
 * in the initialization function. 
 * Also change the interrupt handler from this timer.
 * 
 * @param  pltf       Platform I/O Port Structure
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_PlatformInit(WS0010_PlatformInit_Typedef *pltf)
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Pin = pltf->Pin_DB0 | pltf->Pin_DB1 | pltf->Pin_DB2 | pltf->Pin_DB3 |
					pltf->Pin_DB4 | pltf->Pin_DB5 | pltf->Pin_DB6 | pltf->Pin_DB7;
	GPIO_Init(pltf-Data_Port, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Pin = pltf->RW_Pin;
	GPIO_Init(pltf-RW_Port, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Pin = pltf->EN_Pin;
	GPIO_Init(pltf-EN_Port, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Pin = pltf->RS_Pin;
	GPIO_Init(pltf-RS_Port, &gpio);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef tim;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 42;
	tim.TIM_Period = 10;
	TIM_TimeBaseInit(TIM3, &tim);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);	
}

/**
 * TIM3 interrupt handler
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		if (_ws0010_timeout !=0)
			_ws0010_timeout--;
	}
}

/**
 * WS0010_DelayUs
 * @param  ticks      Time in us
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_DelayUs(uint32_t ticks)
{
	_ws0010_timeout = ticks;

	ws0010_start_timer();
	while(_ws0010_timeout != 0);
	ws0010_stop_timer();
}

/**
 * WS0010_Delay
 * @param  ticks      Time in ms
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Delay(uint32_t ticks)
{
	_ws0010_timeout = ticks*1000;

	ws0010_start_timer();
	while(_ws0010_timeout != 0);
	ws0010_stop_timer();
}

/**
 * Read or write mode selection function
 * @param  pltf       Platform I/O Port Structure
 * @param  mode       See the WS0010_RW_Mode_Typedef enum
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_SetMode_RW(WS0010_PlatformInit_Typedef *pltf, WS0010_Mode_Typedef mode)
{
	GPIO_WriteBit(pltf->RW_Port, pltf->RW_Pin, (BitAction)mode);
}

/**
 * Data or control mode selection function
 * @param  pltf       Platform I/O Port Structure
 * @param  mode       See the WS0010_RS_Mode_Typedef enum
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_SetMode_RS(WS0010_PlatformInit_Typedef *pltf, WS0010_RS_Mode_Typedef mode)
{
	GPIO_WriteBit(pltf->RS_Port, pltf->RS_Pin, (BitAction)mode);
}

/**
 * Function for writing data to function pins
 * @param  pltf       Platform I/O Port Structure
 * @param  data       Data to write
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Pins_Data_Write(WS0010_PlatformInit_Typedef *pltf, uint8_t data)
{
	GPIO_Write(pltf->Data_Port, (data&0xFF));
}

/**
 * Function for reading data from function pins
 * @param  pltf       Platform I/O Port Structure
 * @return            Read data
 * @author M. Beletsky
 * @date:  2019-11-11
 */
uint8_t WS0010_Pins_Data_Read(WS0010_PlatformInit_Typedef *pltf)
{
	return (uint8_t)(GPIO_ReadInputData(pltf->Data_Port) & 0xFF);
}

/**
 * Enable pin management function
 * @param  pltf       Platform I/O Port Structure
 * @param  state      See WS0010_State_Typedef
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Pin_Enable_Write(WS0010_PlatformInit_Typedef *pltf, WS0010_State_Typedef state)
{
	GPIO_WriteBit(pltf->EN_Port, pltf->EN_Pin, (BitAction)state);
}

/**
 * Display Initialization Function
 * @param  pltf       Platform I/O Port Structure
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Init(WS0010_PlatformInit_Typedef *pltf)
{
    // Wait for power-on
    WS0010_Delay(75);

    // Put in 8-bit mode
    WS0010_SendNibble(pltf, 0x03);
    WS0010_DelayUs(5000);
    WS0010_SendNibble(pltf, 0x08);
    WS0010_DelayUs(5000);
    
    // Put back in 4-bit mode    
    WS0010_SendNibble(pltf, 0x02);
    WS0010_DelayUs(5000);
    WS0010_SendNibble(pltf, 0x02);
    WS0010_DelayUs(5000);
    WS0010_SendNibble(pltf, 0x08);
    WS0010_DelayUs(5000);

    // Turn display off
    WS0010_WriteControl(pltf, LCD_DISPLAY_CONTROL | LCD_DISPLAY_OFF);
    WS0010_DelayUs(500);

    // Clear Display
    WS0010_WriteControl(pltf, LCD_CLEAR_DISPLAY);
    WS0010_DelayUs(500);

    // Set Entry Mode - Left
    WS0010_WriteControl(pltf, LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT);
    WS0010_DelayUs(500);

    // Home Cursor
    WS0010_WriteControl(pltf, LCD_RETURN_HOME);
    WS0010_DelayUs(500);

    // Turn On
    WS0010_Wakeup(pltf);
}

/**
 * Function for writing a command to the control register
 * @param  pltf       Platform I/O Port Structure
 * @param  cmd        Control commands see in @defgroup WS0010_Commands
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_WriteControl(WS0010_PlatformInit_Typedef *pltf, unsigned char cmd)
{
    // Control + Write mode
    WS0010_SetMode_RS(pltf, MODE_CONTROL);
    WS0010_SetMode_RW(pltf, MODE_WRITE);

    // Send bytes
    WS0010_SendByte(pltf, cmd);

    // Wait for the display to stop working
    WS0010_IsReady(pltf);
}

/**
 * Function of data writing for display on the screen
 * @param  pltf       Platform I/O Port Structure
 * @param  data       Byte data to write
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_WriteData(WS0010_PlatformInit_Typedef *pltf, unsigned char data)
{
    // Data + Write mode
    WS0010_SetMode_RS(pltf, MODE_DATA);
    WS0010_SetMode_RW(pltf, MODE_WRITE);
    
    // Send bytes
    WS0010_SendByte(pltf, data);

    // Wait for the display to stop working
    WS0010_IsReady(pltf);
}

/**
 * Single byte send function
 * @param  pltf       Platform I/O Port Structure
 * @param  byte       Byte to send
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_SendByte(WS0010_PlatformInit_Typedef *pltf, unsigned char byte)
{
	// Send high
	WS0010_SendNibble(pltf, byte >> 4);
	// Send low
	WS0010_SendNibble(pltf, byte);
}

/**
 * Half byte send function
 * @param  pltf       Platform I/O Port Structure
 * @param  nibble     Half byte to send
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_SendNibble(WS0010_PlatformInit_Typedef *pltf, unsigned char nibble)
{
    // Send high
    WS0010_Pins_Data_Write(pltf, nibble);
    WS0010_DelayUs(50);
    WS0010_Pin_Enable_Write(pltf, HIGH);
    WS0010_DelayUs(50);
    WS0010_Pin_Enable_Write(pltf LOW);
    WS0010_DelayUs(50);
}

/**
 * [WS0010_IsReady description]
 * @param  pltf       Platform I/O Port Structure
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_IsReady(WS0010_PlatformInit_Typedef *pltf)
{
    unsigned char isBusy = 1;
    
    WS0010_SetMode_RS(pltf, MODE_CONTROL); // Control mode
    WS0010_SetMode_RW(pltf, MODE_READ); // Read mode
    WS0010_Pins_Data_Write(pltf, 0x8);

    do {
        // Get the four first bits
        WS0010_Pin_Enable_Write(pltf, LOW);
        WS0010_DelayUs(10);
        WS0010_Pin_Enable_Write(pltf, HIGH);
        
        WS0010_DelayUs(10);
        isBusy = (WS0010_Pins_Data_Read() >> 3);
        WS0010_Pin_Enable_Write(pltf, LOW);
        
        // Get the rest 4 bits, not used
        WS0010_DelayUs(50);
        WS0010_Pin_Enable_Write(pltf, HIGH);
        WS0010_DelayUs(50);
        WS0010_Pin_Enable_Write(pltf, LOW);
    } while (isBusy == 1);
    
    // Set to write mode
    WS0010_SetMode_RW(pltf, MODE_WRITE);
}

/**
 * Function to set the cursor to a given position
 * @param  pltf       Platform I/O Port Structure
 * @param  column     Number of column
 * @param  row        Number of row
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Position(WS0010_PlatformInit_Typedef *pltf, uint8_t column, uint8_t row)
{
  if ( row >= 2 ) {
    row = 0;  //write to first line if out off bounds
  }
  
  WS0010_WriteControl(pltf, LCD_SET_DDRAM_ADDR | (column + row_offsets[row]));
}

/**
 * Display Screen Off Function
 * @param  pltf       Platform I/O Port Structure
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Sleep(WS0010_PlatformInit_Typedef *pltf)
{
    WS0010_WriteControl(pltf, LCD_DISPLAY_CONTROL | LCD_DISPLAY_OFF);
    WS0010_WriteControl(pltf, LCD_CURSOR_SHIFT | LCD_POWER_OFF);
}

/**
 * Display enable function
 * @param  pltf       Platform I/O Port Structure
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Wakeup(WS0010_PlatformInit_Typedef *pltf)
{
    WS0010_WriteControl(pltf, LCD_CURSOR_SHIFT | LCD_POWER_ON | LCD_CHARACTER_MODE);
    WS0010_WriteControl(pltf, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON);
}

/**
 * The function of displaying a string on the display screen
 * @param  pltf       Platform I/O Port Structure
 * @param  string     String to print
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_PrintString(WS0010_PlatformInit_Typedef *pltf, char *string)
{
    unsigned char index = 0;
    while (string[index] != '\0') {
        WS0010_WriteData(pltf, string[index]);
        index++;
    }
}

/**
 * The function of displaying a symbol on the display screen
 * @param  pltf       Platform I/O Port Structure
 * @param  byte       Byte to print
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_PrintByte(WS0010_PlatformInit_Typedef *pltf, unsigned char byte)
{
    char string[3];
    string[0] = NibbleToAscii(byte >> 4);
    string[1] = NibbleToAscii(byte & 0x0F);
    string[2] = '\0';
    
    WS0010_PrintString(pltf, string);
}

/**
 * The function of converting half byte to ASCII
 * @param  nibble     Half byte to convert
 * @return            Char ASCII symbol
 * @author M. Beletsky
 * @date:  2019-11-11
 */
char NibbleToAscii(char nibble)
{
    if (nibble < 0xA) {
        return nibble + 48;
    } else {
        return nibble + 55; // +65 to reach ABCDEF, -9 to remove 0-9
    }
}

/**************************** END OF FILE ****************************/

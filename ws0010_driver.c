/**
 * @file            ws0010_driver.c
 * @description     Library of OLED display management functions 
 *                  based on the WS0010 controller 
 *                  for microcontrollers STM32F4 series.
 * @author          M.Beletsky
 * @date            2019-11-11
 */

#include "ws0010_driver.h"
#include <string.h>

// Extern variable for delay
volatile uint32_t _ws0010_timeout = 0;

const char utf_recode[] =
       { 0x41,0xa0,0x42,0xa1,0xe0,0x45,0xa3,0xa4,0xa5,0xa6,0x4b,0xa7,0x4d,0x48,0x4f,
         0xa8,0x50,0x43,0x54,0xa9,0xaa,0x58,0xe1,0xab,0xac,0xe2,0xad,0xae,0x62,0xaf,0xb0,0xb1,
         0x61,0xb2,0xb3,0xb4,0xe3,0x65,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0x6f,
         0xbe,0x70,0x63,0xbf,0x79,0xe4,0x78,0xe5,0xc0,0xc1,0xe6,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7
        };   

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

    /** Configure data pins */
    gpio.GPIO_Pin = pltf->Pin_DB0 | pltf->Pin_DB1 | pltf->Pin_DB2 | pltf->Pin_DB3 |
                    pltf->Pin_DB4 | pltf->Pin_DB5 | pltf->Pin_DB6 | pltf->Pin_DB7;
#ifdef USE_HAL
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pltf->Data_Port, &gpio);
#else
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(pltf->Data_Port, &gpio);
#endif

    /** Configure RW pin */
    gpio.GPIO_Pin = pltf->RW_Pin;
#ifdef USE_HAL
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pltf->RW_Port, &gpio);
#else
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(pltf->RW_Port, &gpio);
#endif

    /** Configure EN pin */
    gpio.GPIO_Pin = pltf->EN_Pin;
#ifdef USE_HAL
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pltf->EN_Port, &gpio);
#else
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(pltf->EN_Port, &gpio);
#endif

    /** Configure RS pin */
    gpio.GPIO_Pin = pltf->RS_Pin;
#ifdef USE_HAL
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pltf->RS_Port, &gpio);
#else
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(pltf->RS_Port, &gpio);
#endif

#ifndef USE_HAL
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
#endif  
}

#ifndef USE_HAL
/**
 * TIM3 interrupt handler
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (_ws0010_timeout !=0)
            _ws0010_timeout--;
    }
}
#endif

/**
 * WS0010_Delay
 * @param  ticks      Time in ms
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_Delay(uint32_t ticks)
{
#ifdef USE_HAL
    HAL_Delay(ticks);
#else
    _ws0010_timeout = ticks*1000;

    ws0010_start_timer();
    while(_ws0010_timeout != 0){}
    ws0010_stop_timer();
#endif
}

/**
 * Read or write mode selection function
 * @param  pltf       Platform I/O Port Structure
 * @param  mode       See the WS0010_RW_Mode_Typedef enum
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_SetMode_RW(WS0010_PlatformInit_Typedef *pltf, WS0010_RW_Mode_Typedef mode)
{
    GPIO_InitTypeDef gpio;

    ws0010_write_bit(pltf->RW_Port, pltf->RW_Pin, (BitAction)mode);
    
    if (mode == MODE_READ)
    {
        gpio.GPIO_Pin = pltf->Pin_DB0 | pltf->Pin_DB1 | pltf->Pin_DB2 | pltf->Pin_DB3 |
                    pltf->Pin_DB4 | pltf->Pin_DB5 | pltf->Pin_DB6 | pltf->Pin_DB7;
#ifdef USE_HAL
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(pltf->Data_Port, &gpio);
#else
        gpio.GPIO_Mode = GPIO_Mode_IN;
        gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
        gpio.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(pltf->Data_Port, &gpio);
#endif
    }
    else
    {
        gpio.GPIO_Pin = pltf->Pin_DB0 | pltf->Pin_DB1 | pltf->Pin_DB2 | pltf->Pin_DB3 |
                    pltf->Pin_DB4 | pltf->Pin_DB5 | pltf->Pin_DB6 | pltf->Pin_DB7;      
#ifdef USE_HAL
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(pltf->Data_Port, &gpio);
#else
        gpio.GPIO_Mode = GPIO_Mode_OUT;
        gpio.GPIO_OType = GPIO_OType_PP;
        gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
        gpio.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(pltf->Data_Port, &gpio);
#endif
    }
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
    ws0010_write_bit(pltf->RS_Port, pltf->RS_Pin, (BitAction)mode);
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
    ws0010_write_port(pltf->Data_Port, (data&0xFF));
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
    return (uint8_t)(ws0010_read_port(pltf->Data_Port) & 0xFF);
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
    ws0010_write_bit(pltf->EN_Port, pltf->EN_Pin, (BitAction)state);
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
    WS0010_Delay(100);

        // Functions set
    WS0010_WriteControl(pltf, LCD_FUNCTION_SET | LCD_8BIT_MODE | 0x08 | 0x00 |LCD_RUSSIAN);
    WS0010_Delay(50);
    
    // Turn display off
    WS0010_WriteControl(pltf, LCD_DISPLAY_CONTROL | LCD_DISPLAY_OFF);
    WS0010_Delay(5);

        // Clear Display
    WS0010_WriteControl(pltf, LCD_CLEAR_DISPLAY);
    WS0010_Delay(5);

    // Set Entry Mode - Left
    WS0010_WriteControl(pltf, LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT);
    WS0010_Delay(5);

    // Home Cursor
    WS0010_WriteControl(pltf, LCD_RETURN_HOME);
    WS0010_Delay(5);

    // Turn On
    WS0010_Wakeup(pltf);
    WS0010_Delay(5);
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
    // Sometimes not work
    //WS0010_IsReady(pltf);
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
    /** Uncommment this line if you use 4bit mode */
    //WS0010_SendNibble(pltf, byte >> 4);
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
    WS0010_DelayUs();
    WS0010_Pin_Enable_Write(pltf, HIGH);
    WS0010_DelayUs();
    WS0010_Pin_Enable_Write(pltf, LOW);
    WS0010_DelayUs();
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
    //WS0010_Pins_Data_Write(pltf, 0x8);  
    WS0010_SetMode_RW(pltf, MODE_READ); // Read mode

    do {
        // Get the four first bits
        WS0010_Pin_Enable_Write(pltf, LOW);
        WS0010_DelayUs();
        WS0010_Pin_Enable_Write(pltf, HIGH);
        
        WS0010_DelayUs();
        isBusy = (WS0010_Pins_Data_Read(pltf) >> 3);
        WS0010_Pin_Enable_Write(pltf, LOW);
        
        WS0010_DelayUs();
        WS0010_Pin_Enable_Write(pltf, HIGH);
        WS0010_DelayUs();
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
    uint8_t address = 0;
    
    if (column < 20 && row < 2)
    {
        address = column + (row * 0x40);
    }

    WS0010_WriteControl(pltf, LCD_SET_DDRAM_ADDR | (address & 0x7F));
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
    WS0010_WriteControl(pltf, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_ON);
}

/**
 * The function of displaying a string on the display screen
 * @param  pltf       Platform I/O Port Structure
 * @param  string     String to print
 * @author M. Beletsky
 * @date:  2019-11-11
 */
void WS0010_PrintString(WS0010_PlatformInit_Typedef *pltf, char *string, TypeChar_Typedef type)
{
    if (type == RUS)
    {
        while (*string != '\0')
        {
            WS0010_WriteData(pltf, utf_recode[*string % 0x40]);
            string++;
        }
    }
    else
    {
        while (*string != '\0')
        {
            WS0010_WriteData(pltf, *string);
            string++;
        }
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
    
    WS0010_PrintString(pltf, string, EN);
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

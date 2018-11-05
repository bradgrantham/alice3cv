#include <stm32l0xx_hal.h>
#include "leds.h"
#include "delay.h"
#include "uart.h"

extern "C" {

void panic_worse()
{
    LED_set_panic(1);
    for(;;);
}

void panic(void)
{
    static int entered = 0;

    LED_set_panic(1);

    int pin = 0;
    for(;;) {
        if(!entered) {
            // SERIAL_flush() can itself panic(), so stop reentry here
            entered = 1;
            // SERIAL_flush();
            entered = 0;
        }

        LED_set_panic(pin);
        pin = pin ? 0 : 1;
        delay_ms(1000);
    }
}

};


//----------------------------------------------------------------------------
// System Initialization Goop

void SystemClock_Config(void)
{
  static RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  static RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    panic();
  }
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0)!= HAL_OK)
  {
    /* Initialization Error */
    panic();
  }
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
  
}


void system_init()
{
    HAL_Init();

    SystemClock_Config();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

static inline void set_GPIO_value(GPIO_TypeDef* gpio, int mask, int value)
{
    unsigned long int data = value ? mask : 0;
    gpio->ODR = (gpio->ODR & ~mask) | data;
    __asm__ volatile("" ::: "memory"); // Force all memory operations before to come before and all after to come after.
}

static inline void set_GPIO_iotype(GPIO_TypeDef* gpio, int pin, unsigned int iotype)
{
    long unsigned int mask = ~(3U << (pin * 2));
    long unsigned int value = iotype << (pin * 2);
    gpio->MODER = (gpio->MODER & mask) | value;
    __asm__ volatile("" ::: "memory"); // Force all memory operations before to come before and all after to come after.
}

struct gpio {
    GPIO_TypeDef* port;
    int pin;
    unsigned int mask;
    gpio(GPIO_TypeDef* port_, int pin_) :
        port(port_),
        pin(pin_)
    {
        mask = 1U << pin;
    }
};

/* 
DB9's are wired backwards.

actual  schem	GPIO
SK1     N1	PA6
E1	S1	PA4
W1	W1	PA2
S1	E1	PA1
N1	SK      PC15
QB1	F1      PA5
SJ1	QA      PA3
QA1	SJ      PC14
F1	QB      PA0

SK2     N2	PB3
E2	S2	PA14
W2	W2	PA8
S2	E2	PB1
N2	SK      PC15
QB2	F2      PA15
SJ2	QA      PA13
QA2	SJ      PC14
F2	QB      PB0
*/

gpio select_joystick_1 = { GPIOA, 3};
gpio select_keypad_1 = { GPIOA, 6};
gpio joystick_1_north = { GPIOC, 15};
gpio joystick_1_south = { GPIOA, 1};
gpio joystick_1_east = { GPIOA, 4};
gpio joystick_1_west = { GPIOA, 2};
gpio joystick_1_fire = { GPIOA, 0};

gpio select_joystick_2 = { GPIOA, 13};
gpio select_keypad_2 = { GPIOB, 3};
gpio joystick_2_north = { GPIOC, 15};
gpio joystick_2_south = { GPIOB, 1};
gpio joystick_2_east = { GPIOA, 14};
gpio joystick_2_west = { GPIOA, 8};
gpio joystick_2_fire = { GPIOB, 0};

const int CONTROLLER_FIRE_BIT = 0x40;
const int CONTROLLER_NORTH_BIT = 0x01;
const int CONTROLLER_EAST_BIT = 0x02;
const int CONTROLLER_SOUTH_BIT = 0x04;
const int CONTROLLER_WEST_BIT = 0x08;
const int CONTROLLER_KEYPAD_MASK = 0x0F;
const int CONTROLLER_KEYPAD_0 = 0x05;
const int CONTROLLER_KEYPAD_1 = 0x02;
const int CONTROLLER_KEYPAD_2 = 0x08;
const int CONTROLLER_KEYPAD_3 = 0x03;
const int CONTROLLER_KEYPAD_4 = 0x0D;
const int CONTROLLER_KEYPAD_5 = 0x0C;
const int CONTROLLER_KEYPAD_6 = 0x01;
const int CONTROLLER_KEYPAD_7 = 0x0A;
const int CONTROLLER_KEYPAD_8 = 0x0E;
const int CONTROLLER_KEYPAD_9 = 0x04;
const int CONTROLLER_KEYPAD_asterisk = 0x09;
const int CONTROLLER_KEYPAD_pound = 0x06;

unsigned char joystick_1_state;
unsigned char joystick_1_changed;
unsigned char keypad_1_state;
unsigned char keypad_1_changed;
unsigned char joystick_2_state;
unsigned char joystick_2_changed;
unsigned char keypad_2_state;
unsigned char keypad_2_changed;

void flash_it(int flashes)
{
    for(int i = 0; i < flashes; i++) {
        LED_set_info(1);
        delay_ms(200);
        LED_set_info(0);
        delay_ms(200);
    }
    delay_ms(500);
}

void print(const char *s)
{
    while(*s) {
        if(*s == '\n')
            SERIAL_send_one_char('\r');
        SERIAL_send_one_char(*s++);
    }
}

void init_controller_1(void)
{
    joystick_1_state = 0;
    joystick_1_changed = 0;
    keypad_1_state = 0;
    keypad_1_changed = 0;

    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = joystick_1_north.mask;
    HAL_GPIO_Init(joystick_1_north.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_south.mask;
    HAL_GPIO_Init(joystick_1_south.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_west.mask;
    HAL_GPIO_Init(joystick_1_west.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_east.mask;
    HAL_GPIO_Init(joystick_1_east.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_1_fire.mask;
    HAL_GPIO_Init(joystick_1_fire.port, &GPIO_InitStruct);
}

void init_controller_2(void)
{
    joystick_2_state = 0;
    joystick_2_changed = 0;
    keypad_2_state = 0;
    keypad_2_changed = 0;

    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = joystick_2_north.mask;
    HAL_GPIO_Init(joystick_2_north.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_south.mask;
    HAL_GPIO_Init(joystick_2_south.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_west.mask;
    HAL_GPIO_Init(joystick_2_west.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_east.mask;
    HAL_GPIO_Init(joystick_2_east.port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = joystick_2_fire.mask;
    HAL_GPIO_Init(joystick_2_fire.port, &GPIO_InitStruct);
}

void probe_controller_1(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_1.mask;
    HAL_GPIO_WritePin(select_joystick_1.port, select_joystick_1.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_SET);
    HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 

    delay_ms(1);

    unsigned int joystick_value = (
        (HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask) << 6)
        ) ^ 0x4F;

    HAL_GPIO_WritePin(select_joystick_1.port, select_joystick_1.mask, GPIO_PIN_SET);

    HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_RESET);

    unsigned int keypad_value = 
        ((HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask) << 6)
        ) ^ 0x4F;

    HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_SET);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_1.mask;
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 
    delay_ms(1);

    unsigned char changed = joystick_value ^ joystick_1_state;
    joystick_1_changed |= changed;
    joystick_1_state = joystick_value;

    changed = keypad_value ^ keypad_1_state;
    keypad_1_changed |= changed;
    keypad_1_state = keypad_value;
}

void probe_controller_2(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_2.mask;
    HAL_GPIO_WritePin(select_joystick_2.port, select_joystick_2.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_joystick_2.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_2.mask;
    HAL_GPIO_WritePin(select_keypad_2.port, select_keypad_2.mask, GPIO_PIN_SET);
    HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 

    delay_ms(1);

    unsigned int joystick_value = (
        (HAL_GPIO_ReadPin(joystick_2_north.port, joystick_2_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_2_east.port, joystick_2_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_2_south.port, joystick_2_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_2_west.port, joystick_2_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_2_fire.port, joystick_2_fire.mask) << 6)
        ) ^ 0x4F;

    HAL_GPIO_WritePin(select_joystick_2.port, select_joystick_2.mask, GPIO_PIN_SET);

    HAL_GPIO_WritePin(select_keypad_2.port, select_keypad_2.mask, GPIO_PIN_RESET);

    unsigned int keypad_value = 
        ((HAL_GPIO_ReadPin(joystick_2_north.port, joystick_2_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_2_east.port, joystick_2_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_2_south.port, joystick_2_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_2_west.port, joystick_2_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_2_fire.port, joystick_2_fire.mask) << 6)
        ) ^ 0x4F;

    HAL_GPIO_WritePin(select_keypad_2.port, select_keypad_2.mask, GPIO_PIN_SET);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_2.mask;
    HAL_GPIO_Init(select_joystick_2.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_2.mask;
    HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 

    delay_ms(1);


    unsigned char changed = joystick_value ^ joystick_2_state;
    joystick_2_changed |= changed;
    joystick_2_state = joystick_value;

    changed = keypad_value ^ keypad_2_state;
    keypad_2_changed |= changed;
    keypad_2_state = keypad_value;
}


int main()
{
    system_init();

    SERIAL_init();

    LED_init();

    LED_set_info(1);
        delay_ms(250);
    LED_set_info(0);
        delay_ms(250);
    LED_set_info(1);
        delay_ms(250);
    LED_set_info(0);
        delay_ms(250);
    LED_set_info(1);
        delay_ms(250);
    LED_set_info(0);
        delay_ms(500);

    print("OK\n");

    // XXX

    init_controller_1();
    init_controller_2();

    while(1) {
        int on;

        probe_controller_1();

        if(joystick_1_changed & CONTROLLER_NORTH_BIT)
            print((joystick_1_state & CONTROLLER_NORTH_BIT) ? "N1 press\n" : "N1 release\n");
        if(joystick_1_changed & CONTROLLER_SOUTH_BIT)
            print((joystick_1_state & CONTROLLER_SOUTH_BIT) ? "S1 press\n" : "S1 release\n");
        if(joystick_1_changed & CONTROLLER_WEST_BIT)
            print((joystick_1_state & CONTROLLER_WEST_BIT) ? "W1 press\n" : "W1 release\n");
        if(joystick_1_changed & CONTROLLER_EAST_BIT)
            print((joystick_1_state & CONTROLLER_EAST_BIT) ? "E1 press\n" : "E1 release\n");
        if(joystick_1_changed & CONTROLLER_FIRE_BIT)
            print((joystick_1_state & CONTROLLER_FIRE_BIT) ? "FL1 press\n" : "FL1 release\n");

        joystick_1_changed = 0;

        if((keypad_1_changed & CONTROLLER_KEYPAD_MASK) != 0) {
            switch(keypad_1_state & CONTROLLER_KEYPAD_MASK) {
                case CONTROLLER_KEYPAD_0: print("KEYPAD 1 0\n"); break;
                case CONTROLLER_KEYPAD_1: print("KEYPAD 1 1\n"); break;
                case CONTROLLER_KEYPAD_2: print("KEYPAD 1 2\n"); break;
                case CONTROLLER_KEYPAD_3: print("KEYPAD 1 3\n"); break;
                case CONTROLLER_KEYPAD_4: print("KEYPAD 1 4\n"); break;
                case CONTROLLER_KEYPAD_5: print("KEYPAD 1 5\n"); break;
                case CONTROLLER_KEYPAD_6: print("KEYPAD 1 6\n"); break;
                case CONTROLLER_KEYPAD_7: print("KEYPAD 1 7\n"); break;
                case CONTROLLER_KEYPAD_8: print("KEYPAD 1 8\n"); break;
                case CONTROLLER_KEYPAD_9: print("KEYPAD 1 9\n"); break;
                case CONTROLLER_KEYPAD_asterisk: print("KEYPAD 1 asterisk\n"); break;
                case CONTROLLER_KEYPAD_pound: print("KEYPAD 1 pound\n"); break;
                case 0x7: print("KEYPAD 1 UNKNOWN - 0x7\n"); break;
                case 0xB: print("KEYPAD 1 UNKNOWN - 0xB\n"); break;
                case 0xF: print("KEYPAD 1 UNKNOWN - 0xF\n"); break;
            }
        }

        if(keypad_1_changed & CONTROLLER_FIRE_BIT)
            print((keypad_1_state & CONTROLLER_FIRE_BIT) ? "FR1 press\n" : "FR1 release\n");

        keypad_1_changed = 0;

        probe_controller_2();

        if(joystick_2_changed & CONTROLLER_NORTH_BIT)
            print((joystick_2_state & CONTROLLER_NORTH_BIT) ? "N2 press\n" : "N2 release\n");
        if(joystick_2_changed & CONTROLLER_SOUTH_BIT)
            print((joystick_2_state & CONTROLLER_SOUTH_BIT) ? "S2 press\n" : "S2 release\n");
        if(joystick_2_changed & CONTROLLER_WEST_BIT)
            print((joystick_2_state & CONTROLLER_WEST_BIT) ? "W2 press\n" : "W2 release\n");
        if(joystick_2_changed & CONTROLLER_EAST_BIT)
            print((joystick_2_state & CONTROLLER_EAST_BIT) ? "E2 press\n" : "E2 release\n");
        if(joystick_2_changed & CONTROLLER_FIRE_BIT)
            print((joystick_2_state & CONTROLLER_FIRE_BIT) ? "FL2 press\n" : "FL2 release\n");

        joystick_2_changed = 0;

        if((keypad_2_changed & CONTROLLER_KEYPAD_MASK) != 0) {
            switch(keypad_2_state & CONTROLLER_KEYPAD_MASK) {
                case CONTROLLER_KEYPAD_0: print("KEYPAD 2 0\n"); break;
                case CONTROLLER_KEYPAD_1: print("KEYPAD 2 1\n"); break;
                case CONTROLLER_KEYPAD_2: print("KEYPAD 2 2\n"); break;
                case CONTROLLER_KEYPAD_3: print("KEYPAD 2 3\n"); break;
                case CONTROLLER_KEYPAD_4: print("KEYPAD 2 4\n"); break;
                case CONTROLLER_KEYPAD_5: print("KEYPAD 2 5\n"); break;
                case CONTROLLER_KEYPAD_6: print("KEYPAD 2 6\n"); break;
                case CONTROLLER_KEYPAD_7: print("KEYPAD 2 7\n"); break;
                case CONTROLLER_KEYPAD_8: print("KEYPAD 2 8\n"); break;
                case CONTROLLER_KEYPAD_9: print("KEYPAD 2 9\n"); break;
                case CONTROLLER_KEYPAD_asterisk: print("KEYPAD 2 asterisk\n"); break;
                case CONTROLLER_KEYPAD_pound: print("KEYPAD 2 pound\n"); break;
                case 0x7: print("KEYPAD 2 UNKNOWN - 0x7\n"); break;
                case 0xB: print("KEYPAD 2 UNKNOWN - 0xB\n"); break;
                case 0xF: print("KEYPAD 2 UNKNOWN - 0xF\n"); break;
            }
        }

        if(keypad_2_changed & CONTROLLER_FIRE_BIT)
            print((keypad_2_state & CONTROLLER_FIRE_BIT) ? "FR2 press\n" : "FR2 release\n");

        keypad_2_changed = 0;

        delay_ms(100);
    }

    panic();
}

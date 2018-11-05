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

unsigned char joystick_1_state;
unsigned char joystick_1_changed;
unsigned char keypad_1_state;
unsigned char keypad_1_changed;
unsigned char joystick_2_state;
unsigned char joystick_2_changed;
unsigned char keypad_2_state;
unsigned char keypad_2_changed;

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

gpio select_joystick_1 = { GPIOA, 3};
gpio select_keypad_1 = { GPIOA, 6};
gpio joystick_1_north = { GPIOC, 15};
gpio joystick_1_south = { GPIOA, 1};
gpio joystick_1_east = { GPIOA, 4};
gpio joystick_1_west = { GPIOA, 2};
gpio joystick_1_fire = { GPIOA, 0};

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

void reset_controllers(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    joystick_1_state = 0;
    joystick_1_changed = 0;
    keypad_1_state = 0;
    keypad_1_changed = 0;
    joystick_2_state = 0;
    joystick_2_changed = 0;
    keypad_2_state = 0;
    keypad_2_changed = 0;

#if 0
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_1.mask;
    set_GPIO_value(select_joystick_1.port, select_joystick_1.mask, 1);
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_1.mask;
    set_GPIO_value(select_keypad.port, select_keypad.mask, 1);
    HAL_GPIO_Init(select_keypad.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = joystick_1_north.mask;
    HAL_GPIO_Init(joystick_1_north.port, &GPIO_InitStruct); 
#endif
}

void probe_joysticks(void)
{
#if 0
    set_GPIO_value(select_joystick.port, select_joystick.mask, 0);
    delay_ms(1);

    unsigned char new_state =
         (HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.pin) ? 0 : CONTROLLER_NORTH_BIT);

    unsigned char changed = new_state ^ joystick_1_state;
    joystick_1_changed |= changed;
    joystick_1_state = new_state;

    // if(joystick_1_state & CONTROLLER_NORTH_BIT)
    if (!HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.pin))
        LED_set_info(1);
    else
        LED_set_info(0);

    // read lines for joystick 1
    // set changed flags
    // store state

    set_GPIO_value(select_joystick.port, select_joystick.mask, 1);
    delay_ms(1);
#endif
}

void probe_keypads(void)
{
    // ground SELECT_KEYPAD
    // read lines for keypad 1
    // set changed flags
    // store state
    // read lines for keypad 2
    // set changed flags
    // store state
    // tristate SELECT_KEYPAD
}

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

    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_1.mask;
    HAL_GPIO_WritePin(select_joystick_1.port, select_joystick_1.mask, GPIO_PIN_SET);
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_SET);
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

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

    while(1) {
        int on;

        HAL_GPIO_WritePin(select_joystick_1.port, select_joystick_1.mask, GPIO_PIN_RESET);

        on = HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask);
        if(!on)
            print("N1\n");
        
        on = HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask);
        if(!on)
            print("S1\n");
        
        on = HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask);
        if(!on)
            print("W1\n");
        
        on = HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask);
        if(!on)
            print("E1\n");
        
        on = HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask);
        if(!on)
            print("FL1\n");

        HAL_GPIO_WritePin(select_joystick_1.port, select_joystick_1.mask, GPIO_PIN_SET);

        HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_RESET);

        unsigned int keypad_value = 
            ((HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
            (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
            (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
            (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3)) ^ 0xF;

        switch(keypad_value & CONTROLLER_KEYPAD_MASK) {
            case CONTROLLER_KEYPAD_0: print("KEYPAD 0\n"); break;
            case CONTROLLER_KEYPAD_1: print("KEYPAD 1\n"); break;
            case CONTROLLER_KEYPAD_2: print("KEYPAD 2\n"); break;
            case CONTROLLER_KEYPAD_3: print("KEYPAD 3\n"); break;
            case CONTROLLER_KEYPAD_4: print("KEYPAD 4\n"); break;
            case CONTROLLER_KEYPAD_5: print("KEYPAD 5\n"); break;
            case CONTROLLER_KEYPAD_6: print("KEYPAD 6\n"); break;
            case CONTROLLER_KEYPAD_7: print("KEYPAD 7\n"); break;
            case CONTROLLER_KEYPAD_8: print("KEYPAD 8\n"); break;
            case CONTROLLER_KEYPAD_9: print("KEYPAD 9\n"); break;
            case CONTROLLER_KEYPAD_asterisk: print("KEYPAD asterisk\n"); break;
            case CONTROLLER_KEYPAD_pound: print("KEYPAD pound\n"); break;
            case 0x7: print("KEYPAD UNKNOWN - 0x7\n"); break;
            case 0xB: print("KEYPAD UNKNOWN - 0xB\n"); break;
            case 0xF: print("KEYPAD UNKNOWN - 0xF\n"); break;
        };

        on = HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask);
        if(!on)
            print("FR1\n");

        HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_SET);

        delay_ms(100);
    }

#if 0
    reset_controllers();

    while(1) {
        probe_joysticks();
        probe_keypads();
    }
#endif

    panic();
}

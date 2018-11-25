#include <stm32l0xx_hal.h>
#include "leds.h"
#include "delay.h"
#include "uart.h"
#include "defs.h"

#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_i2c.h"

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

    // Joystick directions and fire are hooked to select_joystick
    // and select_keyboard within the controller.

    // Set all joystick directions and fire button to inputs

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

    // Set select signals to high-impedance

    GPIO_InitStruct.Pin = select_joystick_1.mask;
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 
}

void init_controller_2(void)
{
    joystick_2_state = 0;
    joystick_2_changed = 0;
    keypad_2_state = 0;
    keypad_2_changed = 0;

    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    // Joystick directions and fire are hooked to select_joystick
    // and select_keyboard within the controller.

    // Set all joystick directions and fire button to inputs

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

    // Set select signals to high-impedance

    GPIO_InitStruct.Pin = select_joystick_2.mask;
    HAL_GPIO_Init(select_joystick_2.port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = select_keypad_2.mask;
    HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 
}

void probe_controller_1(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    // Set select_joystick to RESET, which grounds joystick and fire-left switches
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_1.mask;
    HAL_GPIO_WritePin(select_joystick_1.port, select_joystick_1.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    delay_ms(1);

    // read joystick and fire-left
    unsigned int joystick_value = (
        (HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask) << 6)
        ) ^ 0x4F;

    // set select_joystick to high-impedance
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_joystick_1.mask;
    HAL_GPIO_Init(select_joystick_1.port, &GPIO_InitStruct); 

    delay_ms(1);

    // Set select_keypad to RESET, which grounds keypad and fire-right switches
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_WritePin(select_keypad_1.port, select_keypad_1.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 

    delay_ms(1);

    // read keypad and fire-right
    unsigned int keypad_value = 
        ((HAL_GPIO_ReadPin(joystick_1_north.port, joystick_1_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_1_east.port, joystick_1_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_1_south.port, joystick_1_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_1_west.port, joystick_1_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_1_fire.port, joystick_1_fire.mask) << 6)
        ) ^ 0x4F;

    // set select_keypad to high-impedance
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_keypad_1.mask;
    HAL_GPIO_Init(select_keypad_1.port, &GPIO_InitStruct); 

    delay_ms(1);

    // note which signals have changed and update those signals

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

    // Set select_joystick to RESET, which grounds joystick and fire-left switches
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = select_joystick_2.mask;
    HAL_GPIO_WritePin(select_joystick_2.port, select_joystick_2.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_joystick_2.port, &GPIO_InitStruct); 

    delay_ms(1);

    // read joystick and fire-left
    unsigned int joystick_value = (
        (HAL_GPIO_ReadPin(joystick_2_north.port, joystick_2_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_2_east.port, joystick_2_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_2_south.port, joystick_2_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_2_west.port, joystick_2_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_2_fire.port, joystick_2_fire.mask) << 6)
        ) ^ 0x4F;

    // set select_joystick to high-impedance
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_joystick_2.mask;
    HAL_GPIO_Init(select_joystick_2.port, &GPIO_InitStruct); 

    delay_ms(1);

    // Set select_keypad to RESET, which grounds keypad and fire-right switches
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_keypad_2.mask;
    HAL_GPIO_WritePin(select_keypad_2.port, select_keypad_2.mask, GPIO_PIN_RESET);
    HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 

    delay_ms(1);

    // read keypad and fire-right
    unsigned int keypad_value = 
        ((HAL_GPIO_ReadPin(joystick_2_north.port, joystick_2_north.mask) << 0) | 
        (HAL_GPIO_ReadPin(joystick_2_east.port, joystick_2_east.mask) << 1) | 
        (HAL_GPIO_ReadPin(joystick_2_south.port, joystick_2_south.mask) << 2) | 
        (HAL_GPIO_ReadPin(joystick_2_west.port, joystick_2_west.mask) << 3) |
        (HAL_GPIO_ReadPin(joystick_2_fire.port, joystick_2_fire.mask) << 6)
        ) ^ 0x4F;

    // set select_keypad to high-impedance
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = select_keypad_2.mask;
    HAL_GPIO_Init(select_keypad_2.port, &GPIO_InitStruct); 

    delay_ms(1);

    // note which signals have changed and update those signals

    unsigned char changed = joystick_value ^ joystick_2_state;
    joystick_2_changed |= changed;
    joystick_2_state = joystick_value;

    changed = keypad_value ^ keypad_2_state;
    keypad_2_changed |= changed;
    keypad_2_state = keypad_value;
}

I2C_HandleTypeDef i2c_handle;
#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
#define I2C_ADDRESS	0x5A
#define I2C_ADDRESS_CODE	(I2C_ADDRESS << 1) // ARGH all of STM32's LL calls take I2C_ADDRESS << 1

#define I2C_MAX_RECEIVE_LENGTH 8
volatile static uint32_t i2c_receive_length = 0;
volatile static bool i2c_receive_overflowed = false;
const char * volatile i2c_error = NULL;
const char * volatile i2c_info = NULL;
volatile static uint8_t i2c_receive_buffer[I2C_MAX_RECEIVE_LENGTH];

/*
    - on last bit, data is stored in RXDR if RXNE=0.  If RXNE=1, clock is stretched until RXNE=0
    - transmit byte in TXDR if TXE=0
    - TXIS is generated when I2C_TXDR becomes empty, interrupt is generated if TXIE is set in I2C_CR1
        - TXIS is cleared when I2C_TXDR is written with the next byte to transmit
    - When the I2C is selected by one of its enabled addresses, ADDR interrupt status flag is set, and interrupt generated if ADDRIE is set then clear ADDRCF
*/

volatile bool an_interrupt = false;
volatile bool i2c_underrun = false;

extern "C" {

void I2C1_IRQHandler(void)
{
    an_interrupt = true;
    if(LL_I2C_IsActiveFlag_ADDR(I2C1)) {

	/* Clear ADDR flag value in ISR register */
	LL_I2C_ClearFlag_ADDR(I2C1);

	if(LL_I2C_GetAddressMatchCode(I2C1) == I2C_ADDRESS_CODE) {

	    /* Verify the transfer direction, a read direction, Slave enters transmitter mode */
	    if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ) {
		/* Enable Transmit Interrupt */
		// LL_I2C_EnableIT_TX(I2C1);
		i2c_info = "i2c read from this slave";
	    }

	} else {
	      
	    i2c_info = "wrong address?";
	}

    } else if(LL_I2C_IsActiveFlag_OVR(I2C1)) {

	/* overrun or underrun */
	/* i2cdetect will cause this because it tries to read a byte */
	LL_I2C_ClearFlag_OVR(I2C1);
	i2c_underrun = true;

    } else if(LL_I2C_IsActiveFlag_NACK(I2C1)) {

	/* Check NACK flag value in ISR register */
	/* End of Transfer */
	LL_I2C_ClearFlag_NACK(I2C1);
	i2c_info = "NACK";

    } else if(LL_I2C_IsActiveFlag_RXNE(I2C1)) {

	if(i2c_receive_length >= I2C_MAX_RECEIVE_LENGTH)
	    i2c_receive_overflowed = true;
	else
	    i2c_receive_buffer[i2c_receive_length++] = LL_I2C_ReceiveData8(I2C1);

    } else if(LL_I2C_IsActiveFlag_STOP(I2C1)) {

        /* Clear STOP flag value in ISR register */
        LL_I2C_ClearFlag_STOP(I2C1);
  
        /* Call function Slave Complete Callback */
        // Slave_Complete_Callback();

    } else if(!LL_I2C_IsActiveFlag_TXE(I2C1)) {

	/* Check TXE flag value in ISR register */
        /* Do nothing */
        /* This Flag will be set by hardware when the TXDR register is empty */
        /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */

    } else {

	NVIC_DisableIRQ(I2C1_IRQn);
	i2c_error = "i2c error";
    }
}

}

void Configure_I2C_Slave(void)
{
    // LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  
    /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);
    // LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP); // RasPi has pullups
  
    /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
    // LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP); // RasPi has pullups
  
    /* (2) Enable the I2C1 peripheral clock and I2C1 clock source ***************/
  
    /* Enable the peripheral clock for I2C1 */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  
    /* Set I2C1 clock source as SYSCLK */
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
  
    /* (3) Configure NVIC for I2C1 **********************************************/
  
    /* Configure Event and Error IT:
     *  - Set priority for I2C1_IRQn
     *  - Enable I2C1_IRQn
     */
    NVIC_SetPriority(I2C1_IRQn, 0);
    NVIC_EnableIRQ(I2C1_IRQn);
  
    /* (4) Configure I2C1 functional parameters *********************************/
  
    /* Disable I2C1 prior modifying configuration registers */
    LL_I2C_Disable(I2C1);
  
    /* Configure the SDA setup, hold time and the SCL high, low period */
    /* Timing register value is computed with the STM32CubeMX Tool,
      * Fast Mode @400kHz with I2CCLK = 32 MHz,
      * rise time = 100ns, fall time = 10ns
      * Timing Value = (uint32_t)0x00601B28
      */
    // uint32_t timing = __LL_I2C_CONVERT_TIMINGS(0x0, 0x6, 0x0, 0x1B, 0x28);
    // LL_I2C_SetTiming(I2C1, timing);
    LL_I2C_SetTiming(I2C1, I2C_TIMING);
  
    LL_I2C_SetOwnAddress1(I2C1, I2C_ADDRESS_CODE, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableOwnAddress1(I2C1);
    // I2C1->OAR1 |= (uint32_t)(I2C_ADDRESS << 1); /* (5) */
    // I2C1->OAR1 |= I2C_OAR1_OA1EN; /* (6) */
  
    LL_I2C_DisableClockStretching(I2C1);
  
    //LL_I2C_SetDigitalFilter(I2C1, 0x00); // default
  
    //LL_I2C_EnableAnalogFilter(I2C1); // default

    LL_I2C_TransmitData8(I2C1, 0x0); // Put 0 on the bus for i2cdetect
  
    LL_I2C_Enable(I2C1);

  
    /* (6) Enable I2C1 address match/error interrupts:
     *  - Enable Address Match Interrupt
     *  - Enable Not acknowledge received interrupt
     *  - Enable Error interrupts
     *  - Enable Stop interrupt
     */
    LL_I2C_EnableIT_RX(I2C1);
    LL_I2C_EnableIT_ADDR(I2C1);
    LL_I2C_EnableIT_NACK(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_EnableIT_STOP(I2C1);
}

void print_hex(uint32_t n)
{
    for(int i = 28; i >= 0 ; i -= 4) {
	uint32_t nybble = (n >> i) & 0xFu;
	if(nybble < 10)
	    SERIAL_send_one_char('0' + nybble);
	else
	    SERIAL_send_one_char('A' + nybble - 10);
    }
}

void print_decimal(int n)
{
    bool keep_printing = false;
    if(n / 100 > 0) {
        keep_printing = true;
        SERIAL_send_one_char('0' + n / 100);
    }
    if(keep_printing || ((n % 100) / 10 > 0)) {
        keep_printing = true;
        SERIAL_send_one_char('0' + (n % 100) / 10);
    }
    SERIAL_send_one_char('0' + n % 10);
}

int main()
{
    system_init();

    SERIAL_init();

    Configure_I2C_Slave();

    LED_init();

    for(int i = 0; i < 3; i++) {
        LED_set_info(1);
        delay_ms(100);
        LED_set_info(0);
        delay_ms(100);
    }

    print("firmware ");
    print(XSTR(FIRMWARE_VERSION));
    print(" OK\n");

    print_hex(0xF5A809EB);
    print(" = print_hex(0xF5A809EB)\n");

    init_controller_1();
    init_controller_2();

    while(1) {

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

	if(false && (i2c_info != NULL)) {
	    print(i2c_info);
	    print(", ISR :");
            print_hex(I2C1->ISR);
	    print("\n");
	    i2c_info = NULL;
	}
	if(i2c_error != NULL) {
	    print(i2c_error);
	    print(", ISR :");
            print_hex(I2C1->ISR);
	    print("\n");
	    panic();
	}
	disable_interrupts();

	    {
		// Critical Section for I2C
		if(an_interrupt) {
		    if(false) print("I2C interrupt!\n");
		    an_interrupt = false;
		}
		if(i2c_underrun) {
		    if(false) print("I2C underrun - i2cdetect?\n");
		    i2c_underrun = false;
		}
		if(LL_I2C_IsActiveFlag_ADDR(I2C1)) {
		    if(false) print("i2c addr!\n");
		    LL_I2C_ClearFlag_ADDR(I2C1);
		}

		if(i2c_receive_overflowed) {
		    print("warning: I2C receive buffer overflowed!\n");
		    i2c_receive_overflowed = false;
		}

		for(uint32_t i = 0; i < i2c_receive_length; i++) {
		    int n = i2c_receive_buffer[i];
		    print("I2C ");
		    print_decimal(n);
		    print("\n");
		}

		i2c_receive_length = 0;
	    }

	enable_interrupts();
    }

    panic();
}

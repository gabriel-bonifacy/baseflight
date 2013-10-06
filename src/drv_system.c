#include "board.h"

#define AIRCR_VECTKEY_MASK ((uint32_t)0x05FA0000)	 // Wykorzytywane przy resecie systemu
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW (0x2 << 24)     // Turn off JTAG port 'cause we're using the GPIO for leds

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
// from system_stm32f10x.c
void SetSysClock(void);

//Inicjalizuj licznik cylki
static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
//Przerwanie wykorzystywane do zliczania cykli wykonanych przez procesor
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void systemInit(void)
{
    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = { //Konfiguracja portów dla diod i buzzzera
#ifdef LED0
        {
            .gpio = LED0_GPIO,
            .cfg = { LED0_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef LED1

        {
            .gpio = LED1_GPIO,
            .cfg = { LED1_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef BUZZER
        {
            .gpio = BEEP_GPIO,
            .cfg = { BEEP_PIN, Mode_Out_OD, Speed_2MHz }
        },
#endif
    };
    gpio_config_t gpio;
    uint32_t i;
    uint8_t gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]); //Liczba portów GPIO

    // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
    // Configure the Flash Latency cycles and enable prefetch buffer
    SetSysClock();

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    gpio.pin = Pin_All;
    gpio.mode = Mode_AIN;
    gpioInit(GPIOA, &gpio);
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);

	//Wyłącz JTAG
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;

    // Configure gpio
    //Wyłącz diody i buzzer
    LED0_OFF;
    LED1_OFF;
    BEEP_OFF;

    for (i = 0; i < gpio_count; i++) {
        if (hse_value == 12000000 && gpio_setup[i].cfg.mode == Mode_Out_OD)
            gpio_setup[i].cfg.mode = Mode_Out_PP;
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Configure the rest of the stuff
#ifdef USE_I2C
    i2cInit(I2C2); //Inicjalizuj I2C
#endif
#ifdef USE_SPI
    spiInit(); //Inicjalizuj SPI
#endif

    // sleep for 100ms
    delay(100);
}

//Opóźnienie w us
//us -- opóźnienie w mikrosekundach
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

//Opóźnienie w ms
//ms -- opóźnienie w milisekundach
void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

//Informuje o błędzie systemu (np. braku akcelerometru).
//Pętla nieskończona, wymusza restart systemu!
//Mode -- określa czas pomiędzy zmianą stanu diody a wł./wy. buzzera.
void failureMode(uint8_t mode)
{
    LED1_ON;
    LED0_OFF;
    while (1) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(475 * mode - 2);
        BEEP_ON
        delay(25);
        BEEP_OFF;
    }
}


//Reset systemu
//toBootloader -- jeżeli true, po resecie przejdź do bootloadera
void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

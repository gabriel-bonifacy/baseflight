#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"

#ifndef __CC_ARM
// only need this garbage on gcc
#define USE_LAME_PRINTF
#include "printf.h"
#endif

#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

typedef enum {
    SENSOR_ACC = 1 << 0,
    SENSOR_BARO = 1 << 1,
    SENSOR_MAG = 1 << 2,
    SENSOR_SONAR = 1 << 3,
    SENSOR_GPS = 1 << 4,
} AvailableSensors;

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
} AccelSensors;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SPEKTRUM = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_GYRO_SMOOTHING = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
    FEATURE_POWERMETER = 1 << 12,
    FEATURE_VARIO = 1 << 13,
    FEATURE_3D = 1 << 14,
} AvailableFeatures;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MTK,
} GPSHardware;

typedef void (* sensorInitFuncPtr)(void);                   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef void (* baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)
typedef void (* uartReceiveCallbackPtr)(uint16_t data);     // used by uart2 driver to return frames to app
typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (* pidControllerFuncPtr)(void);                // pid controller function prototype

typedef struct sensor_t
{
    sensorInitFuncPtr init;                                 // initialize function (wskaźnik na funkcję inicjalizującą)
    sensorReadFuncPtr read;                                 // read 3 axis data function (wskaźnik na funkcję odczytującą)
    sensorReadFuncPtr align;                                // sensor align (wskaźnik na funkcję kalibrującą)
    sensorReadFuncPtr temperature;                          // read temperature if available (wskaźnik na funkcję do odczytania temperatury)
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel) (skala, używane tylko dla żyro)
} sensor_t;

typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    sensorInitFuncPtr start_ut;								//Wskaźnik na funkcję przygotowującą do odczyty temperatury.
    sensorInitFuncPtr get_ut;								//Wskaźnik na funkcję przygotowującą do odczyty temperatury.
    sensorInitFuncPtr start_up;								//Wskaźnik na funkcję przygotowującą do odczyty ciśnienia.
    sensorInitFuncPtr get_up;								//Wskaźnik na funkcję odczytująca ciśnienie.
    baroCalculateFuncPtr calculate;							//Wskaźnik na funkcję obliczającą ciśnienie
} baro_t;

// Hardware definitions and GPIO

#ifdef OLIMEXINO
// OLIMEXINO

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
// LED2 is using one of the pwm pins (PWM2), so we must not use PWM2.  @See pwmInit()
#define LED0_GPIO   GPIOA
#define LED0_PIN    GPIO_Pin_1 // D3, PA1/USART2_RTS/ADC1/TIM2_CH3 - "LED2" on silkscreen, Yellow
#define LED0
#endif

#ifdef OLIMEXINO_UNCUT_LED1_E_JUMPER
#define LED1_GPIO   GPIOA
#define LED1_PIN    GPIO_Pin_5 // D13, PA5/SPI1_SCK/ADC5 - "LED1" on silkscreen, Green
#define LED1
#endif

#define USE_I2C
#define USE_SPI
#define GYRO
#define ACC
#define MAG

#define SENSORS_SET (SENSOR_ACC | SENSOR_MAG)

// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);
#else
#define LED0_TOGGLE
#define LED0_OFF
#define LED0_ON
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);
#else
#define LED1_TOGGLE
#define LED1_OFF
#define LED1_ON
#endif

#ifdef BEEP_GPIO
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON                  digitalLo(BEEP_GPIO, BEEP_PIN);
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif

#undef SOFT_I2C                 // enable to test software i2c

#ifdef OLIMEXINO
// OLIMEXINO
#include "drv_adc.h"
#include "drv_i2c.h"
#include "drv_spi.h"
#include "drv_mpu6050.h"
#include "drv_hmc5883l.h"
#include "drv_pwm.h"
#include "drv_timer.h"
#include "drv_uart.h"
#include "drv_softserial.h"
#endif
#endif

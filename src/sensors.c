#include "board.h"
#include "mw.h"

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
uint16_t calibratingG = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.
int16_t heading, magHold;

extern uint16_t InflightcalibratingA;
extern int16_t AccInflightCalibrationArmed;
extern uint16_t AccInflightCalibrationMeasurementDone;
extern uint16_t AccInflightCalibrationSavetoEEProm;
extern uint16_t AccInflightCalibrationActive;
extern uint16_t batteryWarningVoltage;
extern uint8_t batteryCellCount;
extern float magneticDeclination;

sensor_t acc;                       // acc access functions
sensor_t gyro;                      // gyro access functions
baro_t baro;                        // barometer access functions
uint8_t accHardware = ACC_DEFAULT;  // which accel chip is used/detected


// AfroFlight32 i2c sensors
void sensorsAutodetect(void)
{
    int16_t deg, min;
    
    //----------- Wykrywanie akcelerometru -----------
    bool haveMpu6k = false; //Nie wykryoto MPU6050 (jesteśmy przed autodetekcją). 

    //Autodetekcja MPU6050.
    //Jeżeli MPU6050 nie zostanie wykryte, ustaw failure mode 3.
    if (mpu6050Detect(&acc, &gyro, mcfg.gyro_lpf, &mcfg.mpu6050_scale)) {
		//Ustawienie zmiennej haveMpu6k na true powoduje inicjalizację
		//struktury acc.* domyślnymi wartościami dla MPU6050.
        haveMpu6k = true;
        accHardware = ACC_MPU6050; //Zmienna informuje o typie akcelerometru.
    } else {
		sensorsClear(SENSOR_ACC); //Oznacz brak akcelerometru w systemie
								 //(z racji wywołania failureMode jest to wywołanie czysto formalne).
        failureMode(3); //Wywołanie funkcji nformującej o błędzie systemu (patrz drv_system.*).
    }

	//----------- Wykrywanie barometru -----------
#ifdef BARO
	if (!bmp085Detect(&baro)) {
		sensorsClear(SENSOR_BARO); //Oznacz brak barometru w systemie
	}
#endif

    //Inicjalizuj akcelerometr, jeżeli został wykryty w systemie.
    if (sensors(SENSOR_ACC))
        acc.init();
    //To wywołanie jest bezpieczne, ponieważ jeżeli program doszedł
    //do tej funkcji, oznacza to, że układ MPU6050 został wykryty,
    //zaś ten układ posiada poza akcelerometrem także wbudowany żyroskop.
    gyro.init();

	//----------- Wykrywanie magnetometru -----------
#ifdef MAG
    if (!hmc5883lDetect(mcfg.align[ALIGN_MAG]))
        sensorsClear(SENSOR_MAG); //Oznacz brak magnetometru w systemie
#endif

	//Obliczenia na podstawie wartości zapisanych w pliku config.c.
    //calculate magnetic declination
    deg = cfg.mag_declination / 100;
    min = cfg.mag_declination % 100;
    magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
}

//Oblicz napięcia na baterii na podstawie odczytu z ADC
uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * mcfg.vbatscale;
}

//Inicjalizuj informację o baterii (liczb celli oraz mimimalne napięci na jedną cellę)
void batteryInit(void)
{
    uint32_t i; //Liczba celli bateri (wykrywana na podstawie mcfg.vbatmaxcellvoltage, patrz config.c i mw.h)
    uint32_t voltage = 0; //Napięcie odczytane z ADC

	//Wykonaj 32 odczyty napięcia
    // average up some voltage readings
    for (i = 0; i < 32; i++) {
        voltage += adcGetChannel(ADC_BATTERY);
        delay(10);
    }
	
	//Średnie napięcie na podstawie 32 odczytów.
    voltage = batteryAdcToVoltage((uint16_t)(voltage / 32));

	//Autodetekcja liczby celli na podstawie mcfg.vbatmaxcellvoltage (patrz config.c i mw.h)
    // autodetect cell count, going from 2S..6S
    for (i = 2; i < 6; i++) {
        if (voltage < i * mcfg.vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * mcfg.vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

//Kalibracja sensorów (potrzebna pogłębiona analiza)
// ALIGN_GYRO = 0,
// ALIGN_ACCEL = 1,
// ALIGN_MAG = 2
static void alignSensors(uint8_t type, int16_t *data)
{
    int i;
    int16_t tmp[3];

    // make a copy :(
    tmp[0] = data[0];
    tmp[1] = data[1];
    tmp[2] = data[2];

    for (i = 0; i < 3; i++) {
        int8_t axis = mcfg.align[type][i];
        if (axis > 0)
            data[axis - 1] = tmp[i];
        else
            data[-axis - 1] = -tmp[i];
    }
}

//Funkcja wywoływana po każdym odczycie akclerometru
static void ACC_Common(void)
{
    static int32_t a[3];
    int axis;

	//Kalibracja akcelerometru przy starcie
    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == 400)
                a[axis] = 0;
            // Sum up 400 readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            mcfg.accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            mcfg.accZero[ROLL] = a[ROLL] / 400;
            mcfg.accZero[PITCH] = a[PITCH] / 400;
            mcfg.accZero[YAW] = a[YAW] / 400 - acc_1G;       // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeEEPROM(1, true);      // write accZero in EEPROM
        }
        calibratingA--;
    }

	//Kalibracja akceleromteru w locie
    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };
        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[ROLL] = mcfg.accZero[ROLL];
            accZero_saved[PITCH] = mcfg.accZero[PITCH];
            accZero_saved[YAW] = mcfg.accZero[YAW];
            angleTrim_saved[ROLL] = cfg.angleTrim[ROLL];
            angleTrim_saved[PITCH] = cfg.angleTrim[PITCH];
        }
        if (InflightcalibratingA > 0) {
            for (axis = 0; axis < 3; axis++) {
                // Reset a[axis] at start of calibration
                if (InflightcalibratingA == 50)
                    b[axis] = 0;
                // Sum up 50 readings
                b[axis] += accADC[axis];
                // Clear global variables for next reading
                accADC[axis] = 0;
                mcfg.accZero[axis] = 0;
            }
            // all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = 0;
                AccInflightCalibrationMeasurementDone = 1;
                toggleBeep = 2;      // buzzer for indicatiing the end of calibration
                // recover saved values to maintain current flight behavior until new values are transferred
                mcfg.accZero[ROLL] = accZero_saved[ROLL];
                mcfg.accZero[PITCH] = accZero_saved[PITCH];
                mcfg.accZero[YAW] = accZero_saved[YAW];
                cfg.angleTrim[ROLL] = angleTrim_saved[ROLL];
                cfg.angleTrim[PITCH] = angleTrim_saved[PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm == 1) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = 0;
            mcfg.accZero[ROLL] = b[ROLL] / 50;
            mcfg.accZero[PITCH] = b[PITCH] / 50;
            mcfg.accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeEEPROM(1, true);          // write accZero in EEPROM
        }
    }

    accADC[ROLL] -= mcfg.accZero[ROLL];
    accADC[PITCH] -= mcfg.accZero[PITCH];
    accADC[YAW] -= mcfg.accZero[YAW];
}

//Odczytaj akcelerometr
void ACC_getADC(void)
{
	//Wywołanie funkcji odczytującej dla odpowiedniego typu akcelerometru (patrz sensor_t w board.h)
    acc.read(accADC);
    //Możliwość ręcznej kalibracji przez użytkownika (patrz config.h)
    // if we have CUSTOM alignment configured, user is "assumed" to know what they're doing
    if (mcfg.align[ALIGN_ACCEL][0])
        alignSensors(ALIGN_ACCEL, accADC);
    else
        acc.align(accADC);

    ACC_Common();
}

#ifdef BARO
//Funkcja wywoływana po każdym odczycie temperatury (przed oczytem ciśnienia) z barometru
void Baro_Common(void)
{
	//Tablica historii odczytów z barometru (BARO_TAB_SIZE_MAX ustawiane w mw.h na 48)
    static int32_t baroHistTab[BARO_TAB_SIZE_MAX];
    static int baroHistIdx; //Aktualny indeks w historii
    int indexplus1; //Następny indeks w historii (baroHistIdx + 1)

    indexplus1 = (baroHistIdx + 1);
    //Jeżeli tablica histori jest zapełniona, zacznij od początku (cfg.baro_tab_size patrz config.c)
	//cfg.baro_tab_size <= 48!
    if (indexplus1 == cfg.baro_tab_size)
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure; //Zapisz odczyt od tablicy historii
    baroPressureSum += baroHistTab[baroHistIdx]; //Sumuj aktualny odczyt
    baroPressureSum -= baroHistTab[indexplus1]; //Odejmij najstarszy odczyt
    baroHistIdx = indexplus1;
}

//Funkcja aktualizująca odczyty z barometru
int Baro_update(void)
{
    static uint32_t baroDeadline = 0;
    static int state = 0;

    if ((int32_t)(currentTime - baroDeadline) < 0)
        return 0;

    baroDeadline = currentTime;
    
    //State = 1 -- barometr gotowy do odczytu ciśnienia
    //State = 0 -- barometr gotowy do odczytu temperatury
    if (state) {
        baro.get_up(); //Odczytaj ciśnienie
        baro.start_ut(); //Przygotuj do odczytania temperatury
        baroDeadline += baro.ut_delay; //Czas jaki musi upłynąć do przełączenia
									   //w stan odczytu temperatury.
        //baroPressure i baroTemperature jest zadeklarowane w mw.h
        baro.calculate(&baroPressure, &baroTemperature);
        state = 0; //Barometr gotowy od odczytu temperatury
        return 2;
    } else {
        baro.get_ut(); //Odczytaj temperaturę
        baro.start_up(); //Przgotuj do odczytania ciśnienia
        Baro_Common();
        state = 1; //Barometr w stanie do odczytu ciśnienia
        baroDeadline += baro.up_delay; //Czas jaki musi upłynąć do przełączenia
									   //w stan odczytu ciśnienia.
        return 1;
    }
}
#endif /* BARO */

typedef struct stdev_t
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

static void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

static float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

//Funkcja wykonywana po każdym odczycie z żyroskopu
static void GYRO_Common(void)
{
    int axis;
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    static stdev_t var[3];

	//Kalibracja żyroskopu po starcie
    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == 1000) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += gyroADC[axis];
            devPush(&var[axis], gyroADC[axis]);
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                float dev = devStandardDeviation(&var[axis]);
                // check deviation and startover if idiot was moving the model
                if (mcfg.moron_threshold && dev > mcfg.moron_threshold) {
                    calibratingG = 1000;
                    devClear(&var[0]);
                    devClear(&var[1]);
                    devClear(&var[2]);
                    g[0] = g[1] = g[2] = 0;
                    continue;
                }
                gyroZero[axis] = g[axis] / 1000;
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
    }
    //Odczyty dla wystkich 3 osi
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        //anti gyro glitch, limit the variation between two consecutive readings
        gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        previousGyroADC[axis] = gyroADC[axis];
    }
}

//Odczytanie watoście z żyroskopu
void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    gyro.read(gyroADC); 
    
    //Możliwość ręcznej kalibracji przez użytkownika (patrz config.h)
    // if we have CUSTOM alignment configured, user is "assumed" to know what they're doing
    if (mcfg.align[ALIGN_GYRO][0])
        alignSensors(ALIGN_GYRO, gyroADC);
    else
        gyro.align(gyroADC);

    GYRO_Common();
}

#ifdef MAG
static float magCal[3] = { 1.0f, 1.0f, 1.0f };     // gain for each axis, populated at sensor init, wykorzystywane przy kalibracji
static uint8_t magInit = 0; //Zmiena oznaczająca, że magnetometr został zainicjalizowany (1 -- zainicjalizowany, 0 -- niezainicjalizowany)

//Oczytaj wartości z magnetometru
static void Mag_getRawADC(void)
{
    // MAG driver will align itself, so no need to alignSensors()
    hmc5883lRead(magADC);
}

//Inicjalizacja magnetometru
void Mag_init(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    hmc5883lInit(magCal);
    LED1_OFF;
    magInit = 1; //Zmienna informująca o tym, że magnetometr został zainicjalizowany
}

//Odczytaj wartości z magnetometru
int Mag_getADC(void)
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint32_t axis;

    if ((int32_t)(currentTime - t) < 0)
        return 0;                 //odstęp pomiędzy kolejnymi odczytami 100 ms
    t = currentTime + 100000;

    // Read mag sensor
    Mag_getRawADC();

    magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
    magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
    magADC[YAW]   = magADC[YAW]   * magCal[YAW];

	//Kalibracja magnetometru (f -- flagi flags_t, patrz mw.c)
    if (f.CALIBRATE_MAG) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            mcfg.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0; //Kalibracja magnetometru zakończona
    }
	//Odejmij offset od odczytu dla każdej osi (mcfg patrz config.c)
    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[ROLL] -= mcfg.magZero[ROLL];
        magADC[PITCH] -= mcfg.magZero[PITCH];
        magADC[YAW] -= mcfg.magZero[YAW];
    }

    if (tCal != 0) {
		//Sprawdź, czy upłynęło mniej niż 30 s pomiędzy ostatnim odczytaniem wartości z magnetometru a jego kalibracją
        if ((t - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
			//Zapamiętaj najmniejszą i największą wartość odczytaną w ciągu 30 s dla każdej osi.
			//Wartości te są następnie wykorzystywane do obliczenia średniej i są ustawiane jako magZero (początkowa wartość magnetometru ?)
            LED0_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis])
                    magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis])
                    magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0; //Ustawienie powoduje, że mcfg.magZero zostanie nadpisana tylko raz.
            //Oblicz średnią wartość z początkowych odczytów dla każdej osi.
            for (axis = 0; axis < 3; axis++)
                mcfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
            writeEEPROM(1, true); //Zapisz nową konfigurację do EEPROM
        }
    }
    
    return 1;
}
#endif

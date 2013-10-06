#include "board.h"
#include "mw.h"

int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
int32_t accSum[3];
uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0;
int16_t acc_25deg = 0;
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
int32_t baroPressureSum = 0;
int32_t BaroAlt = 0;
int32_t sonarAlt;              // to think about the unit
int32_t EstAlt;                // in cm
int32_t BaroPID = 0;
int32_t AltHold;
int32_t errorAltitudeI = 0;
int32_t vario = 0;                      // variometer in cm/s
int16_t throttleAngleCorrection = 0;    // correction of throttle in lateral wind,
float magneticDeclination = 0.0f;       // calculated at startup from config
float accVelScale;

// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = { 0, 0, 0 };
int16_t gyroZero[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static void getEstimatedAttitude(void);

void imuInit(void)
{
    acc_25deg = acc_1G * 0.423f;
    accVelScale = 9.80665f / acc_1G / 10000.0f;

#ifdef MAG
    // if mag sensor is enabled, use it
    if (sensors(SENSOR_MAG))
        Mag_init();
#endif
}


void computeIMU(void)
{
    uint32_t axis;
    static int16_t gyroADCprevious[3] = { 0, 0, 0 };
    int16_t gyroADCp[3];
    int16_t gyroADCinter[3];
    static uint32_t timeInterleave = 0;
    static int16_t gyroYawSmooth = 0;

    if (sensors(SENSOR_ACC)) {
        ACC_getADC();
        getEstimatedAttitude();
    }

    Gyro_getADC();

    for (axis = 0; axis < 3; axis++)
        gyroADCp[axis] = gyroADC[axis];
    timeInterleave = micros();
    annexCode();

    if ((micros() - timeInterleave) > 650) {
        annex650_overrun_count++;
    } else {
        while ((micros() - timeInterleave) < 650);  // empirical, interleaving delay between 2 consecutive reads
    }

    Gyro_getADC();
    for (axis = 0; axis < 3; axis++) {
        gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
        // empirical, we take a weighted value of the current and the previous values
        gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
        gyroADCprevious[axis] = gyroADCinter[axis] / 2;
        if (!sensors(SENSOR_ACC))
            accADC[axis] = 0;
    }

    if (feature(FEATURE_GYRO_SMOOTHING)) {
        static uint8_t Smoothing[3] = { 0, 0, 0 };
        static int16_t gyroSmooth[3] = { 0, 0, 0 };
        if (Smoothing[0] == 0) {
            // initialize
            Smoothing[ROLL] = (mcfg.gyro_smoothing_factor >> 16) & 0xff;
            Smoothing[PITCH] = (mcfg.gyro_smoothing_factor >> 8) & 0xff;
            Smoothing[YAW] = (mcfg.gyro_smoothing_factor) & 0xff;
        }
        for (axis = 0; axis < 3; axis++) {
            gyroData[axis] = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1) / Smoothing[axis]);
            gyroSmooth[axis] = gyroData[axis];
        }
    }
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Modified: 19/04/2011  by ziss_dm
// Version: V1.1
//
// code size deduction and tmp vector intermediate step for vector rotation computation: October 2011 by Alex
// **************************************************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)mcfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)mcfg.gyro_cmpfm_factor + 1.0f))

// #define GYRO_SCALE ((1998 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     // 32767 / 16.4lsb/dps for MPU3000

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(-delta[PITCH]);
    sinx = sinf(-delta[PITCH]);
    cosy = cosf(delta[ROLL]);
    siny = sinf(delta[ROLL]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = coszcosy;
    mat[0][1] = sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] = (coszsinx * siny) - sinzcosx;
    mat[1][1] = (sinzsinx * siny) + (coszcosx);
    mat[1][2] = cosy * sinx;
    mat[2][0] = (coszcosx * siny) + (sinzsinx);
    mat[2][1] = (sinzcosx * siny) - (coszsinx);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

static int16_t _atan2f(float y, float x)
{
    // no need for aidsy inaccurate shortcuts on a proper platform
    return (int16_t)(atan2f(y, x) * (180.0f / M_PI * 10.0f));
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

// rotate acc into Earth frame and calculate acceleration in it
void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    float rpy[3];
    t_fp_vector accel_ned;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float) angle[ROLL] * RADX10;
    rpy[1] = -(float) angle[PITCH] * RADX10;
    rpy[2] = -(float) heading * RADX10 * 10;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, rpy);

    if (cfg.acc_unarmedcal == 1) {
        if (!f.ARMED) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc_1G;

    // apply Deadband to reduce integration drift and vibration influence
    accel_ned.V.Z = applyDeadband(accel_ned.V.Z, cfg.accz_deadband);
    accel_ned.V.X = applyDeadband(accel_ned.V.X, cfg.accxy_deadband);
    accel_ned.V.Y = applyDeadband(accel_ned.V.Y, cfg.accxy_deadband);

    // sum up Values for later integration to get velocity and distance
    accTimeSum += deltaT;
    accSumCount++;

    accSum[0] += accel_ned.V.X;
    accSum[1] += accel_ned.V.Y;
    accSum[2] += accel_ned.V.Z;
}

void accSum_reset(void)
{
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accSumCount = 0;
    accTimeSum = 0;
}

// Use original baseflight angle calculation
// #define BASEFLIGHT_CALC
static float invG;

static void getEstimatedAttitude(void)
{
    uint32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstM;
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t deltaT;
    float scale, deltaGyroAngle[3];
#ifndef BASEFLIGHT_CALC
    float sqGZ;
    float sqGX;
    float sqGY;
    float sqGX_sqGZ;
    float invmagXZ;
#endif
    deltaT = currentT - previousT;
    scale = deltaT * gyro.scale;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        if (cfg.acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / cfg.acc_lpf_factor)) + accADC[axis] * (1.0f / cfg.acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);
    if (sensors(SENSOR_MAG))
        rotateV(&EstM.V, deltaGyroAngle);

    if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
        f.SMALL_ANGLES_25 = 1;
    else
        f.SMALL_ANGLES_25 = 0;

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)mcfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    if (sensors(SENSOR_MAG)) {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * (float)mcfg.gyro_cmpfm_factor + magADC[axis]) * INV_GYR_CMPFM_FACTOR;
    }

    // Attitude of the estimated vector
#ifdef BASEFLIGHT_CALC
    // This hack removes gimbal lock (sorta) on pitch, so rolling around doesn't make pitch jump when roll reaches 90deg
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = -asinf(EstG.V.Y / -sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z)) * (180.0f / M_PI * 10.0f);
#else
    // MW2.2 version
    sqGZ = EstG.V.Z * EstG.V.Z;
    sqGX = EstG.V.X * EstG.V.X;
    sqGY = EstG.V.Y * EstG.V.Y;
    sqGX_sqGZ = sqGX + sqGZ;
    invmagXZ = 1.0f / sqrtf(sqGX_sqGZ);
    invG = 1.0f / sqrtf(sqGX_sqGZ + sqGY);
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = _atan2f(EstG.V.Y, invmagXZ * sqGX_sqGZ);
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
#ifdef BASEFLIGHT_CALC
        // baseflight calculation
        float rollRAD = (float)angle[ROLL] * RADX10;
        float pitchRAD = -(float)angle[PITCH] * RADX10;
        float magX = EstM.A[1];                         // Swap X/Y
        float magY = EstM.A[0];                         // Swap X/Y
        float magZ = EstM.A[2];
        float cr = cosf(rollRAD);
        float sr = sinf(rollRAD);
        float cp = cosf(pitchRAD);
        float sp = sinf(pitchRAD);
        float Xh = magX * cp + magY * sr * sp + magZ * cr * sp;
        float Yh = magY * cr - magZ * sr;
        float hd = (atan2f(-Yh, Xh) * 1800.0f / M_PI + magneticDeclination) / 10.0f;
        heading = hd;
#else
        // MW 2.2 calculation
        heading = _atan2f(EstM.V.Z * EstG.V.X - EstM.V.X * EstG.V.Z, EstM.V.Y * invG * sqGX_sqGZ - (EstM.V.X * EstG.V.X + EstM.V.Z * EstG.V.Z) * invG * EstG.V.Y);
        heading = heading + magneticDeclination;
        heading = heading / 10;
#endif
        if (heading > 180)
            heading = heading - 360;
        else if (heading < -180)
            heading = heading + 360;
    }
#endif

    acc_calc(deltaT); // rotate acc vector into earth frame

    if (cfg.throttle_angle_correction) {
        int cosZ = EstG.V.Z / acc_1G * 100.0f;
        throttleAngleCorrection = cfg.throttle_angle_correction * constrain(100 - cosZ, 0, 100) / 8;
        
    }
}

#ifdef BARO
#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)

int getEstimatedAltitude(void)
{
    static int32_t baroGroundPressure;
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t dTime;
    int32_t error;
    int32_t baroVel;
    int32_t vel_tmp;
    int32_t BaroAlt_tmp;
    static float vel = 0.0f;
    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

    dTime = currentT - previousT;
    if (dTime < UPDATE_INTERVAL)
        return 0;
    previousT = currentT;

    if (calibratingB > 0) {
        baroGroundPressure = baroPressureSum / (cfg.baro_tab_size - 1);
        calibratingB--;
        vel = 0;
        accAlt = 0;
    }

    // pressure relative to ground pressure with temperature compensation (fast!)
    // baroGroundPressure is not supposed to be 0 here
    // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
    BaroAlt_tmp = logf(baroGroundPressure * (cfg.baro_tab_size - 1) / (float)baroPressureSum) * (baroTemperature + 27315) * 29.271267f; // in cemtimeter 
    BaroAlt = BaroAlt * cfg.baro_noise_lpf + BaroAlt_tmp * (1.0f - cfg.baro_noise_lpf); // additional LPF to reduce baro noise
    
    // Integrator - velocity, cm/sec
    vel += (float)accSum[2] * accVelScale * (float)accTimeSum / (float)accSumCount;

    // Integrator - Altitude in cm
    accAlt += vel * ((float) accTimeSum * 0.0000005f);                                  // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * cfg.baro_cf_alt + (float) BaroAlt *(1.0f - cfg.baro_cf_alt);      // complementary filter for Altitude estimation (baro & acc)
    EstAlt = accAlt;

#if 0
    debug[0] = accSum[2] / accSumCount; // acceleration
    debug[1] = vel;                     // velocity
    debug[2] = accAlt;                  // height
#endif

    accSum_reset();

    //P
    error = constrain(AltHold - EstAlt, -300, 300);
    error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
    BaroPID = constrain((cfg.P8[PIDALT] * error / 128), -150, +150);

    //I
    errorAltitudeI += cfg.I8[PIDALT] * error / 64;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    BaroPID += errorAltitudeI / 512;     // I in range +/-60

    
    baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -300, 300);    // constrain baro velocity +/- 300cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * cfg.baro_cf_vel + baroVel * (1 - cfg.baro_cf_vel);

    // D
    vel_tmp = vel;
    vel_tmp = applyDeadband(vel_tmp, 5);
    vario = vel_tmp;
    BaroPID -= constrain(cfg.D8[PIDALT] * vel_tmp / 16, -150, 150);

    return 1;
}
#endif /* BARO */

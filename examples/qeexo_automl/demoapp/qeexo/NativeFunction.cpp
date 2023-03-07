/**
  ******************************************************************************
  * @file    QxAutoMLInf.cpp
  * @author  Qeexo Kernel Development team
  * @version V1.0.0
  * @date    30-Sep-2020
  * @brief   Auto ML module for Inference 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 Qeexo Co.
  * All rights reserved
  *
  *
  * ALL INFORMATION CONTAINED HEREIN IS AND REMAINS THE PROPERTY OF QEEXO, CO.
  * THE INTELLECTUAL AND TECHNICAL CONCEPTS CONTAINED HEREIN ARE PROPRIETARY TO
  * QEEXO, CO. AND MAY BE COVERED BY U.S. AND FOREIGN PATENTS, PATENTS IN PROCESS,
  * AND ARE PROTECTED BY TRADE SECRET OR COPYRIGHT LAW. DISSEMINATION OF
  * THIS INFORMATION OR REPRODUCTION OF THIS MATERIAL IS STRICTLY FORBIDDEN UNLESS
  * PRIOR WRITTEN PERMISSION IS OBTAINED OR IS MADE PURSUANT TO A LICENSE AGREEMENT
  * WITH QEEXO, CO. ALLOWING SUCH DISSEMINATION OR REPRODUCTION.
  *
  ******************************************************************************
 */

#include <Arduino.h>
#include "QxAutoMLUser.h"
#include "QxSensorHal_Nano33BLE.h"

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

extern __attribute__((weak)) int QxOS_InitializeBSP(void);
extern "C" int QxAudioHal_init();
extern "C" int QxAudioHal_enable(void *dev, float fs, float odr, BOOL fifo);

/* User sensor driver interface*/

/*
    This function returns accel & gyro fifo sample data count, which is the remainning
    sensor data number, each number's size is 6 bytes 3axis.
    Please refer to https://content.arduino.cc/assets/Nano_BLE_Sense_lsm9ds1.pdf
*/

#if defined (QXAUTOMLCONFIG_SENSOR_ENABLE_ACCEL) || \
    defined (QXAUTOMLCONFIG_SENSOR_ENABLE_GYRO) || \
    defined (QXAUTOMLCONFIG_SENSOR_ENABLE_MAG) 
extern"C" uint16_t lsm9ds1_read_fifocount()
{
    uint16_t fifocount;
    uint8_t reg = 0;

    Sensor_I2CReadReg(LSM9DS1_SLAVE_ADDR, LSM9DS1_REG_FIFO_SRC, &reg, 1);

    fifocount = (reg & 0x3f);
    bool overwritten = !!(reg & 0x40);
    if (overwritten){
        Serial.println("lsm9ds1 over written");
    }
    return fifocount;
}

/*
    This function read the accel and gyro sensor data to the gived input 'data' buffer, the read number
    dependeds on parameter 'remaining'.
    Please refer to https://content.arduino.cc/assets/Nano_BLE_Sense_lsm9ds1.pdf
 */
extern "C" int lsm9ds1_read_fifoData(uint16_t remaining, int16_t* accel_data, int16_t* gyro_data)
{
    int16_t read_samples = 1;

    while(remaining > 0) {
        //read one accel sample data
        Sensor_I2CReadReg(LSM9DS1_SLAVE_ADDR, LSM9DS1_REG_OUT_X_XL, (uint8_t *)accel_data, 6*read_samples);
        accel_data += 3*read_samples;

        //read one gyro sample data
        Sensor_I2CReadReg(LSM9DS1_SLAVE_ADDR, LSM9DS1_REG_OUT_X_G, (uint8_t *)gyro_data, 6*read_samples);
        gyro_data += 3*read_samples;

        /* The remaining indicates the pair of the accel&gyro sample data*/
        remaining -= read_samples;
    }

    return 0;
}
#endif

/*
    This funtion configs the format of sensor data output.
    Please change the parameters to what you set in the Qeexo AutoML data collection page.
*/
extern "C" void NativeInitSensor(void)
{
    static bool senosr_initialized = false;

    if (senosr_initialized) {
        return;
    }

    senosr_initialized = true;
    
    /* Initialize BSP and Sensor HAL layer for Qeexo AutoML */
    QxOS_InitializeBSP();

    /* init and enable accelerometer @& gyrometer */
#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_ACCEL
    lsm9ds1_acc_init(NULL);
    lsm9ds1_acc_enable(NULL,
                      QXAUTOMLCONFIG_SENSOR_ACCEL_FSR,    /* ACCEL FULL SCALE RANGE */
                      QXAUTOMLCONFIG_SENSOR_ACCEL_ODR,   /* ACCEL SAMPLING RATE */
                      TRUE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_GYRO
    lsm9ds1_gyro_init(NULL);
    lsm9ds1_gyro_enable(NULL,
                        QXAUTOMLCONFIG_SENSOR_GYRO_FSR, /* GYRO FULL SCALE RANGE */
                        QXAUTOMLCONFIG_SENSOR_GYRO_ODR,  /* GYRO SAMPLING RATE */
                        TRUE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_MAG
    lsm9ds1_mag_init(NULL);

     /* MAG's params are fixed values */
    lsm9ds1_mag_enable(NULL, 
                       QXAUTOMLCONFIG_SENSOR_MAG_FSR,
                       QXAUTOMLCONFIG_SENSOR_MAG_ODR,
                        FALSE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_PRESSURE
    lps22hb_press_init(NULL);

     /* Press's params are fixed values */
    lps22hb_press_enable(NULL,
                        QXAUTOMLCONFIG_SENSOR_PRESSURE_FSR, 
                        QXAUTOMLCONFIG_SENSOR_PRESSURE_ODR, 
                        FALSE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_TEMPERATURE
    hts221_temperature_init(NULL);

     /* Temp's params are fixed values */
    hts221_temperature_enable(NULL,
                              QXAUTOMLCONFIG_SENSOR_TEMPERATURE_FSR, 
                              QXAUTOMLCONFIG_SENSOR_TEMPERATURE_ODR, 
                              FALSE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_HUMIDITY
    hts221_humidity_init(NULL);

     /* Humidity's params are fixed values */
    hts221_humidity_enable(NULL, 
                           QXAUTOMLCONFIG_SENSOR_HUMIDITY_FSR, 
                           QXAUTOMLCONFIG_SENSOR_HUMIDITY_ODR, 
                           FALSE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_PROXIMITY
    adps9960_proximity_init(NULL);

     /* Proximity's params are fixed values */
    adps9960_proximity_enable(NULL,
                              QXAUTOMLCONFIG_SENSOR_PROXIMITY_FSR, 
                              QXAUTOMLCONFIG_SENSOR_PROXIMITY_ODR, 
                              FALSE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_AMBIENT
    adps9960_light_init(NULL);

     /* Light's params are fixed values */
    adps9960_light_enable(NULL, 
                          QXAUTOMLCONFIG_SENSOR_AMBIENT_FSR, 
                          QXAUTOMLCONFIG_SENSOR_AMBIENT_ODR, 
                          FALSE);
#endif

#ifdef QXAUTOMLCONFIG_SENSOR_ENABLE_MICROPHONE
    QxAudioHal_init();

    /* MIC's params are fixed values */
    QxAudioHal_enable(NULL, 
                                QXAUTOMLCONFIG_SENSOR_MICROPHONE_FSR, 
                                QXAUTOMLCONFIG_SENSOR_MICROPHONE_ODR, 
                                FALSE);
#endif

}

extern "C" void NativeOSDelay(int msec)
{
    delay(msec);
}

/**    
 * @brief Get tick counter in millisecond unit.
 * @return uint32_t : Millisecond tick counter.
 * @note This function is matched with osKernelSysTick() of CMSIS
 */
extern "C" int NativeOSGetTick()
{
    return millis();
}

extern "C"  void NativeOSPrint(char* str)
{
    Serial.println(str);

}

extern "C"  void QxOS_DebugPrint(char const*, ...)
{

}




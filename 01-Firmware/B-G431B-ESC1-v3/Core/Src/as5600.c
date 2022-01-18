/*
 * as5600.c
 *
 *  Created on: 26.03.2021
 *      Author: kai
 */

#include <stdio.h>
#include <stdlib.h>
#include "as5600.h"
#include "math_tool.h"

#define AS5600_SLAVE_ADDRESS 0x36

// config register
#define AS5600_ZMCO_REG_ADDR 0x00
#define AS5600_ZPOS_REG_ADDR 0x01
#define AS5600_MPOS_REG_ADDR 0x03
#define AS5600_MANG_REG_ADDR 0x05
#define AS5600_CONF_REG_ADDR 0x07

// angle register
#define AS5600_RAWANGLE_REG_ADDR 0x0C
#define AS5600_ANGLE_REG_ADDR    0x0E

// status register
#define AS5600_STATUS_REG_ADDR   0x0B
#define AS5600_AGC_REG_ADDR      0x1A
#define AS5600_MAG_REG_ADDR      0x1B

// burn register
#define AS5600_BURN_REG_ADDR     0xFF


#define AS5600_AGC_MIN_GAIN_OVERFLOW  (uint8_t)(1UL << 3) /*Error bit indicates b-field is too string */
#define AS5600_AGC_MAX_GAIN_OVERFLOW  (uint8_t)(1UL << 4) /*Error bit indicates b-field is too weak */
#define AS5600_MAGNET_DETECTED        (uint8_t)(1UL << 5) /*Status bit indicates b-field is detected */

/* AS5600 Configuration Settings */
#define AS5600_POWER_MODE_NOM 0
#define AS5600_POWER_MODE_LPM1 1
#define AS5600_POWER_MODE_LPM2 2
#define AS5600_POWER_MODE_LPM3 3
#define AS5600_HYSTERESIS_OFF 0
#define AS5600_HYSTERESIS_1LSB 1
#define AS5600_HYSTERESIS_2LSB 2
#define AS5600_HYSTERESIS_3LSB 3
#define AS5600_OUTPUT_STAGE_FULL  0 /* Ratiometric analog output ranging from GND-VCC*/
#define AS5600_OUTPUT_STAGE_REDUCED 1 /* Ratiometric analog output ranging from 10% to 90% of VCC */
#define AS5600_OUTPUT_STAGE_PWM 2 /* Digital PWM output */
#define AS5600_PWM_FREQUENCY_115HZ 0
#define AS5600_PWM_FREQUENCY_230HZ 1
#define AS5600_PWM_FREQUENCY_460HZ 2
#define AS5600_PWM_FREQUENCY_920HZ 3
#define AS5600_SLOW_FILTER_16X 0
#define AS5600_SLOW_FILTER_8X 1
#define AS5600_SLOW_FILTER_4X 2
#define AS5600_SLOW_FILTER_2X 3
#define AS5600_FAST_FILTER_SLOW_ONLY 0
#define AS5600_FAST_FILTER_6LSB 1
#define AS5600_FAST_FILTER_7LSB 2
#define AS5600_FAST_FILTER_9LSB 3
#define AS5600_FAST_FILTER_18LSB 4
#define AS5600_FAST_FILTER_21LSB 5
#define AS5600_FAST_FILTER_24LSB 6
#define AS5600_FAST_FILTER_10LSB 7
#define AS5600_WATCHDOG_OFF 0
#define AS5600_WATCHDOG_ON 1

//Config register
typedef union
{
    volatile uint16_t      raw;
    struct
    {
        volatile uint16_t      SF            :  2;
        volatile uint16_t      FTH           :  3;
        volatile uint16_t      WD            :  1;
        volatile uint16_t      notused       :  2;
        volatile uint16_t      PM            :  2;
        volatile uint16_t      HYST          :  2;
        volatile uint16_t      OUTS          :  2;
        volatile uint16_t      PWMF          :  2;

    } bit;
} conf_reg_t;

typedef union
{
    volatile uint16_t      raw;
    struct
    {
        volatile uint16_t      mang8_12      :   4;
        volatile uint16_t      notused       :   4;
        volatile uint16_t      mang0_7       :   8;
    } bit;
} mang_reg_t;

typedef union
{
    volatile uint16_t      raw;
    struct
    {
        volatile uint16_t      mpos8_12      :   4;
        volatile uint16_t      notused       :   4;
        volatile uint16_t      mpos0_7       :   8;
    } bit;
} mpos_reg_t;

typedef union
{
    volatile uint16_t      raw;
    struct
    {
        volatile uint16_t      zpos8_12      :   4;
        volatile uint16_t      notused       :   4;
        volatile uint16_t      zpos0_7       :   8;
    } bit;
} zpos_reg_t;

typedef union
{
    volatile uint8_t       raw;
    struct
    {
        volatile uint8_t       zmco          :   2;
        volatile uint8_t       notused       :   6;
    } bit;
} zmco_reg_t;

// output register
typedef union
{
    volatile uint16_t      raw;
    struct
    {
        volatile uint16_t      angle8_12     :   4;
        volatile uint16_t      notused       :   4;
        volatile uint16_t      angle0_7      :   8;
    } bit;
} angle_reg_t;

// status register
typedef union
{
    volatile uint8_t       raw;
    struct
    {
        volatile uint8_t       notused1              :   3;
        volatile uint8_t       Agc_min_gain_overflow :   1;
        volatile uint8_t       Agc_max_gain_overflow :   1;
        volatile uint8_t       MagnetDetected        :   1;
        volatile uint8_t       notused2              :   2;
    } bit;
} status_reg_t;

typedef union
{
    volatile uint8_t       raw;
    struct
    {
        volatile uint8_t       agc      :   8;
    } bit;
} agc_reg_t;

typedef union
{
    volatile uint16_t      raw;
    struct
    {
        volatile uint16_t      magnitude8_12 :   4;
        volatile uint16_t      notused       :   4;
        volatile uint16_t      magnitude0_7  :   8;
    } bit;
} mag_reg_t;

HAL_StatusTypeDef AS5600_SetConfig(AS5600_TypeDef *const as5600, conf_reg_t conf);
HAL_StatusTypeDef AS5600_GetMagnetStatus(AS5600_TypeDef *const a, status_reg_t *stat);

AS5600_TypeDef *AS5600_New(void) {
    AS5600_TypeDef *as5600 = (AS5600_TypeDef *)calloc(1, sizeof(AS5600_TypeDef));
    return as5600;
}

HAL_StatusTypeDef AS5600_Init(AS5600_TypeDef *as5600) {

	status_reg_t mag_status;
	conf_reg_t config;
	HAL_StatusTypeDef status = HAL_OK;

	// set i2c address
	as5600->i2cAddr = (AS5600_SLAVE_ADDRESS & 0x7f) <<1;

	// configure sensor settings
	// *************************
	config.raw = 0;

	// POWER MODE
	// Possible configuration:
    // AS5600_POWER_MODE_NOM
    // AS5600_POWER_MODE_LPM1
    // AS5600_POWER_MODE_LPM2
    // AS5600_POWER_MODE_LPM3
	config.bit.PM = AS5600_POWER_MODE_NOM;

	// HYSTERESIS
	// Possible configuration:
	// AS5600_HYSTERESIS_OFF
    // AS5600_HYSTERESIS_1LSB
    // AS5600_HYSTERESIS_2LSB
    // AS5600_HYSTERESIS_3LSB
	config.bit.HYST = AS5600_HYSTERESIS_OFF;

	// OUTPUT_STAGE
	// Possible configuration:
    // AS5600_OUTPUT_STAGE_FULL  /* Ratiometric analog output ranging from GND-VCC*/
    // AS5600_OUTPUT_STAGE_REDUCED /* Ratiometric analog output ranging from 10% to 90% of VCC */
    // AS5600_OUTPUT_STAGE_PWM  /* Digital PWM output */
	config.bit.OUTS = AS5600_OUTPUT_STAGE_FULL;

	// PWM_FREQUENCY
	// Possible configuration:
    // AS5600_PWM_FREQUENCY_115HZ
    // AS5600_PWM_FREQUENCY_230HZ
    // AS5600_PWM_FREQUENCY_460HZ
    // AS5600_PWM_FREQUENCY_920HZ
   	config.bit.PWMF = AS5600_PWM_FREQUENCY_115HZ;

   	// SLOW_FILTER
	// Possible configuration:
   	// AS5600_SLOW_FILTER_16X
    // AS5600_SLOW_FILTER_8X
    // AS5600_SLOW_FILTER_4X
    // AS5600_SLOW_FILTER_2X
   	config.bit.SF = AS5600_SLOW_FILTER_2X;

   	// FAST_FILTER
	// Possible configuration:
    // AS5600_FAST_FILTER_SLOW_ONLY
    // AS5600_FAST_FILTER_6LSB
    // AS5600_FAST_FILTER_7LSB
    // AS5600_FAST_FILTER_9LSB
    // AS5600_FAST_FILTER_18LSB
    // AS5600_FAST_FILTER_21LSB
    // AS5600_FAST_FILTER_24LSB
    // AS5600_FAST_FILTER_10LSB
   	config.bit.FTH = AS5600_FAST_FILTER_SLOW_ONLY;

   	// WATCHDOG
	// Possible configuration:
    // AS5600_WATCHDOG_OFF 0
    // AS5600_WATCHDOG_ON 1
   	config.bit.WD = AS5600_WATCHDOG_ON;

	// write configuration to sensor
   	// *****************************
	if (AS5600_SetConfig(as5600, config) != HAL_OK){
        status = HAL_ERROR;
        return status;
 	}

    // Check magnet status
	// *******************
    if (AS5600_GetMagnetStatus(as5600, &mag_status) != HAL_OK) {
        status = HAL_ERROR;
        return status;
    }
    if (!mag_status.bit.MagnetDetected) {
        // Magnet not detected
        status = HAL_ERROR;
        return status;
    }
    if (mag_status.bit.Agc_min_gain_overflow) {
        // B-field is too strong
        status = HAL_ERROR;
        return status;
    }
    if (mag_status.bit.Agc_max_gain_overflow) {
        // B-field is too weak
        status = HAL_ERROR;
        return status;
    }

    return status;
}

HAL_StatusTypeDef AS5600_GetRawAngle(AS5600_TypeDef *const a,
                                     uint16_t *const angle) {
    HAL_StatusTypeDef status = HAL_OK;
    angle_reg_t data;
    data.raw = 0;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    		                AS5600_RAWANGLE_REG_ADDR,
    		                I2C_MEMADD_SIZE_8BIT,
							(uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    *angle = ((data.bit.angle8_12<<8)&0xF00)|data.bit.angle0_7;
    return status;
}

HAL_StatusTypeDef AS5600_GetAngle(AS5600_TypeDef *const a, uint16_t *const angle) {
    HAL_StatusTypeDef status = HAL_OK;
    angle_reg_t data;
    data.raw = 0;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    		                AS5600_ANGLE_REG_ADDR,
    		                I2C_MEMADD_SIZE_8BIT,
							(uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    *angle = ((data.bit.angle8_12<<8)&0xF00)|data.bit.angle0_7;

    return status;
}

HAL_StatusTypeDef AS5600_GetMagnetStatus(AS5600_TypeDef *const a, status_reg_t *stat) {
    HAL_StatusTypeDef status = HAL_OK;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    						 AS5600_STATUS_REG_ADDR,
							 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)stat, 1,2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_GetAGCSetting(AS5600_TypeDef *const a,
                                       uint8_t *const agc) {
    HAL_StatusTypeDef status = HAL_OK;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    		                AS5600_AGC_REG_ADDR,
                            I2C_MEMADD_SIZE_8BIT,
							agc, 1,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    return status;
}

HAL_StatusTypeDef AS5600_GetCORDICMagnitude(AS5600_TypeDef *const a,
                                            uint16_t *const mag) {
    HAL_StatusTypeDef status = HAL_OK;
    mag_reg_t data;
    data.raw = 0;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    		                AS5600_MAG_REG_ADDR,
    		                I2C_MEMADD_SIZE_8BIT,
							(uint8_t*)&data.raw,2,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    *mag = ((data.bit.magnitude8_12<<8)&0xF00)|data.bit.magnitude0_7;

    return status;
}

HAL_StatusTypeDef AS5600_SetMaxAngle(AS5600_TypeDef *const a,
                                     const uint16_t angle) {
    HAL_StatusTypeDef status = HAL_OK;
    mang_reg_t data;
    data.raw=0;
    data.bit.mang0_7 = ((angle>>8)&0xFF);
    data.bit.mang8_12 = angle & 0xFF;
    if (HAL_I2C_Mem_Write(a->i2cHandle, a->i2cAddr,
    						 AS5600_MANG_REG_ADDR,
							 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_GetMaxAngle(AS5600_TypeDef *const a,
                                      uint16_t *angle) {
    HAL_StatusTypeDef status = HAL_OK;
    mang_reg_t data;
    data.raw=0;

    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    						 AS5600_MANG_REG_ADDR,
							 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    *angle = ((data.bit.mang8_12<<8)&0xF00)|data.bit.mang0_7;;

    return status;
}

HAL_StatusTypeDef AS5600_SetStopPosition(AS5600_TypeDef *const a,
                                         const uint16_t pos) {
    HAL_StatusTypeDef status = HAL_OK;
    mpos_reg_t data;
    data.raw=0;
    data.bit.mpos0_7 = ((pos>>8)&0xFF);
    data.bit.mpos8_12 = pos & 0xFF;
    if (HAL_I2C_Mem_Write(a->i2cHandle, a->i2cAddr,
    		                 AS5600_MPOS_REG_ADDR,
							 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_GetStopPosition(AS5600_TypeDef *const a,
                                          uint16_t *pos) {
    HAL_StatusTypeDef status = HAL_OK;
    mpos_reg_t data;
    data.raw=0;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
    		                 AS5600_MPOS_REG_ADDR,
							 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    *pos = ((data.bit.mpos8_12<<8)&0xF00)|data.bit.mpos0_7;;

    return status;
}

HAL_StatusTypeDef AS5600_SetStartPosition(AS5600_TypeDef *const a,
                                          const uint16_t pos) {
    HAL_StatusTypeDef status = HAL_OK;
    zpos_reg_t data;
    data.raw=0;
    data.bit.zpos0_7 = ((pos>>8)&0xFF);
    data.bit.zpos8_12 = pos & 0xFF;
    if (HAL_I2C_Mem_Write(a->i2cHandle, a->i2cAddr,
                             AS5600_ZPOS_REG_ADDR,
			                 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_GetStartPosition(AS5600_TypeDef *const a, uint16_t *pos) {
    HAL_StatusTypeDef status = HAL_OK;
    zpos_reg_t data;
    data.raw=0;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
                             AS5600_ZPOS_REG_ADDR,
			                 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&data.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }
    *pos=((data.bit.zpos8_12<<8)&0xF00)|data.bit.zpos0_7;;

    return status;
}

HAL_StatusTypeDef AS5600_SetConfig(AS5600_TypeDef *const a, conf_reg_t conf) {
    HAL_StatusTypeDef status = HAL_OK;
    if (HAL_I2C_Mem_Write(a->i2cHandle, a->i2cAddr,
                             AS5600_ZPOS_REG_ADDR,
			                 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)&conf.raw, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

HAL_StatusTypeDef AS5600_GetConfig(AS5600_TypeDef *const a, conf_reg_t *conf) {
    HAL_StatusTypeDef status = HAL_OK;
    if (HAL_I2C_Mem_Read(a->i2cHandle, a->i2cAddr,
                             AS5600_ZPOS_REG_ADDR,
			                 I2C_MEMADD_SIZE_8BIT,
							 (uint8_t*)conf, 2,2) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

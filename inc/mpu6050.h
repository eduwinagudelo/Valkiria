/*
 * mpu6050.h
 *
 *  Created on: Jul 2, 2018
 *      Author: edwin
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f0xx_i2c.h"

#define MPU6050_I2C			I2C2
#define MPU6050_ADDRESS		0x68

/* Mapeo de registros */
#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10
#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define MOT_THR				0x1F
#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG 		0x37
#define INT_ENABLE			0x38
#define INT_STATUS 			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67
#define I2C_SIG_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A
#define PWR_MGMT_1			0x6B
#define PWR_MGMT_2			0x6C
#define FIFO_COUNTH			0x72
#define FIFO_COUNTL 		0x73
#define FIFO_R_W 			0x74
#define WHO_AM_I			0x75

/* Defines para el valor de la sensibilidad del giroscopio */
#define MPU6050_GYRO_RANGE_250		((float)131)
#define MPU6050_GYRO_RANGE_500		((float)65.5)
#define MPU6050_GYRO_RANGE_1000		((float)32.8)
#define MPU6050_GYRO_RANGE_2000		((float)16.4)

/* Defines para la sensibilidad del acelerometro */
#define MPU6050_ACCEL_RANGE_2g		((float)16384)
#define MPU6050_ACCEL_RANGE_4g		((float)8192)
#define MPU6050_ACCEL_RANGE_8g		((float)4096)
#define MPU6050_ACCEL_RANGE_16g		((float)2048)

/* Valores masimos de timeout en los bucles de espera.
 * Estos timeouts no son tiempo definido
 * son usado para que la aplicacion no se bloquee en las comunicaciones corruptas de I2C
 */
#define MPU6050_FLAG_TIMEOUT             (uint32_t)0x1000
#define MPU6050_LONG_TIMEOUT             (uint32_t)(10 * MPU6050_FLAG_TIMEOUT)


typedef struct{

	float gyroMul;		//Multiplicador del giroscopio en datos brutos
	float accelMul;		//Multiplicador del acelerometro en datos brutos

}MPU6050_dataStruct;

typedef enum{
	/* MPU6050 I2C success */
	MPU6050_NO_ERROR = 0,
	/* I2C error */
	MPU6050_I2C_ERROR = 1,
	/* TX error */
	MPU6050_I2C_TX_ERROR = 2,
	/* TX error */
	MPU6050_I2C_RX_ERROR = 3,

}MPU6050_errorstatus;

/* Opciones del full scala para el giroscopio @gyro_scale_range */
typedef enum{

	MPU6050_GYRO_250 = 0x00,
	MPU6050_GYRO_500 = 0x08,
	MPU6050_GYRO_1000 = 0x10,
	MPU6050_GYRO_2000 = 0x18

}MPU6050_Gyro_Range;

/* Opciones para la full escala del acelerometro @accel_scale_range */
typedef enum{

	MPU6050_ACCEL_2g = 0x00,
	MPU6050_ACCEL_4g = 0x08,
	MPU6050_ACCEL_8g = 0x10,
	MPU6050_ACCEL_16g = 0x18
}MPU6050_Accel_Range;

/* Power management 1 	@pwr_mngt_1 */
typedef enum{

	MPU6050_INTERNAL_OSC = 0x00,
	MPU6050_PLL_X_GYRO = 0x01,
	MPU6050_PLL_Y_GYRO = 0x02,
	MPU6050_PLL_Z_GYRO = 0x03,
	MPU6050_PLL_EXT_32KHZ = 0x04,
	MPU6050_PLL_EXT_19MHZ = 0x05,
	MPU6050_STOP_CLOCK = 0x07
}MPU6050_Clock_Select;

MPU6050_errorstatus MPU6050_Read(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
MPU6050_errorstatus MPU6050_Write(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t* pBuffer);
MPU6050_errorstatus MPU6050_Test(void);

/* Funciones de rango de full eslaca del giroscopio */
uint8_t MPU6050_Gyro_Get_Range(void);
MPU6050_errorstatus MPU6050_Gyro_Set_Range(MPU6050_Gyro_Range range);

/* Funciones de full escala para el acelerometro */
MPU6050_errorstatus MPU6050_Accel_Get_Range(void);
MPU6050_errorstatus MPU6050_Accel_Set_Range(MPU6050_Accel_Range range);

MPU6050_errorstatus MPU6050_Accel_Config(void);
MPU6050_errorstatus MPU6050_Set_Clock(MPU6050_Clock_Select clock);

MPU6050_errorstatus MPU6050_Initialization(void);

/* Definiciones de las funciones de datos */
MPU6050_errorstatus MPU6050_Get_Gyro_Data_Raw(int16_t* X, int16_t* Y, int16_t* Z);
MPU6050_errorstatus MPU6050_Get_Accel_Data_Raw(int16_t* X, int16_t* Y, int16_t* Z);
MPU6050_errorstatus MPU6050_Get_Gyro_Data(float* X, float* Y, float* Z);
MPU6050_errorstatus MPU6050_Get_Accel_Data(float* X, float* Y, float* Z);
int16_t MPU6050_Get_Temperature(void);




#endif /* INC_MPU6050_H_ */

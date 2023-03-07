/*
 * mpu6050.c
 *
 *  Created on: Jul 2, 2018
 *      Author: edwin
 */

#include "mpu6050.h"

uint32_t MPU6050_Timeout = MPU6050_FLAG_TIMEOUT;
MPU6050_dataStruct dataStruct;

/* @brief Inicializa los valores de reloj y sensibilidad en el sensor
*  Esta funcion debe ser llamada antes de usar el sensor!!!
*
* @retval @MPU6050_errorstatus
*/
MPU6050_errorstatus MPU6050_Initialization(void){

	MPU6050_errorstatus errorstatus;

	/* Seteo la fuente de reloj
	 * posibles valores @pwr_mngt_1
	 */
	errorstatus = MPU6050_Set_Clock(MPU6050_PLL_X_GYRO);
	if(errorstatus != 0) return errorstatus;

	/* Seteo la full escala edl giroscopio
	 * posibles valores @gyro_scale_range
	 */
	errorstatus = MPU6050_Gyro_Set_Range(MPU6050_GYRO_250);
	if(errorstatus != 0) return errorstatus;

	/* Seteo el full rango del acelerometro
	 * posible valores @accel_scale_range
	 */
	errorstatus = MPU6050_Accel_Set_Range(MPU6050_ACCEL_2g);
	if(errorstatus != 0) return errorstatus;

	return MPU6050_NO_ERROR;
}


/* @brief Prueba si el chip es visible en la linea I2C
 * Lee el registro WHO_AM_I
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Test(void){

	MPU6050_errorstatus errorstatus;
	uint8_t tmp;

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, WHO_AM_I, &tmp, 1);
	if(tmp != (uint8_t)0x68){
		return errorstatus;
	}
	return MPU6050_NO_ERROR;
}

/* @brief Llama el valor de full escala del giroscopio
 * Lee el registro GYRO_CONFIG y retorna el valor del rango
 *
 * @retval tmp - value of gyro's range
 */
uint8_t MPU6050_Gyro_Get_Range(void){

	MPU6050_errorstatus errorstatus;
	uint8_t tmp;

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_CONFIG, &tmp, 1);
	if(errorstatus != 0){
		return 1;
	}
	else return tmp;

}

/* @brief Setea el valor full escala del giroscopio
 * @param range - revise @MPU6050_Gyro_Range
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Gyro_Set_Range(MPU6050_Gyro_Range range){

	MPU6050_errorstatus errorstatus;
	dataStruct.gyroMul = range;

	errorstatus = MPU6050_Write((MPU6050_ADDRESS & 0x7f) << 1, GYRO_CONFIG, &range);
	if(errorstatus != 0){
		return errorstatus;
	}
	else return MPU6050_NO_ERROR;

}

/* @brief Toma el valor full escala del acelerometro
 * Lee el registro Accel_CONFIG i retornoa el rango
 *
 * @retval tmp - value of accelerometer's range
 */
uint8_t MPU6050_Accel_Get_Range(void){

	MPU6050_errorstatus errorstatus;
	uint8_t tmp;

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_CONFIG, &tmp, 1);
	if(errorstatus != 0){
		return 1;
	}
	else return tmp;

}

/* @brief Setea el valor full escala del acelerometro
 * @param range - Valida @MPU6050_Accel_Range
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Accel_Set_Range(MPU6050_Accel_Range range){

	MPU6050_errorstatus errorstatus;
	dataStruct.accelMul = range;

	errorstatus = MPU6050_Write((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_CONFIG, &range);
	if(errorstatus != 0){
		return errorstatus;
	}
	else return MPU6050_NO_ERROR;

}

/* @brief Setea la fuente de reloj del MPU6050
 * @param clock - Validar @MPU6050_Clock_Select
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus  MPU6050_Set_Clock(MPU6050_Clock_Select clock){

	MPU6050_errorstatus errorstatus;

	errorstatus = MPU6050_Write((MPU6050_ADDRESS & 0x7f) << 1, PWR_MGMT_1, &clock);
	if(errorstatus != 0){
		return errorstatus;
	}
	else return MPU6050_NO_ERROR;

}

/* @brief Lee la temperatura del MPU6050
 * @retval temp_celsius - temperature en grados celsius
 */
int16_t MPU6050_Get_Temperature(void){

	MPU6050_errorstatus errorstatus;
	uint8_t temp_low;
	uint8_t temp_high;
	int16_t temp;
	int16_t temp_celsius;

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, TEMP_OUT_L, &temp_low, 1);
	if(errorstatus != 0){
		return 1;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, TEMP_OUT_H, &temp_high, 1);
	if(errorstatus != 0){
		return 1;
	}

	temp = (uint16_t)(temp_high << 8 | temp_low);

	temp_celsius = temp/340 + 36;
	return temp_celsius;

}

/* @brief Toma los valores brutos del Giroscopio X,Y,Z
 *
 * @param X - sensor roll on X axis
 * @param Y - sensor pitch on Y axis
 * @param Z - sensor jaw on Z axis
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Get_Gyro_Data_Raw(int16_t* X, int16_t* Y, int16_t* Z){

	MPU6050_errorstatus errorstatus;

	uint8_t xlow, xhigh, ylow, yhigh, zlow, zhigh;

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_XOUT_L, &xlow, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_XOUT_H, &xhigh, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_YOUT_L, &ylow, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_YOUT_H, &yhigh, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_ZOUT_L, &zlow, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, GYRO_ZOUT_H, &zhigh, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	*X = (int16_t)(xhigh << 8 | xlow);
	*Y = (int16_t)(yhigh << 8 | ylow);
	*Z = (int16_t)(zhigh << 8 | zlow);

	return MPU6050_NO_ERROR;
}

/* @brief Toma los valores brutos del Accelerometer X,Y,Z
 *
 * @param X - sensor accel on X axis
 * @param Y - sensor accel on Y axis
 * @param Z - sensor accel on Z axis
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Get_Accel_Data_Raw(int16_t* X, int16_t* Y, int16_t* Z){

	MPU6050_errorstatus errorstatus;

	uint8_t xlow, xhigh, ylow, yhigh, zlow, zhigh;

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_XOUT_L, &xlow, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_XOUT_H, &xhigh, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_YOUT_L, &ylow, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_YOUT_H, &yhigh, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_ZOUT_L, &zlow, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	errorstatus = MPU6050_Read((MPU6050_ADDRESS & 0x7f) << 1, ACCEL_ZOUT_H, &zhigh, 1);
	if(errorstatus != 0){
		return errorstatus;
	}

	*X = (int16_t)(xhigh << 8 | xlow);
	*Y = (int16_t)(yhigh << 8 | ylow);
	*Z = (int16_t)(zhigh << 8 | zlow);

	return MPU6050_NO_ERROR;
}

/* @brief Toma los valores calculados de los datos del giroscopio X,Y,Z
 *
 * @param X - sensor roll on X axis
 * @param Y - sensor pitch on Y axis
 * @param Z - sensor jaw on Z axis
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Get_Gyro_Data(float* X, float* Y, float* Z){

	MPU6050_errorstatus errorstatus;

	float mult;
	int16_t gyro_x, gyro_y, gyro_z;

	errorstatus = MPU6050_Get_Gyro_Data_Raw(&gyro_x, &gyro_y, &gyro_z);

	if(dataStruct.gyroMul == MPU6050_GYRO_250){
		mult = (float)(1/MPU6050_GYRO_RANGE_250);
	}
	else if(dataStruct.gyroMul == MPU6050_GYRO_500){
		mult = (float)(1/MPU6050_GYRO_RANGE_500);
	}
	else if(dataStruct.gyroMul == MPU6050_GYRO_1000){
		mult = (float)(1/MPU6050_GYRO_RANGE_1000);
	}
	else mult = (float)(1/MPU6050_GYRO_RANGE_2000);

	*X = (float)(gyro_x*mult);
	*Y = (float)(gyro_y*mult);
	*Z = (float)(gyro_z*mult);

	return MPU6050_NO_ERROR;
}

/* @brief Toma los valores calculados del acelerometros  X,Y,Z
 *
 * @param X - sensor accel on X axis
 * @param Y - sensor accel on Y axis
 * @param Z - sensor accel on Z axis
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Get_Accel_Data(float* X, float* Y, float* Z){

	MPU6050_errorstatus errorstatus;

	float mult;
	int16_t accel_x, accel_y, accel_z;

	errorstatus = MPU6050_Get_Accel_Data_Raw(&accel_x, &accel_y, &accel_z);

	if(dataStruct.accelMul == MPU6050_ACCEL_2g){
		mult = (float)(1/MPU6050_ACCEL_RANGE_2g);
	}
	else if(dataStruct.accelMul == MPU6050_ACCEL_2g){
		mult = (float)(1/MPU6050_ACCEL_RANGE_4g);
	}
	else if(dataStruct.accelMul == MPU6050_ACCEL_2g){
		mult = (float)(1/MPU6050_ACCEL_RANGE_8g);
	}
	else mult = (float)(1/MPU6050_ACCEL_RANGE_16g);

	*X = (float)(accel_x*mult);
	*Y = (float)(accel_y*mult);
	*Z = (float)(accel_z*mult);

	return MPU6050_NO_ERROR;
}

/* @brief Lee datos desde el MPU6050
 *
 * @param SlaveAddr - Slave I2C address
 * @param RegAddr - registro
 * @param pBuffer - buffer donde escribir la info
 * @ param NumByteToRead - Numero de datos a leer
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Read(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{

	/* Test if SDA line busy */
	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
	}

	I2C_TransferHandling(I2C2, SlaveAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
	}

	if(NumByteToRead>1)
	RegAddr |= 0x80;

	I2C_SendData(I2C2, (uint8_t)RegAddr);

	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TC) == RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_TX_ERROR;
	}

    I2C_TransferHandling(I2C2, SlaveAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    while (NumByteToRead)
    {
    	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
    	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) == RESET)
    	{
    		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_RX_ERROR;
    	}

    	*pBuffer = I2C_ReceiveData(I2C2);
    	pBuffer++;

    	NumByteToRead--;
    }

    MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET)
    {
      if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
    }

    I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);

    return MPU6050_NO_ERROR;
}

/* @brief Escribe datos hacia MPU6050
 *
 * @param SlaveAddr - Slave I2C address
 * @param RegAddr - registro
 * @param pBuffer - buffer donde se toman los datos
 *
 * @retval @MPU6050_errorstatus
 */
MPU6050_errorstatus MPU6050_Write(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t* pBuffer)
{

	/* Test if SDA line busy */
	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
	}

	I2C_TransferHandling(I2C2, SlaveAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
	}

	I2C_SendData(I2C2, (uint8_t) RegAddr);

	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TCR) == RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
	}

	I2C_TransferHandling(I2C2, SlaveAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET)
	{
		if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
	}

	I2C_SendData(I2C2, *pBuffer);

    MPU6050_Timeout = MPU6050_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET)
    {
      if((MPU6050_Timeout--) == 0) return MPU6050_I2C_ERROR;
    }

    I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);

	return MPU6050_NO_ERROR;
}



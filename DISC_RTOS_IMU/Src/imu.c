#include "imu.h"
#include "gpio.h"
#include "mpu9250_register_map.h"
#include "main.h"
#include "MadgwickAHRS.h"
#include "i2c.h"
#include "usbd_cdc.h"

void mpu9250Init(void)
{
	/* Check mpu9250 */
	if (i2c_ReadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) == 0x73)
	{
		for(int i; i<10; i++)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_Delay(100);
		}
		sprintf(data, "MPU9255 Init OK\r\n");
		CDC_Transmit_FS((uint8_t*)data, (uint16_t) strlen(data));
	}
	else Error_Handler();

	/* Set accelerometers low pass filter at 5Hz */
	i2c_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06);
	HAL_Delay(10);

	/* Configure gyroscope range */
	i2c_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FULL_SCALE_250_DPS);
	HAL_Delay(10);

	/* Configure accelerometers range */
	i2c_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACC_FULL_SCALE_2_G);
	HAL_Delay(10);

	/* Doing something magic */
	i2c_WriteByte(MPU9250_ADDRESS, CONFIG, 3<<DLPF_CFG_bit);
	HAL_Delay(10);

	/*Output Freq = InternalFreq / (1+SMPLRT_DIV) == 1000 / 10 = 100Hz*/
//	i2c_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
//	HAL_Delay(10);

	/*
	// Set by pass mode for the magnetometers
	i2c_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);
	HAL_Delay(10);

	i2c_WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
	HAL_Delay(10);

	i2c_WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	HAL_Delay(10);

	// Request continuous magnetometer measurements in 16 bits
	i2c_WriteByte(AK8963_ADDRESS, AK8963_CNTL, 0x16);
	HAL_Delay(10);
	 */
	/* Setup INT pin for Interrupts */
	i2c_WriteByte(MPU9250_ADDRESS, INT_ENABLE, DATA_RDY_EN);
}

void mpu9250Data(void)
{
	uint8_t Buf[14] = {0};
	int16_t raw_data[12] = {0};
	float unit[12] = {0};

	// Read the raw data block
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)MPU9250_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0xFFFF);

	// Create unit-vectors
	for(int i = 0; i <= 12; i = i + 2)
	{
		raw_data[i] = Buf[i]<<8 | Buf[i+1];
		unit[i] = (float)raw_data[i]  * A_RES;

		if(i > 4)
		{
			unit[i] = (float)raw_data[i] * G_RES * DEG_TO_RAD;
		}
	}

	//	Send unit data to the filter
	MadgwickAHRSupdateIMU(unit[8], unit[10], unit[12], unit[0], unit[2], unit[4]);

	//	Calculate angles from quaternions
	calc.roll = RAD_TO_DEG * atan2(2*(q0*q1+q2*q3), q3*q3-q2*q2-q1*q1+q0*q0);
	calc.pitch = RAD_TO_DEG * asin(2.0f*(q1*q3-q0*q2));
	calc.yaw = RAD_TO_DEG * atan2(2*(q0*q3+q1*q2), q1*q1+q0*q0-q3*q3-q2*q2);


//	uint8_t count = 0;
	// Send every 100`s data to UART
//	if(count == 10)
//	{
		sprintf(data, "Pitch: %0.1f,\t Roll: %0.1f,\t Yaw: %0.1f\t\r\n", calc.pitch, calc.roll, calc.yaw);
		CDC_Transmit_FS((uint8_t*)data, (uint16_t) strlen(data));
//		count = 0;
//	}
//	count++;
}

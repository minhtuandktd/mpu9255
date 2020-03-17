#include "mpu9255.h"
#include "IMU.h"

uint8_t HMC_rx[6] ,mpu_rx[21];
uint8_t mpu_buf[4], gy_off[6] , re =0 , re_mag= 0 , buffer[3] , buffer2[7], buffer3[21];
extern I2C_HandleTypeDef hi2c1;
int16_t gyro_offset[3];

int16_t gyro[3], gyro_raw[3] ;
int16_t accel[3], accel_raw[3];
int16_t mag[3], mag_raw[3];
float gyro_that[3],accel_that[3];
uint8_t k=0;
float sum_ax=0, sum_ay=0, sum_az=0,sum_gx=0, sum_gy=0, sum_gz=0,  sum_mx=0, sum_my=0, sum_mz=0;
int16_t magn[3],accel[3], gyro[3];
uint8_t mx_sensitivity =0, my_sensitivity = 0, mz_sensitivity = 0, mag_buff[6];
int32_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
#define n		100

int16_t magn_filter_x[n]={0},magn_filter_y[n]={0},magn_filter_z[n]={0};
int16_t accel_filter_x[n]={0}, accel_filter_y[n]={0},accel_filter_z[n]={0};
int16_t gyro_filter_x[n]={0}, gyro_filter_y[n]={0},gyro_filter_z[n]={0};
uint8_t identify_r=0;

unsigned char BUF[10];

void MPU9255_Init(void) 
{
		// select clock source to gyro
		//1 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
		writeRegisterMPU9250(PWR_MGMT_1,CLOCK_SEL_PLL);
		//bit5 = 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK
		mpu_buf[0] = USER_CTRL;
		mpu_buf[1] = 0x20; // enable I2C master mode
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10);
				
		mpu_buf[0] = I2C_MST_CTRL;
		mpu_buf[1] = 0x0D; // set the I2C bus speed to 400 kHz (13)
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10);
		
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		// reset the MPU9250
		writeRegisterMPU9250(PWR_MGMT_1,PWR_RESET);
		HAL_Delay(1);
		 // reset the AK8963
		writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
		
		// select clock source to gyro
		//1 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
		writeRegisterMPU9250(PWR_MGMT_1,CLOCK_SEL_PLL);
		
		mpu_buf[0] = WHO_AM_I;
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS , mpu_buf , 1, 10);
		HAL_I2C_Master_Receive(&hi2c1, MPU6050_DEVICE_ADRESS , &re ,1 ,10);
		HAL_Delay(100);
		if ( re == WHO_AM_I_VAL) 
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		
		mpu_buf[0] = PWR_MGMNT_2;
		mpu_buf[1] = 0x00;  // enable accelerometer and gyro
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10);
		
		mpu_buf[0] = ACCEL_CONFIG; 
		mpu_buf[1] = 0x18;	//16g
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10);
		
		mpu_buf[0] = GYRO_CONFIG; 
		mpu_buf[1] = 0x00;	//250dp
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10); 
		// setting bandwidth to 184Hz as default
		mpu_buf[0] = ACCEL_CONFIG2; 
		mpu_buf[1] = 0x01;	
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10); 
		
		mpu_buf[0] = CONFIG; 
		mpu_buf[1] = 0x01;	
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10); 
		
		// setting the sample rate divider to 0 as default
		mpu_buf[0] = SMPLRT_DIV ; 
		mpu_buf[1] = 0x00;		
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10); 
		
		mpu_buf[0] = USER_CTRL;
		mpu_buf[1] = 0x20; // enable I2C master mode
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10);
				
		mpu_buf[0] = I2C_MST_CTRL;
		mpu_buf[1] = 0x0D; // set the I2C bus speed to 400 kHz (13)
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf,2,10);
		
		readAK8963Registers(AK8963_WHO_AM_I, 1 , &re_mag);
		HAL_Delay(100);
		if ( re_mag == WHO_AM_I_VAL_MAG) 
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		}	
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		HAL_Delay(100); // long wait between AK8963 mode changes
		// set AK8963 to FUSE ROM access
		writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);
		HAL_Delay(100); // long wait between AK8963 mode changes
		// read the AK8963 ASA registers and compute magnetometer scale factors
		readAK8963Registers(ASAX, 3, buffer);
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		HAL_Delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);
		HAL_Delay(100); // long wait between AK8963 mode changes
		writeRegisterMPU9250(PWR_MGMT_1,CLOCK_SEL_PLL);
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(MAG_XOUT_L, 7 ,buffer2);
		
}

void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
  // set slave 0 to the AK8963 and set for write
	writeRegisterMPU9250(I2C_SLV0_ADDR,AK8963_I2C_ADDR) ;
  // set the register to the desired AK8963 sub address 
	writeRegisterMPU9250(I2C_SLV0_REG,subAddress);
  // store the data for write
	writeRegisterMPU9250(I2C_SLV0_DO,data);
  // enable I2C and send 1 byte
	writeRegisterMPU9250(I2C_SLV0_CTRL,(I2C_SLV0_EN | (uint8_t)1) );
}

void writeRegisterMPU9250(uint8_t subAddress, uint8_t data)
{
		mpu_buf[0] = subAddress ; 
		mpu_buf[1] = data;		
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf, 2, 10);   
}
void readRegisterMPU9250(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
		mpu_buf[0] = subAddress ; 
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_DEVICE_ADRESS, mpu_buf, 1, 10); 
		HAL_I2C_Master_Receive(&hi2c1, MPU6050_DEVICE_ADRESS, dest, count, 10);     
}
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  // set slave 0 to the AK8963 and set for read
	writeRegisterMPU9250(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) ;
  // set the register to the desired AK8963 sub address
	writeRegisterMPU9250(I2C_SLV0_REG,subAddress) ;
  // enable I2C and request the bytes
	writeRegisterMPU9250(I2C_SLV0_CTRL,I2C_SLV0_EN | count);
	
	HAL_Delay(10); // takes some time for these registers to fill
	
	readRegisterMPU9250(EXT_SENS_DATA_00 ,count,dest);
}
void MPU9255_Get_Data(void)
{
		readRegisterMPU9250(ACCEL_XOUT_H, 21 , mpu_rx);
		/* Format accelerometer */
		accel_raw[0] = (int16_t)(mpu_rx[0] << 8 | mpu_rx[1]);
		accel_raw[1] = (int16_t)(mpu_rx[2] << 8 | mpu_rx[3]);
		accel_raw[2] = (int16_t)(mpu_rx[4] << 8 | mpu_rx[5]);
		/* Format temperature */
		uint16_t temp = (mpu_rx[6] << 8 | mpu_rx[7]);
		/* Format gyroscope data */
		gyro_raw[0] = (int16_t)(mpu_rx[8]  << 8 | mpu_rx[9]) - gyro_offset[0] ;
		gyro_raw[1] = (int16_t)(mpu_rx[10] << 8 | mpu_rx[11]) - gyro_offset[1];
		gyro_raw[2] = (int16_t)(mpu_rx[12] << 8 | mpu_rx[13]) - gyro_offset[2];
//		gyro_raw[0] = (int16_t)(mpu_rx[8]  << 8 | mpu_rx[9]) ;
//		gyro_raw[1] = (int16_t)(mpu_rx[10] << 8 | mpu_rx[11]) ;
//		gyro_raw[2] = (int16_t)(mpu_rx[12] << 8 | mpu_rx[13]) ;
		/* Format compass */
		mag_raw[0]=(((uint16_t) mpu_rx[15])<<8)|mpu_rx[14];
		mag_raw[1]=(((uint16_t) mpu_rx[17])<<8)|mpu_rx[16];
		mag_raw[2]=(((uint16_t) mpu_rx[19])<<8)|mpu_rx[18];		
		Average_Filter(accel_raw[0], accel_raw[1], accel_raw[2], mag_raw[0], mag_raw[1], mag_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]);
}
void InitGyrOffset(void)
{
	uint8_t i, gyro_off[6]={0};
	int32_t	TempGx = 0, TempGy = 0, TempGz = 0;
	
 	for(i = 0; i < 128; i ++)
 	{
//		readRegisterMPU9250(GYRO_XOUT_H, 6, gyro_off);	
//		TempGx += (gyro_off[0]<<8)|gyro_off[1];
//		TempGy += (gyro_off[2]<<8)|gyro_off[3];
//		TempGz += (gyro_off[4]<<8)|gyro_off[5];
		readRegisterMPU9250(ACCEL_XOUT_H, 21 , mpu_rx);
		TempGx = TempGx + (int16_t)(mpu_rx[8]  << 8 | mpu_rx[9]) ;
		TempGy = TempGy + (int16_t)(mpu_rx[10] << 8 | mpu_rx[11]) ;
		TempGz = TempGz + (int16_t)(mpu_rx[12] << 8 | mpu_rx[13]) ;
		HAL_Delay(20);
	}
	gyro_offset[0]= TempGx/128;
	gyro_offset[1] = TempGy/128;
	gyro_offset[2] = TempGz/128;
}
void Average_Filter(int16_t ax,int16_t ay,int16_t az,int16_t mx,int16_t my, int16_t mz,int16_t gx,int16_t gy, int16_t gz)
{
	sum_ax = sum_ax + ax - accel_filter_x[k];
	sum_ay = sum_ay + ay - accel_filter_y[k];
	sum_az = sum_az + az - accel_filter_z[k];
	sum_mx = sum_mx + mx - magn_filter_x[k];
	sum_my = sum_my + my - magn_filter_y[k];
	sum_mz = sum_mz + mz - magn_filter_z[k];
	sum_gx = sum_gx + gx - gyro_filter_x[k];
	sum_gy = sum_gy + gy - gyro_filter_y[k];
	sum_gz = sum_gz + gz - gyro_filter_z[k];
	accel_filter_x[k]= ax;
	accel_filter_y[k]= ay;
	accel_filter_z[k]= az;
	magn_filter_x[k] = mx;
	magn_filter_y[k] = my;
	magn_filter_z[k] = mz;
	gyro_filter_x[k] = gx;
	gyro_filter_y[k] = gy;
	gyro_filter_z[k] = gz;
	
	accel[0] = sum_ax/n;
	accel[1] = sum_ay/n;
	accel[2] = sum_az/n;
	mag[0] = sum_mx/n;
	mag[1] = sum_my/n;
	mag[2] = sum_mz/n;
	gyro[0] = sum_gx/n;
	gyro[1] = sum_gy/n;
	gyro[2] = sum_gz/n;
	accel_that[0] = accel[0]/2048.0;
	accel_that[1] = accel[1]/2048.0;
	accel_that[2] = accel[2]/2048.0;
	gyro_that[0] = gyro[0]/131.072;
	gyro_that[1] = gyro[1]/131.072;
	gyro_that[2] = gyro[2]/131.072;
	k++;
	if(k==n) k=0;
}

#define    	MPU9255_ADDRESS	0x68
#define		GYRO_ADDRESS   	0xD0
#define 	ACCEL_ADDRESS  	0xD0 

#define    GYRO_FULL_SCALE_250_DPS    0x00 
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define 	GYRO_SCALE_FACTOR_0	131
#define 	GYRO_SCALE_FACTOR_1	65.5
#define 	GYRO_SCALE_FACTOR_2	32.8
#define 	GYRO_SCALE_FACTOR_3	16.4

#define		GYRO_XOUT_H	0x43
#define		GYRO_XOUT_L	0x44
#define		GYRO_YOUT_H	0x45
#define		GYRO_YOUT_L	0x46
#define		GYRO_ZOUT_H	0x47
#define		GYRO_ZOUT_L	0x48

#define		GYRO_CONFIG	0x1b

#define    ACC_FULL_SCALE_2_G        0x00 
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define		ACC_XOUT_H	0x3b
#define		ACC_XOUT_L	0x3c
#define		ACC_YOUT_H	0x3d
#define		ACC_YOUT_L	0x3e
#define		ACC_ZOUT_H	0x3f
#define		ACC_ZOUT_L	0x40

#define		ACC_CONFIG	0x1c

#define M_PI 3.14159265358979323846
#define RAD_TO_DEG 57.29578

#define DT 0.02
#define AA 0.97

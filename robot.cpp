/*Compile with
 * 
 * g++ -std=c++11 robot.cpp -o robot -lwiringPi -lm
 * 
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <signal.h>

#include "motorhat.h"
#include "mpu9255.h"

typedef unsigned short word;
int fd;
int shutdown = 0;

class Adafruit_MotorHAT {

    word i2caddr, i2c;
    word in1[4] = {99, 99, 99, 99}, 
		 in2[4] = {99, 99, 99, 99}, 
		 pwm[4] = {99, 99, 99, 99};

  public:
    static word Hat_Addr[4];
    Adafruit_MotorHAT(word addr = ADAFRUIT_MOTORHAT) {

		for (int i = 0; i < 4; i++) {
		    if (Hat_Addr[i] == addr) {
			printf("Moror Hat %x already in use!\n", addr);
			exit(3);
		    }
		}
		int j = 99;
		for (int i = 0; i < 4; i++) {
		    if (Hat_Addr[i] == 0)
			j = i;
		}
	
		if (j == 99) {
		    printf("No room for more Hats (max 4 allowed)!\n");
		    exit(4);
		} else {
		    Hat_Addr[j] = addr;
		}
	
		i2caddr = addr;
	
		// Setup I2C
		i2c = wiringPiI2CSetup(i2caddr);
		printf("Motor Hat %x created!\n", addr);
	
		// Setup PWM
		setAllPWM(i2c, 0, 0);
		wiringPiI2CWriteReg8(i2c, PWM_MODE2, PWM_OUTDRV);
		wiringPiI2CWriteReg8(i2c, PWM_MODE1, PWM_ALLCALL);
		delay(5);
		word mode1 = wiringPiI2CReadReg8(i2c, PWM_MODE1) & ~PWM_SLEEP;
		wiringPiI2CWriteReg8(i2c, PWM_MODE1, mode1);
		delay(5);
	
		// Set PWM frequency
		word prescale =
		    (int) (ceil(25000000.0 / 4096.0 / PWM_FREQUENCY - 1.0));
		//printf("prescale value is %d\n", prescale);
		word oldmode = wiringPiI2CReadReg8(i2c, PWM_MODE1);
		word newmode = oldmode & 0x7F | 0x10;
		wiringPiI2CWriteReg8(i2c, PWM_MODE1, newmode);
		wiringPiI2CWriteReg8(i2c, PWM_PRESCALE, prescale);
		wiringPiI2CWriteReg8(i2c, PWM_MODE1, oldmode);
		delay(5);
		wiringPiI2CWriteReg8(i2c, PWM_MODE1, oldmode | 0x80);
    };

    word getAddr() {
		return i2caddr;
    };

    void setAllPWM(word i2c, word on, word off) {
		// Sets all PWM channels
		wiringPiI2CWriteReg8(i2c, PWM_ALL_LED_ON_L, on & 0xFF);
		wiringPiI2CWriteReg8(i2c, PWM_ALL_LED_ON_H, on >> 8);
		wiringPiI2CWriteReg8(i2c, PWM_ALL_LED_OFF_L, off & 0xFF);
		wiringPiI2CWriteReg8(i2c, PWM_ALL_LED_OFF_H, off >> 8);
    };

    void setPWM(word i2c, word channel, word on, word off) {
		// Sets a single PWM channel
		wiringPiI2CWriteReg8(i2c, PWM_LED0_ON_L + 4 * channel, on & 0xFF);
		wiringPiI2CWriteReg8(i2c, PWM_LED0_ON_H + 4 * channel, on >> 8);
		wiringPiI2CWriteReg8(i2c, PWM_LED0_OFF_L + 4 * channel,
				     off & 0xFF);
		wiringPiI2CWriteReg8(i2c, PWM_LED0_OFF_H + 4 * channel, off >> 8);
    };

    void setPin(word i2c, word pin, word value) {
		if (pin < 0 || pin > 15) {
		    printf("PWM pin must be between 0 and 15 inclusive.  Received '%d'\n",pin);
			return;
		}

		switch (value) {
		case 0:
		    setPWM(i2c, pin, 0, 4096);
		    break;
		case 1:
		    setPWM(i2c, pin, 4096, 0);
		    break;
		default:
		    printf("PWM pin value must be 0 or 1.  Received '%d'\n", pin);
		    return;
		}
    };

    void addDCMotor(word motor) {
		if (in1[motor - 1] != 99) {
		    printf("DC Motor %d already added!\n", motor);
		    exit(1);
		}
		switch (motor) {
		case 1:
		    in1[0] = PWM_M1_IN1;
		    in2[0] = PWM_M1_IN2;
		    break;
		case 2:
		    in1[1] = PWM_M2_IN1;
		    in2[1] = PWM_M2_IN2;
		    break;
		case 3:
		    in1[2] = PWM_M3_IN1;
		    in2[2] = PWM_M3_IN2;
		    break;
		case 4:
		    in1[3] = PWM_M4_IN1;
		    in2[3] = PWM_M4_IN2;
		    break;
		default:
		    printf("Invalid DC motor number '%d'\n", motor);
		    return;
		}
    };

    void runDCMotor(word motor, word command) {
		switch (command) {
		case MOTOR_FORWARD:
		    setPin(i2c, in2[motor - 1], 0);
		    setPin(i2c, in1[motor - 1], 1);
		    break;
		case MOTOR_BACKWARD:
		    setPin(i2c, in1[motor - 1], 0);
		    setPin(i2c, in2[motor - 1], 1);
		    break;
		case MOTOR_RELEASE:
		    setPin(i2c, in1[motor - 1], 0);
		    setPin(i2c, in2[motor - 1], 0);
		    break;
		default:
		    printf("Unsupported command '%d'\n", command);
		    return;
		}
    };

    void setDCMotorSpeed(word motor, word speed) {
		if (speed < 0 || speed > 255) {
		    printf("Speed must be between 0 and 255 inclusive.  Received '%d'\n",speed);
		    return;
		}
	
		switch (motor) {
		case 1:
		    pwm[0] = PWM_M1_PWM;
		    break;
		case 2:
		    pwm[1] = PWM_M2_PWM;
		    break;
		case 3:
		    pwm[2] = PWM_M3_PWM;
		    break;
		case 4:
		    pwm[3] = PWM_M4_PWM;
		    break;
		default:
		    printf("Unsupported motor '%s'\n", motor);
		    break;
		}
		setPWM(i2c, pwm[motor - 1], 0, speed * 16);
    };
};

void DCTest(word mtr){
    Adafruit_MotorHAT hat;
    hat.addDCMotor(mtr);

    printf("Forward!\n");
    hat.runDCMotor(mtr, MOTOR_FORWARD);

    printf("\tSpeed up...\n");
    for (int i = 1; i < 256; i++) {
		hat.setDCMotorSpeed(mtr, i);
		delay(100);
    };

    printf("\tSlow down...\n");
    for (int i = 255; i > 0; i--) {
		hat.setDCMotorSpeed(mtr, i);
		delay(100);
    };

    printf("Backward!\n");
    hat.runDCMotor(mtr, MOTOR_BACKWARD);

    printf("\tSpeed up...\n");
    for (int i = 1; i < 256; i++) {
		hat.setDCMotorSpeed(mtr, i);
		delay(100);
    };

    printf("\tSlow down...\n");
    for (int i = 255; i > 0; i--) {
		hat.setDCMotorSpeed(mtr, i);
		delay(100);
    };

    printf("Release\n");
    hat.runDCMotor(mtr, MOTOR_RELEASE);

    for (int i = 0; i < 4; i++) {
		if (Adafruit_MotorHAT::Hat_Addr[i] == hat.getAddr()) {
		    Adafruit_MotorHAT::Hat_Addr[i] = 0;
		    printf("Moror Hat %x removed!\n", hat.getAddr());
		}
    }
}

char I2C_readByte(int reg){
    return (char)wiringPiI2CReadReg8(fd,reg);
}

void I2C_writeByte(int reg,int val){
    wiringPiI2CWriteReg8(fd,reg,val);
}

void setup9255(){
	// Configure gyroscope range
	I2C_writeByte(GYRO_CONFIG,GYRO_FULL_SCALE_250_DPS);
	// Configure accelerometer range
	I2C_writeByte(ACC_CONFIG,ACC_FULL_SCALE_2_G);
}

void read_accelerometer(short accRaw[2]){
	char Buf[6];
	
	Buf[0]=I2C_readByte(ACC_XOUT_H);
	Buf[1]=I2C_readByte(ACC_XOUT_L);
	Buf[2]=I2C_readByte(ACC_YOUT_H);
	Buf[3]=I2C_readByte(ACC_YOUT_L);
	Buf[4]=I2C_readByte(ACC_ZOUT_H);
	Buf[5]=I2C_readByte(ACC_ZOUT_L);
	
	// Create 16 bit values from 8 bits data
	
	accRaw[0] = Buf[0]<<8 | Buf[1];
	accRaw[1] = Buf[2]<<8 | Buf[3];
	accRaw[2] = Buf[4]<<8 | Buf[5];
	
	//printf("Ax = %d Ay = %d Az = %d\n",accRaw[0],accRaw[1],accRaw[2]);

	return;
}

void read_gyroscope(short gyrRaw[3]){
	char Buf[6];
	
	Buf[0]=I2C_readByte(GYRO_XOUT_H);
	Buf[1]=I2C_readByte(GYRO_XOUT_L);
	Buf[2]=I2C_readByte(GYRO_YOUT_H);
	Buf[3]=I2C_readByte(GYRO_YOUT_L);
	Buf[4]=I2C_readByte(GYRO_ZOUT_H);
	Buf[5]=I2C_readByte(GYRO_ZOUT_L);
	
	// Create 16 bit values from 8 bits data

	gyrRaw[0] = Buf[0]<<8 | Buf[1];
	gyrRaw[1] = Buf[2]<<8 | Buf[3];
	gyrRaw[2] = Buf[4]<<8 | Buf[5];  
	
	//printf("Gx = %d Gy = %d Gz = %d\n",gyrRaw[0],gyrRaw[1],gyrRaw[2]);
	
	return;
}

void die(int signum) {
      shutdown = 1;
}

word Adafruit_MotorHAT::Hat_Addr[4] = { 0, 0, 0, 0 };

int main(int argc, char *argv[]){
	float rate_gyr_y = 0.0;   
	float rate_gyr_x = 0.0;  
	float rate_gyr_z = 0.0;   

	short  accRaw[3];
	short  gyrRaw[3];

	float gyroXangle = 0.0;
	float gyroYangle = 0.0;
	float gyroZangle = 0.0;
	float AccXangle = 0.0;
	float AccYangle = 0.0;
	float AccZangle = 0.0;
	float CFangleX = 0.0;
	float CFangleY = 0.0;
	
	//P- determines the force with which the robot will correct itself.
	//   A lower P shows robot’s inability to balance itself and a 
	//   higher P will shows the violent behavior.
    //I- determines the response time of robot for correcting itself. 
    //   Higher the I, Faster it will response.
    //D- determines the sensitivity of robot to the error in its state.
    //   It is used to smoothen/depress the robot oscillations. 
    //   A lower D is unable to remove oscillations and a higher D 
    //   will cause violent vibrations.
	float ki=3.0,kd=4.0,kp=12.0;
	
	float iTerm=0.0, dTerm=0.0, pTerm=0.0;
	float prevAngle=0.0, thrust=0.0;
	
	int startInt;
	
	//PID tuning parameters
	if(argc == 4) {	
		ki=atof(argv[1]);
		kd=atof(argv[2]);
		kp=atof(argv[3]);
	}
	
    Adafruit_MotorHAT hat;
    word motorSpeed = 0;
    
    signal(SIGINT,die);
    
    setup9255();
    
    fd = wiringPiI2CSetup(MPU9255_ADDRESS);
    
    hat.addDCMotor(1);
    hat.addDCMotor(2);
    hat.runDCMotor(1, MOTOR_FORWARD);
    hat.runDCMotor(2, MOTOR_FORWARD);
    hat.setDCMotorSpeed(1, motorSpeed);
    hat.setDCMotorSpeed(2, motorSpeed);
    
    while(1){
		startInt = millis();
	
		//read ACC and GYR data
		read_accelerometer(accRaw);
		read_gyroscope(gyrRaw);
	
		//Convert Gyro raw to degrees per second
		rate_gyr_x = (float) gyrRaw[0]  * 1.0/GYRO_SCALE_FACTOR_0;
		rate_gyr_y = (float) gyrRaw[1]  * 1.0/GYRO_SCALE_FACTOR_0;
		rate_gyr_z = (float) gyrRaw[2]  * 1.0/GYRO_SCALE_FACTOR_0;
	
		//Calculate the angles from the gyro
		gyroXangle=rate_gyr_x*DT;
		gyroYangle=rate_gyr_y*DT;
		gyroZangle=rate_gyr_z*DT;
	
		//Convert Accelerometer values to degrees
		AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
		AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;
		AccZangle = (float) (atan2(accRaw[0],accRaw[1])+M_PI)*RAD_TO_DEG;
	
	    //Change the rotation value of the accelerometer to -/+ 180 
	    //and move the Y axis '0' point to up.
	    //Two different pieces of code are used depending on how 
	    //your IMU is mounted.
	    //If IMU is upside down

	        //if (AccXangle >180)
	                //AccXangle -= (float)360.0;
	        //AccYangle-=90;
	        //if (AccYangle >180)
	                //AccYangle -= (float)360.0;
	
	    //If IMU is up the correct way, use these lines
	    AccXangle -= (float)180.0;
		if (AccYangle > 90)
		    AccYangle -= (float)270;
		else
			AccYangle += (float)90;
			
		//Complementary filter used to combine the accelerometer and gyro values.
		CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
		CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;
	
		//printf("X = %6.2f Y = %6.2f\n",CFangleX,CFangleY);
	
		//Compute PID values to get the thrust needed to apply to motors
		pTerm = kp * CFangleX;
		iTerm += ki * CFangleX;
		dTerm = kd * (CFangleX - prevAngle);
		prevAngle = CFangleX;
		thrust = pTerm + iTerm + dTerm;
		//printf("Thrust is %6.2f\n", thrust);
		
		/*	
	    1.Set KI and KD to zero.
	    2.Set KP high enougth so that the motors drive the wheels under 
			the robot in the direction it is falling. The robot should 
			overshoot Point Zero a little bit and then start to oscillate.
	    3.Reduce KP by about 10% to so you get just below the oscillation.
	    4.Increase KI. This will help the robot reach Point Zero faster 
			and will also start to oscillate. Try and get a value so that 
			the robot just oscillates
	    5.Increase KD to dampen the oscillation and until the robot balances.
		*/
		
		//Get direction
		if(thrust < 0.0){
			hat.runDCMotor(1, MOTOR_BACKWARD);
			hat.runDCMotor(2, MOTOR_BACKWARD);
			printf("...BACKWARD\n");
			thrust = -thrust;
		}
		else {
			hat.runDCMotor(1, MOTOR_FORWARD);
			hat.runDCMotor(2, MOTOR_FORWARD);
			printf("...FORWARD\n");
		}
		//Convert thrust(float) to speed(word)
		if(thrust > 255) thrust = 255;
		motorSpeed = thrust;
		printf("Speed is %d\n", motorSpeed);
		
		hat.setDCMotorSpeed(1, motorSpeed);
		hat.setDCMotorSpeed(2, motorSpeed);
		
		//Each loop should be at least 20ms.
	    while(millis() - startInt < (DT*1000))
	            usleep(100);
	
		printf("Loop Time %d\t", millis()- startInt);
		
		//Shutdown when interrupt with CTRL-C
		if(shutdown) {
			hat.setDCMotorSpeed(1, 0);
			hat.setDCMotorSpeed(2, 0);
			hat.runDCMotor(1, MOTOR_RELEASE);
			hat.runDCMotor(2, MOTOR_RELEASE);
			exit(0);
		}
	}
    
    return 0;
}

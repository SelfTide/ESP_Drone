/*
 *
 *	Motor Control
 *
 *	PWM	 -	Motor throttle control
 *	Gyro -	MPU6050
 *	PID	 -	Stablization control, steering	
 *
 */
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include "PID_v2.h"
#include "Motor_control.h"
#include "common.h"


extern Control_data controller;

/*
 *	Globals PWM LEDC
 */
ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,	// this sets our duty cycle to 0 - 255
    .timer_num  = LEDC_TIMER_0,
    .freq_hz = 498,
    .clk_cfg = LEDC_AUTO_CLK,
};

ledc_channel_config_t ledc_channel[4];

int target_speed[4];

/*
 * 	Globals MPU6050
 */
Quaternion q;				// [w, x, y, z]         quaternion container
VectorFloat gravity;		// [x, y, z]            gravity vector
float ypr[3];				// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;			// count of all bytes currently in FIFO
uint8_t fifoBuffer[64];		// FIFO storage buffer
uint8_t mpuIntStatus;		// holds actual interrupt status byte from MPU

MPU6050 mpu;

/*
 *	Globals PID
 */
double roll_setpoint, roll_input, roll_output;
double pitch_setpoint, pitch_input, pitch_output;
double yaw_setpoint, yaw_input, yaw_output;

float ypr_cal[3];

//Define the aggressive and conservative Tuning Parameters
double consKp = 0.5, consKi = 0.05, consKd = 0.05;

PID roll_PID(&pitch_input, &pitch_output, &pitch_setpoint, consKp, consKi, consKd, DIRECT);
PID pitch_PID(&roll_input, &roll_output, &roll_setpoint, consKp, consKi, consKd, DIRECT);
PID yaw_PID(&yaw_input, &yaw_output, &yaw_setpoint, consKp, consKi, consKd, DIRECT);

void motor_control_init(void) {
	
	// initalize MPU6050
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	mpu = MPU6050();
	mpu.initialize();
	mpu.dmpInitialize();

	// This needs to be setup individually
	mpu.setXGyroOffset(-22);
	mpu.setYGyroOffset(48);
	mpu.setZGyroOffset(68);
	mpu.setZAccelOffset(1760); // 1688 factory default for my test chip
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);

	mpu.setDMPEnabled(true);
	
	// initalize PWM LEDC
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
	
	ledc_channel[0].channel = LEDC_CHANNEL_0;
	ledc_channel[0].gpio_num = FL_MOTOR;
	
	ledc_channel[1].channel = LEDC_CHANNEL_1;
	ledc_channel[1].gpio_num = FR_MOTOR;

	ledc_channel[2].channel = LEDC_CHANNEL_2;
	ledc_channel[2].gpio_num = BL_MOTOR;
	
	ledc_channel[3].channel = LEDC_CHANNEL_3;
	ledc_channel[3].gpio_num = BR_MOTOR;
	
	for (int i = 0; i < 4; i++)
	{   
		ledc_channel[i].speed_mode = LEDC_LOW_SPEED_MODE;
		ledc_channel[i].timer_sel = LEDC_TIMER_0;
		ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
		ledc_channel[i].flags.output_invert = 1;
		ledc_channel[i].duty = 0;
		ledc_channel[i].hpoint = 0;

		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
	}
	
	for (int i = 0; i < 4; i++) {
		target_speed[i] = 0;
	}
	
	// initalize PID
	for (int i = 0; i < 3; i++)
	{
		ypr_cal[i] = 0.0;
	}
	
	roll_input = 0.0;
	pitch_input = 0.0;
	yaw_input = 0.0;

	roll_setpoint = 0.0;
	pitch_setpoint = 0.0;
	yaw_setpoint = 0.0;
	
	roll_PID.SetMode(AUTOMATIC);
	pitch_PID.SetMode(AUTOMATIC);
	yaw_PID.SetMode(AUTOMATIC);

	roll_PID.SetOutputLimits(-20, 20);
	pitch_PID.SetOutputLimits(-20, 20);
	yaw_PID.SetOutputLimits(-20, 20);
}

void set_motors_speed (int *Speed) 
{
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, Speed[0]));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
	
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, Speed[1]));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, Speed[2]));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
	
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, Speed[3]));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
}

void motor_control_stabilization (int *curr_speed, int *act_speed, double roll_diff, double pitch_diff, double yaw_diff) 
{
	/*
	 *		(roll left/right)	   (pitch up/down)	(yaw turn left/right)
	 *		 (-)	(+)		    (+)    (+)           (-)    (+) 
	 *		   \   /              	      \   /                \   /
	 *	             O                  	O                    O
	 *         	   /   \              	      /   \                /   \
	 *      	(-)     (+)        	   (-)     (-)          (+)     (-)
	 */
	act_speed[0] = (int) curr_speed[0] - (roll_diff) + (pitch_diff) - (yaw_diff); 	// FL
	act_speed[1] = (int) curr_speed[1] + (roll_diff) + (pitch_diff) + (yaw_diff);	// FR
	act_speed[2] = (int) curr_speed[2] - (roll_diff) - (pitch_diff) + (yaw_diff);	// BL
	act_speed[3] = (int) curr_speed[3] + (roll_diff) - (pitch_diff) - (yaw_diff);	// BR

	for (int i = 0; i < 4; i ++)
	{
		if (act_speed[i] < 0 )
			act_speed[i] = 0;
	}
}

int current_time, previous_time;

void motor_control_run(void)
{	
	mpuIntStatus = mpu.getIntStatus();
	
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();

	  // otherwise, check for DMP data ready interrupt frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}
	
	// PID Cal while throttle is at 0
	if (target_speed[0] == 0)
	{
		// CALIBRATING...
		ypr_cal[0] = ypr[0] * 180 / M_PI;
		ypr_cal[1] = ypr[1] * 180 / M_PI;
		ypr_cal[2] = ypr[2] * 180 / M_PI;
	}
	
	roll_input = ypr[2] * 180 / M_PI - ypr_cal[2];
	pitch_input = ypr[1] * 180 / M_PI - ypr_cal[1];
	yaw_input = ypr[1] * 180 / M_PI - ypr_cal[1];
	
	roll_setpoint = controller.roll;
	pitch_setpoint = controller.pitch;
	yaw_setpoint = controller.yaw;
	
	roll_PID.Compute();
	pitch_PID.Compute();
	yaw_PID.Compute();
	
	for (int i = 0; i < 4; i++) 
		target_speed[i] = controller.throttle;
	
	set_motors_speed(target_speed);
	
	int act_speed[4];
	motor_control_stabilization(target_speed, act_speed, roll_output, pitch_output, yaw_output);
/*
	Leaving this in for debug

	current_time = esp_timer_get_time();
	// limit diag display to only print once every second
	if(((current_time - previous_time)/1000) > 1000)
	{
		printf("MPU int status: %d\n", mpuIntStatus);
		printf("Time: %d\n", (current_time - previous_time));
		printf("YAW: %3.1f, ", ypr[0] * 180 / M_PI);
		printf("PITCH: %3.1f, ", ypr[1] * 180 / M_PI);
		printf("ROLL: %3.1f\n", ypr[2] * 180 / M_PI);
	
		printf("Current Speed: mot[0] = %d, mot[1] = %d, mot[2] = %d, mot[3] = %d\n", act_speed[0], act_speed[1], act_speed[2], act_speed[3]);
		printf("Current PID values: Roll = %f, Pitch = %f, Yaw = %f\n", roll_output, pitch_output, yaw_output);
		
		previous_time = current_time;
	}
*/
}


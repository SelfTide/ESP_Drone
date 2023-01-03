#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H


// Motor PINS
#define FL_MOTOR 13
#define FR_MOTOR 12
#define BR_MOTOR 27
#define BL_MOTOR 14
#define FL_MOTOR_CHANNEL 1
#define FR_MOTOR_CHANNEL 2
#define BL_MOTOR_CHANNEL 3
#define BR_MOTOR_CHANNEL 4

// MPU6050 PINS
#define PIN_SDA 17
#define PIN_CLK 16

#ifdef __cplusplus
extern "C" {
#endif

	extern void motor_control_init(void);
	extern void motor_control_run(void);
	
#ifdef __cplusplus
}
#endif

#endif
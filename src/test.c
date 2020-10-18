#include "hal.h"
#include "test.h"
#include "usb.h"

// This file contains test functions for the code

// Test moving the specified servo by 5% of its full range every 2 seconds
void test_move_servo(enum motor_num motor){
	float set_percent = 0;
	bool direction = 0;
	while(1){
		hal_set_motor_duty_cycle(motor, set_percent);

		if(set_percent > 95){
			// We are at the top range - count down
			direction = 1;
		}
		else if (set_percent < 5){
			// We are at the bottom range - count up
			direction = 0;
		}

		if(direction == 0){
			// Count up mode
			set_percent += 5;
		}
		else{
			// Count down mode
			set_percent -= 5;
		}
		hal_delay_ms(2000);
	}
}

// Test moving all servos by shifting each one by 5% of its full range every 8 seconds
void test_move_all_servos(){
	float set_percent = 0;
	bool direction = 0;
	enum motor_num motor = MOTOR1;
	while(1){
		hal_set_motor_duty_cycle(motor, set_percent);
		if(motor >= MOTOR_ENUM_END){
			motor = MOTOR1;
		}
		else{
			motor++;
		}

		if(set_percent > 95){
			// We are at the top range - count down
			direction = 1;
		}
		else if (set_percent < 5){
			// We are at the bottom range - count up
			direction = 0;
		}

		if(direction == 0){
			// Count up mode
			set_percent += 5;
		}
		else{
			// Count down mode
			set_percent -= 5;
		}
		hal_delay_ms(1000);
	}
}

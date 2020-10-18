#include <stdint.h>
#include <stdbool.h>

enum motor_num{
	MOTOR1 = 1,
	MOTOR2 = 2,
	MOTOR3 = 3,
	MOTOR4 = 4,
	MOTOR5 = 5,
	MOTOR6 = 6,
	MOTOR7 = 7,
	MOTOR8 = 8,
	MOTOR_ENUM_END = 9,
};

extern float motor_duty_cycle[8];

// Initialization
void hal_init(void);

// Motor Control
void hal_set_motor_duty_cycle(enum motor_num motor, float duty_cycle);

// LED Control
void hal_control_spi_led(bool enable);
void hal_control_i2c_led(bool enable)
void hal_control_general_led1(bool enable);
void hal_control_general_led2(bool enable);
void hal_control_general_led3(bool enable);

// Logic fault handler
void __attribute__((noreturn)) LOGIC_FAULT(void);



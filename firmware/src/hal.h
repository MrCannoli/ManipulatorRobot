#pragma once

#include <stdbool.h>
#include <stdint.h>

enum motor_num {
    MOTOR1 = 0,
    MOTOR2 = 1,
    MOTOR3 = 2,
    MOTOR4 = 3,
    MOTOR5 = 4,
    MOTOR6 = 5,
    MOTOR7 = 6,
    MOTOR8 = 7,
    MOTOR_ENUM_END = 8,
};

enum {
    DISABLE = 0,
    ENABLE = 1,
};

/// \struct Data structure for holding different counters. Used for debugging and statistics.
struct counters {
    uint32_t ik_success_count;
    uint32_t ik_timeout_count;
    uint32_t ik_iteration_overrun_count;
    uint32_t ik_too_large_angle_count;
    uint32_t ik_negative_angle_count;
    uint32_t ik_nan_position;
    uint32_t invalid_set_angle;
};

/// \var Struct containing debugging and logging counters
extern struct counters counters;

/// \var Array holding the PWM duty cycle for each motor
extern float motor_duty_cycle[8];

// Initialization
void hal_init(void);

// Motor Control
void hal_set_motor_duty_cycle(enum motor_num motor, float duty_cycle);
void hal_motor_enable();
void hal_motor_disable();

// LED Control
void hal_control_spi_led(bool enable);
void hal_control_i2c_led(bool enable);
void hal_control_general_led1(bool enable);
void hal_control_general_led2(bool enable);
void hal_control_general_led3(bool enable);

// Delay Functions
void hal_delay_ms(uint32_t delay);

// Counter Functions
uint32_t hal_get_systick_counter(void);

// Compiler barrier
void hal_compiler_barrier(void);

// Timer control
void hal_movement_timer_enable();
void hal_movement_timer_disable();

// Logic fault handler
void __attribute__((noreturn)) logic_fault(void);

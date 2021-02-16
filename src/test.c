#include "test.h"
#include "hal.h"
#include "position_control.h"
#include "usb.h"
#include <stddef.h>
#include <stdint.h>

// This file contains test functions for the code

// Test points for IK and movement
const struct Point tp1 = {10, 10, 10};
const struct Point tp2 = {100, 20, 30}; // Unreachable
const struct Point tp3 = {90, 0, 0}; // 1 solution possible
const struct Point tp4 = {40, 20, 80}; // Unreachable
const struct Point tp5 = {-1, 37, 12};
const struct Point tp6 = {1000, 70, 70}; // No solutions
const struct Point tp7 = {-10, 40, 40}; // Close to 120 deg limit, reachable
const struct Point tp8 = {-40, -10, 40}; // Beyond the range of the joints; difficult for IK to determine

const struct Point test_points[] = {tp1, tp2, tp3, tp4, tp5, tp6, tp7, tp8};

uint32_t test_success[10];
uint32_t test_fail[10];

// Debug variables
volatile uint32_t temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9;

// Test moving the specified servo by 5% of its full range every 2 seconds
void test_move_servo(enum motor_num motor) {
    static float set_percent = 0;
    static bool direction = 0;

    hal_set_motor_duty_cycle(motor, set_percent);

    if (set_percent > 95) {
        // We are at the top range - count down
        direction = 1;
    } else if (set_percent < 5) {
        // We are at the bottom range - count up
        direction = 0;
    }

    if (direction == 0) {
        // Count up mode
        set_percent += 5;
    } else {
        // Count down mode
        set_percent -= 5;
    }
    hal_delay_ms(2000);
}

// Test moving all servos by shifting each one by 5% of its full range every 8
// seconds
void test_move_all_servos() {
    static float set_percent = 0;
    static float set_percent_increment = 0.1f;

    // Make sure the increment is <= 10% per second
    if (set_percent_increment > 10) {
        return;
    }
    // Set the delay so that only 10% of the full range is moved per second
    uint32_t step_delay = (uint32_t)(1000.0f * set_percent_increment / 10.0f);

    // Force a delay if the delay is zeroed out
    if (step_delay == 0) step_delay = 1;

    for (set_percent = 0; set_percent < 100; set_percent += set_percent_increment) {
        // Move all motors to the noted duty cycle
        for (enum motor_num motor = MOTOR1; motor <= MOTOR8; motor++) {
            hal_set_motor_duty_cycle(motor, set_percent);
        }
        hal_delay_ms(step_delay);
    }

    for (set_percent = 100; set_percent > 0; set_percent -= set_percent_increment) {
        // Move all motors to the noted duty cycle
        for (enum motor_num motor = MOTOR1; motor <= MOTOR8; motor++) {
            hal_set_motor_duty_cycle(motor, set_percent);
        }
        hal_delay_ms(step_delay);
    }
}

// Test toggling all LEDs off and on
void test_led_toggle() {
    // Turn on all LEDs
    hal_control_spi_led(ENABLE);
    hal_control_i2c_led(ENABLE);
    hal_control_general_led1(ENABLE);
    hal_control_general_led2(ENABLE);
    hal_control_general_led3(ENABLE);
    hal_delay_ms(1000);

    // Turn off all LEDs
    hal_control_spi_led(DISABLE);
    hal_control_i2c_led(DISABLE);
    hal_control_general_led1(DISABLE);
    hal_control_general_led2(DISABLE);
    hal_control_general_led3(DISABLE);
    hal_delay_ms(1000);
}

void test_move_to_points() {
    for (size_t i = 0; i < (sizeof(test_points) / sizeof(test_points[0])); i++) {
        // Set the next target position and move towards it
        if (position_control_set_target_position(test_points[i])) {
            // Wait until movement to the point completes
            while (!position_control_check_angles())
                ;
            hal_control_i2c_led(1);
            hal_delay_ms(300);
            hal_control_i2c_led(0);
            hal_delay_ms(1000);
        } else {
            // Do a short LED blink to signal that the position failed
            hal_control_spi_led(1);
            hal_delay_ms(300);
            hal_control_spi_led(0);
            hal_delay_ms(1000);
        }
    }
}

void test_ik(){
    float test_angles[NUM_JOINTS];
    for (size_t i = 0; i < (sizeof(test_points) / sizeof(test_points[0])); i++) {
        if(position_control_fabrik(test_points[i],test_angles)){
            test_success[i]++;
        }
        else{
            test_fail[i]++;
        }
    }
}

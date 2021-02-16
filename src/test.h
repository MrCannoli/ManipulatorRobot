#pragma once

#include "hal.h"

extern volatile uint32_t temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9;

void test_move_servo(enum motor_num motor);
void test_move_all_servos(void);
void test_led_toggle(void);
void test_move_to_points(void);
void test_ik(void);

#include "position_control.h"
#include "hal.h"

// For now this is defined in the C code, but eventually should be moved to a "settings" section of ROM
enum robot_type robot_type = ROBOT_TYPE_V1;

float position_control_get_angle_from_duty_cycle(enum motor_num motor){
    // Need to confirm accuracy of this!!
    return (float)((270.0 / 360.0) * motor_duty_cycle[motor-1]);
}

void position_control_set_motor_angle(enum motor_num motor, float angle){
    // Need to confirm accuracy of this!!
	float duty_cycle = (angle / 270.0) * 100;
	hal_set_motor_duty_cycle(motor, duty_cycle);
}

void position_control_get_current_position(){
    switch(robot_type){
        case ROBOT_TYPE_V1:
            // Look up math from robotics course to do this!

        	// First rotation is the base (motor 1)

        	// Second rotation is motor 2 at offset ROBOT_ARM_A_OFFSET

        	// Third rotation is motor 3 at offset ROBOT_ARM_A_LEN - ROBOT_ARM_A_B_TRANSITION_OFFSET from the second rotation

        	// Fourth rotation is motor 4 at offset ROBOT_ARM_B_LEN - ROBOT_ARM_B_C_TRANSITION_OFFSET from the third rotation

        	// Fifth (wrist) rotation is motor 5 at offset ROBOT_ARM_C_LEN - ROBOT_ARM_C_WRIST_TRANSITION from the fourth rotation


            break;
        default:
            LOGIC_FAULT();
            break;
    }
}

void position_control_set_goal_position(){

}

void position_control_move_to_goal_position(){
    // Need to set a maximum speed - enforce by systick waits?
}

void position_control_move_to_start(){

}

enum robot_type{
	// Robot Type V1 = Rotary base, vertical rotation, vertical rotation, vertical rotation, grasper
	ROBOT_TYPE_V1 = 0,
};

// Arm lengths and offsets in centimeters
#define ROBOT_ARM_A_OFFSET 3 // Not correct, needs to be updated
#define ROBOT_ARM_A_LEN 30 // Not correct, needs to be updated
#define ROBOT_ARM_A_B_TRANSITION_OFFSET 3 // Not correct, needs to be updated
#define ROBOT_ARM_B_LEN 30 // Not correct, needs to be updated
#define ROBOT_ARM_B_C_TRANSITION_OFFSET 3 // Not correct, needs to be updated
#define ROBOT_ARM_C_LEN 30 // Not correct, needs to be updated
#define ROBOT_ARM_C_WRIST_TRANSITION OFFSET 3 // Not correct, needs to be updated
#define ROBOT_WRIST_LEN 5 // Not correct, needs to be updated

extern enum robot_type robot_type;

void position_control_get_current_position();
void position_control_set_goal_position();
void position_control_move_to_goal_position();
void position_control_move_to_start();

#include "hal.h"
#include "test.h"
#include "position_control.h"

int main() {
    hal_init();
    position_control_move_to_start();

    // Test loop
    while (1) {
        // test_led_toggle();
        // test_move_all_servos();
        test_move_to_points();
    }
}

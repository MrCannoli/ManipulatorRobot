#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"
#include "position_control.h"

// Want value on the prescaler such that
// Tfreq = 50Hz -> Tperiod = 1/50 = 20ms
// Choose period value closest to 65535 to maximize fine control of the waveform
// Prescaler value less than 65535

// Frequency = (Fclk / prescaler) / Period Length
// 50Hz = (42MHZ / Prescaler) / 65535
// Prescaler = 12.82 -> 13, which shifts the period to 64615

// Need to multiply APB1 frequency *2 due to APB1 clock doubling, -1 due to the
// register adding 1.
#define TIMER_PRESCALER (uint32_t)(13 * 2 - 1)
// Set the period length to be at 50Hz
#define TIMER_PERIOD_LENGTH (uint32_t)(64615)

// Counter which increments once every millisecond
static uint32_t systick_counter;

float motor_duty_cycle[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void hal_init(void) {
    // Enable clocks
    // rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOH);
    // rcc_periph_clock_enable(RCC_I2C1); // May not be right I2C, check this
    rcc_periph_clock_enable(RCC_OTGFS);
    // rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM4);
    rcc_periph_clock_enable(RCC_TIM9);

    // Perform clock setup (Fastest available for this chip: 84MHz HSI PLL)
    rcc_clock_setup_pll(&rcc_hsi_configs[0]);

    // Enable Systick
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(rcc_ahb_frequency / 1000); // Tick every 1ms
    systick_interrupt_enable();
    systick_counter_enable();

    // Future feature: Enable USB first and wait on communication from a host
    // computer This will allow us to make sure a robot is set back to the
    // starting position without jumping during initialization It also will
    // provide the host computer with full control of robot operation

    // Motor control enable pin
    // PH0
    gpio_mode_setup(GPIOH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_set_output_options(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0);
    hal_motor_enable();
    // hal_motor_disable();

    // LED Pin Setup
    // SPI = PC11, I2C = PC12, General1-3 = PC13-15
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

    // Set up TIM3, TIM4 for motor PWM generation
    // Timers need to support different duty cycles of a 50Hz PWM
    // TIM3 CH1-4 = PC6-9
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7 | GPIO8 | GPIO9);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6 | GPIO7 | GPIO8 | GPIO9);
    gpio_set_af(GPIOC, GPIO_AF2, GPIO6 | GPIO7 | GPIO8 | GPIO9);

    // TIM4 CH1-4 = PB6-9
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7 | GPIO8 | GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6 | GPIO7 | GPIO8 | GPIO9);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO6 | GPIO7 | GPIO8 | GPIO9);

    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, TIMER_PRESCALER);
    timer_set_repetition_counter(TIM3, 0);
    timer_set_period(TIM3, TIMER_PERIOD_LENGTH); // 50Hz Period

    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, TIMER_PRESCALER);
    timer_set_repetition_counter(TIM4, 0);
    timer_set_period(TIM4, TIMER_PERIOD_LENGTH); // 50Hz Period

    /* -- Output Capture Config (Duty cycle for PWM) -- */
    /* Disable outputs. */
    timer_disable_oc_output(TIM3, TIM_OC1);
    timer_disable_oc_output(TIM3, TIM_OC2);
    timer_disable_oc_output(TIM3, TIM_OC3);
    timer_disable_oc_output(TIM3, TIM_OC4);
    timer_disable_oc_output(TIM4, TIM_OC1);
    timer_disable_oc_output(TIM4, TIM_OC2);
    timer_disable_oc_output(TIM4, TIM_OC3);
    timer_disable_oc_output(TIM4, TIM_OC4);

    /* Configure global mode of the output capture lines. */
    timer_disable_oc_clear(TIM3, TIM_OC1);
    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_set_oc_slow_mode(TIM3, TIM_OC1);
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM3, TIM_OC2);
    timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_set_oc_slow_mode(TIM3, TIM_OC2);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM3, TIM_OC3);
    timer_enable_oc_preload(TIM3, TIM_OC3);
    timer_set_oc_slow_mode(TIM3, TIM_OC3);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM3, TIM_OC4);
    timer_enable_oc_preload(TIM3, TIM_OC4);
    timer_set_oc_slow_mode(TIM3, TIM_OC4);
    timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM4, TIM_OC1);
    timer_enable_oc_preload(TIM4, TIM_OC1);
    timer_set_oc_slow_mode(TIM4, TIM_OC1);
    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM4, TIM_OC2);
    timer_enable_oc_preload(TIM4, TIM_OC2);
    timer_set_oc_slow_mode(TIM4, TIM_OC2);
    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM4, TIM_OC3);
    timer_enable_oc_preload(TIM4, TIM_OC3);
    timer_set_oc_slow_mode(TIM4, TIM_OC3);
    timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM4, TIM_OC4);
    timer_enable_oc_preload(TIM4, TIM_OC4);
    timer_set_oc_slow_mode(TIM4, TIM_OC4);
    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);

    // Set the PWM polarity active high
    timer_set_oc_polarity_high(TIM3, TIM_OC1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);
    timer_set_oc_polarity_high(TIM3, TIM_OC3);
    timer_set_oc_polarity_high(TIM3, TIM_OC4);
    timer_set_oc_polarity_high(TIM4, TIM_OC1);
    timer_set_oc_polarity_high(TIM4, TIM_OC2);
    timer_set_oc_polarity_high(TIM4, TIM_OC3);
    timer_set_oc_polarity_high(TIM4, TIM_OC4);

    // Set Output Capture value (duty cycle) to be 0 to initialize all servos to
    // their base position
    timer_set_oc_value(TIM3, TIM_OC1, 0);
    timer_set_oc_value(TIM3, TIM_OC2, 0);
    timer_set_oc_value(TIM3, TIM_OC3, 0);
    timer_set_oc_value(TIM3, TIM_OC4, 0);
    timer_set_oc_value(TIM4, TIM_OC1, 0);
    timer_set_oc_value(TIM4, TIM_OC2, 0);
    timer_set_oc_value(TIM4, TIM_OC3, 0);
    timer_set_oc_value(TIM4, TIM_OC4, 0);

    /* Reenable outputs. */
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_enable_oc_output(TIM3, TIM_OC2);
    timer_enable_oc_output(TIM3, TIM_OC3);
    timer_enable_oc_output(TIM3, TIM_OC4);
    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_oc_output(TIM4, TIM_OC2);
    timer_enable_oc_output(TIM4, TIM_OC3);
    timer_enable_oc_output(TIM4, TIM_OC4);

    /* ARR reload enable. */
    timer_enable_preload(TIM3);
    timer_enable_preload(TIM4);

    /* Counter enable. */
    timer_enable_counter(TIM3);
    timer_enable_counter(TIM4);

    // Robot movement timer
    timer_set_mode(TIM9, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM9, 2 - 1); // Prescale by 2 to remove clock doubling
    timer_set_repetition_counter(TIM9, 0);
    timer_set_period(TIM9, rcc_apb2_frequency / STEPS_PER_SECOND); // Time step every millisecond

    // If more interrupts are added, need to check and confirm priority setting
    // works correctly
    nvic_set_priority(NVIC_TIM1_BRK_TIM9_IRQ, 3);
    // nvic_enable_irq(NVIC_TIM1_BRK_TIM9_IRQ);

    timer_enable_counter(TIM9);

    // Set up USB
    // ID = PA10, DataM = PA11, DataP = PA12
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11 | GPIO12);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO10 | GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO10 | GPIO11 | GPIO12);
    // TODO: More USB peripheral and interrupt setup

    /* Currently unused but may be used peripherals:
    // SPI2 Setup (TBD)
    // NSS = PB12, SCLK = PB13, MISO = PB14, MOSI = PB15

    // I2C Setup (TBD)
    // SDA = PB3, SCL = PB10

    // ADC1 Setup (TBD)
    // ADC1 IN0-3 = PA0-3
    */
    return;
}

///////////////////// Motor Control Functions /////////////////////

// Set the duty cycle (0 - 100%) for the specified motor
void hal_set_motor_duty_cycle(enum motor_num motor, float duty_cycle) {
    // Limit the duty cycle to 0-100%
    if (duty_cycle > 100) {
        duty_cycle = 100;
    } else if (duty_cycle < 1) {
        duty_cycle = 0;
    }

    // Control of the motor is limited to 1-2ms of the 20ms pulse
    const uint32_t timer_offset = (uint32_t)(TIMER_PERIOD_LENGTH / 20);
    // Divide by 15 here because it gives better control in testing
    uint32_t timer_duty_cycle_val = timer_offset + (uint32_t)((float)TIMER_PERIOD_LENGTH * duty_cycle / 100.0 / 15.0);

    switch (motor) {
        case MOTOR1:
            timer_disable_oc_output(TIM3, TIM_OC1);
            timer_set_oc_value(TIM3, TIM_OC1, timer_duty_cycle_val);
            timer_enable_oc_output(TIM3, TIM_OC1);
            motor_duty_cycle[0] = duty_cycle;
            break;
        case MOTOR2:
            timer_disable_oc_output(TIM3, TIM_OC2);
            timer_set_oc_value(TIM3, TIM_OC2, timer_duty_cycle_val);
            timer_enable_oc_output(TIM3, TIM_OC2);
            motor_duty_cycle[1] = duty_cycle;
            break;
        case MOTOR3:
            timer_disable_oc_output(TIM3, TIM_OC3);
            timer_set_oc_value(TIM3, TIM_OC3, timer_duty_cycle_val);
            timer_enable_oc_output(TIM3, TIM_OC3);
            motor_duty_cycle[2] = duty_cycle;
            break;
        case MOTOR4:
            timer_disable_oc_output(TIM3, TIM_OC4);
            timer_set_oc_value(TIM3, TIM_OC4, timer_duty_cycle_val);
            timer_enable_oc_output(TIM3, TIM_OC4);
            motor_duty_cycle[3] = duty_cycle;
            break;
        case MOTOR5:
            timer_disable_oc_output(TIM4, TIM_OC1);
            timer_set_oc_value(TIM4, TIM_OC1, timer_duty_cycle_val);
            timer_enable_oc_output(TIM4, TIM_OC1);
            motor_duty_cycle[4] = duty_cycle;
            break;
        case MOTOR6:
            timer_disable_oc_output(TIM4, TIM_OC2);
            timer_set_oc_value(TIM4, TIM_OC2, timer_duty_cycle_val);
            timer_enable_oc_output(TIM4, TIM_OC2);
            motor_duty_cycle[5] = duty_cycle;
            break;
        case MOTOR7:
            timer_disable_oc_output(TIM4, TIM_OC3);
            timer_set_oc_value(TIM4, TIM_OC3, timer_duty_cycle_val);
            timer_enable_oc_output(TIM4, TIM_OC3);
            motor_duty_cycle[6] = duty_cycle;
            break;
        case MOTOR8:
            timer_disable_oc_output(TIM4, TIM_OC4);
            timer_set_oc_value(TIM4, TIM_OC4, timer_duty_cycle_val);
            timer_enable_oc_output(TIM4, TIM_OC4);
            motor_duty_cycle[7] = duty_cycle;
            break;
        default:
            // Should not happen!!
            logic_fault();
            break;
    }
}

// Enable PWM control of the motors
void hal_motor_enable(void) { gpio_set(GPIOH, GPIO0); }

// Disable PWM control of the motors
void hal_motor_disable(void) { gpio_clear(GPIOH, GPIO0); }

///////////////////// LED Control Functions /////////////////////
void hal_control_spi_led(bool enable) {
    if (enable) {
        gpio_set(GPIOC, GPIO11);
    } else {
        gpio_clear(GPIOC, GPIO11);
    }
}

void hal_control_i2c_led(bool enable) {
    if (enable) {
        gpio_set(GPIOC, GPIO12);
    } else {
        gpio_clear(GPIOC, GPIO12);
    }
}

void hal_control_general_led1(bool enable) {
    if (enable) {
        gpio_set(GPIOC, GPIO13);
    } else {
        gpio_clear(GPIOC, GPIO13);
    }
}

void hal_toggle_general_led1(){
    gpio_toggle(GPIOC, GPIO13);
}

void hal_control_general_led2(bool enable) {
    if (enable) {
        gpio_set(GPIOC, GPIO14);
    } else {
        gpio_clear(GPIOC, GPIO14);
    }
}

// Note: LED3 is toggled every half second by the systick handler
void hal_control_general_led3(bool enable) {
    if (enable) {
        gpio_set(GPIOC, GPIO15);
    } else {
        gpio_clear(GPIOC, GPIO15);
    }
}

// Return the current systick counter value
static uint32_t hal_get_systick_counter(void) {
    hal_compiler_barrier();
    return systick_counter;
}

// Delay the specified number of milliseconds. Simple systick implementation,
// should not be used for high precision delays.
void hal_delay_ms(uint32_t delay) {
    uint32_t end_time_ms = systick_counter + delay;
    while (hal_get_systick_counter() < end_time_ms)
        ;
}

// Forces the compiler to not optimize through the barrier
// Provides a more efficient workaround to using "volatile" prefixes on
// variables touched by interrupts
void hal_compiler_barrier(void) { __asm__ volatile("" : : : "memory"); }

// Enable the movement interrupt
void hal_movement_timer_enable() { nvic_enable_irq(NVIC_TIM1_BRK_TIM9_IRQ); }

// Disable the movement interrupt
void hal_movement_timer_disable() { nvic_disable_irq(NVIC_TIM1_BRK_TIM9_IRQ); }

// Runs every 1ms
void sys_tick_handler(void) {
    hal_compiler_barrier();
    systick_counter++;
}

// Periodic timer used to move the robot joints in steps
// Ticks at 1KHz when active
void tim9_handler(void) {
    hal_compiler_barrier();
    static size_t step_count = 0;

    if (position_control_check_angles()) {
        // We are at our target destination, turn off movement
        hal_movement_timer_disable();
        hal_control_general_led1(0);
    }

    if(step_count == STEPS_PER_SECOND/5){
        // Toggle LED1 at 5Hz while moving
        hal_toggle_general_led1();
        // Reset the step count
        step_count = 0;
    }
    // Attempt to perform a movement step
    if(position_control_step_towards_position()){
        // Stepped successfully
        step_count++;
        hal_control_general_led2(0);
    }
    else{
        // Failed to step towards the target
        hal_control_general_led2(1);
    }
}

// Software logic fault
void __attribute__((noreturn)) logic_fault() {
    hal_compiler_barrier();
    hal_motor_disable(); // Disable motors so we don't have any funky behavior
    // Future feature: store and transmit the current joint angles of the robot
    // arm over USB to the host computer This can be used to return the robot to
    // the initial position without jumping
    while (1) {
        // Blink LED 3 at 1Hz to note the error
        hal_delay_ms(1000);
        hal_control_general_led3(1);
        hal_delay_ms(1000);
        hal_control_general_led3(0);
    }
}

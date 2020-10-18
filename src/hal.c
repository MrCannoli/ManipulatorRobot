#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <stdint.h>
#include <stdbool.h>

// Need to multiply APB1 frequency *2 since the prescaler due to APB1 clock doubling?
#define TIMER_PERIOD_LENGTH (uint32_t)(rcc_apb1_frequency * 2 / 50)

static uint32_t systick_counter;
static uint32_t delay_ms;

float motor_duty_cycle[8] = {0, 0, 0, 0, 0, 0, 0, 0};


void hal_init(){
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

	// Perform clock setup
	rcc_clock_setup_pll(rcc_hsi_configs[2]); // Fastest clock setup

	// Enable Systick
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(rcc_ahb_frequency/1000); // Tick every 1ms
	systick_interrupt_enable();
	systick_counter_enable();

	// Motor control enable pin
	// PH0
	gpio_mode_setup(GPIOH, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
	gpio_set_output_options(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO0);

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
	timer_set_prescaler(TIM3, 0);
	timer_set_repetition_counter(TIM3, 0);
	timer_set_period(TIM3, TIMER_PERIOD_LENGTH); // 50Hz Period

	timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM4, 0);
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
	timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM2);
	timer_disable_oc_clear(TIM3, TIM_OC3);
	timer_enable_oc_preload(TIM3, TIM_OC3);
	timer_set_oc_slow_mode(TIM3, TIM_OC3);
	timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM3);
	timer_disable_oc_clear(TIM3, TIM_OC4);
	timer_enable_oc_preload(TIM3, TIM_OC4);
	timer_set_oc_slow_mode(TIM3, TIM_OC4);
	timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM4);
	timer_disable_oc_clear(TIM4, TIM_OC1);
	timer_enable_oc_preload(TIM4, TIM_OC1);
	timer_set_oc_slow_mode(TIM4, TIM_OC1);
	timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
	timer_disable_oc_clear(TIM4, TIM_OC2);
	timer_enable_oc_preload(TIM4, TIM_OC2);
	timer_set_oc_slow_mode(TIM4, TIM_OC2);
	timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM2);
	timer_disable_oc_clear(TIM4, TIM_OC3);
	timer_enable_oc_preload(TIM4, TIM_OC3);
	timer_set_oc_slow_mode(TIM4, TIM_OC3);
	timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM3);
	timer_disable_oc_clear(TIM4, TIM_OC4);
	timer_enable_oc_preload(TIM4, TIM_OC4);
	timer_set_oc_slow_mode(TIM4, TIM_OC4);
	timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM4);

	// Set the PWM polarity active high
	timer_set_oc_polarity_high(TIM3, TIM_OC1);
	timer_set_oc_polarity_high(TIM3, TIM_OC2);
	timer_set_oc_polarity_high(TIM3, TIM_OC3);
	timer_set_oc_polarity_high(TIM3, TIM_OC4);
	timer_set_oc_polarity_high(TIM4, TIM_OC1);
	timer_set_oc_polarity_high(TIM4, TIM_OC2);
	timer_set_oc_polarity_high(TIM4, TIM_OC3);
	timer_set_oc_polarity_high(TIM4, TIM_OC4);

	// Set Output Capture value (duty cycle) to be 0 (Duty Cycle = 0%?)
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

	// Set up USB
	// ID = PA10, DataM = PA11, DataP = PA12
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11 | GPIO12);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO10 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO10 | GPIO11 | GPIO12);
	// TODO: More USB peripheral setup

	/* Currently unused but may be used peripherals:
	// SPI2 Setup (TBD)
	// NSS = PB12, SCLK = PB13, MISO = PB14, MOSI = PB15

	// I2C Setup (TBD)
	// SDA = PB3, SCL = PB10

	// ADC1 Setup
	// ADC1 IN0-3 = PA0-3
	*/
	return;
}

// Set the duty cycle (0 - 100%) for the specified motor
void hal_set_motor_duty_cycle(enum motor_num motor, float duty_cycle){
	// Limit the duty cycle to 0-100%
	if(duty_cycle > 100){
		duty_cycle = 100;
	}
	else if(duty_cycle < 1){
		duty_cycle = 0;
	}

	uint32_t timer_duty_cycle_val = (uint32_t)(TIMER_PERIOD_LENGTH * duty_cycle / 100);
	switch(motor){
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
			LOGIC_FAULT();
			break;
	}
}


///////////// LED Control Functions /////////////////////
void hal_control_spi_led(bool enable){
	if (enable){
		gpio_set(GPIOC, GPIO11);
	}
	else{
		gpio_clear(GPIOC, GPIO11);
	}
}

void hal_control_i2c_led(bool enable){
	if (enable){
		gpio_set(GPIOC, GPIO12);
	}
	else{
		gpio_clear(GPIOC, GPIO12);
	}
}

void hal_control_general_led1(bool enable){
	if (enable){
		gpio_set(GPIOC, GPIO13);
	}
	else{
		gpio_clear(GPIOC, GPIO13);
	}
}

void hal_control_general_led2(bool enable){
	if (enable){
		gpio_set(GPIOC, GPIO14);
	}
	else{
		gpio_clear(GPIOC, GPIO14);
	}
}

// Note: LED3 is toggled every half second by the systick handler
void hal_control_general_led3(bool enable){
	if (enable){
		gpio_set(GPIOC, GPIO15);
	}
	else{
		gpio_clear(GPIOC, GPIO15);
	}
}

// Delay the specified number of milliseconds. Simple systick implementation, should not be used for complicated delays.
void hal_delay_ms(uint32_t delay){
	delay_ms = delay;
	while(delay_ms);
}

// Runs every 1ms
void sys_tick_handler(void){
	if(delay_ms){
		delay_ms--;
	}

	systick_counter++;
	if(systick_counter > 500){
		systick_counter = 0;
		// Toggle general LED3 every half second
		gpio_toggle(GPIOC, GPIO15);
	}
}

// Software logic fault
void __attribute__((noreturn)) LOGIC_FAULT(){
	while(1){
		__asm__("nop");
	}
}

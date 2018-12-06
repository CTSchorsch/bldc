#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout

#define SPEED_STEP	10
#define SPEED_MAX	100
#define SPEED_MIN	10
#define SPEED_OFF	0

//Trigger and Speed Hall Pins
#define HW_HALL_TRIGGER_GPIO            GPIOB
#define HW_HALL_TRIGGER_PIN             5
#define HW_HALL_ROTARY_A_GPIO           GPIOA
#define HW_HALL_ROTARY_A_PIN            5
#define HW_HALL_ROTARY_B_GPIO           GPIOA
#define HW_HALL_ROTARY_B_PIN            6
#define HW_HALL_ROTARY_A_EXTI_PORTSRC   EXTI_PortSourceGPIOA
#define HW_HALL_ROTARY_A_EXTI_PINSRC    EXTI_PinSource5
#define HW_HALL_ROTARY_A_EXTI_CH        EXTI9_5_IRQn
#define HW_HALL_ROTARY_A_EXTI_LINE      EXTI_Line5
#define HW_HALL_ROTARY_A_EXTI_ISR_VEC   EXTI9_5_IRQHandler

//private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static unsigned char targetSpeed=SPEED_MIN;

//Threads
static THD_FUNCTION(dpv_thread, arg);
static THD_WORKING_AREA(dpv_thread_wa, 2048); // 2kb stack for this thread

//private functions
void dpv_rotary_isr(void);

CH_IRQ_HANDLER(HW_HALL_ROTARY_A_EXTI_ISR_VEC) {
        if (EXTI_GetITStatus(HW_HALL_ROTARY_A_EXTI_LINE) != RESET) {

                dpv_rotary_isr();
                // Clear the EXTI line pending bit
                EXTI_ClearITPendingBit(HW_HALL_ROTARY_A_EXTI_LINE);
        }
}


void dpv_rotary_isr(void) {

	if (! palReadPad(HW_HALL_ROTARY_B_GPIO, HW_HALL_ROTARY_B_PIN) ) {
		targetSpeed += SPEED_STEP;
		if (targetSpeed > SPEED_MAX) targetSpeed = SPEED_MAX;
	} else {
		targetSpeed -= SPEED_STEP;
		if (targetSpeed < SPEED_MIN) targetSpeed = SPEED_MIN;
	}
}

void app_custom_configure(app_configuration *conf)
{

}

void app_custom_stop(void) 
{
        stop_now = true;

        if (is_running) {
		mc_interface_release_motor();
        }

        while (is_running) {
                chThdSleepMilliseconds(1);
        }

}

void app_custom_start(void) {
        EXTI_InitTypeDef   EXTI_InitStructure;

	stop_now = false;

	// Set the UART TX pin as an input with pulldown
	palSetPadMode(HW_HALL_TRIGGER_GPIO, HW_HALL_TRIGGER_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ROTARY_A_GPIO, HW_HALL_ROTARY_A_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ROTARY_B_GPIO, HW_HALL_ROTARY_B_PIN, PAL_MODE_INPUT_PULLUP);

        // Interrupt on HALL ROTARY A Pin
        // Connect EXTI Line to pin
        SYSCFG_EXTILineConfig(HW_HALL_ROTARY_A_EXTI_PORTSRC, HW_HALL_ROTARY_A_EXTI_PINSRC);

        // Configure EXTI Line
        EXTI_InitStructure.EXTI_Line = HW_HALL_ROTARY_A_EXTI_LINE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        // Enable and set EXTI Line Interrupt to the highest priority
        nvicEnableVector(HW_HALL_ROTARY_A_EXTI_CH, 0);

	// Start the dv thread
	chThdCreateStatic(dpv_thread_wa, sizeof(dpv_thread_wa),
		NORMALPRIO, dpv_thread, NULL);
}

static THD_FUNCTION(dpv_thread, arg) {
	(void)arg;

	chRegSetThreadName("app_dpv");
	static bool first=true;
	static unsigned char motorSpeed=SPEED_MIN;

	for(;;) {

                if (stop_now) {
                        is_running = false;
                        return;
                }

		if ( ! palReadPad(HW_HALL_TRIGGER_GPIO, HW_HALL_TRIGGER_PIN)) {
			if (first) {
				first=false;
			}
			if (motorSpeed != targetSpeed) {
				if (targetSpeed == 0) {
			        	motorSpeed = 0;
			        } else if ( motorSpeed < targetSpeed ) {
					motorSpeed ++;
			        } else if (motorSpeed > targetSpeed ) {
	        			motorSpeed --;
			        }
			}
			mc_interface_set_duty(motorSpeed/100.);

		} else {
			first=true;
			motorSpeed = SPEED_MIN;
			mc_interface_release_motor();
		}

		// Run this loop at 100Hz
		chThdSleepMilliseconds(50);

		// Reset the timeout
		timeout_reset();
	}
}

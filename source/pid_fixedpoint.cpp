
#include "ti_msp_dl_config.h"
#include "fixed_point.hpp"

import pid_fixedpoint;

using fp_t = arm_cmsis_dsp::Q15;
using pid_controller_t = pid_fixedpoint::pid<fp_t>;

volatile bool perform = false;

volatile bool checkADC;
fp_t feedback;
fp_t target_command = fp_t::f(0.66f); // Example target command in Q15 per-unit (0.0 to 1.0)
fp_t output_pid {0}; // Initialize output_pid to 0.0 in Q15 per-unit

pid_controller_t controller(fp_t::f(0.1f), fp_t::f(0.01f), fp_t::f(0.05f)); // Example PID gains for voltage control

void perform_adc() {
    DL_ADC12_startConversion(ADC12_0_INST);
    while (false == checkADC) {
        __WFE();
    }

    // Shift left by 3 bits
    feedback.v = static_cast<int16_t>((DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0)) << 3); // Direct to 0.0 - 1.0 per-unit in Q15!

    checkADC = false;
    DL_ADC12_enableConversions(ADC12_0_INST);
}

void perform_pwm() {
    // Convert Q15 per-unit (0.0 to 1.0 excluded) to raw timer counts (0 to 65535)
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, (static_cast<uint32_t>(output_pid.v) << 1), DL_TIMER_CC_0_INDEX); // update ccr0 value  
}

void perform_pid() {         
    output_pid = controller.compute(target_command, feedback);
}

int main() {
    SYSCFG_DL_init();

    /* timer 5 interrupt ticks per second */ 
    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    checkADC = false;

    /* Start PWM */
    DL_TimerG_startCounter(PWM_0_INST);

    /* Start Timer counting */
    DL_TimerG_startCounter(TIMER_0_INST);
    
    while (1) {
        if (perform) {
            perform = false;
            DL_GPIO_setPins(GPIO_GRP_LEDS_PORT,
                GPIO_GRP_LEDS_PIN_LED_GREEN_PIN);

            perform_adc();
            perform_pid();
            perform_pwm();

            DL_GPIO_clearPins(GPIO_GRP_LEDS_PORT,
                GPIO_GRP_LEDS_PIN_LED_GREEN_PIN);
        }
    }
}

extern "C" {

void TIMER_0_INST_IRQHandler(void) {
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            perform = true;
            break;
        default:
            break;
    }
}

void ADC12_0_INST_IRQHandler(void) {
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            checkADC = true;
            break;
        default:
            break;
    }
}

}

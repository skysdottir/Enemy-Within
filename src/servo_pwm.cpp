#include "servo_pwm.h"

#include <avr/io.h>

// At a 64x divider, 1 tick = 6.4uS
// maximum count = 20ms in ticks
#define MAX_TIMER_COUNT 3125

uint16_t pwm_to_ticks(uint16_t throttle_pct) {
    uint16_t us = 1000 + (throttle_pct * 10);
    
    // convert to ticks- 10mhz (10 cycles/us), 64x divider
    us *= 10;
    us /= 64;
    return MAX_TIMER_COUNT - us;
}

void set_weapon_speed(uint16_t throttle_pct) {
    uint16_t setpt = pwm_to_ticks(throttle_pct);
    TCD0.CMPASET = setpt;
    TCD0.CMPBSET = setpt;

    // and resync the registers at the end of the next cycle
    TCD0.CTRLE = TCD_SYNCEOC_bm;
}

void pwm_init() {
    _PROTECTED_WRITE(TCD0.FAULTCTRL, 0);
    TCD0.CTRLA = 0;
    PORTC.DIRSET = 0x01;

    // set TCD0 into one-ramp mode
    TCD0.CTRLB = 0x00;

    // Set the two additional outputs (WOC and WOD, PC0 and PC1)
    TCD0.CTRLC |= TCD_CMPCSEL_bm;
    _PROTECTED_WRITE(TCD0.FAULTCTRL, TCD_CMPCEN_bm | TCD_CMPDEN_bm);

    uint16_t setpt = pwm_to_ticks(0);

    // Set the initial values to all max_TIMER_count
    TCD0.CMPACLR = MAX_TIMER_COUNT;
    TCD0.CMPBCLR = MAX_TIMER_COUNT;
    TCD0.CMPASET = setpt;
    TCD0.CMPBSET = setpt;

    // and set the dividers and start the counter
    TCD0.CTRLA = TCD_ENABLE_bm |
                    TCD_CLKSEL_SYSCLK_gc |
                    TCD_SYNCPRES_DIV2_gc |
                    TCD_CNTPRES_DIV32_gc;

    // and init the weapon timer to 0 (1000us pulse);
    set_weapon_speed(0);
}

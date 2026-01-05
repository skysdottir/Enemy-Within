#include <stdint.h>

uint16_t pwm_to_ticks(uint16_t throttle_pct);

void set_weapon_speed(uint16_t throttle_pct);

void pwm_init();

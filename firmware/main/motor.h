#pragma once
void motor_init(void);
void motor_set_duty(int motor_idx, float duty); /* duty in [-1.0, 1.0] */
void motor_stop_all(void);

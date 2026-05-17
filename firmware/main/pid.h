#pragma once
typedef struct { float kp,ki,kd,integral,prev_error,out_min,out_max,dt; } pid_t;
void  pid_init(pid_t *p,float kp,float ki,float kd,float dt,float mn,float mx);
void  pid_reset(pid_t *p);
float pid_update(pid_t *p,float setpoint,float measured);

#include "pid.h"
void pid_init(pid_t *p,float kp,float ki,float kd,float dt,float mn,float mx){
    p->kp=kp;p->ki=ki;p->kd=kd;p->dt=dt;p->out_min=mn;p->out_max=mx;
    p->integral=0;p->prev_error=0;
}
void pid_reset(pid_t *p){p->integral=0;p->prev_error=0;}
float pid_update(pid_t *p,float sp,float meas){
    float e=sp-meas, d=(e-p->prev_error)/p->dt;
    p->prev_error=e;
    float out=p->kp*e+p->ki*p->integral+p->kd*d;
    if(out>p->out_min&&out<p->out_max) p->integral+=e*p->dt;
    if(out>p->out_max) out=p->out_max;
    if(out<p->out_min) out=p->out_min;
    return out;
}

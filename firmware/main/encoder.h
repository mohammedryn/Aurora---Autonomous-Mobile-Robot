#pragma once
#include <stdint.h>

#define MOT_FL 0
#define MOT_FR 1
#define MOT_RL 2
#define MOT_RR 3

void encoder_init(void);
void encoder_get_deltas(int32_t deltas[4]); /* thread-safe, resets on read */

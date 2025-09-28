//
// Created by Yevhen Arteshchuk on 05.09.2025.
//

#ifndef RATE_HPP
#define RATE_HPP

#define ONE_SECOND 1.0
#define DEFAULT_PWM_FREQUENCY 290
#define PWM_CYCLE_INTERVAL_US (ONE_SECOND / DEFAULT_PWM_FREQUENCY * 1000000)
#define PWM_12_BIT_RESOLUTION ((2 << 11) - 1) * 1.0
#define PWM_12_BIT_MULTIPLIER (PWM_12_BIT_RESOLUTION / PWM_CYCLE_INTERVAL_US)

typedef struct {
	float roll;
	float pitch;
	float yaw;
} Rate;

typedef struct {
	bool arm;
	int throttle;
	Rate rate;
} ControlState;

#endif //RATE_HPP

/*
 * PWM.h
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void hal_ledpwm(uint32_t a);
//void hal_ledpwm(float duty);

#ifdef __cplusplus
}
#endif

#endif /* INC_PWM_H_ */

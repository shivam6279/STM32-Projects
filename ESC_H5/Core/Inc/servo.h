#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdbool.h>

extern void servoInit(float);
extern void setServo_us(float);
extern void servoOff();
extern void servoBrake();

#endif

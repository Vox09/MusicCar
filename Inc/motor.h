#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define PI 3.1415926
#define MOTOR_TASK_FREQ 40
#define ENCODER_ONE_CIRCLE 695.1f
#define WHEEL_ONE_CIRCLE_CM 9.5
#define WHEELS_DISTANCE 8.4

// LEFT: Encoder tim2, PWM CH1/2
// RIGHT:Encoder tim3,   PWM CH3/4
typedef enum Wheel_LR{
    WHEEL_LEFT,     
    WHEEL_RIGHT     
}Wheel_LR;

typedef struct Encoder{
    uint16_t last;
    uint16_t current;
    int16_t overflow;
}Encoder_t;

typedef struct PID_Controller{
    float kp;
    float ki;
    float kd;
    float iMax;
    float inte;
}PID_Controller_t;

// Public Functions
// positive means forward negative means backward
void carMove(float cm);
// positive means counterclockwise
void carTurn(float deg);
// dir = WHEEL_LEFT/WHEEL_RIGHT
float getWheelAngDeg(Wheel_LR dir);
// dir = WHEEL_LEFT/WHEEL_RIGHT
float getWheelVelDeg(Wheel_LR dir);
// Interupt Handler
void encoderRightOverflowHandler(void);
void encoderLeftOverflowHandler(void);
// PID param seter
void setPIDParam(PID_Controller_t* p, float kp, float ki, float kd, float iMax);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

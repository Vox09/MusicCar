#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#define ARM_MATH_CM3
#include "arm_math.h"

#define MOTOR_TASK_FREQ 10
#define ENCODER_ONE_CIRCLE 11700.0f // TODO!
#define WHEEL_ONE_CIRCLE_CM 18.22f
#define CHESSIS_RADIUS 10.5
#define SQRT3 1.7320508

// LEFT: Encoder tim4, PWM tim3 CH1/2
// RIGHT:Encoder tim5, PWM tim3 CH3/4
// BACK: Encoder tim2, PWM tim1 CH1/4
typedef enum Wheel_LR{
    WHEEL_LEFT,     
    WHEEL_RIGHT,    
    WHEEL_BACK
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
void carMove(float deg, float cm);
// positive means counterclockwise
void carTurn(float deg);
// dir = WHEEL_LEFT/WHEEL_RIGHT/WHEEL_BACK
float getWheelAngDeg(Wheel_LR dir);
// dir = WHEEL_LEFT/WHEEL_RIGHT/WHEEL_BACK
float getWheelVelDeg(Wheel_LR dir);
// Interupt Handler
void encoderBackOverflowHandler(void);
void encoderRightOverflowHandler(void);
void encoderLeftOverflowHandler(void);
// PID param seter
void setPIDParam(PID_Controller_t* p, float kp, float ki, float kd, float iMax);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

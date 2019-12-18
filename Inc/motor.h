#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "arm_math.h"

#define MOTOR_TASK_FREQ 25
#define ENCODER_ONE_CIRCLE 390.0f 
#define WHEEL_ONE_CIRCLE_CM 18.22f
#define CHESSIS_RADIUS 10.5
#define SQRT3 1.7320508



// LEFT: Encoder tim4, PWM tim1 CH1(PA8 ), BACK PA6
// RIGHT:Encoder tim3, PWM tim5 CH2(PA1), BACK PA5
// BACK: Encoder tim2, PWM tim1 CH4(PA11), BACK PB0
typedef enum Wheel_LR{
    WHEEL_LEFT,     
    WHEEL_RIGHT,    
    WHEEL_BACK
}Wheel_LR;

typedef struct Encoder{
    volatile uint16_t last;
    volatile uint16_t current;
    volatile int16_t overflow;
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
// void carMove(float deg, float cm);
void carBrake(void);
void carMoveVel(float deg, float cm_s);
// positive means counterclockwise
// void carTurn(float deg);
void carTurnVel(float deg_s);
// dir = WHEEL_LEFT/WHEEL_RIGHT/WHEEL_BACK
// float getWheelAngDeg(Wheel_LR dir);
// dir = WHEEL_LEFT/WHEEL_RIGHT/WHEEL_BACK
float getWheelVelDeg(Wheel_LR dir);
// Interupt Handler
void encoderBackOverflowHandler(void);
void encoderRightOverflowHandler(void);
void encoderLeftOverflowHandler(void);
// PID param seter
void setPIDParam(PID_Controller_t* p, float kp, float ki, float kd, float iMax);

void servoUp(void);
void servoDown(void);
void carSpeedUp(void);
void carSpeedDown(void);

void setWheelVel(Wheel_LR dir, int16_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

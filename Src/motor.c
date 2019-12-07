#include "tim.h"
#include "motor.h"

// For Debug
// #include "lcd.h"

PID_Controller_t vel_pid[3] = {0};
PID_Controller_t pos_pid[3] = {0};

static Encoder_t encoders[3]={0};
static float pos_cmd[3] = {0};
static void encoderUpdate(void);
static void setWheelVel(Wheel_LR dir, int16_t cmd);
static int16_t velPIDControl(Wheel_LR dir, float target);
static float posPIDControl(Wheel_LR dir);
static void pidDoubleControl(Wheel_LR dir);

void StartMotorTask(void const * argument)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // PB12 and PC10
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // PB12 and PC10
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // PB12 and PC10
    encoders[WHEEL_LEFT].overflow = 0;
    encoders[WHEEL_RIGHT].overflow = 0;
    encoders[WHEEL_BACK].overflow = 0;

    setPIDParam(&vel_pid[WHEEL_LEFT] , 10, 0.1, 10, 10000.0);
    setPIDParam(&vel_pid[WHEEL_RIGHT], 10, 0.1, 10, 10000.0);
    setPIDParam(&vel_pid[WHEEL_BACK], 10, 0.1, 10, 10000.0);
    setPIDParam(&pos_pid[WHEEL_LEFT] , 5, 0.1, 1.0, 300.0);
    setPIDParam(&pos_pid[WHEEL_RIGHT], 5, 0.1, 1.0, 300.0);
    setPIDParam(&pos_pid[WHEEL_BACK], 5, 0.1, 1.0, 300.0);

    // Command should be used outside the loop. e.g.
    // carMove(10.0);
    // carTurn(90.0);
    for(;;)
    {
        encoderUpdate();
        pidDoubleControl(WHEEL_LEFT);
        pidDoubleControl(WHEEL_RIGHT);
        pidDoubleControl(WHEEL_BACK);
        osDelay(1000/MOTOR_TASK_FREQ);
    };
}

void encoderBackOverflowHandler()
{
    (encoders[WHEEL_BACK].last < 0x7fff)?
    (encoders[WHEEL_BACK].overflow -- ):
    (encoders[WHEEL_BACK].overflow ++ );
}

void encoderRightOverflowHandler()
{
    (encoders[WHEEL_RIGHT].last < 0x7fff)?
    (encoders[WHEEL_RIGHT].overflow -- ):
    (encoders[WHEEL_RIGHT].overflow ++ );
}

void encoderLeftOverflowHandler()
{
    (encoders[WHEEL_LEFT].last < 0x7fff)?
    (encoders[WHEEL_LEFT].overflow -- ):
    (encoders[WHEEL_LEFT].overflow ++ );
}

static void encoderUpdate()
{
    encoders[WHEEL_LEFT].last = encoders[WHEEL_LEFT].current;
    encoders[WHEEL_LEFT].current = TIM4->CNT;
    encoders[WHEEL_RIGHT].last = encoders[WHEEL_RIGHT].current;
    encoders[WHEEL_RIGHT].current = TIM5->CNT;
    encoders[WHEEL_BACK].last = encoders[WHEEL_BACK].current;
    encoders[WHEEL_BACK].current = TIM2->CNT;
}

static void setWheelVel(Wheel_LR dir, int16_t cmd)
{
    if(dir==WHEEL_LEFT)
    {
        TIM3->CCR1=(int16_t)(cmd>0?cmd:0);
        TIM3->CCR2=(int16_t)(cmd>0?0:-cmd);
        return;
    }
    if(dir==WHEEL_RIGHT)
    {
        TIM3->CCR3=(uint16_t)(cmd>0?cmd:0);
        TIM3->CCR4=(uint16_t)(cmd>0?0:-cmd);
        return;
    }
    if(dir==WHEEL_BACK)
    {
        TIM1->CCR1=(uint16_t)(cmd>0?cmd:0);
        TIM1->CCR4=(uint16_t)(cmd>0?0:-cmd);
        return;
    }
}

float getWheelAngDeg(Wheel_LR dir)
{
    return (encoders[dir].overflow * 0xffff + encoders[dir].current)
    / ENCODER_ONE_CIRCLE * 360.0f;
}

float getWheelVelDeg(Wheel_LR dir)
{
    return (encoders[dir].current - encoders[dir].last)
    / ENCODER_ONE_CIRCLE * 360.0f * MOTOR_TASK_FREQ;
}

void setPIDParam(PID_Controller_t* p, float kp, float ki, float kd, float iMax)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->iMax = iMax;
}

static int16_t velPIDControl(Wheel_LR dir, float target)
{
    static float last_error[3];
    static float current_error[3];

    last_error[dir] = current_error[dir];
    current_error[dir] = target - getWheelVelDeg(dir);
    vel_pid[dir].inte += current_error[dir];
    vel_pid[dir].inte = vel_pid[dir].inte>vel_pid[dir].iMax?vel_pid[dir].iMax:vel_pid[dir].inte;
    vel_pid[dir].inte = vel_pid[dir].inte<-vel_pid[dir].iMax?-vel_pid[dir].iMax:vel_pid[dir].inte;

    int rtv =  vel_pid[dir].kp * current_error[dir] + 
        vel_pid[dir].ki * vel_pid[dir].inte + 
        vel_pid[dir].kd * (current_error[dir]-last_error[dir]);

    if(rtv > 32767)rtv=32767;
    else if (rtv < -32767)rtv=-32767;
    return rtv;
}

static float posPIDControl(Wheel_LR dir)
{
    static float last_error[3];
    static float current_error[3];

    last_error[dir] = current_error[dir];
    current_error[dir] = pos_cmd[dir] - getWheelAngDeg(dir);
    pos_pid[dir].inte += current_error[dir];
    pos_pid[dir].inte = pos_pid[dir].inte>pos_pid[dir].iMax?pos_pid[dir].iMax:pos_pid[dir].inte;
    pos_pid[dir].inte = pos_pid[dir].inte<-pos_pid[dir].iMax?-pos_pid[dir].iMax:pos_pid[dir].inte;

    return pos_pid[dir].kp * current_error[dir] + 
        pos_pid[dir].ki * pos_pid[dir].inte + 
        pos_pid[dir].kd * (current_error[dir]-last_error[dir]);
}

static void pidDoubleControl(Wheel_LR dir)
{
    setWheelVel(dir,velPIDControl(dir,posPIDControl(dir)));
}

void carMove(float deg, float cm)
{
    encoderUpdate();
    float rad =  deg * PI / 180;
    float back_ang = cm * arm_cos_f32(-rad)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
    float left_ang = cm * arm_cos_f32(2*PI/3-rad)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
    float right_ang = cm * arm_cos_f32(PI/3 -rad)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
    pos_cmd[WHEEL_LEFT] = getWheelAngDeg(WHEEL_LEFT) + left_ang;
    pos_cmd[WHEEL_RIGHT] = getWheelAngDeg(WHEEL_RIGHT) + right_ang;
    pos_cmd[WHEEL_BACK] = getWheelAngDeg(WHEEL_BACK) + back_ang;
}

void carTurn(float deg)
{
    encoderUpdate();
    float ang = (deg * PI / 180 * CHESSIS_RADIUS)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
    pos_cmd[WHEEL_LEFT] = getWheelAngDeg(WHEEL_LEFT) + ang;
    pos_cmd[WHEEL_RIGHT] = getWheelAngDeg(WHEEL_RIGHT) + ang;
    pos_cmd[WHEEL_BACK] = getWheelAngDeg(WHEEL_BACK) + ang;
}

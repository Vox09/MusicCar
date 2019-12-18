#include "tim.h"
#include "motor.h"

// For Debug
#include "lcd.h"
char dbg[8]={0};

PID_Controller_t vel_pid[3] = {0};
PID_Controller_t pos_pid[3] = {0};

static Encoder_t encoders[3]={0};
// static float pos_cmd[3] = {0};
static uint8_t servo_deg = 90;
static float vel_cmd[3] = {0};
uint8_t vel = 30;

static void encoderUpdate(void);
void setWheelVel(Wheel_LR dir, int16_t cmd);
static int16_t velPIDControl(Wheel_LR dir, float target);
// static float posPIDControl(Wheel_LR dir);
// static void pidDoubleControl(Wheel_LR dir);
extern float arm_cos_f32(float);
extern float arm_sin_f32(float);


void StartMotorTask(void const * argument)
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    TIM8->CCR1 = 20000 - 500 - servo_deg * 10;

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); 
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    TIM2->CNT = 0x7fff;
    TIM3->CNT = 0x7fff;
    TIM4->CNT = 0x7fff;

    setPIDParam(&vel_pid[WHEEL_LEFT] , 20, 1, 2, 10000.0);
    setPIDParam(&vel_pid[WHEEL_RIGHT], 20, 1, 2, 10000.0);
    setPIDParam(&vel_pid[WHEEL_BACK] , 20, 1, 2, 10000.0);

    // setPIDParam(&pos_pid[WHEEL_LEFT] , 1, 0.1, 0.01, 300.0);
    // setPIDParam(&pos_pid[WHEEL_RIGHT], 1, 0.1, 0.01, 300.0);
    // setPIDParam(&pos_pid[WHEEL_BACK] , 1, 0.1, 0.01, 300.0);

    // Command should be used outside the loop. e.g.
    // carTurnVel(0.0);
    LCD_DrawString(10,200, "turn speed deg/s: ");
    LCD_DrawString(10,220, "LEFT enc: ");
    LCD_DrawString(10,240, "RIGHT enc: ");
    LCD_DrawString(10,260, "BACK enc: %d ");
    LCD_DrawString(150, 220, "cmd: ");
    LCD_DrawString(150, 240, "cmd: ");
    LCD_DrawString(150, 260, "cmd: ");
    carBrake();
    for(;;)
    {
        encoderUpdate();
        // if(!pid_vel_flag)
        // {
            // pidDoubleControl(WHEEL_LEFT);
            // pidDoubleControl(WHEEL_RIGHT);
            // pidDoubleControl(WHEEL_BACK);
        // }
        // else
        // {
            setWheelVel(WHEEL_LEFT,velPIDControl(WHEEL_LEFT,vel_cmd[WHEEL_LEFT]));
            setWheelVel(WHEEL_RIGHT,velPIDControl(WHEEL_RIGHT,vel_cmd[WHEEL_RIGHT]));
            setWheelVel(WHEEL_BACK,velPIDControl(WHEEL_BACK,vel_cmd[WHEEL_BACK]));
        // }

        sprintf(dbg,"%d ", vel);
        LCD_DrawString(10+18*8,200, dbg);

        sprintf(dbg,"%d ", encoders[WHEEL_LEFT].current-0x7fff);
        LCD_DrawString(10+10*8,220, dbg);

        sprintf(dbg,"%d ", encoders[WHEEL_RIGHT].current-0x7fff);
        LCD_DrawString(10+11*8,240, dbg);

        sprintf(dbg,"%d ", encoders[WHEEL_BACK].current-0x7fff);
        LCD_DrawString(10+10*8,260, dbg);


        osDelay(1000/MOTOR_TASK_FREQ);
    };
}

void encoderBackOverflowHandler()
{
    setWheelVel(WHEEL_BACK,0);
    (encoders[WHEEL_BACK].last < 0x7fff)?
    (encoders[WHEEL_BACK].overflow -- ):
    (encoders[WHEEL_BACK].overflow ++ );
}

void encoderRightOverflowHandler()
{
    setWheelVel(WHEEL_RIGHT,0);
    (encoders[WHEEL_RIGHT].last < 0x7fff)?
    (encoders[WHEEL_RIGHT].overflow -- ):
    (encoders[WHEEL_RIGHT].overflow ++ );
}

void encoderLeftOverflowHandler()
{
    setWheelVel(WHEEL_LEFT,0);
    (encoders[WHEEL_LEFT].last < 0x7fff)?
    (encoders[WHEEL_LEFT].overflow -- ):
    (encoders[WHEEL_LEFT].overflow ++ );
}

static void encoderUpdate()
{
    encoders[WHEEL_LEFT].last = encoders[WHEEL_LEFT].current;
    encoders[WHEEL_LEFT].current = TIM4->CNT;
    encoders[WHEEL_RIGHT].last = encoders[WHEEL_RIGHT].current;
    encoders[WHEEL_RIGHT].current = TIM3->CNT;
    encoders[WHEEL_BACK].last = encoders[WHEEL_BACK].current;
    encoders[WHEEL_BACK].current = TIM2->CNT;
}

void setWheelVel(Wheel_LR dir, int16_t cmd)
{
    if(dir==WHEEL_LEFT)
    {
        TIM1->CCR1=(int16_t)cmd>0?cmd:-cmd;
        HAL_GPIO_WritePin(L_BACK_GPIO_Port, L_BACK_Pin, !(cmd>0));
        sprintf(dbg,"%d ", cmd);
        LCD_DrawString(150+5*8, 220, dbg);
        return;
    }
    if(dir==WHEEL_RIGHT)
    {
        TIM1->CCR4=(int16_t)cmd>0?cmd:-cmd;
        HAL_GPIO_WritePin(R_BACK_GPIO_Port, R_BACK_Pin, !(cmd>0));
        sprintf(dbg,"%d ", cmd);
        LCD_DrawString(150+5*8, 240, dbg);
        return;
    }
    if(dir==WHEEL_BACK)
    {
        TIM5->CCR2=(int16_t)cmd>0?cmd:-cmd;
        HAL_GPIO_WritePin(B_BACK_GPIO_Port, B_BACK_Pin, !(cmd>0));
        sprintf(dbg,"%d ", cmd);
        LCD_DrawString(150+5*8, 260, dbg);
        return;
    }
}

// float getWheelAngDeg(Wheel_LR dir)
// {
//     return (encoders[dir].overflow * 0xffff + encoders[dir].current)
//     / ENCODER_ONE_CIRCLE * 360.0f;
// }

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

// static float posPIDControl(Wheel_LR dir)
// {
//     static float last_error[3];
//     static float current_error[3];

//     last_error[dir] = current_error[dir];
//     current_error[dir] = pos_cmd[dir] - getWheelAngDeg(dir);
//     pos_pid[dir].inte += current_error[dir];
//     pos_pid[dir].inte = pos_pid[dir].inte>pos_pid[dir].iMax?pos_pid[dir].iMax:pos_pid[dir].inte;
//     pos_pid[dir].inte = pos_pid[dir].inte<-pos_pid[dir].iMax?-pos_pid[dir].iMax:pos_pid[dir].inte;

//     return pos_pid[dir].kp * current_error[dir] + 
//         pos_pid[dir].ki * pos_pid[dir].inte + 
//         pos_pid[dir].kd * (current_error[dir]-last_error[dir]);
// }

// static void pidDoubleControl(Wheel_LR dir)
// {
//     setWheelVel(dir,velPIDControl(dir,posPIDControl(dir)));
// }

// void carMove(float deg, float cm)
// {
//     encoderUpdate();
//     float rad =  deg * PI / 180;
//     float cos = arm_cos_f32(rad);
//     float sin = arm_sin_f32(rad);
//     float back_ang = cm * cos / WHEEL_ONE_CIRCLE_CM * 360.0f;
//     float left_ang = cm * (cos/2 + sin*SQRT3/2) / WHEEL_ONE_CIRCLE_CM * 360.0f;
//     float right_ang = cm * (-cos/2 + sin*SQRT3/2)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
//     pos_cmd[WHEEL_LEFT] = getWheelAngDeg(WHEEL_LEFT) + left_ang;
//     pos_cmd[WHEEL_RIGHT] = getWheelAngDeg(WHEEL_RIGHT) + right_ang;
//     pos_cmd[WHEEL_BACK] = getWheelAngDeg(WHEEL_BACK) + back_ang;
// }

// void carTurn(float deg)
// {
//     encoderUpdate();
//     float ang = (deg * PI / 180 * CHESSIS_RADIUS)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
//     pos_cmd[WHEEL_LEFT] = getWheelAngDeg(WHEEL_LEFT) + ang;
//     pos_cmd[WHEEL_RIGHT] = getWheelAngDeg(WHEEL_RIGHT) + ang;
//     pos_cmd[WHEEL_BACK] = getWheelAngDeg(WHEEL_BACK) + ang;
// }

void carMoveVel(float deg, float cm_s)
{
    encoderUpdate();
    float rad =  deg * PI / 180;
    float cos = arm_cos_f32(rad);
    float sin = arm_sin_f32(rad);
    float back_ang = cm_s * cos / WHEEL_ONE_CIRCLE_CM * 360.0f;
    float left_ang = cm_s * (-cos/2 - sin*SQRT3/2) / WHEEL_ONE_CIRCLE_CM * 360.0f;
    float right_ang = cm_s * (-cos/2 + sin*SQRT3/2)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
    vel_cmd[WHEEL_LEFT] = left_ang;
    vel_cmd[WHEEL_RIGHT] = right_ang;
    vel_cmd[WHEEL_BACK] = back_ang;
}

void carTurnVel(float deg_s)
{
    encoderUpdate();
    float ang = (deg_s * PI / 180 * CHESSIS_RADIUS)/ WHEEL_ONE_CIRCLE_CM * 360.0f;
    vel_cmd[WHEEL_LEFT] = ang;
    vel_cmd[WHEEL_RIGHT] = ang;
    vel_cmd[WHEEL_BACK] = ang;
}

void carBrake()
{
    encoderUpdate();
    vel_cmd[WHEEL_LEFT] = 0.0;
    vel_cmd[WHEEL_RIGHT] = 0.0;
    vel_cmd[WHEEL_BACK] = 0.0;
}

void servoUp()
{
    if(servo_deg != 105)
    TIM8->CCR1 = (20000-500 - (++servo_deg) * 10);
}

void servoDown()
{
    if(servo_deg != 0)
    TIM8->CCR1 = (20000-500 - (--servo_deg) * 10);
}

void carSpeedUp() {vel = vel<125?vel+5:vel;}
void carSpeedDown() {vel = vel>5?vel-5:vel;}


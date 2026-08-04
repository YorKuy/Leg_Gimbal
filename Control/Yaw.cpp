#include "Yaw.h"
#include "RM_Lib.h"
#include "SMC.h"
#include "RM.h"
#include "PID.h"
#include "DM.h"


#define YAW_ST 6600
#define ZERO_HEAD 1.564F
#define ZERO_BACK -1.5776F
#define PI 3.1415926F
#define RAD_TO_DEG 180.0f / PI
#define DEG_TO_RAD PI / 180.0f
#define YAW_ENCODER_INPUT_SCALE 5.0F
UpDown_check_class UD_Yaw_Back(0),UD_Yaw_StandUp(0);
SMC Yaw(20,30, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Encoder(100, 105, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Zm(25, 33, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Back(15, 50, 0, 0.001, 25000, 0.9, 1, 1),
    Yaw_Stand(15, 50, 0, 0.001, 25000, 0.9, 1, 1);
static YAW yaw_instance;
YAW *yaw = &yaw_instance;
extern MOTOR_RM M6020_YAW;
extern MOTOR_DM DM_YAW;
extern float Zm_Yaw_Vel, Zm_Yaw_Acc;
static const YAW::StateFunc kYawStateFuncTable[static_cast<u8>(YAW::State::COUNT)] = {
    &YAW::stateNORMAL,
    &YAW::stateBACKING,
    &YAW::stateSTANDUP,
    &YAW::stateENCODER,
    &YAW::stateAUTO_ZM,
    &YAW::stateZERO,
};

static float max_short_mang(float target, float current)
{
    float delta = target - current;
    while (delta > PI)
        delta -= 2.0f * PI;
    while (delta < -PI)
        delta += 2.0f * PI;
    return delta;
}
//转力矩
float gm6020to_torq(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
//Yaw轴模式切换
void YAW::yaw_mode_deal(u8 GIMBAL_088_State,u8 YK_Mode,u8 zm_request)
{
    if (GIMBAL_088_State == BMI088_OK &&
        (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || YK_Mode == SHOOT_MODE ||
         YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) &&
        !zm_request)
    {
        currentMode = Mode::GYRO;
        YAW_Mode = GYRO_MODE;
    }
    else if (GIMBAL_088_State != BMI088_OK &&
             (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || YK_Mode == SHOOT_MODE ||
              YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) &&
             !zm_request)
    {
        currentMode = Mode::ENCODER;
        YAW_Mode = ENCODER_MODE;
    }
    else if (zm_request)
    {
        currentMode = Mode::AUTO;
        YAW_Mode = AUTO_MODE;
    }
    else
    {
        currentMode = Mode::PROTECT;
        YAW_Mode = PROTECT_MODE;
    }
}

void YAW::set_SMCref(f Target_Angle)
{
    Yaw.ref = Target_Angle;
    Yaw_Zm.ref = Target_Angle;
    Yaw_Back.ref = Target_Angle;
    Yaw_Stand.ref = Target_Angle;
}
void YAW::set_Yaw_Angle(f Target_Angle)
{
    this->Target_Angle = Target_Angle;
}

YAW::StateFunc YAW::stateFuncOf(State state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kYawStateFuncTable) / sizeof(kYawStateFuncTable[0])))
    {
        return &YAW::stateZERO;
    }
    return kYawStateFuncTable[index];
}

void YAW::switchState(State newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &YAW::stateZERO && newState != State::ZERO)
    {
        currentState = State::ZERO;
    }
}

f YAW::runCurrentState(u8 jianshu_flag)
{
    if (currentStateFunc == nullptr)
    {
        switchState(State::ZERO);
    }
    return (this->*currentStateFunc)(jianshu_flag);
}

void YAW::updateEncoderMang(f mang)
{
    if (!Encoder_First)
    {
        Last_Encoder_Mang = mang;
        Encoder_Mang = mang;
        Encoder_First = 1;
        return;
    }

    const f delta = mang - Last_Encoder_Mang;
    if (delta < -PI)
        Encoder_Mang_Offset += 2.0f * PI;
    else if (delta > PI)
        Encoder_Mang_Offset -= 2.0f * PI;

    Encoder_Mang = mang + Encoder_Mang_Offset;
    Last_Encoder_Mang = mang;
}

float YAW::stateNORMAL(u8 jianshu_flag)
{
    const f yaw_delta = jianshu_flag ?
        (f)LIMIT(YK.shubiao.x, -500, 500) / 1200.0f :
        (f)YK.yaogan.ch2 / 4400.0f;
    Target_Angle -= yaw_delta;

    if (UD_Yaw_Back.updata(YK.Pressed_Check(KEY_PRESSED_R)) == UpDown_check_rising)
    {
        Target_Angle += 180.0f;
        set_SMCref(Target_Angle);
        Communicate_Send_Flag_1 |= (0x0001 << 1);
        Back_Flag = 1;
        StandUp_Flag = 0;
        switchState(State::BACKING);
        return stateBACKING(jianshu_flag);
    }

    if(UD_Yaw_StandUp.updata(YK.Pressed_Check(KEY_PRESSED_X)) == UpDown_check_rising)
    {
        diff_stand_L = max_short_mang(ZERO_HEAD, DM_YAW.mang);
        diff_stand_R = max_short_mang(ZERO_BACK, DM_YAW.mang);

        if (fabsf(diff_stand_L) <= fabsf(diff_stand_R))
        {
            Yaw_Now_Flag = 1;
            Stand_Target_Mang = ZERO_HEAD;
            Target_Angle = GIMBAL_088.realAngle.yaw + diff_stand_L * RAD_TO_DEG;
        }
        else
        {
            Yaw_Now_Flag = 0;
            Stand_Target_Mang = ZERO_BACK;
            Target_Angle = GIMBAL_088.realAngle.yaw + diff_stand_R * RAD_TO_DEG;
        }
        Communicate_Send_Flag_1 |= (0x0001 << 1);
        Back_Flag = 0;
        StandUp_Flag = 1;
        set_SMCref(Target_Angle);
        switchState(State::STANDUP);
        return stateSTANDUP(jianshu_flag);
    }

    Back_Flag = 0;
    set_SMCref(Target_Angle);
    Yaw.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw.u);
    return Yaw_Out;
}
float YAW::stateENCODER(u8 jianshu_flag)
{
    if (!Encoder_Initialized)
    {
        Encoder_Target_Mang = Encoder_Mang;
        Encoder_Initialized = 1;
    }

    const f yaw_delta_deg = jianshu_flag ?
        (f)LIMIT(YK.shubiao.x, -500, 500) / 1200.0f :
        (f)YK.yaogan.ch2 / 4400.0f;
    Encoder_Target_Mang -= yaw_delta_deg * YAW_ENCODER_INPUT_SCALE * DEG_TO_RAD;

    Yaw_Encoder.ref = Encoder_Target_Mang;
    Yaw_Encoder.SMC_Tick(Encoder_Mang, DM_YAW.sp);
    Yaw_Out = gm6020to_torq(Yaw_Encoder.u);
    return Yaw_Out;
}
float YAW::stateAUTO_ZM(u8 jianshu_flag)
{
    (void)jianshu_flag; //消除警告
    set_SMCref(Target_Angle);
    Yaw_Zm.SMC_AngleSpeed(Target_Angle, Zm_Yaw_Vel, Zm_Yaw_Acc, GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw_Zm.u);
    return Yaw_Out;
}

float YAW::stateSTANDUP(u8 jianshu_flag)
{
    (void)jianshu_flag;
    const float motor_error = max_short_mang(Stand_Target_Mang, DM_YAW.mang); 

    if (fabsf(motor_error) < 0.2f &&
        fabsf(DM_YAW.sp) <= 0.2f)
    {
        StandUp_Flag = 0;
        Target_Angle = GIMBAL_088.realAngle.yaw;
        set_SMCref(Target_Angle);
        Communicate_Send_Flag_1 &= ~(0x0001 << 1);
        switchState(State::NORMAL);
        Yaw_Out = 0.0f;
        return Yaw_Out;
    }

    Target_Angle = GIMBAL_088.realAngle.yaw + motor_error * RAD_TO_DEG;
    set_SMCref(Target_Angle);
    Yaw_Stand.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw_Stand.u);
    return Yaw_Out;
}

float YAW::stateBACKING(u8 jianshu_flag)
{
    set_SMCref(Target_Angle);
    if (fabsf(Target_Angle - GIMBAL_088.realAngle.yaw) > 5.0f)
    {
        Yaw_Back.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
        Yaw_Out = gm6020to_torq(Yaw_Back.u);
        return Yaw_Out;
    }

    Communicate_Send_Flag_1 &= ~(0x0001 << 1);
    Back_Flag = 0;
    StandUp_Flag = 0;
    switchState(State::NORMAL);
    return stateNORMAL(jianshu_flag);
}

float YAW::stateZERO(u8 jianshu_flag)
{
    (void)jianshu_flag;
    Target_Angle = GIMBAL_088.realAngle.yaw;
    set_SMCref(Target_Angle);
    Back_Flag = 0;
    StandUp_Flag = 0;
    Communicate_Send_Flag_1 &= ~(0x0001 << 1);
    Yaw_Out = 0.0f;
    return Yaw_Out;
}
float YAW::Yaw_Out_Interface(u8 jianshu_flag)
{
    // 每次 YAW 电机新反馈都更新，保证切入编码器模式时多圈角已连续。
    updateEncoderMang(DM_YAW.mang);

    // 同步全局 YAW_Mode 到状态机
    if (YAW_Mode == GYRO_MODE)
        currentMode = Mode::GYRO;
    else if (YAW_Mode == ENCODER_MODE)
        currentMode = Mode::ENCODER;
    else if (YAW_Mode == AUTO_MODE)
        currentMode = Mode::AUTO;
    else
        currentMode = Mode::PROTECT;

    State nextState = State::ZERO;

    switch (currentMode)
    {
        case Mode::GYRO:
            if (StandUp_Flag)
                nextState = State::STANDUP;
            else
                nextState = Back_Flag ? State::BACKING : State::NORMAL;
            break;
        case Mode::ENCODER:
            Back_Flag = 0;
            StandUp_Flag = 0;
            Communicate_Send_Flag_1 &= ~(0x0001 << 1);
            nextState = State::ENCODER;
            break;
        case Mode::AUTO:
            Back_Flag = 0;
            StandUp_Flag = 0;
            Communicate_Send_Flag_1 &= ~(0x0001 << 1);
            nextState = State::AUTO_ZM;
            break;
        case Mode::PROTECT:
        default:
            nextState = State::ZERO;
            break;
    }
    if (currentMode != Mode::ENCODER)
    {
        Encoder_Initialized = 0;
    }
    if (currentState == State::ENCODER && nextState == State::NORMAL)
    {
        Target_Angle = GIMBAL_088.realAngle.yaw;
        set_SMCref(Target_Angle);
    }
    if (currentState != nextState)
    {
        switchState(nextState);
    }
    return runCurrentState(jianshu_flag);
}

#include "Pitch.h"
#include "DM.h"
#include "PID.h"
#include "SMC.h"

#define PITCH_HIGH   -1.0146F //-1.13636f  
#define PITCH_LOW    -0.00332F //-0.144892f
#define PI 3.1415926F
#define PITCH_LIMIT_MARGIN 0.01F
#define PITCH_GYRO_TARGET_MIN -30.5F
#define PITCH_GYRO_TARGET_MAX 20.5F
#define PITCH_OUTPUT_MIN -9.0F
#define PITCH_OUTPUT_MAX 9.0F
#define RAD PI / 180.0f
SMC         Pitch(45,70,0,0.001,15000,0.9,1,1),
            Pitch_Zm(60, 130, 0, 0.1, 16000, 1, 1, 1);
SMC_PITCH SMC_Pitch(35,46, 2.0f, 0.01f, 20000, 0.8f, 1),
            SMC_Pitch_Encoder(120,135, 10.0f, 0.01f, 20000, 0.8f, 1),
            SMC_Pitch_Zm(38,50, 22.0f, 0.01f, 30000, 0.8f, 1.0f);
static PITCH pitch_instance;
PITCH *pitch = &pitch_instance;
extern MOTOR_DM DM_PITCH;
extern float Zm_Pitch_Vel, Zm_Pitch_Acc;

float gm6020to_torq_pitch(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
// 状态函数表
static const PITCH::StateFunc kPitchStateFuncTable[static_cast<u8>(PITCH::State::COUNT)] = {
    &PITCH::stateNORMAL,
    &PITCH::stateENCODER,
    &PITCH::stateAUTO_ZM,
    &PITCH::stateZERO,
};

PITCH::StateFunc PITCH::stateFuncOf(State state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kPitchStateFuncTable) / sizeof(kPitchStateFuncTable[0])))
    {
        return &PITCH::stateZERO;
    }
    return kPitchStateFuncTable[index];
}

void PITCH::switchState(State newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &PITCH::stateZERO && newState != State::ZERO)
    {
        currentState = State::ZERO;
    }
}

f PITCH::runCurrentState(u8 jianshu_flag)
{
    if (currentStateFunc == nullptr)
    {
        switchState(State::ZERO);
    }
    return (this->*currentStateFunc)(jianshu_flag);
}

// NORMAL: 遥控/键鼠手动控制
f PITCH::stateNORMAL(u8 jianshu_flag)
{
    Angle_buf = jianshu_flag ? (float)(LIMIT(YK.shubiao.y, -400, 400) / 1000.0) : (float)(YK.yaogan.ch3 / 3300.0);
    Target_Angle -= Angle_buf;
    if (DM_PITCH.mang < PITCH_HIGH - PITCH_LIMIT_MARGIN && Angle_buf > 0)
        Target_Angle = Last_Angle;
    if (DM_PITCH.mang > PITCH_LOW + PITCH_LIMIT_MARGIN && Angle_buf < 0)
        Target_Angle = Last_Angle;
    Last_Angle = Target_Angle;
    Target_Angle = LIMIT(Target_Angle, PITCH_GYRO_TARGET_MIN, PITCH_GYRO_TARGET_MAX);
    Pitch.ref = Target_Angle;
    SMC_Pitch.SMC_Tick(Target_Angle, Angle_buf, 0, GIMBAL_088.realAngle.roll, GIMBAL_088.Anglespeed.Deal_roll);
    Pitch_Out = gm6020to_torq_pitch(SMC_Pitch.u);
    Pitch_Out = LIMIT(Pitch_Out, PITCH_OUTPUT_MIN, PITCH_OUTPUT_MAX);
    return Pitch_Out;
}

f PITCH::stateENCODER(u8 jianshu_flag)
{
    if (!Encoder_Initialized)
    {
        Encoder_Target_Mang = LIMIT(DM_PITCH.mang, PITCH_HIGH, PITCH_LOW);
        Encoder_Initialized = 1;
    }

    Angle_buf = jianshu_flag ?(float)(LIMIT(YK.shubiao.y, -400, 400) / 5000.0) :(float)(YK.yaogan.ch3 / 3300.0 * 5.0f);
    const f encoder_delta = Angle_buf * RAD;
    Encoder_Target_Mang -= encoder_delta;
    Encoder_Target_Mang = LIMIT(Encoder_Target_Mang, PITCH_HIGH, PITCH_LOW);

    SMC_Pitch_Encoder.SMC_Tick(Encoder_Target_Mang, -encoder_delta, 0.0f,
                               DM_PITCH.mang, DM_PITCH.sp);
    Pitch_Out = gm6020to_torq_pitch(SMC_Pitch_Encoder.u);
    Pitch_Out = LIMIT(Pitch_Out, PITCH_OUTPUT_MIN, PITCH_OUTPUT_MAX);
    return Pitch_Out;
}

// AUTO_ZM: 自瞄零力矩控制
f PITCH::stateAUTO_ZM(u8 jianshu_flag)
{
    (void)jianshu_flag;
    SMC_Pitch_Zm.SMC_Tick(Target_Angle, Zm_Pitch_Vel, Zm_Pitch_Acc, GIMBAL_088.realAngle.roll, GIMBAL_088.Anglespeed.Deal_roll);
    Pitch_Out = gm6020to_torq_pitch(SMC_Pitch_Zm.u);
    Pitch_Out = LIMIT(Pitch_Out, PITCH_OUTPUT_MIN, PITCH_OUTPUT_MAX);
    return Pitch_Out;
}

// ZERO: 保护模式，输出置零
f PITCH::stateZERO(u8 jianshu_flag)
{
    (void)jianshu_flag; //消警告的
    Target_Angle = GIMBAL_088.realAngle.roll;
    Pitch_Out = 0;
    return Pitch_Out;
}

// 主入口：根据全局 PITCH_Mode 同步 currentMode 并切换 State
f PITCH::Pitch_Out_Interface(u8 jianshu_flag)
{
    // 同步全局 Mode 到状态机
    if (PITCH_Mode == GYRO_MODE)
        currentMode = Mode::GYRO;
    else if (PITCH_Mode == ENCODER_MODE)
        currentMode = Mode::ENCODER;
    else if (PITCH_Mode == AUTO_MODE)
        currentMode = Mode::AUTO;
    else
        currentMode = Mode::PROTECT;

    State nextState = State::ZERO;

    switch (currentMode)
    {
        case Mode::GYRO:
            nextState = State::NORMAL;
            break;
        case Mode::ENCODER:
            nextState = State::ENCODER;
            break;
        case Mode::AUTO:
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
        Target_Angle = GIMBAL_088.realAngle.roll;
        Last_Angle = Target_Angle;
    }
    if (currentState != nextState)
    {
        switchState(nextState);
    }
    return runCurrentState(jianshu_flag);
}

void PITCH::set_Pitch_Target(f Target_Angle)
{
    pitch->Target_Angle = Target_Angle;
}
void PITCH::set_SMCref(f Target_Angle)
{
    Pitch.ref = Target_Angle;
    Pitch_Zm.ref = Target_Angle;
}

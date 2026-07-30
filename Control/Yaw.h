#ifndef __YAW_H__
#define __YAW_H__

#include "main.h"
#include "RM_Lib.h"

extern u8 YAW_Mode;
extern RC YK;
extern u16 Communicate_Send_Flag_1;
extern BMI088	GIMBAL_088;
class YAW
{
    public:      
        f Yaw_Out;        //Yaw最后的输出
        f Target_Angle;   //目标角度
        f Encoder_Mang;   // 由 mang 过圈累加得到的连续角（rad）
        f Encoder_Target_Mang; // 编码器模式的连续目标角（rad）
        u8 Yaw_Now_Flag; // 0: ZERO_BACK, 1: ZERO_HEAD
        float diff_stand_L,diff_stand_R;
        float Stand_Target_Mang;
        void set_SMCref(f Target_Angle);//接口
        void set_Yaw_Angle(f Target_Angle);
    YAW():Yaw_Out(0),Target_Angle(0),Encoder_Mang(0),Encoder_Target_Mang(0),Yaw_Now_Flag(0),diff_stand_L(0),diff_stand_R(0),Stand_Target_Mang(0),Yaw_real(0),Yaw_vel(0),Yaw_acc(0),Back_Flag(0),StandUp_Flag(0),Encoder_Initialized(0),Encoder_First(0),Last_Encoder_Mang(0),Encoder_Mang_Offset(0){}
    enum class State
    {
        NORMAL,
        BACKING,
        STANDUP,
        ENCODER,
        AUTO_ZM,
        ZERO,
        COUNT
    };
    enum class Mode
    {
        PROTECT,
        GYRO,
        ENCODER,
        AUTO
    };
    using StateFunc = f (YAW::*)(u8 jianshu_flag);
    void yaw_mode_deal(u8 GIMBAL_088_State,u8 YK_Mode,u8 zm_request); //输入接口
    f stateNORMAL(u8 jianshu_flag);
    f stateBACKING(u8 jianshu_flag);
    f stateSTANDUP(u8 jianshu_flag);
    f stateENCODER(u8 jianshu_flag);
    f stateAUTO_ZM(u8 jianshu_flag);
    f stateZERO(u8 jianshu_flag);
    Mode  currentMode = Mode::PROTECT;
    State currentState = State::ZERO;
    StateFunc currentStateFunc = &YAW::stateZERO;
    void switchState(State newState);
    f Yaw_Out_Interface(u8 jianshu_flag); //输出接口
    private:
        static StateFunc stateFuncOf(State state);
        f runCurrentState(u8 jianshu_flag);
        void updateEncoderMang(f mang);
        f Yaw_real,Yaw_vel,Yaw_acc;  //陀螺仪实际角度，角速度，角加速度
        u8 Back_Flag;
        u8 StandUp_Flag;
        u8 Encoder_Initialized;
        u8 Encoder_First;
        f Last_Encoder_Mang;
        f Encoder_Mang_Offset;
        
};
#endif

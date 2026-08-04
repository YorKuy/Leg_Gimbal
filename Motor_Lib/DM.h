#ifndef __DM_H__
#define __DM_H__

#include "RM_Lib.h"

////////***********************************  DA MIAO �� �� ********************************//////////
class MOTOR_DM
{ // ������ ,�����ﶨ��Ķ�����Ҫʹ��this����ȡ
public:
    const uint16_t ID; // �������ID
    USER_CAN *can_rev;

    int16_t id;  // �ɴ���Ĵ�����������
    int16_t ERR; // ���������ĵ��������Ϣ��8����ѹ 9��Ƿѹ A�������� B��mos���� C����Ȧ���� D��ͨѶ��ʧ E������
    int p_int;
    int v_int;
    int t_int;
    float mang;    // λ�� 16λ
    float sp;      //   �ٶ�  12λ
    float Torque;  // Ť�� 12λ
    float T_Rotor; // ��ʾ����ڲ���Ȧ��ƽ���¶� ��λ�����϶�
    float T_MOS;   // ��ʾ������ MOS ��ƽ���¶�

    float nsqd_8PI_Cnt_mang; // Ȧ��*����ֵ
    float mang_inf;          // ��Ȧ����ֵ
    uint8_t first = 0;       // ��ʼ��־
    float Last_mang;         // �ϴεĽǶ�ֵ���жϹ�Ȧ��
    int16_t motor_number;    // Ȧ��

    uint32_t motor_send_error_cnt = 0;                // ���������ʹ���ƴ�
    HAL_StatusTypeDef motor_send_state = HAL_TIMEOUT; // �������״̬/�Ƿ��е��ñ�־
    HAL_StatusTypeDef DM_Start(uint16_t id);
    HAL_StatusTypeDef DM_End(uint16_t id);
    HAL_StatusTypeDef DM_Savezero(uint16_t id);
    HAL_StatusTypeDef DM_MIT(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
    HAL_StatusTypeDef DM_POS(uint16_t id, float _pos, float _vel);
    HAL_StatusTypeDef DM_VEL(uint16_t id, float _vel);
    void update_4PI_mang_inf_basic_zeromang(void); // ���ı�0��Ĺ�Ȧ���
    HAL_StatusTypeDef DM_update(void);             // �õ��ٶȣ�λ�õȲ���
    HAL_StatusTypeDef DM_Clear_Err(uint16_t Id);
    MOTOR_DM(const uint16_t id, class USER_CAN *CAN_rev) : ID(id), can_rev(CAN_rev) {}
        
        float P_MIN = -3.141593f,
          P_MAX = 3.141593f,
          V_MIN = -30.0f,
          V_MAX = 30.0f,
          KP_MIN = 0.0f,
          KP_MAX = 500.0f,
          KD_MIN = 0.0f,
          KD_MAX = 5.0f,
          T_MIN = -12.0f,
          T_MAX = 12.0f;

private:
    
    // ���ﶨ��Ķ������ø�class������ĺ�����������
};

#endif

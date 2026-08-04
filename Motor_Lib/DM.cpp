#include "DM.h"

#define PI 3.1415926
HAL_StatusTypeDef MOTOR_DM::DM_Start(uint16_t Id) // ������  ����ָ��
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFC;
    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;//DM��������
    }
    return motor_send_state;
}

HAL_StatusTypeDef MOTOR_DM::DM_End(uint16_t Id) // �˳����   ����ָ��
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFD;
    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
HAL_StatusTypeDef MOTOR_DM::DM_Savezero(uint16_t Id) // ����λ�����   ����ָ��
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFE;
    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

//�޷������� �� ��������ӳ��
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  MITģʽ���¿���֡,kp=0,kd��Ϊ0��kd��0���𵴣�
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 * @param  _KP    λ�ñ���ϵ��
 * @param  _KD    λ��΢��ϵ��
 * @param  _torq  ת�ظ���ֵ
 */
HAL_StatusTypeDef MOTOR_DM::DM_MIT(uint16_t Id, float _pos, float _vel, float _KP, float _KD, float _torq) // MIT ģʽ
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);//λ��
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);//�ٶ�
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);//λ�ñ���ϵ��
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);//λ��΢��ϵ��
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    uint8_t pData[8];
    pData[0] = (pos_tmp >> 8);
    pData[1] = pos_tmp;
    pData[2] = (vel_tmp >> 4);
    pData[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    pData[4] = kp_tmp;
    pData[5] = (kd_tmp >> 4);
    pData[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    pData[7] = tor_tmp;
    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
/**
 * @brief֡ ID Ϊ�趨�� CAN ID ֵ���� 0x100 ��ƫ��
 * @param _pos��λ�ø����������ͣ���λ��ǰ����λ�ں�
 * @param _vel���ٶȸ����������ͣ���λ��ǰ����λ�ں�
 * @param
 * @param �˴���������� CAN ID �� 0x100+ID���ٶȸ��������μ��ٶ�����������ٶȵģ���Ϊ���ٶε��ٶ�ֵ��
 */
HAL_StatusTypeDef MOTOR_DM::DM_POS(uint16_t Id, float _pos, float _vel) // λ���ٶ�ģʽ
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;

    uint8_t pData[8];
    pData[0] = *pbuf;
    pData[1] = *(pbuf + 1);
    pData[2] = *(pbuf + 2);
    pData[3] = *(pbuf + 3);
    pData[4] = *vbuf;
    pData[5] = *(vbuf + 1);
    pData[6] = *(vbuf + 2);
    pData[7] = *(vbuf + 3);
    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}

/**
 * @brief  �ٶ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _vel   �ٶȸ���
 */

HAL_StatusTypeDef MOTOR_DM::DM_VEL(uint16_t Id, float _vel) // �ٶ�ģʽ
{
    uint8_t *vbuf;
    vbuf = (uint8_t *)&_vel;

    uint8_t pData[4];
    pData[0] = *vbuf;
    pData[1] = *(vbuf + 1);
    pData[2] = *(vbuf + 2);
    pData[3] = *(vbuf + 3);

    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
void MOTOR_DM::update_4PI_mang_inf_basic_zeromang(void)
{
    if (this->first == 0)
    {
        this->Last_mang = this->mang;
        this->first = 1;
    }

    if ((this->mang - this->Last_mang) < -4 * PI)
    {
        this->nsqd_8PI_Cnt_mang += 8 * PI;
        this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 4 * PI)
    {
        this->nsqd_8PI_Cnt_mang -= 8 * PI;
        this->motor_number--;
    }
    this->mang_inf = this->nsqd_8PI_Cnt_mang + this->mang;
    this->Last_mang = this->mang;
}
HAL_StatusTypeDef MOTOR_DM::DM_update(void) // �õ�����
{
    if (this->can_rev->RxHeader.StdId != ID)
    {
        return HAL_ERROR;
    }
    id = (this->can_rev->rx_buf[0]) & 0x0F;
    ERR = (this->can_rev->rx_buf[0]) >> 4;
    p_int = ((this->can_rev->rx_buf[1] << 8) | this->can_rev->rx_buf[2]);
    v_int = (this->can_rev->rx_buf[3] << 4) | (this->can_rev->rx_buf[4] >> 4) % 16384;
    t_int = ((this->can_rev->rx_buf[4] & 0xF) << 8) | this->can_rev->rx_buf[5];
    mang = uint_to_float(p_int, P_MIN, P_MAX, 16);   // (-PI,PI)
    sp = uint_to_float(v_int, V_MIN, V_MAX, 12);     // (-30.0,30.0)
    Torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    T_Rotor = (float)(this->can_rev->rx_buf[6]);
    T_MOS = (float)(this->can_rev->rx_buf[7]);
    return HAL_OK;
}
HAL_StatusTypeDef MOTOR_DM::DM_Clear_Err(uint16_t Id) // 清除错误   发送指令
{
    uint8_t pData[8];
    pData[0] = 0xFF;
    pData[1] = 0xFF;
    pData[2] = 0xFF;
    pData[3] = 0xFF;
    pData[4] = 0xFF;
    pData[5] = 0xFF;
    pData[6] = 0xFF;
    pData[7] = 0xFB;
    motor_send_state = this->can_rev->Send(Id, pData);
    if (motor_send_state != HAL_OK)
    {
        motor_send_error_cnt++;
    }
    return motor_send_state;
}
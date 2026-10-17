//
// Created by dyna0 on 2026/10/11.
//
#include "Kalman_offline.h";

OfflineKalman::OfflineKalman(float pos, float vel, float dt,
                  float K0, float K1, float v_max)
                :pos_(pos), vel_(vel), dt_(dt),K0_(K0), K1_(K1), v_max_(v_max)
{

}
float OfflineKalman::Update(float z_pos)
{
    //1.预测
    float pos_pred = pos_ + vel_ * dt_;
    float vel_pred = vel_;

    //2.计算消息
    float innovation = z_pos - pos_pred;;

    //3.更新
    pos_ = pos_pred + innovation * K0_;
    vel_ = vel_pred+ innovation * K1_;

    //4.限幅
    if (vel_ >0 && vel_ > v_max_)
    {
        vel_ = v_max_;
    }
    else if (vel_ <0 && vel_ < -v_max_)
    {
        vel_ = -v_max_;
    }

}

void OfflineKalman::Reset(float pos0, float vel0)
{
    pos_ = pos0;
    vel_ = vel0;
}



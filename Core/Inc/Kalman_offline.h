//
// Created by dyna0 on 2026/10/11.
//

#ifndef ROBOMASTERTEST001_KALMAN_OFFLINE_H
#define ROBOMASTERTEST001_KALMAN_OFFLINE_H
#ifdef __cplusplus
extern "C" {
#endif

class OfflineKalman
{
public:
    OfflineKalman(float pos, float vel, float dt,
                  float K0, float K1, float v_max);//pos:位置的估计值;vel:速度的估计值;dt:采样周期;k0:位置增益;k1:速度增益;v_max_速度限幅

    float Update(float z_pos);
    void  Reset(float pos0, float vel0);

    float GetPos() const { return pos_; }
    float GetVel() const { return vel_; }

private:
    float pos_;
    float vel_;
    float dt_;
    float K0_;
    float K1_;
    float v_max_;
};



#ifdef __cplusplus
}
#endif
#endif //ROBOMASTERTEST001_KALMAN_OFFLINE_H

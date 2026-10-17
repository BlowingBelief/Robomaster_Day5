#include "Motor.h"
#include "can_user.h"
#include <cmath>

Motor::Motor(float ratio)
    : ratio_(ratio),
      ppid_(117.05f, 0, 902.0f, 1000.0f, 600.0f, 0.3f),
      spid_(0.08f, 0, 0.17f, 1000.0f, 6.0f, 0.8f),
      ff_peak_a_(0.35425f),
      ff_scale_(1.4f)
{
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    uint16_t raw_ecd = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
    int16_t  raw_rpm = (static_cast<int16_t>(rx_data[2]) << 8) | rx_data[3];
    int16_t  raw_cur = (static_cast<int16_t>(rx_data[4]) << 8) | rx_data[5];
    uint8_t  raw_tmp = rx_data[6];

    if (raw_ecd >= 8192) {
        return;
    }

    ecd_angle_ = raw_ecd * 360.0f / 8192.0f;
    speedRpm_  = static_cast<float>(raw_rpm);
    currentA_  = static_cast<float>(raw_cur) / kUnitsPerAmp;
    tempC_     = static_cast<float>(raw_tmp);

    if (!received_) {
        last_ecd_ = ecd_angle_;
        received_ = true;
    } else {
        float diff = ecd_angle_ - last_ecd_;
        if (diff > 180.0f) {
            diff -= 360.0f;
        } else if (diff < -180.0f) {
            diff += 360.0f;
        }
        angle_ += diff;
        last_ecd_ = ecd_angle_;
    }
}

float Motor::angle() const
{
    return (angle_ / ratio_) - zero_offset_deg_;
}

float Motor::speedDegS() const
{
    return speedRpm_ * 6.0f / ratio_;
}

float Motor::currentAmps() const { return currentA_; }
float Motor::temperature() const { return tempC_; }
bool  Motor::hasFeedback() const { return received_; }

void Motor::setPosition(float target_position, float feedforward_speed, float feedforward_intensity)
{
    (void)feedforward_speed;
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_intensity_ = feedforward_intensity;
}

void Motor::setSpeed(float target_speed, float feedforward_intensity)
{
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void Motor::setIntensity(float intensity)
{
    control_method_ = TORQUE;
    feedforward_intensity_ = intensity;
}

void Motor::setFeedforwardOnly()
{
    control_method_ = FEEDFORWARD;
}

void Motor::setStopFlag(bool flag)
{
    stop_flag_ = flag;
}

void Motor::requestZero()
{
    if (stop_flag_ && received_) {
        zero_offset_deg_ = angle_ / ratio_;
        ppid_.Reset();
        spid_.Reset();
    }
}

void Motor::handle()
{
    fdb_angle_ = angle();
    fdb_speed_ = speedDegS();

    if (stop_flag_) {
        intensity_ = 0.0f;
        ppid_.Reset();
        spid_.Reset();
        setTxCurrent(0.0f, 1);
        pid_mon.angle = fdb_angle_;
        pid_mon.target_angle = target_angle_;
        pid_mon.intensity = 0.0f;
        pid_mon.spid_integral = 0.0f;
        pid_mon.ppid_integral = 0.0f;
        pid_mon.ppid_last_d = 0.0f;
        pid_mon.error = target_angle_ - fdb_angle_;
        pid_mon.speed = fdb_speed_;
        return;
    }

    float gravity_sine = sinf(fdb_angle_ * kPi / 180.0f);
    float gravity_ff_a = ff_peak_a_ * ff_scale_ * gravity_sine;

    switch (control_method_) {
    case FEEDFORWARD:
        intensity_ = gravity_ff_a;
        break;
    case POSITION_SPEED: {
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_);
        intensity_ = spid_.calc(target_speed_, fdb_speed_, gravity_ff_a) + gravity_ff_a;
        break;
    }
    case SPEED:
        intensity_ = spid_.calc(target_speed_, fdb_speed_, gravity_ff_a) + gravity_ff_a;
        break;
    case TORQUE:
        intensity_ = feedforward_intensity_;
        break;
    }

    setTxCurrent(intensity_, 1);

    pid_mon.angle = fdb_angle_;
    pid_mon.target_angle = target_angle_;
    pid_mon.intensity = intensity_;
    pid_mon.spid_integral = spid_.integral();
    pid_mon.ppid_integral = ppid_.integral();
    pid_mon.ppid_last_d = ppid_.lastD();
    pid_mon.error = target_angle_ - fdb_angle_;
    pid_mon.speed = fdb_speed_;
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id)
{
    if (amperes >  kDebugCurrentLimitA) amperes =  kDebugCurrentLimitA;
    if (amperes < -kDebugCurrentLimitA) amperes = -kDebugCurrentLimitA;
    if (amperes >  kMaxCurrentA) amperes =  kMaxCurrentA;
    if (amperes < -kMaxCurrentA) amperes = -kMaxCurrentA;

    int16_t raw = static_cast<int16_t>(amperes * kUnitsPerAmp);

    uint8_t idx = static_cast<uint8_t>((motor_id - 1) * 2);
    if (idx > 6) {
        return;
    }
    tx_data_[idx]     = static_cast<uint8_t>((raw >> 8) & 0xFF);
    tx_data_[idx + 1] = static_cast<uint8_t>(raw & 0xFF);
}

uint8_t* Motor::getTxData()
{
    return tx_data_;
}
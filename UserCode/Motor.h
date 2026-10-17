#ifndef ROBOMASTERTEST001_MOTOR_H
#define ROBOMASTERTEST001_MOTOR_H

#include <cstdint>
#include "pids.h"

class Motor {
public:
    explicit Motor(float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);

    float angle() const;
    float speedDegS() const;
    float currentAmps() const;
    float temperature() const;
    bool  hasFeedback() const;

    typedef enum ControlMethod {
        POSITION_SPEED,
        SPEED,
        TORQUE,
        FEEDFORWARD
    } ControlMethod_e;

    void handle();
    void setPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void setSpeed(float target_speed, float feedforward_intensity);
    void setIntensity(float intensity);
    void setFeedforwardOnly();
    void setStopFlag(bool flag);
    void requestZero();
    void setTxCurrent(float amperes, uint8_t motor_id);
    uint8_t* getTxData();

private:
    const float ratio_;

    float ecd_angle_ = 0.0f;
    float speedRpm_  = 0.0f;
    float currentA_  = 0.0f;
    float tempC_     = 0.0f;

    float angle_ = 0.0f;
    float last_ecd_ = 0.0f;
    bool  received_ = false;
    float zero_offset_deg_ = 0.0f;

    uint8_t tx_data_[8] = {};

    ControlMethod_e control_method_ = FEEDFORWARD;

    PID ppid_;
    PID spid_;

    float target_angle_ = 0.0f;
    float fdb_angle_ = 0.0f;
    float target_speed_ = 0.0f;
    float fdb_speed_ = 0.0f;
    float feedforward_intensity_ = 0.0f;
    float intensity_ = 0.0f;
    bool stop_flag_ = true;

    float ff_peak_a_;
    float ff_scale_;

    static constexpr float kPi = 3.1415926f;
    static constexpr float kUnitsPerAmp = 1000.0f;
    static constexpr float kMaxCurrentA = 10.0f;
    static constexpr float kDebugCurrentLimitA = 6.0f;
};

#endif
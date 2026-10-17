#ifndef ROBOMASTERTEST001_PIDS_H
#define ROBOMASTERTEST001_PIDS_H

class PID
{
public:
    PID(float Kp, float Ki, float Kd, float i_max, float out_max, float d_filter_k);
    void Reset();
    float calc(float ref, float fdb, float feedforward = 0.0f);
    float integral() const { return integral_; }
    float lastD() const { return prev_d_; }

private:
    static float clamp(float value, float minimum, float maximum);

    float kp_, ki_, kd_, i_max_, out_max_, d_filter_k_;
    float integral_;
    float prev_error_;
    float prev_d_;
    bool first_calc_;
};

#endif
#include "pids.h"
#include <cmath>

PID::PID(float kp, float ki, float kd,
         float i_max, float out_max, float d_filter_k)
    : kp_(kp), ki_(ki), kd_(kd),
      i_max_(fabsf(i_max)), out_max_(fabsf(out_max)),
      d_filter_k_(clamp(d_filter_k, 0.0f, 0.99f)),
      integral_(0.0f), prev_error_(0.0f),
      prev_d_(0.0f), first_calc_(true)
{
}

void PID::Reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_d_ = 0.0f;
    first_calc_ = true;
}

float PID::clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float PID::calc(float ref, float fdb, float feedforward)
{
    float error = ref - fdb;
    if (!isfinite(error) || !isfinite(feedforward)) {
        Reset();
        return 0.0f;
    }

    float derivative = first_calc_ ? 0.0f : (error - prev_error_);
    derivative = d_filter_k_ * prev_d_ + (1.0f - d_filter_k_) * derivative;

    float next_integral = clamp(integral_ + error, -i_max_, i_max_);
    float output = kp_ * error + ki_ * next_integral + kd_ * derivative + feedforward;

    if (!((output > out_max_ && ki_ * error > 0.0f) ||
          (output < -out_max_ && ki_ * error < 0.0f))) {
        integral_ = next_integral;
          }

    prev_error_ = error;
    prev_d_ = derivative;
    first_calc_ = false;

    output = kp_ * error + ki_ * integral_ + kd_ * derivative + feedforward;
    return clamp(output, -out_max_, out_max_) - feedforward;
}
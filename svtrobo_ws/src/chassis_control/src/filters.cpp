#include "chassis_control/filters.h"

double LowPassFilter::filter(double input)
{
    if (!initialized_) {
      output_ = input;
      initialized_ = true;
      return output_;
    }
    // 一阶低通滤波: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    output_ = alpha_ * input + (1.0 - alpha_) * output_;
    return output_;
}

void LowPassFilter::reset()
{
    initialized_ = false;
    output_ = 0.0;
}

double LowPassFilter::getOutput() const
{
    return output_; 
}

double RateLimiter::limit(double target, double dt)
{
    if (!initialized_) {
      last_output_ = target;
      initialized_ = true;
      return target;
    }
    
    double diff = target - last_output_;
    double max_change = max_rate_ * dt;
    
    if (diff > max_change) {
      last_output_ += max_change;
    } else if (diff < -max_change) {
      last_output_ -= max_change;
    } else {
      last_output_ = target;
    }
    
    return last_output_;
}

void RateLimiter::reset(void)
{
    initialized_ = false;
    last_output_ = 0.0;
}



void VectorRateLimiter::limit(double targets[], int n, double dt)
{
    if (n <= 0 || dt <= 0.0) return;

    if (!initialized_) {
        for (int i = 0; i < n && i < MAX_WHEEL_COUNT; i++) {
            last_output_[i] = targets[i];
        }
        initialized_ = true;
        return;
    }

    // 计算 delta 向量范数
    double delta_sq = 0.0;
    for (int i = 0; i < n && i < MAX_WHEEL_COUNT; i++) {
        double d = targets[i] - last_output_[i];
        delta_sq += d * d;
    }
    double delta_mag = std::sqrt(delta_sq);

    double max_delta = max_rate_ * dt;

    if (delta_mag > max_delta && delta_mag > 1e-9) {
        // 超限：等比缩放所有 delta
        double scale = max_delta / delta_mag;
        for (int i = 0; i < n && i < MAX_WHEEL_COUNT; i++) {
            double d = targets[i] - last_output_[i];
            last_output_[i] += d * scale;
            targets[i] = last_output_[i];
        }
    } else {
        // 未超限：直接跟踪目标
        for (int i = 0; i < n && i < MAX_WHEEL_COUNT; i++) {
            last_output_[i] = targets[i];
        }
    }
}

void VectorRateLimiter::reset(void)
{
    initialized_ = false;
    memset(last_output_, 0, sizeof(last_output_));
}

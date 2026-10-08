#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <iostream>
#include <cmath>
#include <cstring>

#define FILTER_ALPHA 0.75f        // 低通滤波系数 (0-1, 越小越平滑但响应越慢)
#define MAX_ANGLE_RATE 3.0f       // 最大角度变化率 (rad/s)
#define MAX_WHEEL_SPEED 100.0f    // 最大轮子速度 (rpm)
#define MAX_WHEEL_COUNT 4         // 最大轮子数量

class LowPassFilter {
public:
  LowPassFilter(double alpha = FILTER_ALPHA) : alpha_(alpha), output_(0.0), initialized_(false) {}

  double filter(double input);

  void reset();

  double getOutput() const;

private:
  double alpha_;
  double output_;
  bool initialized_;
};

// 单值变化率限制器（用于舵角等独立限制场景）
class RateLimiter {
public:
  RateLimiter(double max_rate = MAX_ANGLE_RATE) : max_rate_(max_rate), last_output_(0.0), initialized_(false) {}

  double limit(double target, double dt);
  void reset(void);

private:
  double max_rate_;
  double last_output_;
  bool initialized_;
};

// 合成矢量变化率限制器
// 对 N 个值计算 delta 向量的范数，超限时等比缩放，所有值同步变化保持相对比例
// 适用场景：轮速同步限制（避免左右不对称偏移）、舵角同步限制（保持转向几何）
class VectorRateLimiter {
public:
  VectorRateLimiter(double max_rate = 1.0) : max_rate_(max_rate), initialized_(false) {
    memset(last_output_, 0, sizeof(last_output_));
  }

  // targets: 目标值数组, n: 维度(<=MAX_WHEEL_COUNT), dt: 时间步
  // 结果直接写回 targets 数组
  void limit(double targets[], int n, double dt);
  void reset(void);

  // 运行时动态调参
  void set_max_rate(double max_rate) { max_rate_ = max_rate; }

private:
  double max_rate_;
  double last_output_[MAX_WHEEL_COUNT];
  bool initialized_;
};

#endif

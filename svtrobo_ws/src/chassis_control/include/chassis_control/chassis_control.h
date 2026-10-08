#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "chassis_control/msg/chassis_diagnostics.hpp"
#include <atomic>
#include <memory>
#include <thread>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <exception>
#include <chrono>

#include "stdint.h"

#include "chassis_control/steering_motor.h"
#include "chassis_control/wheel_motor.h"
#include "chassis_control/filters.h"

#define PI 3.1415926535897932384626433832795f

#define motor_kp 4.0f
#define motor_kd 0.4f

#define WHEEL_FL_DIRETION -1 // 左前
#define WHEEL_FR_DIRETION 1 // 右前
#define WHEEL_RL_DIRETION 1 // 左后
#define WHEEL_RR_DIRETION -1 // 右后

#define STEER_MOTOR_CAN "can2" 
#define WHEEL_MOTOR_CAN "can3"

typedef struct chassis_control_para_t
{
    double vx_set;                  //m/s
    double vy_set;                  //m/s
    double wz_set;                  //rad/s
    double front_left_angle;
    double front_right_angle;
    double rear_left_angle;
    double rear_right_angle;
    double front_left_speed;        //rad/s
    double front_right_speed;
    double rear_left_speed;
    double rear_right_speed;
    /* data */
} chassis_control_para_t;

typedef struct chassis_param_t
{
    double chassis_radius;
    double wheel_perimeter;
    double fl_motor_start_angle;
    double fr_motor_start_angle;
    double rl_motor_start_angle;
    double rr_motor_start_angle;
} chassis_param_t;

class ChassisControlNode : public rclcpp::Node {
public:
    ChassisControlNode(void);
    void excute_loop(void);
    ~ChassisControlNode(void);

private:
    std::thread worker_thread_;
    std::atomic<bool> running_ = true;
    double control_rate_hz_;

    chassis_control_para_t chassis_control_para;
    chassis_param_t chassis_param;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr svtrobot_cmd_sub;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_feedback_pub_;
    rclcpp::Publisher<chassis_control::msg::ChassisDiagnostics>::SharedPtr diagnostics_pub_;
    
    RobStrideMotor motor1;
    RobStrideMotor motor2;
    RobStrideMotor motor3;
    RobStrideMotor motor4;

    LowPassFilter fl_angle_filter;
    LowPassFilter fr_angle_filter;
    LowPassFilter rl_angle_filter;
    LowPassFilter rr_angle_filter;

    RateLimiter fl_rate_limiter;
    RateLimiter fr_rate_limiter;
    RateLimiter rl_rate_limiter;
    RateLimiter rr_rate_limiter;

    VectorRateLimiter speed_limiter;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::unique_ptr<ZLAC8015D> front_;
    std::unique_ptr<ZLAC8015D> rear_;

    double dt;
    int publish_decimation_ = 0;
    int vbus_decimation_ = 0;
    int wheel_speed_decimation_ = 0;
    std::chrono::steady_clock::time_point last_cmd_time_;
    float wheel_actual_fl_ = 0.0f;
    float wheel_actual_fr_ = 0.0f;
    float wheel_actual_rl_ = 0.0f;
    float wheel_actual_rr_ = 0.0f;

    void chassis_control_loop(void);
    void arc_judge(void);
    void filter_limited(void);
    void motor_move(void);
    void svtrobot_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
#endif

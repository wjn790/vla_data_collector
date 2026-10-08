#include "chassis_control/chassis_control.h"

ChassisControlNode::ChassisControlNode(void):rclcpp::Node("chassis_control_node"),
                                        motor1(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x65, 0)),
                                        motor2(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x66, 0)),
                                        motor3(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x67, 0)),
                                        motor4(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x68, 0)),
                                        fl_angle_filter(FILTER_ALPHA),
                                        fr_angle_filter(FILTER_ALPHA),
                                        rl_angle_filter(FILTER_ALPHA),
                                        rr_angle_filter(FILTER_ALPHA),
                                        fl_rate_limiter(MAX_ANGLE_RATE),
                                        fr_rate_limiter(MAX_ANGLE_RATE),
                                        rl_rate_limiter(MAX_ANGLE_RATE),
                                        rr_rate_limiter(MAX_ANGLE_RATE),
                                        speed_limiter(1.0){
    this->declare_parameter<double>("robot.chassis_radius");
    this->declare_parameter<double>("robot.wheel_perimeter");

    this->declare_parameter<double>("robot.fl_motor_start_angle");
    this->declare_parameter<double>("robot.fr_motor_start_angle");
    this->declare_parameter<double>("robot.rl_motor_start_angle");
    this->declare_parameter<double>("robot.rr_motor_start_angle");

    this->declare_parameter<double>("limit.max_angle_rate");
    this->declare_parameter<double>("limit.max_wheel_speed_rate");

    this->get_parameter("robot.chassis_radius", chassis_param.chassis_radius);
    this->get_parameter("robot.wheel_perimeter",chassis_param.wheel_perimeter);

    this->get_parameter("robot.fl_motor_start_angle", chassis_param.fl_motor_start_angle);
    this->get_parameter("robot.fr_motor_start_angle", chassis_param.fr_motor_start_angle);
    this->get_parameter("robot.rl_motor_start_angle", chassis_param.rl_motor_start_angle);
    this->get_parameter("robot.rr_motor_start_angle", chassis_param.rr_motor_start_angle);

    // 读取速率限制参数
    double angle_rate, speed_rate;
    this->get_parameter("limit.max_angle_rate", angle_rate);
    this->get_parameter("limit.max_wheel_speed_rate", speed_rate);
    fl_rate_limiter = RateLimiter(angle_rate);
    fr_rate_limiter = RateLimiter(angle_rate);
    rl_rate_limiter = RateLimiter(angle_rate);
    rr_rate_limiter = RateLimiter(angle_rate);
    speed_limiter = VectorRateLimiter(speed_rate);

    // 运行时动态调参回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& param : params) {
                if (param.get_name() == "limit.max_angle_rate") {
                    double v = param.as_double();
                    fl_rate_limiter = RateLimiter(v);
                    fr_rate_limiter = RateLimiter(v);
                    rl_rate_limiter = RateLimiter(v);
                    rr_rate_limiter = RateLimiter(v);
                    RCLCPP_INFO(this->get_logger(), "max_angle_rate updated: %.2f rad/s", v);
                } else if (param.get_name() == "limit.max_wheel_speed_rate") {
                    double v = param.as_double();
                    speed_limiter.set_max_rate(v);
                    RCLCPP_INFO(this->get_logger(), "max_wheel_speed_rate updated: %.2f RPM/s", v);
                }
            }
            return result;
        });

    svtrobot_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("/svtrobot_cmd", 10,
                       std::bind(&ChassisControlNode::svtrobot_cmd_callback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/chassis/joint_states", 10);
    cmd_feedback_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/chassis/cmd_feedback", 10);
    diagnostics_pub_ = this->create_publisher<chassis_control::msg::ChassisDiagnostics>("/chassis/diagnostics", 10);

    motor1.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);
    motor2.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);
    motor3.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);
    motor4.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);

    // Read initial VBUS
    motor1.Get_RobStrite_Motor_parameter(0x701C);
    usleep(100);

    motor1.enable_motor();
    usleep(100);
    motor2.enable_motor();
    usleep(100);
    motor3.enable_motor();
    usleep(100);
    motor4.enable_motor();
    usleep(100);

    // RCLCPP_INFO(this->get_logger(), "Creating front and rear ZLAC8015D on interface '%s'...", WHEEL_MOTOR_CAN);
    try
    {
      front_ = std::make_unique<ZLAC8015D>(WHEEL_MOTOR_CAN, 1, 0.3);
      rear_  = std::make_unique<ZLAC8015D>(WHEEL_MOTOR_CAN, 2, 0.3);

      for (auto *drv : std::vector<ZLAC8015D*>{front_.get(), rear_.get()})
      {
        int hb = drv->wait_heartbeat(1.0);
        (void)hb;
        try
        {
          drv->clear_fault();
          }
        catch (...)
        {

        }
        drv->set_velocity_mode();
        drv->enable_operation();
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error during initial device setup: %s", e.what());
      throw;
    }
    signal(SIGINT, [](int sig)
    {
      (void)sig;
      rclcpp::shutdown();
    });
    RCLCPP_INFO(this->get_logger(), "chassis is initing");
    last_cmd_time_ = std::chrono::steady_clock::now();
    chassis_control_para.front_left_angle = chassis_param.fl_motor_start_angle;
    chassis_control_para.front_right_angle = chassis_param.fr_motor_start_angle;
    chassis_control_para.rear_left_angle = chassis_param.rl_motor_start_angle;
    chassis_control_para.rear_right_angle = chassis_param.rr_motor_start_angle;
    RCLCPP_INFO(this->get_logger(), "chassis init finished");
    worker_thread_ = std::thread(&ChassisControlNode::excute_loop, this);
}

void ChassisControlNode::excute_loop(void)
{
    const double LOOP_DT = 0.001;      // 循环周期 1ms

    // 记录上次循环时间
    auto last_time = std::chrono::steady_clock::now();

    while (running_) {
      try {
        // 计算实际循环时间
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time).count();
        last_time = current_time;
        // 限制dt在合理范围内，避免异常值
        if (dt > 0.1) dt = LOOP_DT;  // 如果dt过大，使用默认值
        if (dt < 0.0001) dt = LOOP_DT;  // 如果dt过小，使用默认值

        // 指令超时保护：0.5s 无新指令则自动归零
        double cmd_age = std::chrono::duration<double>(current_time - last_cmd_time_).count();
        if (cmd_age > 0.5) {
            chassis_control_para.vx_set = 0.0;
            chassis_control_para.vy_set = 0.0;
            chassis_control_para.wz_set = 0.0;
        }

        chassis_control_loop();
        arc_judge();

        double fl_angle_target = chassis_control_para.front_left_angle;
        double fr_angle_target = chassis_control_para.front_right_angle;
        double rl_angle_target = chassis_control_para.rear_left_angle;
        double rr_angle_target = chassis_control_para.rear_right_angle;

            // 应用变化率限制
        double fl_angle_limited = fl_rate_limiter.limit(fl_angle_target, dt);
        double fr_angle_limited = fr_rate_limiter.limit(fr_angle_target, dt);
        double rl_angle_limited = rl_rate_limiter.limit(rl_angle_target, dt);
        double rr_angle_limited = rr_rate_limiter.limit(rr_angle_target, dt);

        // 应用低通滤波
        chassis_control_para.front_left_angle = fl_angle_filter.filter(fl_angle_limited);
        chassis_control_para.front_right_angle = fr_angle_filter.filter(fr_angle_limited);
        chassis_control_para.rear_left_angle = rl_angle_filter.filter(rl_angle_limited);
        chassis_control_para.rear_right_angle = rr_angle_filter.filter(rr_angle_limited);

        if (!std::isfinite(chassis_control_para.front_left_angle ))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Invalid fl_angle: %f, using previous value", chassis_control_para.front_left_angle );
            chassis_control_para.front_left_angle  = fl_angle_filter.getOutput();  // 使用上次有效值
        }
        if (!std::isfinite(chassis_control_para.front_right_angle))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Invalid fr_angle: %f, using previous value", chassis_control_para.front_right_angle);
            chassis_control_para.front_right_angle = fr_angle_filter.getOutput();
        }
        if (!std::isfinite(chassis_control_para.rear_left_angle ))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Invalid rl_angle: %f, using previous value", chassis_control_para.rear_left_angle );
            chassis_control_para.rear_left_angle  = rl_angle_filter.getOutput();
        }
        if (!std::isfinite(chassis_control_para.rear_right_angle)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Invalid rr_angle: %f, using previous value", chassis_control_para.rear_right_angle);
            chassis_control_para.rear_right_angle = rr_angle_filter.getOutput();
        }
        try {
          float fl_angle_f = static_cast<float>(chassis_control_para.front_left_angle);
          if (std::isfinite(fl_angle_f)) {
            // motor1.send_motion_command(0.0f, fl_angle_f, 0.0f, motor_kp, motor_kd);
            motor1.RobStrite_Motor_PosCSP_control(20.0f, fl_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Motor1: Skipping invalid angle %f", fl_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Motor1 command failed: %s", e.what());
        }

        try {
          float fr_angle_f = static_cast<float>(chassis_control_para.front_right_angle);
          if (std::isfinite(fr_angle_f)) {
            // motor2.send_motion_command(0.0f, fr_angle_f, 0.0f, motor_kp, motor_kd);
            motor2.RobStrite_Motor_PosCSP_control(20.0f, fr_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Motor2: Skipping invalid angle %f", fr_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Motor2 command failed: %s", e.what());
        }

        try {
          float rl_angle_f = static_cast<float>(chassis_control_para.rear_left_angle);
          if (std::isfinite(rl_angle_f)) {
           // motor3.send_motion_command(0.0f, rl_angle_f, 0.0f, motor_kp, motor_kd);
            motor3.RobStrite_Motor_PosCSP_control(20.0f, rl_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Motor3: Skipping invalid angle %f", rl_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Motor3 command failed: %s", e.what());
        }

        try {
          float rr_angle_f = static_cast<float>(chassis_control_para.rear_right_angle);
          if (std::isfinite(rr_angle_f)) {
            // motor4.send_motion_command(0.0f, rr_angle_f, 0.0f, motor_kp, motor_kd);
            motor4.RobStrite_Motor_PosCSP_control(20.0f, rr_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Motor4: Skipping invalid angle %f", rr_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Motor4 command failed: %s", e.what());
        }
      }catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Error in control loop: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 出错时稍长等待
      }
      if((fabs(chassis_control_para.rear_right_angle-motor4.position_)<=0.1)
        && fabs(chassis_control_para.rear_left_angle-motor3.position_)<=0.1
        && fabs(chassis_control_para.front_right_angle-motor2.position_)<=0.1
        && fabs(chassis_control_para.front_left_angle-motor1.position_)<=0.1)
      {
          // 舵角到位：合成矢量限制轮速变化率（4轮同步缩放，保持运动方向）
          double speeds[4] = {
            chassis_control_para.front_left_speed,
            chassis_control_para.front_right_speed,
            chassis_control_para.rear_left_speed,
            chassis_control_para.rear_right_speed
          };
          speed_limiter.limit(speeds, 4, dt);

          front_->set_target_speed_lr_rpm(speeds[1]*WHEEL_FR_DIRETION,
                                        speeds[0]*WHEEL_FL_DIRETION);
          rear_->set_target_speed_lr_rpm(speeds[2]*WHEEL_RL_DIRETION,
                                        speeds[3]*WHEEL_RR_DIRETION);

      }
      else
      {
        // 舵角未到位：轮速强制为 0，同时喂 0 给 VectorRateLimiter 使其跟踪实际发送值
        double zero_speeds[4] = {0.0, 0.0, 0.0, 0.0};
        speed_limiter.limit(zero_speeds, 4, dt);

        front_->set_target_speed_lr_rpm(0,0);
        rear_->set_target_speed_lr_rpm(0,0);
      }

      // 发布状态 (降频到 100Hz, 每 10 次循环发布一次)
      publish_decimation_++;
      if (publish_decimation_ >= 10)
      {
        publish_decimation_ = 0;

        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.name = {"fl_steer", "fr_steer", "rl_steer", "rr_steer",
                          "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"};
        joint_msg.position = {
          motor1.position_, motor2.position_, motor3.position_, motor4.position_,
          0.0, 0.0, 0.0, 0.0
        };
        joint_msg.velocity = {
          motor1.velocity_, motor2.velocity_, motor3.velocity_, motor4.velocity_,
          chassis_control_para.front_left_speed, chassis_control_para.front_right_speed,
          chassis_control_para.rear_left_speed, chassis_control_para.rear_right_speed
        };
        joint_msg.effort = {
          motor1.torque_, motor2.torque_, motor3.torque_, motor4.torque_,
          0.0, 0.0, 0.0, 0.0
        };
        joint_state_pub_->publish(joint_msg);

        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = chassis_control_para.vx_set;
        cmd_msg.linear.y = chassis_control_para.vy_set;
        cmd_msg.angular.z = chassis_control_para.wz_set;
        cmd_feedback_pub_->publish(cmd_msg);

        // VBUS polling (every 100 publish cycles ≈ 20s)
        vbus_decimation_++;
        if (vbus_decimation_ >= 100) {
          vbus_decimation_ = 0;
          motor1.Get_RobStrite_Motor_parameter(0x701C);
        }

        // Wheel actual speed polling (every 50 publish cycles ≈ 10s)
        wheel_speed_decimation_++;
        if (wheel_speed_decimation_ >= 50) {
          wheel_speed_decimation_ = 0;
          try {
            auto [fl_spd, fr_spd] = front_->read_actual_speed_lr_0p1rpm();
            auto [rl_spd, rr_spd] = rear_->read_actual_speed_lr_0p1rpm();
            wheel_actual_fl_ = static_cast<float>(fl_spd) * 0.1f;
            wheel_actual_fr_ = static_cast<float>(fr_spd) * 0.1f;
            wheel_actual_rl_ = static_cast<float>(rl_spd) * 0.1f;
            wheel_actual_rr_ = static_cast<float>(rr_spd) * 0.1f;
          } catch (...) {
            // ZLAC8015D read failure — keep last known values
          }
        }

        // Publish diagnostics
        auto diag_msg = chassis_control::msg::ChassisDiagnostics();
        diag_msg.header.stamp = this->now();
        diag_msg.vbus = motor1.drw.VBUS.data;
        diag_msg.motor_temperatures = {
          motor1.temperature_, motor2.temperature_,
          motor3.temperature_, motor4.temperature_};
        diag_msg.motor_error_codes = {
          motor1.error_code, motor2.error_code,
          motor3.error_code, motor4.error_code};
        diag_msg.wheel_speeds_actual = {
          wheel_actual_fl_, wheel_actual_fr_,
          wheel_actual_rl_, wheel_actual_rr_};
        diagnostics_pub_->publish(diag_msg);
      }
    }

}

ChassisControlNode::~ChassisControlNode(void)
{
    motor1.Disenable_Motor(0);
    motor2.Disenable_Motor(0);
    motor3.Disenable_Motor(0);
    motor4.Disenable_Motor(0);


    running_ = false; // 停止线程
    if (worker_thread_.joinable())
      worker_thread_.join(); // 等待线程结束
}

void ChassisControlNode::chassis_control_loop(void)
{
    this->get_parameter("robot.chassis_radius", chassis_param.chassis_radius);
    this->get_parameter("robot.wheel_perimeter",chassis_param.wheel_perimeter);

    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f/chassis_param.wheel_perimeter;

    chassis_control_para.front_left_speed = sqrt(	pow(chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_control_para.rear_left_speed = sqrt(	pow(chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_control_para.front_right_speed = sqrt(	pow(chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_control_para.rear_right_speed = sqrt(	pow(chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;

    // // 按比例限制速度：找出最大速度，如果超过限制则按比例缩放所有速度
    double max_speed = fabs(chassis_control_para.front_left_speed);
    if (fabs(chassis_control_para.front_right_speed) > max_speed)
        max_speed = fabs(chassis_control_para.front_right_speed);
    if (fabs(chassis_control_para.rear_left_speed) > max_speed)
        max_speed = fabs(chassis_control_para.rear_left_speed);
    if (fabs(chassis_control_para.rear_right_speed) > max_speed)
        max_speed = fabs(chassis_control_para.rear_right_speed);

    // // 如果最大速度超过限制，按比例缩放所有速度（保持运动方向）
    if (max_speed > MAX_WHEEL_SPEED) {
        double scale = MAX_WHEEL_SPEED / max_speed;
        chassis_control_para.front_left_speed *= scale;
        chassis_control_para.front_right_speed *= scale;
        chassis_control_para.rear_left_speed *= scale;
        chassis_control_para.rear_right_speed *= scale;
    }

    //舵向控制
    chassis_control_para.front_left_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.fl_motor_start_angle;
    chassis_control_para.front_right_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.fr_motor_start_angle;
    chassis_control_para.rear_left_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.rl_motor_start_angle;
    chassis_control_para.rear_right_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.rr_motor_start_angle;
}

void ChassisControlNode::arc_judge(void)
{


        double fl_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));
        double fr_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));
        double rl_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));
        double rr_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));

        if(fabs(chassis_control_para.front_left_angle- motor1.position_)>PI/2.0f+0.02f)
        {
          if(fl_angle>0)
          {
            chassis_control_para.front_left_angle = chassis_param.fl_motor_start_angle-(PI-fabs(fl_angle));
            chassis_control_para.front_left_speed = -chassis_control_para.front_left_speed;
          }
          else
          {
            chassis_control_para.front_left_angle = chassis_param.fl_motor_start_angle+PI-fabs(fl_angle);
            chassis_control_para.front_left_speed = -chassis_control_para.front_left_speed;
          }
        }
        if(fabs(chassis_control_para.front_right_angle-motor2.position_)>PI/2.0f+0.02f)
        {
          if(fr_angle>0)
          {
            chassis_control_para.front_right_angle = chassis_param.fr_motor_start_angle-(PI-fabs(fr_angle));
            chassis_control_para.front_right_speed = -chassis_control_para.front_right_speed;
          }
          else
          {
            chassis_control_para.front_right_angle = chassis_param.fr_motor_start_angle+PI-fabs(fr_angle);
            chassis_control_para.front_right_speed = -chassis_control_para.front_right_speed;
          }
        }
        if(fabs(chassis_control_para.rear_left_angle-motor3.position_)>PI/2.0f+0.02f)
        {
          if(rl_angle>0)
          {
            chassis_control_para.rear_left_angle = chassis_param.rl_motor_start_angle-(PI-fabs(rl_angle));
            chassis_control_para.rear_left_speed = -chassis_control_para.rear_left_speed;
          }
          else
          {
            chassis_control_para.rear_left_angle = chassis_param.rl_motor_start_angle+PI-fabs(rl_angle);
            chassis_control_para.rear_left_speed = -chassis_control_para.rear_left_speed;
          }
        }

        if(fabs(chassis_control_para.rear_right_angle-motor4.position_)>PI/2.0f+0.02f)
        {
          if(rr_angle>0)
          {
            chassis_control_para.rear_right_angle = chassis_param.rr_motor_start_angle-(PI-fabs(rr_angle));
            chassis_control_para.rear_right_speed = -chassis_control_para.rear_right_speed;
          }
          else
          {
            chassis_control_para.rear_right_angle = chassis_param.rr_motor_start_angle+PI-fabs(rr_angle);
            chassis_control_para.rear_right_speed = -chassis_control_para.rear_right_speed;
          }
        }
}

void ChassisControlNode::svtrobot_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_time_ = std::chrono::steady_clock::now();
    chassis_control_para.vx_set = msg->linear.x;
    chassis_control_para.vy_set = msg->linear.y;
    chassis_control_para.wz_set = msg->angular.z;
}

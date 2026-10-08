/******************************************************************************
 * @file    lift_RS485_control.cpp
 * @brief   lift_RS485_control implementation
 *
 * @author  luzhongfa
 * @company svtrobo
 * @date    2026-01-05
 *
 *****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <poll.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <chrono>
#include <limits>

/*
# 发布控制指令（示例）
rostopic pub -1 /lift_control_cmd std_msgs/Int32MultiArray "data: [1, 300]"  # 正转300rpm
rostopic pub -1 /lift_control_cmd std_msgs/Int32MultiArray "data: [-1, 200]" # 反转200rpm
rostopic pub -1 /lift_control_cmd std_msgs/Int32MultiArray "data: [0, 0]"    # 停止
*/

class MotorController : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr relative_position_pub_;
    rclcpp::TimerBase::SharedPtr position_timer_;
    int serial_fd_;
    std::mutex serial_mutex_;
    std::atomic<bool> is_running_;
    std::atomic<bool> serial_needs_reopen_{false};  // USB 重枚举等导致 fd 失效时标记
    int32_t position_zero_raw_;

    // 配置参数（写死以避免 get_param 类型转换等问题）
    const std::string SERIAL_DEV_ = "/dev/lift_motor";
    const uint8_t SLAVE_ADDR_ = 0x01;
    const speed_t BAUDRATE_ = B57600;  // termios 波特率常量
    const int COMM_TIMEOUT_MS_ = 500;  // 等待应答超时（ms）
    const int POSITION_PUBLISH_PERIOD_MS_ = 500;
    const int16_t MAX_SPEED_ = 6000;
    const int16_t MIN_SPEED_ = 1;

    int32_t clamp_to_int32(int64_t value) {
        if (value > std::numeric_limits<int32_t>::max()) {
            return std::numeric_limits<int32_t>::max();
        }
        if (value < std::numeric_limits<int32_t>::min()) {
            return std::numeric_limits<int32_t>::min();
        }
        return static_cast<int32_t>(value);
    }

    // CRC16-Modbus
    uint16_t crc16_modbus(const uint8_t* data, uint16_t len) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
            }
        }
        return crc;
    }

    // 写入所有字节（处理短写）
    bool write_all(const uint8_t* buf, size_t len, int timeout_ms = 500) {
        size_t sent = 0;
        while (sent < len) {
            struct pollfd pfd;
            pfd.fd = serial_fd_;
            pfd.events = POLLOUT;
            int rv = poll(&pfd, 1, timeout_ms);
            if (rv <= 0) {
                RCLCPP_ERROR(this->get_logger(), "串口写等待超时或出错: %s", rv == 0 ? "timeout" : strerror(errno));
                return false;
            }
            ssize_t w = write(serial_fd_, buf + sent, len - sent);
            if (w < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(this->get_logger(), "串口写入失败: %s", strerror(errno));
                // USB 重枚举/掉线导致 fd 失效：标记需要重开
                if (errno == EIO || errno == EBADF || errno == ENXIO || errno == ENODEV) {
                    serial_needs_reopen_ = true;
                }
                return false;
            }
            sent += static_cast<size_t>(w);
        }
        return true;
    }

    // 读取精确长度（使用 poll 等待数据，不会长期阻塞）
    ssize_t read_exact(uint8_t* buf, size_t len, int timeout_ms) {
        size_t received = 0;
        int elapsed = 0;
        const int step_ms = 20; // poll 最小步长
        while (received < len && elapsed < timeout_ms) {
            struct pollfd pfd;
            pfd.fd = serial_fd_;
            pfd.events = POLLIN;
            int wait = std::min(step_ms, timeout_ms - elapsed);
            int rv = poll(&pfd, 1, wait);
            elapsed += wait;
            if (rv < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(this->get_logger(), "串口 poll 出错: %s", strerror(errno));
                return -1;
            } else if (rv == 0) {
                continue;
            } else {
                ssize_t r = read(serial_fd_, buf + received, len - received);
                if (r < 0) {
                    if (errno == EINTR) continue;
                    RCLCPP_ERROR(this->get_logger(), "串口读取失败: %s", strerror(errno));
                    // USB 重枚举/掉线导致 fd 失效：标记需要重开
                    if (errno == EIO || errno == EBADF || errno == ENXIO || errno == ENODEV) {
                        serial_needs_reopen_ = true;
                    }
                    return -1;
                } else if (r == 0) {
                    continue;
                } else {
                    received += static_cast<size_t>(r);
                }
            }
        }
        return static_cast<ssize_t>(received);
    }

    // termios 串口配置（raw, 2 stop bits, no parity）
    bool set_serial_config() {
        struct termios cfg;
        if (tcgetattr(serial_fd_, &cfg) < 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr 失败: %s", strerror(errno));
            return false;
        }

        cfmakeraw(&cfg);
        if (cfsetispeed(&cfg, BAUDRATE_) < 0 || cfsetospeed(&cfg, BAUDRATE_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "设置波特率失败");
            return false;
        }

        cfg.c_cflag &= ~CSIZE;
        cfg.c_cflag |= CS8;
        cfg.c_cflag &= ~PARENB;
        cfg.c_cflag |= CSTOPB;
        cfg.c_cflag &= ~CRTSCTS;
        cfg.c_cflag |= CLOCAL | CREAD;

        cfg.c_cc[VTIME] = 0;
        cfg.c_cc[VMIN] = 0;

        tcflush(serial_fd_, TCIOFLUSH);
        if (tcsetattr(serial_fd_, TCSANOW, &cfg) < 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr 失败: %s", strerror(errno));
            return false;
        }
        return true;
    }

    // 写寄存器（注意：函数保证在 mutex 内调用）
    bool write_reg_locked(uint16_t reg_addr, int16_t data) {
        uint8_t frame[8] = {0};
        frame[0] = SLAVE_ADDR_;
        frame[1] = 0x06;  // 写单寄存器
        frame[2] = (reg_addr >> 8) & 0xFF;
        frame[3] = reg_addr & 0xFF;
        frame[4] = (data >> 8) & 0xFF;
        frame[5] = data & 0xFF;

        uint16_t crc = crc16_modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = (crc >> 8) & 0xFF;

        if (!write_all(frame, sizeof(frame), COMM_TIMEOUT_MS_)) {
            RCLCPP_ERROR(this->get_logger(), "寄存器写入请求发送失败（地址0x%X）", reg_addr);
            return false;
        }

        uint8_t resp[8];
        ssize_t got = read_exact(resp, sizeof(resp), COMM_TIMEOUT_MS_);
        if (got != static_cast<ssize_t>(sizeof(resp))) {
            RCLCPP_ERROR(this->get_logger(), "寄存器写入响应超时或长度不符（期望8，实际%zd）", got);
            return false;
        }
        if (resp[0] != SLAVE_ADDR_ || resp[1] != 0x06) {
            if (resp[1] == static_cast<uint8_t>(0x86)) {
                RCLCPP_ERROR(this->get_logger(), "写寄存器异常响应，错误码0x%X", (int)resp[2]);
            } else {
                RCLCPP_ERROR(this->get_logger(), "寄存器写入响应格式错误");
            }
            return false;
        }
        return true;
    }

    // 公共写寄存器入口（带锁）
    bool write_reg(uint16_t reg_addr, int16_t data) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        return write_reg_locked(reg_addr, data);
    }

    // 读取 1 个寄存器（用于故障码）
    bool check_error(uint16_t& err_code) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        uint8_t frame[8] = {0};
        frame[0] = SLAVE_ADDR_;
        frame[1] = 0x03;
        frame[2] = 0x0B;  // 高
        frame[3] = 0x22;  // 低 => 0x0B22 = 2850
        frame[4] = 0x00;
        frame[5] = 0x01;  // 读 1 个寄存器

        uint16_t crc = crc16_modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = (crc >> 8) & 0xFF;

        if (!write_all(frame, 8, COMM_TIMEOUT_MS_)) {
            RCLCPP_ERROR(this->get_logger(), "故障码请求发送失败");
            return false;
        }

        uint8_t resp[7] = {0};
        ssize_t got = read_exact(resp, sizeof(resp), COMM_TIMEOUT_MS_);
        if (got <= 0) {
            RCLCPP_ERROR(this->get_logger(), "故障码响应超时或出错（len=%zd）", got);
            return false;
        }
        if (resp[1] == 0x03) {
            if (got < 7) {
                RCLCPP_ERROR(this->get_logger(), "故障码响应长度不足（%zd）", got);
                return false;
            }
            err_code = (resp[3] << 8) | resp[4];
            return true;
        } else if (resp[1] == 0x83) {
            RCLCPP_ERROR(this->get_logger(), "读取故障码被拒绝，异常码0x%X", resp[2]);
            return false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知故障码响应函数码: 0x%X", resp[1]);
            return false;
        }
    }

    // 读取 32-bit 位置（核心修复：修正字节解析顺序）
    bool read_position(int32_t& position) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        uint8_t frame[8] = {0};
        frame[0] = SLAVE_ADDR_;
        frame[1] = 0x03;
        frame[2] = (2823 >> 8) & 0xFF;  // H0B_07 地址=2823
        frame[3] = 2823 & 0xFF;
        frame[4] = 0x00;
        frame[5] = 0x02;  // 读2个寄存器（32位）

        uint16_t crc = crc16_modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = (crc >> 8) & 0xFF;

        if (!write_all(frame, 8, COMM_TIMEOUT_MS_)) {
            RCLCPP_ERROR(this->get_logger(), "位置读取请求发送失败");
            return false;
        }

        uint8_t resp[9] = {0};
        ssize_t got = read_exact(resp, sizeof(resp), COMM_TIMEOUT_MS_);
        if (got <= 0) {
            RCLCPP_ERROR(this->get_logger(), "位置读取响应超时或出错（len=%zd）", got);
            return false;
        }
        if (resp[1] == 0x03) {
            if (got < 9) {
                RCLCPP_ERROR(this->get_logger(), "位置响应长度不足（%zd）", got);
                return false;
            }
            // 核心修复：按 Modbus 标准解析 32 位数据
            // 响应数据格式：resp[3]（高字高字节）、resp[4]（高字低字节）、resp[5]（低字高字节）、resp[6]（低字低字节）
            int16_t high_word = (resp[3] << 8) | resp[4]; // 高字：高字节<<8 | 低字节
            int16_t low_word = (resp[5] << 8) | resp[6];  // 低字：高字节<<8 | 低字节
            position = ((int32_t)high_word << 16) | (uint16_t)low_word;  // 符号扩展正确
            return true;
        } else if (resp[1] == 0x83) {
            RCLCPP_ERROR(this->get_logger(), "位置读取异常响应，错误码0x%X", resp[2]);
            return false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知位置响应函数码: 0x%X", resp[1]);
            return false;
        }
    }

    // 紧急停止（不可在 signal handler 中调用）
    void emergency_stop_locked() {
        uint16_t addr = 771;
        write_reg_locked(addr, 0);
        RCLCPP_WARN(this->get_logger(), "电机紧急停止（已发送断开使能）");
        is_running_ = false;
    }

    void emergency_stop() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ >= 0) {
            emergency_stop_locked();
        } else {
            is_running_ = false;
        }
    }

    // 初始化电机参数（不在信号处理里调用）
    bool init_motor_params() {
        RCLCPP_INFO(this->get_logger(), "初始化电机参数...");
        if (!write_reg(770, 1)) return false;   // H03_02 = 1
        if (!write_reg(771, 0)) return false;   // H03_03 = 0
        if (!write_reg(512, 0)) return false;   // H02_00 = 0 (速度模式)
        if (!write_reg(1538, 0)) return false;  // H06_02 = 0 (内部速度来源)
        if (!write_reg(3085, 1)) {              // 保存到 EEPROM
            RCLCPP_WARN(this->get_logger(), "保存参数到 EEPROM 失败（非致命）");
        }
        RCLCPP_INFO(this->get_logger(), "电机参数初始化完成");
        return true;
    }

    // 控制回调（subscriber）
    void cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!is_running_) return;
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "收到无效控制指令长度");
            return;
        }

        int8_t direction = static_cast<int8_t>(msg->data[0]);
        int16_t speed = static_cast<int16_t>(msg->data[1]);

        if (direction == 0) {
            if (!write_reg(771, 0)) {
                RCLCPP_ERROR(this->get_logger(), "发送停止命令失败");
            } else {
                RCLCPP_INFO(this->get_logger(), "收到停止命令，已断开使能");
            }
            return;
        }

        speed = std::max<int16_t>(MIN_SPEED_, std::min<int16_t>(MAX_SPEED_, speed));

        if (direction == 1) {
            if (!write_reg(1539, speed)) {
                RCLCPP_ERROR(this->get_logger(), "设置正转速度失败");
                return;
            }
        } else if (direction == -1) {
            if (!write_reg(1539, static_cast<int16_t>(-speed))) {
                RCLCPP_ERROR(this->get_logger(), "设置反转速度失败");
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "无效方向指令：%d（仅支持1/-1/0）", direction);
            return;
        }

        if (!write_reg(771, 1)) {
            RCLCPP_ERROR(this->get_logger(), "使能电机失败");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "电机已使能，方向=%d 速度=%d", direction, abs(speed));
        }
    }

    // 串口自愈：USB 重枚举等导致 fd 失效时，重新打开并初始化
    bool reopen_serial() {
        RCLCPP_WARN(this->get_logger(), "检测到串口失效，尝试重开 %s ...", SERIAL_DEV_.c_str());

        // 关闭旧 fd（加锁防止与 emergency_stop 并发）
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (serial_fd_ >= 0) {
                ::close(serial_fd_);
                serial_fd_ = -1;
            }
        }

        // 给 udev 一点时间稳定 symlink（USB 刚重枚举时可能短暂不一致）
        usleep(100 * 1000);

        // 重新打开（/dev/lift_motor 是 symlink，open 会跟随到当前实际设备）
        serial_fd_ = open(SERIAL_DEV_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "重开串口失败：%s (%s)", SERIAL_DEV_.c_str(), strerror(errno));
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "重开：串口已打开 (fd=%d)", serial_fd_);

        if (!set_serial_config()) {
            RCLCPP_ERROR(this->get_logger(), "重开：串口配置失败");
            ::close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // 检查故障码并复位
        uint16_t err_code = 0;
        if (check_error(err_code)) {
            if (err_code != 0) {
                RCLCPP_WARN(this->get_logger(), "重开后检测到故障码: 0x%X，尝试复位...", err_code);
                if (!write_reg(3329, 1)) {
                    RCLCPP_ERROR(this->get_logger(), "重开：故障复位失败");
                } else {
                    usleep(200 * 1000);
                }
            }
        }

        // 重新初始化电机参数
        if (!init_motor_params()) {
            RCLCPP_ERROR(this->get_logger(), "重开：电机参数初始化失败");
            ::close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "串口自愈成功，恢复正常控制");
        return true;
    }

    void publish_position() {
        if (!is_running_) return;

        // 自愈：fd 失效（USB 重枚举等）时自动重开串口
        if (serial_needs_reopen_.exchange(false) || serial_fd_ < 0) {
            if (!reopen_serial()) {
                return;  // 重开失败，下个周期(500ms)再试
            }
        }

        int32_t position = 0;
        if (!read_position(position)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "读取升降机原始位置失败"
            );
            return;
        }

        std_msgs::msg::Int32 msg;
        msg.data = position;
        position_pub_->publish(msg);

        const int64_t relative_position =
            static_cast<int64_t>(position) - static_cast<int64_t>(position_zero_raw_);
        std_msgs::msg::Int32 relative_msg;
        relative_msg.data = clamp_to_int32(relative_position);
        relative_position_pub_->publish(relative_msg);
    }

public:
    MotorController() :
        Node("lift_control_node"),
        serial_fd_(-1),
        is_running_(false),
        position_zero_raw_(0) {

        const int64_t zero_param = this->declare_parameter<int64_t>("position_zero_raw", 0);
        position_zero_raw_ = clamp_to_int32(zero_param);

        serial_fd_ = open(SERIAL_DEV_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "串口打开失败：%s (%s)", SERIAL_DEV_.c_str(), strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "串口打开：%s", SERIAL_DEV_.c_str());

        if (!set_serial_config()) {
            RCLCPP_FATAL(this->get_logger(), "串口配置失败");
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        uint16_t err_code = 0;
        if (check_error(err_code)) {
            if (err_code != 0) {
                RCLCPP_WARN(this->get_logger(), "检测到驱动器故障码: 0x%X，尝试复位...", err_code);
                if (!write_reg(3329, 1)) {
                    RCLCPP_FATAL(this->get_logger(), "故障复位失败，请人工检查驱动器");
                    close(serial_fd_);
                    serial_fd_ = -1;
                    return;
                }
                usleep(200 * 1000);
                RCLCPP_INFO(this->get_logger(), "故障复位发送完毕");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "读取故障码失败（继续尝试初始化）");
        }

        if (!init_motor_params()) {
            RCLCPP_FATAL(this->get_logger(), "电机参数初始化失败，退出");
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/lift_control_cmd", 10,
            std::bind(&MotorController::cmd_callback, this, std::placeholders::_1)
        );

        position_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/lift/position_raw", 10
        );
        relative_position_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/lift/position_relative_raw", 10
        );

        is_running_ = true;
        position_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(POSITION_PUBLISH_PERIOD_MS_),
            std::bind(&MotorController::publish_position, this)
        );

        RCLCPP_INFO(this->get_logger(), "电机控制节点启动成功");
    }

    ~MotorController() {
        if (serial_fd_ >= 0) {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            write_reg_locked(771, 0);
            ::close(serial_fd_);
            serial_fd_ = -1;
            RCLCPP_INFO(this->get_logger(), "串口已关闭（析构）");
        }
    }

    void run() {
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "串口未就绪，退出节点");
            rclcpp::shutdown();
            return;
        }
        rclcpp::spin(shared_from_this());
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    node->run();
    rclcpp::shutdown();
    return 0;
}


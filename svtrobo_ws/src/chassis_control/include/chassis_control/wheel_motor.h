#ifndef __WHEEL_MOTOR_H__
#define __WHEEL_MOTOR_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <ifaddrs.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <mutex>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <poll.h>
#include <sstream>
#include <iomanip>
#include <optional>
#include <atomic>

// CANopen SDO异常类声明
class CanopenSdoError : public std::runtime_error {
public:
    explicit CanopenSdoError(const std::string &msg);
};

// SocketCAN封装类声明
class SocketCan
{
public:
    explicit SocketCan(const std::string &ifname, const std::vector<can_filter>& filters);
    ~SocketCan();

    void send_frame(const struct can_frame &frame);
    std::pair<bool, struct can_frame> recv_frame(int timeout_ms);
    void drain(int max_iters = 200);

private:
    int sock_{-1};
    std::mutex tx_mutex_;
};

// ZLAC8015D电机驱动器控制类声明
class ZLAC8015D {
public:
    ZLAC8015D(const std::string &channel, int node_id, double recv_timeout = 0.3);
    ZLAC8015D(std::shared_ptr<SocketCan> external_socket, int node_id, double recv_timeout = 0.3);
    ~ZLAC8015D() = default;

    // NMT控制
    void nmt_start();
    int wait_heartbeat(double timeout_s = 2.0);

    // SDO读写通用接口
    void sdo_write(uint16_t index, uint8_t sub, const std::vector<uint8_t>& payload);
    std::vector<uint8_t> sdo_read(uint16_t index, uint8_t sub, double timeout_s = -1.0);

    // 类型化SDO读写接口
    void sdo_write_i8(uint16_t index, uint8_t sub, int8_t val);
    void sdo_write_u16(uint16_t index, uint8_t sub, uint16_t val);
    int32_t sdo_read_i32(uint16_t index, uint8_t sub);
    uint32_t sdo_read_u32(uint16_t index, uint8_t sub);

    // 电机操作接口
    void set_velocity_mode();
    void enable_operation();
    void stop();
    void quick_stop();
    void clear_fault();

    // 速度控制与读取
    void set_target_speed_lr_rpm(int16_t left_rpm, int16_t right_rpm);
    std::pair<int16_t,int16_t> read_actual_speed_lr_0p1rpm();
    std::pair<int32_t,int32_t> read_encoder_lr();
    std::pair<uint16_t,uint16_t> read_statusword_lr();
    uint32_t read_fault_code_u32();

private:
    int node_id_;
    uint32_t sdo_tx_;
    uint32_t sdo_rx_;
    uint32_t hb_id_;
    double recv_timeout_s_;
    std::shared_ptr<SocketCan> socket_;

    // 私有辅助函数声明
    void init_socket(const std::string &channel);
    std::optional<struct can_frame> wait_sdo_resp(uint16_t index, uint8_t sub, double timeout_s);
    uint32_t extract_abort_code(const struct can_frame &frame);
    static std::string frame_dump(const struct can_frame &f);
};

#endif
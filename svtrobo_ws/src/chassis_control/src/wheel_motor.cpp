#include "chassis_control/wheel_motor.h"

// ====================== CanopenSdoError 实现 ======================
CanopenSdoError::CanopenSdoError(const std::string &msg) : std::runtime_error(msg) {}

// ====================== SocketCan 实现 ======================
SocketCan::SocketCan(const std::string &ifname, const std::vector<can_filter>& filters) {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        throw std::runtime_error("socket(PF_CAN) failed: " + std::string(strerror(errno)));
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        close(sock_);
        throw std::runtime_error("SIOCGIFINDEX failed for " + ifname + ": " + std::string(strerror(errno)));
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 禁用回环帧
    int loopback = 0;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback)) < 0) {
        std::ostringstream ss;
        ss << "Warning: CAN_RAW_RECV_OWN_MSGS setsockopt failed: " << strerror(errno);
        std::cerr << ss.str() << std::endl;
    }

    // 过滤错误帧
    int err_mask = 0;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        std::ostringstream ss;
        ss << "Warning: CAN_RAW_ERR_FILTER setsockopt failed: " << strerror(errno);
        std::cerr << ss.str() << std::endl;
    }

    // 设置CAN过滤器
    if (!filters.empty()) {
        if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), static_cast<int>(filters.size() * sizeof(can_filter))) < 0) {
            close(sock_);
            throw std::runtime_error("setsockopt CAN_RAW_FILTER failed: " + std::string(strerror(errno)));
        }
    }

    if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock_);
        throw std::runtime_error("bind CAN socket failed: " + std::string(strerror(errno)));
    }
}

SocketCan::~SocketCan() {
    if (sock_ >= 0) close(sock_);
}

void SocketCan::send_frame(const struct can_frame &frame) {
    std::lock_guard<std::mutex> lk(tx_mutex_);
    ssize_t n = write(sock_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        std::ostringstream ss;
        ss << "CAN send failed: wrote=" << n << " (" << strerror(errno) << ")";
        throw std::runtime_error(ss.str());
    }
}

std::pair<bool, struct can_frame> SocketCan::recv_frame(int timeout_ms) {
    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
        return {false, {}};
    }
    struct can_frame frame;
    ssize_t n = read(sock_, &frame, sizeof(frame));
    if (n < 0) {
        std::ostringstream ss;
        ss << "CAN read failed: " << strerror(errno);
        throw std::runtime_error(ss.str());
    }
    if (n < static_cast<ssize_t>(sizeof(struct can_frame))) {
        std::ostringstream ss;
        ss << "Short CAN frame read: n=" << n;
        throw std::runtime_error(ss.str());
    }
    return {true, frame};
}

void SocketCan::drain(int max_iters) {
    for (int i = 0; i < max_iters; ++i) {
        struct can_frame f;
        ssize_t n = recv(sock_, &f, sizeof(f), MSG_DONTWAIT);
        if (n <= 0) break;
    }
}

// ====================== ZLAC8015D 实现 ======================
ZLAC8015D::ZLAC8015D(const std::string &channel, int node_id, double recv_timeout)
    : node_id_(node_id),
      sdo_tx_(0x600 + node_id),
      sdo_rx_(0x580 + node_id),
      hb_id_(0x700 + node_id),
      recv_timeout_s_(recv_timeout)
{
    init_socket(channel);
}

ZLAC8015D::ZLAC8015D(std::shared_ptr<SocketCan> external_socket, int node_id, double recv_timeout)
    : node_id_(node_id),
      sdo_tx_(0x600 + node_id),
      sdo_rx_(0x580 + node_id),
      hb_id_(0x700 + node_id),
      recv_timeout_s_(recv_timeout),
      socket_(std::move(external_socket))
{
    if (!socket_) throw std::invalid_argument("external_socket is null");
}

void ZLAC8015D::nmt_start() {
    struct can_frame f {};
    f.can_id = 0x000;
    f.can_dlc = 2;
    f.data[0] = 0x01;
    f.data[1] = static_cast<uint8_t>(node_id_);
    socket_->send_frame(f);
}

int ZLAC8015D::wait_heartbeat(double timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining < 0) remaining = 0;
        auto [ok, frame] = socket_->recv_frame(static_cast<int>(remaining));
        if (!ok) continue;
        uint32_t id = frame.can_id & CAN_SFF_MASK;
        if (id == hb_id_ && frame.can_dlc >= 1) {
            return static_cast<int>(frame.data[0]);
        }
    }
    return -1;
}

void ZLAC8015D::sdo_write(uint16_t index, uint8_t sub, const std::vector<uint8_t>& payload) {
    if (!(payload.size() == 1 || payload.size() == 2 || payload.size() == 4)) {
        throw std::invalid_argument("payload must be 1,2 or 4 bytes for expedited write");
    }
    uint8_t cmd = (payload.size() == 1) ? 0x2F : (payload.size() == 2 ? 0x2B : 0x23);
    struct can_frame req {};
    req.can_id = sdo_tx_;
    req.can_dlc = 8;
    req.data[0] = cmd;
    req.data[1] = static_cast<uint8_t>(index & 0xFF);
    req.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    req.data[3] = static_cast<uint8_t>(sub & 0xFF);
    for (size_t i = 0; i < payload.size(); ++i) req.data[4 + i] = payload[i];
    for (size_t i = payload.size(); i < 4; ++i) req.data[4 + i] = 0;

    socket_->drain();
    socket_->send_frame(req);
    auto resp = wait_sdo_resp(index, sub, recv_timeout_s_);
    if (!resp.has_value()) {
        std::ostringstream ss;
        ss << "SDO write timeout 0x" << std::hex << std::setw(4) << std::setfill('0') << index
           << ":" << std::setw(2) << static_cast<int>(sub);
        throw std::runtime_error(ss.str());
    }
    const auto &frame = resp.value();
    if (frame.can_dlc >= 1 && frame.data[0] == 0x80) {
        uint32_t abort = extract_abort_code(frame);
        std::ostringstream ss;
        ss << "SDO abort 0x" << std::hex << std::setw(8) << std::setfill('0') << abort
           << " on write 0x" << std::setw(4) << index << ":" << std::setw(2) << static_cast<int>(sub);
        throw CanopenSdoError(ss.str());
    }
    if (!(frame.can_dlc >= 1 && frame.data[0] == 0x60)) {
        std::ostringstream ss;
        ss << "Unexpected SDO write response: data=" << frame_dump(frame);
        throw CanopenSdoError(ss.str());
    }
}

std::vector<uint8_t> ZLAC8015D::sdo_read(uint16_t index, uint8_t sub, double timeout_s) {
    if (timeout_s < 0) timeout_s = recv_timeout_s_;
    struct can_frame req {};
    req.can_id = sdo_tx_;
    req.can_dlc = 8;
    req.data[0] = 0x40;
    req.data[1] = static_cast<uint8_t>(index & 0xFF);
    req.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    req.data[3] = sub;
    for (int i = 4; i < 8; ++i) req.data[i] = 0;

    socket_->drain();
    socket_->send_frame(req);
    auto resp = wait_sdo_resp(index, sub, timeout_s);
    if (!resp.has_value()) {
        std::ostringstream ss;
        ss << "SDO read timeout 0x" << std::hex << std::setw(4) << std::setfill('0') << index
           << ":" << std::setw(2) << static_cast<int>(sub);
        throw std::runtime_error(ss.str());
    }
    const auto &frame = resp.value();
    if (frame.can_dlc >= 1 && frame.data[0] == 0x80) {
        uint32_t abort = extract_abort_code(frame);
        std::ostringstream ss;
        ss << "SDO abort 0x" << std::hex << std::setw(8) << std::setfill('0') << abort
           << " on read 0x" << std::setw(4) << index << ":" << std::setw(2) << static_cast<int>(sub);
        throw CanopenSdoError(ss.str());
    }
    if (frame.can_dlc < 4) {
        std::ostringstream ss;
        ss << "SDO read response too short: dlc=" << frame.can_dlc;
        throw std::runtime_error(ss.str());
    }
    uint8_t cmd = frame.data[0];
    if (cmd == 0x4F) {
        return { frame.data[4] };
    } else if (cmd == 0x4B) {
        return { frame.data[4], frame.data[5] };
    } else if (cmd == 0x43) {
        return { frame.data[4], frame.data[5], frame.data[6], frame.data[7] };
    } else {
        std::vector<uint8_t> out;
        for (int i = 4; i < static_cast<int>(frame.can_dlc) && i < 8; ++i) out.push_back(frame.data[i]);
        return out;
    }
}

void ZLAC8015D::sdo_write_i8(uint16_t index, uint8_t sub, int8_t val) {
    std::vector<uint8_t> p{ static_cast<uint8_t>(val) };
    sdo_write(index, sub, p);
}

void ZLAC8015D::sdo_write_u16(uint16_t index, uint8_t sub, uint16_t val) {
    std::vector<uint8_t> p{ static_cast<uint8_t>(val & 0xFF), static_cast<uint8_t>((val >> 8) & 0xFF) };
    sdo_write(index, sub, p);
}

int32_t ZLAC8015D::sdo_read_i32(uint16_t index, uint8_t sub) {
    auto b = sdo_read(index, sub);
    if (b.size() >= 4) {
        int32_t v = static_cast<int32_t>(
            (static_cast<uint32_t>(b[0])      ) |
            (static_cast<uint32_t>(b[1]) << 8 ) |
            (static_cast<uint32_t>(b[2]) << 16) |
            (static_cast<uint32_t>(b[3]) << 24));
        return v;
    }
    uint8_t sign_byte = b.empty() ? 0x00 : b.back();
    bool negative = (sign_byte & 0x80) != 0;
    std::vector<uint8_t> b4 = b;
    while (b4.size() < 4) b4.push_back(negative ? 0xFF : 0x00);
    int32_t v = static_cast<int32_t>(
        (static_cast<uint32_t>(b4[0])      ) |
        (static_cast<uint32_t>(b4[1]) << 8 ) |
        (static_cast<uint32_t>(b4[2]) << 16) |
        (static_cast<uint32_t>(b4[3]) << 24));
    return v;
}

uint32_t ZLAC8015D::sdo_read_u32(uint16_t index, uint8_t sub) {
    auto b = sdo_read(index, sub);
    if (b.size() >= 4) {
        uint32_t v =
            (static_cast<uint32_t>(b[0])      ) |
            (static_cast<uint32_t>(b[1]) << 8 ) |
            (static_cast<uint32_t>(b[2]) << 16) |
            (static_cast<uint32_t>(b[3]) << 24);
        return v;
    }
    uint32_t v = 0;
    for (size_t i = 0; i < b.size(); ++i) v |= (static_cast<uint32_t>(b[i]) << (8 * i));
    return v;
}

void ZLAC8015D::set_velocity_mode() { sdo_write_i8(0x6060, 0x00, 3); }

void ZLAC8015D::enable_operation() {
    sdo_write_u16(0x6040, 0x00, 0x0006);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    sdo_write_u16(0x6040, 0x00, 0x0007);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    sdo_write_u16(0x6040, 0x00, 0x000F);
}

void ZLAC8015D::stop() { sdo_write_u16(0x6040, 0x00, 0x0000); }

void ZLAC8015D::quick_stop() { sdo_write_u16(0x6040, 0x00, 0x0002); }

void ZLAC8015D::clear_fault() { sdo_write_u16(0x6040, 0x00, 0x0080); }

void ZLAC8015D::set_target_speed_lr_rpm(int16_t left_rpm, int16_t right_rpm) {
    std::vector<uint8_t> p(4);
    p[0] = static_cast<uint8_t>(left_rpm & 0xFF); p[1] = static_cast<uint8_t>((left_rpm>>8)&0xFF);
    p[2] = static_cast<uint8_t>(right_rpm & 0xFF); p[3] = static_cast<uint8_t>((right_rpm>>8)&0xFF);
    sdo_write(0x60FF, 0x03, p);
}

std::pair<int16_t,int16_t> ZLAC8015D::read_actual_speed_lr_0p1rpm() {
    auto raw = sdo_read(0x606C, 0x03);
    while (raw.size() < 4) raw.push_back(0);
    int16_t left = static_cast<int16_t>((raw[0]) | (raw[1] << 8));
    int16_t right= static_cast<int16_t>((raw[2]) | (raw[3] << 8));
    return {left, right};
}

std::pair<int32_t,int32_t> ZLAC8015D::read_encoder_lr() {
    return { sdo_read_i32(0x6064, 0x01), sdo_read_i32(0x6064, 0x02) };
}

std::pair<uint16_t,uint16_t> ZLAC8015D::read_statusword_lr() {
    uint32_t sw = sdo_read_u32(0x6041, 0x00);
    return { static_cast<uint16_t>(sw & 0xFFFF), static_cast<uint16_t>((sw >> 16) & 0xFFFF) };
}

uint32_t ZLAC8015D::read_fault_code_u32() {
    return sdo_read_u32(0x603F, 0x00);
}

void ZLAC8015D::init_socket(const std::string &channel) {
    std::vector<can_filter> filters;
    can_filter f1{static_cast<__u32>(sdo_rx_), 0x7FF};
    can_filter f2{static_cast<__u32>(hb_id_),  0x7FF};
    filters.push_back(f1);
    filters.push_back(f2);
    socket_ = std::make_shared<SocketCan>(channel, filters);
}

std::optional<struct can_frame> ZLAC8015D::wait_sdo_resp(uint16_t index, uint8_t sub, double timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    uint8_t idx_lo = static_cast<uint8_t>(index & 0xFF);
    uint8_t idx_hi = static_cast<uint8_t>((index >> 8) & 0xFF);
    while (std::chrono::steady_clock::now() < deadline) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining < 0) remaining = 0;
        auto [ok, frame] = socket_->recv_frame(static_cast<int>(remaining));
        if (!ok) continue;
        uint32_t id = frame.can_id & CAN_SFF_MASK;
        if (id != sdo_rx_) continue;
        if (frame.can_dlc < 4) continue;
        if (frame.data[1] == idx_lo && frame.data[2] == idx_hi && frame.data[3] == sub) {
            return frame;
        }
    }
    return std::nullopt;
}

uint32_t ZLAC8015D::extract_abort_code(const struct can_frame &frame) {
    if (frame.can_dlc >= 8) {
        uint32_t abort = (static_cast<uint32_t>(frame.data[4])      ) |
                         (static_cast<uint32_t>(frame.data[5]) << 8 ) |
                         (static_cast<uint32_t>(frame.data[6]) << 16) |
                         (static_cast<uint32_t>(frame.data[7]) << 24);
        return abort;
    }
    return 0;
}

std::string ZLAC8015D::frame_dump(const struct can_frame &f) {
    std::ostringstream ss;
    ss << "can_id=0x" << std::hex << (f.can_id & CAN_SFF_MASK) << " dlc=" << std::dec << int(f.can_dlc) << " data=";
    for (int i = 0; i < f.can_dlc && i < 8; ++i) {
        ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(static_cast<uint8_t>(f.data[i]));
    }
    return ss.str();
}
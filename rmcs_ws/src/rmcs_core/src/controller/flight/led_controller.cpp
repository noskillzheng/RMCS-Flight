#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <string>
#include <string_view>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <fcntl.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/string.hpp>

namespace rmcs_core::controller::flight {

namespace {

constexpr std::uint32_t kSwdMailboxBase = 0x20000000U;
constexpr std::uint32_t kSwdMailboxMagicAddr = kSwdMailboxBase + 0x00U;
constexpr std::uint32_t kSwdMailboxCmdAddr = kSwdMailboxBase + 0x04U;
constexpr std::uint32_t kSwdMailboxStatusAddr = kSwdMailboxBase + 0x08U;
constexpr std::uint32_t kSwdMailboxSeqAddr = kSwdMailboxBase + 0x0CU;
constexpr std::uint32_t kSwdMailboxMagic = 0x4C454431U; // "LED1"
constexpr std::uint32_t kGpioaOdrAddr = 0x4001080CU;

constexpr std::uint8_t kDapConnect = 0x02U;
constexpr std::uint8_t kDapDisconnect = 0x03U;
constexpr std::uint8_t kDapTransferConfigure = 0x04U;
constexpr std::uint8_t kDapTransfer = 0x05U;
constexpr std::uint8_t kDapSwjClock = 0x11U;
constexpr std::uint8_t kDapSwjSequence = 0x12U;

constexpr std::uint8_t kDapPortSwd = 0x01U;
constexpr std::uint8_t kDapTransferOk = 0x01U;
constexpr std::uint8_t kDapTransferWait = 0x02U;

constexpr std::uint8_t kDpReadIdcode = 0x02U;
constexpr std::uint8_t kDpWriteAbort = 0x00U;
constexpr std::uint8_t kDpWriteCtrlStat = 0x04U;
constexpr std::uint8_t kDpReadCtrlStat = 0x06U;
constexpr std::uint8_t kDpWriteSelect = 0x08U;

constexpr std::uint8_t kApWriteCsw = 0x01U;
constexpr std::uint8_t kApWriteTar = 0x05U;
constexpr std::uint8_t kApWriteDrw = 0x0DU;
constexpr std::uint8_t kApReadDrw = 0x0FU;

constexpr std::uint32_t kCsw32BitNoInc = 0x23000002U;

struct LedSnapshot {
    bool connected = false;
    std::uint8_t live_mask = 0U;
    std::uint8_t target_mask = 0U;
    std::uint8_t effect_mode = 0U;
    char last_cmd = '0';
    std::uint32_t seq = 0U;
    std::uint32_t gpioa_odr = 0x0EU;
    std::string transport_desc = "disconnected";
    std::string detail{};
};

std::string to_upper_ascii(std::string_view input) {
    std::string out{input};
    std::ranges::transform(
        out, out.begin(), [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    return out;
}

std::string to_lower_ascii(std::string_view input) {
    std::string out{input};
    std::ranges::transform(
        out, out.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return out;
}

std::string_view trim_ascii(std::string_view sv) {
    while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.front())) != 0)
        sv.remove_prefix(1);
    while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.back())) != 0)
        sv.remove_suffix(1);
    return sv;
}

const char* color_name_from_mask(std::uint8_t mask) noexcept {
    switch (mask & 0x07U) {
    case 0x00U: return "OFF";
    case 0x01U: return "RED (PA1)";
    case 0x02U: return "GREEN (PA2)";
    case 0x03U: return "YELLOW (PA1+PA2)";
    case 0x04U: return "BLUE (PA3)";
    case 0x05U: return "PURPLE (PA1+PA3)";
    case 0x06U: return "CYAN (PA2+PA3)";
    case 0x07U: return "WHITE (PA1+PA2+PA3)";
    default: return "OFF";
    }
}

const char* effect_name_from_mode(std::uint8_t mode) noexcept {
    switch (mode) {
    case 0U: return "STATIC";
    case 1U: return "FLOW (R->Y->G->C->B->M)";
    case 2U: return "BLINK";
    case 3U: return "BREATH";
    default: return "UNKNOWN";
    }
}

bool is_valid_command_char(char ch) noexcept {
    switch (ch) {
    case 'r':
    case 'R':
    case 'g':
    case 'G':
    case 'b':
    case 'B':
    case 'y':
    case 'Y':
    case 'c':
    case 'C':
    case 'm':
    case 'M':
    case 'p':
    case 'P':
    case 'w':
    case 'W':
    case 'f':
    case 'F':
    case 'k':
    case 'K':
    case 'e':
    case 'E':
    case 'o':
    case 'O':
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7': return true;
    default: return false;
    }
}

std::optional<char> parse_command_token(std::string_view raw) {
    const auto token = to_lower_ascii(trim_ascii(raw));
    if (token.empty())
        return std::nullopt;
    if (token.size() == 1U && is_valid_command_char(token[0]))
        return token[0];
    if (token == "red")
        return 'r';
    if (token == "green")
        return 'g';
    if (token == "blue")
        return 'b';
    if (token == "yellow")
        return 'y';
    if (token == "cyan")
        return 'c';
    if (token == "purple" || token == "magenta")
        return 'm';
    if (token == "white")
        return 'w';
    if (token == "flow")
        return 'f';
    if (token == "blink")
        return 'k';
    if (token == "breath")
        return 'e';
    if (token == "off")
        return 'o';
    if (token == "status")
        return '?';
    return std::nullopt;
}

void write_u32_le(std::uint8_t* dst, std::uint32_t value) noexcept {
    dst[0] = static_cast<std::uint8_t>(value & 0xFFU);
    dst[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    dst[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    dst[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::uint32_t read_u32_le(const std::uint8_t* src) noexcept {
    return static_cast<std::uint32_t>(src[0]) | (static_cast<std::uint32_t>(src[1]) << 8U)
         | (static_cast<std::uint32_t>(src[2]) << 16U)
         | (static_cast<std::uint32_t>(src[3]) << 24U);
}

class CmsisDapMailboxClient {
public:
    CmsisDapMailboxClient() = default;
    ~CmsisDapMailboxClient() { close_device(); }

    CmsisDapMailboxClient(const CmsisDapMailboxClient&) = delete;
    CmsisDapMailboxClient& operator=(const CmsisDapMailboxClient&) = delete;

    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }
    [[nodiscard]] const std::string& description() const noexcept { return description_; }

    static std::optional<std::pair<std::string, std::string>> discover_hidraw() {
        namespace fs = std::filesystem;
        std::error_code ec;
        if (!fs::exists("/sys/class/hidraw", ec))
            return std::nullopt;

        std::vector<fs::path> entries;
        for (const auto& entry : fs::directory_iterator("/sys/class/hidraw", ec))
            entries.push_back(entry.path());
        std::ranges::sort(entries);

        for (const auto& sys_dir : entries) {
            const auto dev_name = sys_dir.filename().string();
            const auto dev_node = std::string{"/dev/"} + dev_name;
            if (!fs::exists(dev_node, ec))
                continue;

            std::ifstream uevent{sys_dir / "device" / "uevent"};
            if (!uevent.is_open())
                continue;

            std::string hid_name;
            std::string hid_id;
            std::string line;
            while (std::getline(uevent, line)) {
                if (line.starts_with("HID_NAME="))
                    hid_name = line.substr(9);
                else if (line.starts_with("HID_ID="))
                    hid_id = to_upper_ascii(line.substr(7));
            }

            const auto upper_name = to_upper_ascii(hid_name);
            const bool known_vid_pid =
                hid_id.ends_with("00000416:00005021") || hid_id.ends_with("00000D28:00000204")
                || hid_id.ends_with("0000C251:0000F001") || hid_id.ends_with("00001A86:00008011")
                || hid_id.ends_with("00002E8A:0000000C");
            if (upper_name.find("CMSIS-DAP") != std::string::npos
                || upper_name.find("FIREDAP") != std::string::npos || known_vid_pid) {
                return std::pair{dev_node, hid_name.empty() ? dev_name : hid_name};
            }
        }
        return std::nullopt;
    }

    bool connect_device(const std::string& explicit_path, std::string& error_out) {
        close_device();

        std::string node_path = explicit_path;
        std::string dev_label = explicit_path;
        if (node_path.empty()) {
            const auto found = discover_hidraw();
            if (!found.has_value()) {
                error_out = "No CMSIS-DAP HID device found under /dev/hidraw*";
                return false;
            }
            node_path = found->first;
            dev_label = found->second;
        }

        fd_ = ::open(node_path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            error_out = std::format("Cannot open {}: {}", node_path, std::strerror(errno));
            return false;
        }

        // Drain stale HID reports
        std::array<std::uint8_t, 128> drain_buf{};
        while (::read(fd_, drain_buf.data(), drain_buf.size()) > 0) {}

        if (!init_swd(error_out)) {
            close_device();
            return false;
        }

        std::uint32_t magic = 0U;
        if (!read32(kSwdMailboxMagicAddr, magic, error_out)) {
            close_device();
            return false;
        }
        if (magic != kSwdMailboxMagic) {
            error_out = std::format(
                "SWD mailbox magic mismatch at 0x20000000 (got 0x{:08X}, expected 0x{:08X})", magic,
                kSwdMailboxMagic);
            close_device();
            return false;
        }

        description_ = std::format("SWD {} ({})", dev_label, node_path);
        return true;
    }

    void close_device() noexcept {
        if (fd_ >= 0) {
            std::array<std::uint8_t, 64> resp{};
            const std::array<std::uint8_t, 1> req{kDapDisconnect};
            (void)send_packet(req.data(), req.size(), resp);
            (void)::close(fd_);
            fd_ = -1;
            csw_initialized_ = false;
        }
    }

    bool send_led_command(char ch, LedSnapshot& snapshot_out, std::string& error_out) {
        const auto code = static_cast<std::uint32_t>(static_cast<unsigned char>(ch));
        if (!write32(kSwdMailboxCmdAddr, code, error_out))
            return false;

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{250};
        while (std::chrono::steady_clock::now() < deadline) {
            std::uint32_t pending = 1U;
            if (!read32(kSwdMailboxCmdAddr, pending, error_out))
                return false;
            if (pending == 0U)
                return read_snapshot(snapshot_out, error_out);
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }

        error_out = "Timed out waiting for MCU SWD mailbox acknowledgement";
        return false;
    }

    bool read_snapshot(LedSnapshot& snapshot_out, std::string& error_out) {
        std::uint32_t status = 0U;
        std::uint32_t seq = 0U;
        std::uint32_t odr = 0U;
        if (!read32(kSwdMailboxStatusAddr, status, error_out)
            || !read32(kSwdMailboxSeqAddr, seq, error_out)
            || !read32(kGpioaOdrAddr, odr, error_out))
            return false;

        snapshot_out.connected = true;
        snapshot_out.live_mask = static_cast<std::uint8_t>(status & 0x07U);
        snapshot_out.effect_mode = static_cast<std::uint8_t>((status >> 8U) & 0xFFU);
        const auto raw_cmd = static_cast<unsigned char>((status >> 16U) & 0xFFU);
        snapshot_out.last_cmd =
            (raw_cmd >= 32U && raw_cmd <= 126U) ? static_cast<char>(raw_cmd) : '?';
        snapshot_out.target_mask = static_cast<std::uint8_t>((status >> 24U) & 0x07U);
        snapshot_out.seq = seq;
        snapshot_out.gpioa_odr = odr;
        snapshot_out.transport_desc = description_;
        snapshot_out.detail.clear();
        return true;
    }

private:
    struct SwdOp {
        std::uint8_t req;
        std::uint32_t value;
    };

    bool send_packet(
        const std::uint8_t* payload, std::size_t len, std::array<std::uint8_t, 64>& resp_out,
        std::string* error_out = nullptr) const {
        if (fd_ < 0 || len == 0U || len > 64U) {
            if (error_out != nullptr)
                *error_out = "Invalid CMSIS-DAP packet state";
            return false;
        }

        std::array<std::uint8_t, 65> tx{};
        tx[0] = 0x00U; // Linux hidraw report ID prefix
        std::memcpy(tx.data() + 1, payload, len);
        if (::write(fd_, tx.data(), tx.size()) < 0) {
            if (error_out != nullptr)
                *error_out = std::format("HID write failed: {}", std::strerror(errno));
            return false;
        }

        pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
        const int poll_rc = ::poll(&pfd, 1, 500);
        if (poll_rc <= 0) {
            if (error_out != nullptr)
                *error_out = "Timeout waiting for CMSIS-DAP HID response";
            return false;
        }

        std::array<std::uint8_t, 65> rx{};
        const ssize_t nread = ::read(fd_, rx.data(), rx.size());
        if (nread <= 0) {
            if (error_out != nullptr)
                *error_out = "Empty CMSIS-DAP HID response";
            return false;
        }

        const std::size_t offset =
            (rx[0] != payload[0] && nread > 1 && rx[1] == payload[0]) ? 1U : 0U;
        if (rx[offset] != payload[0]) {
            if (error_out != nullptr)
                *error_out = std::format(
                    "Unexpected CMSIS-DAP response 0x{:02X} for 0x{:02X}", rx[offset], payload[0]);
            return false;
        }

        resp_out.fill(0U);
        const auto copy_len =
            std::min<std::size_t>(resp_out.size(), static_cast<std::size_t>(nread) - offset);
        std::memcpy(resp_out.data(), rx.data() + offset, copy_len);
        return true;
    }

    bool swj_reset_to_swd(std::string& error_out) const {
        static constexpr std::array<std::uint8_t, 19> kSeqPacket{
            kDapSwjSequence, 17U * 8U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x9EU,
            0xE7U,           0xFFU,    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U,
        };
        std::array<std::uint8_t, 64> resp{};
        if (!send_packet(kSeqPacket.data(), kSeqPacket.size(), resp, &error_out))
            return false;
        if (resp[1] != 0x00U) {
            error_out = "DAP_SWJ_Sequence failed";
            return false;
        }
        return true;
    }

    bool transfer(
        const SwdOp* ops, std::size_t count, std::uint32_t* reads_out, std::string& error_out) {
        for (int attempt = 0; attempt < 4; ++attempt) {
            std::array<std::uint8_t, 64> req{};
            req[0] = kDapTransfer;
            req[1] = 0x00U;
            req[2] = static_cast<std::uint8_t>(count);
            std::size_t pos = 3U;
            for (std::size_t i = 0; i < count; ++i) {
                req[pos++] = ops[i].req;
                if ((ops[i].req & 0x02U) == 0U) {
                    write_u32_le(req.data() + pos, ops[i].value);
                    pos += 4U;
                }
            }

            std::array<std::uint8_t, 64> resp{};
            if (!send_packet(req.data(), pos, resp, &error_out))
                return false;

            const auto completed = resp[1];
            const auto ack = resp[2];
            if (completed == count && ack == kDapTransferOk) {
                std::size_t rpos = 3U;
                std::size_t ridx = 0U;
                for (std::size_t i = 0; i < count; ++i) {
                    if ((ops[i].req & 0x02U) != 0U) {
                        if (reads_out != nullptr)
                            reads_out[ridx] = read_u32_le(resp.data() + rpos);
                        ++ridx;
                        rpos += 4U;
                    }
                }
                return true;
            }

            std::array<std::uint8_t, 8> abort_pkt{kDapTransfer, 0x00U, 0x01U, kDpWriteAbort};
            write_u32_le(abort_pkt.data() + 4, 0x1EU);
            std::array<std::uint8_t, 64> ignore_resp{};
            (void)send_packet(abort_pkt.data(), abort_pkt.size(), ignore_resp);

            if (ack == kDapTransferWait && attempt < 3) {
                std::this_thread::sleep_for(std::chrono::milliseconds{2});
                continue;
            }
            if (attempt == 2) {
                (void)swj_reset_to_swd(error_out);
                const SwdOp id_op{.req = kDpReadIdcode, .value = 0U};
                std::uint32_t dummy_id = 0U;
                (void)transfer_single(id_op, &dummy_id);
                csw_initialized_ = false;
            }
        }
        error_out = "SWD DAP_Transfer failed";
        return false;
    }

    bool transfer_single(const SwdOp& op, std::uint32_t* read_val) const {
        std::array<std::uint8_t, 8> req{kDapTransfer, 0x00U, 0x01U, op.req};
        std::size_t len = 4U;
        if ((op.req & 0x02U) == 0U) {
            write_u32_le(req.data() + 4, op.value);
            len = 8U;
        }
        std::array<std::uint8_t, 64> resp{};
        if (!send_packet(req.data(), len, resp))
            return false;
        if (resp[1] == 1U && resp[2] == kDapTransferOk && read_val != nullptr
            && (op.req & 0x02U) != 0U) {
            *read_val = read_u32_le(resp.data() + 3);
        }
        return resp[1] == 1U && resp[2] == kDapTransferOk;
    }

    bool init_swd(std::string& error_out) {
        std::array<std::uint8_t, 64> resp{};
        const std::array<std::uint8_t, 2> connect_req{kDapConnect, kDapPortSwd};
        if (!send_packet(connect_req.data(), connect_req.size(), resp, &error_out))
            return false;
        if (resp[1] != kDapPortSwd) {
            error_out = "DAP_Connect(SWD) rejected";
            return false;
        }

        std::array<std::uint8_t, 5> clk_req{kDapSwjClock};
        write_u32_le(clk_req.data() + 1, 1'000'000U);
        if (!send_packet(clk_req.data(), clk_req.size(), resp, &error_out))
            return false;

        const std::array<std::uint8_t, 6> cfg_req{
            kDapTransferConfigure, 0x00U, 0xC8U, 0x00U, 0x00U, 0x00U};
        if (!send_packet(cfg_req.data(), cfg_req.size(), resp, &error_out))
            return false;

        if (!swj_reset_to_swd(error_out))
            return false;

        std::uint32_t idcode = 0U;
        const SwdOp id_op{.req = kDpReadIdcode, .value = 0U};
        if (!transfer(&id_op, 1U, &idcode, error_out))
            return false;

        const std::array<SwdOp, 3> power_ops{{
            {.req = kDpWriteAbort, .value = 0x0000001EU},
            {.req = kDpWriteSelect, .value = 0x00000000U},
            {.req = kDpWriteCtrlStat, .value = 0x50000000U},
        }};
        if (!transfer(power_ops.data(), power_ops.size(), nullptr, error_out))
            return false;

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{500};
        while (true) {
            std::uint32_t stat = 0U;
            const SwdOp stat_op{.req = kDpReadCtrlStat, .value = 0U};
            if (!transfer(&stat_op, 1U, &stat, error_out))
                return false;
            if ((stat & 0xF0000000U) == 0xF0000000U)
                break;
            if (std::chrono::steady_clock::now() > deadline) {
                error_out = "SWD debug power-up ACK timeout";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{5});
        }

        csw_initialized_ = false;
        return true;
    }

    bool read32(std::uint32_t addr, std::uint32_t& out_val, std::string& error_out) {
        if (!csw_initialized_) {
            const std::array<SwdOp, 3> ops{{
                {.req = kApWriteCsw, .value = kCsw32BitNoInc},
                {.req = kApWriteTar, .value = addr},
                {.req = kApReadDrw, .value = 0U},
            }};
            if (!transfer(ops.data(), ops.size(), &out_val, error_out))
                return false;
            csw_initialized_ = true;
            return true;
        }

        const std::array<SwdOp, 2> ops{{
            {.req = kApWriteTar, .value = addr},
            {.req = kApReadDrw, .value = 0U},
        }};
        return transfer(ops.data(), ops.size(), &out_val, error_out);
    }

    bool write32(std::uint32_t addr, std::uint32_t val, std::string& error_out) {
        if (!csw_initialized_) {
            const std::array<SwdOp, 3> ops{{
                {.req = kApWriteCsw, .value = kCsw32BitNoInc},
                {.req = kApWriteTar, .value = addr},
                {.req = kApWriteDrw, .value = val},
            }};
            if (!transfer(ops.data(), ops.size(), nullptr, error_out))
                return false;
            csw_initialized_ = true;
            return true;
        }

        const std::array<SwdOp, 2> ops{{
            {.req = kApWriteTar, .value = addr},
            {.req = kApWriteDrw, .value = val},
        }};
        return transfer(ops.data(), ops.size(), nullptr, error_out);
    }

    int fd_ = -1;
    bool csw_initialized_ = false;
    std::string description_ = "SWD disconnected";
};

class PosixUartClient {
public:
    PosixUartClient() = default;
    ~PosixUartClient() { close_port(); }

    PosixUartClient(const PosixUartClient&) = delete;
    PosixUartClient& operator=(const PosixUartClient&) = delete;

    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }
    [[nodiscard]] const std::string& description() const noexcept { return description_; }

    static std::optional<std::string> discover_serial_port() {
        namespace fs = std::filesystem;
        std::error_code ec;
        std::vector<std::string> matches;
        for (const auto& entry : fs::directory_iterator("/dev", ec)) {
            const auto name = entry.path().filename().string();
            if (name.starts_with("ttyUSB") || name.starts_with("ttyACM"))
                matches.push_back(entry.path().string());
        }
        if (matches.empty())
            return std::nullopt;
        std::ranges::sort(matches);
        return matches.front();
    }

    bool open_port(const std::string& explicit_port, int baudrate, std::string& error_out) {
        close_port();

        std::string port = explicit_port;
        if (port.empty()) {
            const auto found = discover_serial_port();
            if (!found.has_value()) {
                error_out = "No /dev/ttyUSB* or /dev/ttyACM* serial device found";
                return false;
            }
            port = *found;
        }

        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            error_out = std::format("Cannot open {}: {}", port, std::strerror(errno));
            return false;
        }

        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) {
            error_out = std::format("tcgetattr({}) failed: {}", port, std::strerror(errno));
            close_port();
            return false;
        }

        ::cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD | CS8);
        tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CRTSCTS);

        const speed_t speed = (baudrate == 9600) ? B9600 : B115200;
        ::cfsetispeed(&tio, speed);
        ::cfsetospeed(&tio, speed);

        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
            error_out = std::format("tcsetattr({}) failed: {}", port, std::strerror(errno));
            close_port();
            return false;
        }
        ::tcflush(fd_, TCIOFLUSH);
        description_ = std::format("UART {} @ {} 8N1", port, baudrate);
        return true;
    }

    void close_port() noexcept {
        if (fd_ >= 0) {
            (void)::close(fd_);
            fd_ = -1;
        }
    }

    bool send_led_command(char ch, LedSnapshot& snapshot_out, std::string& error_out) {
        if (fd_ < 0) {
            error_out = "UART is not open";
            return false;
        }
        if (::write(fd_, &ch, 1U) != 1) {
            error_out = std::format("UART write failed: {}", std::strerror(errno));
            return false;
        }

        std::string reply;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{120};
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
            if (::poll(&pfd, 1, 20) > 0) {
                std::array<char, 128> buf{};
                const ssize_t n = ::read(fd_, buf.data(), buf.size());
                if (n > 0) {
                    reply.append(buf.data(), static_cast<std::size_t>(n));
                    if (reply.find(static_cast<char>(10)) != std::string::npos)
                        break;
                }
            }
        }

        snapshot_out.connected = true;
        snapshot_out.last_cmd = ch;
        snapshot_out.seq += 1U;
        snapshot_out.transport_desc = description_;
        snapshot_out.detail = std::string{trim_ascii(reply)};
        apply_expected_state_from_char(ch, snapshot_out);
        return true;
    }

    static void apply_expected_state_from_char(char ch, LedSnapshot& snap) noexcept {
        const auto lower = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (lower == 'f') {
            snap.effect_mode = 1U;
            snap.target_mask = 0x01U;
            snap.live_mask = 0x01U;
        } else if (lower == 'k') {
            snap.effect_mode = 2U;
            if (snap.target_mask == 0U)
                snap.target_mask = 0x07U;
        } else if (lower == 'e') {
            snap.effect_mode = 3U;
            if (snap.target_mask == 0U)
                snap.target_mask = 0x07U;
        } else {
            snap.effect_mode = 0U;
            switch (lower) {
            case 'r':
            case '1': snap.live_mask = 0x01U; break;
            case 'g':
            case '2': snap.live_mask = 0x02U; break;
            case 'y':
            case '3': snap.live_mask = 0x03U; break;
            case 'b':
            case '4': snap.live_mask = 0x04U; break;
            case 'm':
            case 'p':
            case '5': snap.live_mask = 0x05U; break;
            case 'c':
            case '6': snap.live_mask = 0x06U; break;
            case 'w':
            case '7': snap.live_mask = 0x07U; break;
            default: snap.live_mask = 0x00U; break;
            }
            snap.target_mask = snap.live_mask;
        }
        snap.gpioa_odr = static_cast<std::uint32_t>((~snap.live_mask & 0x07U) << 1U);
    }

private:
    int fd_ = -1;
    std::string description_ = "UART disconnected";
};

std::string format_status_line(const LedSnapshot& snap) {
    if (!snap.connected) {
        return std::format(
            "DISCONNECTED | transport={} | {}", snap.transport_desc,
            snap.detail.empty() ? "waiting for board" : snap.detail);
    }

    const std::uint8_t display_mask = (snap.effect_mode == 0U) ? snap.live_mask : snap.target_mask;
    const int pa1_on = ((snap.gpioa_odr & (1U << 1U)) == 0U) ? 1 : 0;
    const int pa2_on = ((snap.gpioa_odr & (1U << 2U)) == 0U) ? 1 : 0;
    const int pa3_on = ((snap.gpioa_odr & (1U << 3U)) == 0U) ? 1 : 0;

    std::string line = std::format(
        "{} | mode={} | cmd='{}' | GPIOA(R={},G={},B={}) | seq={} | via {}",
        color_name_from_mask(display_mask), effect_name_from_mode(snap.effect_mode), snap.last_cmd,
        pa1_on, pa2_on, pa3_on, snap.seq, snap.transport_desc);
    if (!snap.detail.empty())
        line += std::format(" | {}", snap.detail);
    return line;
}

} // namespace

class LedController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LedController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        register_input("/flight/led/serial", led_serial_, false);
        register_input("/flight/led/command_byte", internal_command_, false);

        register_output("/flight/led/active_mask", active_mask_output_, std::uint8_t{0});
        register_output("/flight/led/effect_mode", effect_mode_output_, std::uint8_t{0});

        get_parameter_or<std::string>("transport", transport_mode_, "auto");
        get_parameter_or<std::string>("serial_port", serial_port_, "");
        get_parameter_or<int>("baudrate", baudrate_, 115200);
        get_parameter_or<std::string>("hidraw_path", hidraw_path_, "");
        get_parameter_or<std::string>("command_topic", command_topic_, "/flight/led/command");
        get_parameter_or<std::string>("status_topic", status_topic_, "/flight/led/status");
        get_parameter_or<std::string>("initial_command", initial_command_, "");

        status_publisher_ = create_publisher<std_msgs::msg::String>(
            status_topic_, rclcpp::QoS{1}.reliable().transient_local());

        command_subscription_ = create_subscription<std_msgs::msg::String>(
            command_topic_, rclcpp::QoS{16}.reliable(),
            [this](const std_msgs::msg::String::UniquePtr msg) { on_command_message(msg->data); });

        if (!initial_command_.empty()) {
            if (const auto parsed = parse_command_token(initial_command_);
                parsed.has_value() && *parsed != '?') {
                enqueue_command(*parsed);
            }
        }

        RCLCPP_INFO(
            logger_,
            "Flight LedController initialized (command_topic=%s, status_topic=%s, mode=%s)",
            command_topic_.c_str(), status_topic_.c_str(), transport_mode_.c_str());
    }

    LedController(const LedController&) = delete;
    LedController& operator=(const LedController&) = delete;
    LedController(LedController&&) = delete;
    LedController& operator=(LedController&&) = delete;
    ~LedController() override = default;

    void before_updating() override {
        io_thread_ = std::jthread{[this](const std::stop_token& st) { io_worker_loop(st); }};
    }

    void update() override {
        if (internal_command_.ready()) {
            const auto raw = static_cast<char>(*internal_command_);
            if (raw != last_internal_command_ && is_valid_command_char(raw)) {
                last_internal_command_ = raw;
                enqueue_command(raw);
            }
        }

        *active_mask_output_ = atomic_live_mask_.load(std::memory_order_relaxed);
        *effect_mode_output_ = atomic_effect_mode_.load(std::memory_order_relaxed);
    }

private:
    enum class ActiveBackend : std::uint8_t {
        None,
        RmcsSerial,
        Uart,
        Swd,
    };

    void on_command_message(std::string_view payload) {
        const auto trimmed = trim_ascii(payload);
        if (trimmed.empty())
            return;

        if (const auto single_token = parse_command_token(trimmed); single_token.has_value()) {
            enqueue_command(*single_token);
            return;
        }

        bool queued_any = false;
        for (const char ch : trimmed) {
            if (std::isspace(static_cast<unsigned char>(ch)) != 0)
                continue;
            if (is_valid_command_char(ch)) {
                enqueue_command(ch);
                queued_any = true;
            } else {
                RCLCPP_WARN(
                    logger_, "Unsupported LED command '%c' in '%.*s'", ch,
                    static_cast<int>(trimmed.size()), trimmed.data());
            }
        }
        if (!queued_any) {
            publish_status_string(
                std::format("INVALID_CMD '{}' | valid: r/g/b/y/c/m/w/f/k/e/o/status", trimmed));
        }
    }

    void enqueue_command(char cmd) {
        {
            const std::scoped_lock lock{queue_mutex_};
            if (command_queue_.size() >= 64U)
                command_queue_.pop_front();
            command_queue_.push_back(cmd);
        }
        queue_cv_.notify_one();
    }

    void publish_snapshot(const LedSnapshot& snap) {
        atomic_live_mask_.store(snap.live_mask, std::memory_order_relaxed);
        atomic_effect_mode_.store(snap.effect_mode, std::memory_order_relaxed);
        publish_status_string(format_status_line(snap));
    }

    void publish_status_string(std::string text) {
        std_msgs::msg::String msg;
        msg.data = std::move(text);
        status_publisher_->publish(msg);
    }

    void io_worker_loop(const std::stop_token& st) {
        CmsisDapMailboxClient swd_client;
        PosixUartClient uart_client;
        ActiveBackend backend = ActiveBackend::None;
        LedSnapshot snapshot{};
        auto last_poll_time = std::chrono::steady_clock::now() - std::chrono::seconds{2};

        auto disconnect_all = [&]() {
            swd_client.close_device();
            uart_client.close_port();
            backend = ActiveBackend::None;
            snapshot.connected = false;
        };

        auto ensure_connected = [&]() -> bool {
            if (backend == ActiveBackend::RmcsSerial && led_serial_.ready()
                && static_cast<bool>(led_serial_->write))
                return true;
            if (backend == ActiveBackend::Uart && uart_client.is_open())
                return true;
            if (backend == ActiveBackend::Swd && swd_client.is_open())
                return true;

            disconnect_all();
            const auto mode = to_lower_ascii(transport_mode_);
            std::string err;

            if ((mode == "auto" || mode == "rmcs_serial") && led_serial_.ready()
                && static_cast<bool>(led_serial_->write)) {
                backend = ActiveBackend::RmcsSerial;
                snapshot.connected = true;
                snapshot.transport_desc = "RMCS /flight/led/serial";
                RCLCPP_INFO(
                    logger_, "Connected to LED board via %s", snapshot.transport_desc.c_str());
                publish_snapshot(snapshot);
                return true;
            }

            if (mode == "auto" || mode == "uart") {
                if (uart_client.open_port(serial_port_, baudrate_, err)) {
                    backend = ActiveBackend::Uart;
                    snapshot.connected = true;
                    snapshot.transport_desc = uart_client.description();
                    RCLCPP_INFO(
                        logger_, "Connected to LED board via %s", snapshot.transport_desc.c_str());
                    publish_snapshot(snapshot);
                    return true;
                }
                if (mode == "uart") {
                    snapshot.detail = err;
                    return false;
                }
            }

            if (mode == "auto" || mode == "swd") {
                if (swd_client.connect_device(hidraw_path_, err)) {
                    backend = ActiveBackend::Swd;
                    if (swd_client.read_snapshot(snapshot, err)) {
                        RCLCPP_INFO(
                            logger_, "Connected to LED board via %s",
                            snapshot.transport_desc.c_str());
                        publish_snapshot(snapshot);
                        return true;
                    }
                    swd_client.close_device();
                    backend = ActiveBackend::None;
                }
                snapshot.detail = err;
            }

            return false;
        };

        while (!st.stop_requested()) {
            char next_cmd = '\0';
            bool has_cmd = false;
            {
                std::unique_lock lock{queue_mutex_};
                queue_cv_.wait_for(lock, st, std::chrono::milliseconds{250}, [&] {
                    return !command_queue_.empty() || st.stop_requested();
                });
                if (st.stop_requested())
                    break;
                if (!command_queue_.empty()) {
                    next_cmd = command_queue_.front();
                    command_queue_.pop_front();
                    has_cmd = true;
                }
            }

            if (!ensure_connected()) {
                const auto now = std::chrono::steady_clock::now();
                if ((now - last_poll_time) >= std::chrono::seconds{2}) {
                    last_poll_time = now;
                    RCLCPP_WARN_THROTTLE(
                        logger_, *get_clock(), 5000, "LED board offline: %s",
                        snapshot.detail.c_str());
                    publish_snapshot(snapshot);
                }
                continue;
            }

            if (has_cmd) {
                std::string err;
                bool ok = false;
                if (next_cmd == '?') {
                    if (backend == ActiveBackend::Swd)
                        ok = swd_client.read_snapshot(snapshot, err);
                    else
                        ok = true;
                } else if (backend == ActiveBackend::Swd) {
                    ok = swd_client.send_led_command(next_cmd, snapshot, err);
                } else if (backend == ActiveBackend::Uart) {
                    ok = uart_client.send_led_command(next_cmd, snapshot, err);
                } else if (
                    backend == ActiveBackend::RmcsSerial && led_serial_.ready()
                    && static_cast<bool>(led_serial_->write)) {
                    const auto byte_val =
                        static_cast<std::byte>(static_cast<unsigned char>(next_cmd));
                    (void)led_serial_->write(&byte_val, 1U);
                    snapshot.connected = true;
                    snapshot.last_cmd = next_cmd;
                    snapshot.seq += 1U;
                    PosixUartClient::apply_expected_state_from_char(next_cmd, snapshot);
                    ok = true;
                }

                if (ok) {
                    last_poll_time = std::chrono::steady_clock::now();
                    const auto formatted = format_status_line(snapshot);
                    RCLCPP_INFO(logger_, "[cmd='%c'] %s", next_cmd, formatted.c_str());
                    publish_snapshot(snapshot);
                } else {
                    RCLCPP_WARN(logger_, "LED transport error on '%c': %s", next_cmd, err.c_str());
                    snapshot.detail = err;
                    disconnect_all();
                    publish_snapshot(snapshot);
                }
                continue;
            }

            // Periodically refresh SWD hardware state for dynamic modes (FLOW / BLINK / BREATH)
            const auto now = std::chrono::steady_clock::now();
            if (backend == ActiveBackend::Swd
                && (now - last_poll_time) >= std::chrono::milliseconds{500}) {
                last_poll_time = now;
                std::string err;
                const auto prev_mask = snapshot.live_mask;
                const auto prev_mode = snapshot.effect_mode;
                if (swd_client.read_snapshot(snapshot, err)) {
                    atomic_live_mask_.store(snapshot.live_mask, std::memory_order_relaxed);
                    atomic_effect_mode_.store(snapshot.effect_mode, std::memory_order_relaxed);
                    if (snapshot.live_mask != prev_mask || snapshot.effect_mode != prev_mode)
                        publish_snapshot(snapshot);
                } else {
                    snapshot.detail = err;
                    disconnect_all();
                    publish_snapshot(snapshot);
                }
            }
        }
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::SerialInterface> led_serial_;
    InputInterface<std::uint8_t> internal_command_;
    OutputInterface<std::uint8_t> active_mask_output_;
    OutputInterface<std::uint8_t> effect_mode_output_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;

    std::string transport_mode_;
    std::string serial_port_;
    int baudrate_ = 115200;
    std::string hidraw_path_;
    std::string command_topic_;
    std::string status_topic_;
    std::string initial_command_;

    char last_internal_command_ = '\0';
    std::atomic<std::uint8_t> atomic_live_mask_{0U};
    std::atomic<std::uint8_t> atomic_effect_mode_{0U};

    std::mutex queue_mutex_;
    std::condition_variable_any queue_cv_;
    std::deque<char> command_queue_;

    // Declared last so stop_token + join run before destroying any member used by the worker
    std::jthread io_thread_;
};

} // namespace rmcs_core::controller::flight

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::flight::LedController, rmcs_executor::Component)

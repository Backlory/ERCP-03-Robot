#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <windows.h>
#include "TcAdsDef.h"
#include "TcAdsAPI.h"

namespace {

enum class Access { ReadOnly, SafeZeroWrite, DangerousAction };

struct SymbolSpec {
    std::string name;
    std::string expected_type;
    std::uint32_t expected_size;
    Access access;
    bool required = true;
};

/**
 * @brief 按数组下标批量生成 PLC symbol 检查项。
 * @details 使用统一的前缀、类型、字节长度和访问级别构造名称，减少探测清单中的重复文字。
 */
void AddIndexedSymbols(std::vector<SymbolSpec> &symbols,
                       const char *prefix,
                       const char *type,
                       std::uint32_t size,
                       Access access,
                       std::size_t count,
                       bool required = false)
{
    for (std::size_t i = 0; i < count; ++i) {
        symbols.push_back(
            {std::string(prefix) + std::to_string(i + 1) + "]", type, size, access, required});
    }
}

/**
 * @brief 构造 Beckhoff 接口探测所需的完整 symbol 清单。
 * @details 清单同时覆盖必需字段、可选字段、只读字段和安全归零写字段，供主流程逐项查询和校验。
 */
std::vector<SymbolSpec> BuildSymbolSpecs()
{
    std::vector<SymbolSpec> symbols{
        {"MAIN.iPrepare_State", "INT", 2, Access::ReadOnly},
        {"MAIN.bErro_State_Drive", "BOOL", 1, Access::ReadOnly},
        {"MAIN.bErro_State_Motor", "BOOL", 1, Access::ReadOnly},
        {"MAIN.type_of_scope", "DINT", 4, Access::ReadOnly},
        {"MAIN.Status_Comand_FromMaster", "DINT", 4, Access::DangerousAction},
        // Diagnostic alias used by the current RobotSystem implementation.
        {"MAIN.Status_Command_FromMaster", "DINT", 4, Access::DangerousAction, false},
        {"MAIN.Status_Feedback_ToMaster", "DINT", 4, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Follow_Length", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Switch_Water", "BOOL", 1, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Switch_Gas", "BOOL", 1, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Switch_Suck", "BOOL", 1, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Big_Wheel", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Small_Wheel", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Power_level", "INT", 2, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.lifter", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Deliver_Force", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Rotate_Degree", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Info_Feedback_ToMaster.Follow_Force", "LREAL", 8, Access::ReadOnly},
        {"MAIN.Emergency_Stop_FromMaster", "DINT", 4, Access::DangerousAction},
        {"MAIN.Follow_Control_Cmd.Cmd_Follow_Comp_Joy_FromMaster",
         "LREAL",
         8,
         Access::SafeZeroWrite},
        {"MAIN.Follow_Control_Cmd.Cmd_Operator_Joy_FromMaster",
         "ARRAY[1..9] OF LREAL",
         72,
         Access::SafeZeroWrite},
        {"MAIN.Follow_Control_Cmd.Cmd_Home_Joy_FromMaster",
         "ARRAY[1..3] OF BOOL",
         3,
         Access::SafeZeroWrite},
        {"MAIN.Follow_Control_Cmd.Cmd_IO_Joy_FromMaster",
         "ARRAY[1..3] OF BOOL",
         3,
         Access::SafeZeroWrite},
        {"MAIN.ERCP_Online_flag", "BOOL", 1, Access::ReadOnly},
        {"POU_Ercp_CycleExecute.Ercp_Ready_State", "BOOL", 1, Access::ReadOnly},
        {"MAIN_ERCP.bERCP_Operate_State_FromMaster", "BOOL", 1, Access::SafeZeroWrite},
        {"MAIN_ERCP.bErro_State_Drive_ERCP", "BOOL", 1, Access::ReadOnly},
        {"MAIN_ERCP.bErro_State_Motor_ERCP", "BOOL", 1, Access::ReadOnly},
        {"MAIN_ERCP.type_of_ERCP", "DINT", 4, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Status_Feedback_ToMaster", "DINT", 4, Access::ReadOnly},
        {"MAIN_ERCP.bERCP_Load_Exchange_Dir", "BOOL", 1, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Force",
         "LREAL",
         8,
         Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.GuideWire_Force", "LREAL", 8, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Bow_Force", "LREAL", 8, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Pos",
         "LREAL",
         8,
         Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.GuideWire_Pos", "LREAL", 8, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_CurPos_01",
         "LREAL",
         8,
         Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_CurPos_02",
         "LREAL",
         8,
         Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_State_01", "DINT", 4, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_State_02", "DINT", 4, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Balloon_Pressure", "INT", 2, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Operator_Pos", "LREAL", 8, Access::ReadOnly},
        {"MAIN_ERCP.ERCP_Control_Cmd.Cmd_6Dhandle_Joy_FromMaster",
         "ARRAY[1..6] OF LREAL",
         48,
         Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Control_Cmd.Cmd_Button_Joy_FromMaster",
         "ARRAY[1..3] OF BOOL",
         3,
         Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Vel_01", "LREAL", 8, Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Vel_02", "LREAL", 8, Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Pos_01", "LREAL", 8, Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Pos_02", "LREAL", 8, Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Enable_01", "BOOL", 1, Access::SafeZeroWrite},
        {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Enable_02", "BOOL", 1, Access::SafeZeroWrite},
        {"MAIN_ERCP.bErcp_Cooperate_Enable", "BOOL", 1, Access::SafeZeroWrite},
    };

    AddIndexedSymbols(symbols, "MAIN.DriveErrorState[", "BOOL", 1, Access::ReadOnly, 22);
    AddIndexedSymbols(symbols, "MAIN.MotorErrorState[", "BOOL", 1, Access::ReadOnly, 19);
    AddIndexedSymbols(symbols,
                      "MAIN.Info_Feedback_ToMaster.Force_Sensor[",
                      "LREAL",
                      8,
                      Access::ReadOnly,
                      10);
    AddIndexedSymbols(symbols,
                      "MAIN.Info_Feedback_ToMaster.Axes_Pos[",
                      "LREAL",
                      8,
                      Access::ReadOnly,
                      21);
    AddIndexedSymbols(symbols,
                      "MAIN_ERCP.DriveErrorState_ERCP[",
                      "BOOL",
                      1,
                      Access::ReadOnly,
                      13);
    AddIndexedSymbols(symbols,
                      "MAIN_ERCP.MotorErrorState_ERCP[",
                      "BOOL",
                      1,
                      Access::ReadOnly,
                      11);
    return symbols;
}

const auto kSymbols = BuildSymbolSpecs();

struct Options {
    std::string net_id = "169.254.213.62.1.1";
    std::uint16_t port = 851;
    bool write_safe_zero = false;
};

std::string AdsHex(long error)
{
    std::ostringstream stream;
    stream << "0x" << std::hex << std::uppercase << static_cast<std::uint32_t>(error);
    return stream.str();
}

/**
 * @brief 将常见 ADS 错误码转换为稳定的英文诊断名称。
 * @details 覆盖连接、超时、尺寸、符号和权限错误；未登记的错误统一返回 UNKNOWN。
 */
const char *ErrorName(long error)
{
    switch (error) {
    case ADSERR_NOERR:
        return "OK";
    case 0x06:
        return "TARGET_PORT_NOT_FOUND";
    case 0x07:
        return "TARGET_MACHINE_NOT_FOUND";
    case 0x15:
        return "AMS_SYNC_TIMEOUT";
    case 0x1D:
        return "TLS_SEND_FAILED";
    case ADSERR_DEVICE_INVALIDSIZE:
        return "ADS_INVALID_SIZE";
    case ADSERR_DEVICE_SYMBOLNOTFOUND:
        return "ADS_SYMBOL_NOT_FOUND";
    case ADSERR_DEVICE_ACCESSDENIED:
        return "ADS_ACCESS_DENIED";
    case ADSERR_CLIENT_SYNCTIMEOUT:
        return "ADS_CLIENT_SYNC_TIMEOUT";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief 将点分隔的六段 AMS Net ID 文本解析为字节数组。
 * @details 严格检查段数、分隔符和 0~255 范围，只有完整合法时才写入目标地址。
 */
bool ParseNetId(const std::string &text, AmsNetId &net_id)
{
    std::array<unsigned int, 6> parts{};
    std::istringstream stream(text);
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (!(stream >> parts[i]))
            return false;
        if (i + 1 != parts.size()) {
            char separator = '\0';
            if (!(stream >> separator) || separator != '.')
                return false;
        }
    }
    if (stream.peek() != std::char_traits<char>::eof())
        return false;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (parts[i] > 255)
            return false;
        net_id.b[i] = static_cast<unsigned char>(parts[i]);
    }
    return true;
}

/**
 * @brief 解析接口探测工具的命令行选项。
 * @details 支持目标 Net ID、ADS 端口和安全归零写开关，未知参数或非法端口直接返回失败。
 */
bool ParseOptions(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--net-id" && i + 1 < argc) {
            options.net_id = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            const int value = std::stoi(argv[++i]);
            if (value < 1 || value > 65535)
                return false;
            options.port = static_cast<std::uint16_t>(value);
        } else if (arg == "--write-safe-zero") {
            options.write_safe_zero = true;
        } else {
            return false;
        }
    }
    return true;
}

struct OnlineSymbol {
    std::uint32_t index_group = 0;
    std::uint32_t index_offset = 0;
    std::uint32_t size = 0;
    std::string type;
};

/**
 * @brief 通过 ADS 按名称查询 PLC symbol 的在线地址、大小和类型。
 * @details 发送 SYM_INFOBYNAMEEX 请求，校验返回长度后解码 AdsSymbolEntry，供后续读写使用。
 */
long QuerySymbol(AmsAddr &address, const char *name, OnlineSymbol &result)
{
    std::array<std::uint8_t, 4096> buffer{};
    unsigned long bytes_read = 0;
    const auto name_size = static_cast<unsigned long>(std::strlen(name) + 1);
    const long error = AdsSyncReadWriteReqEx(&address,
                                             ADSIGRP_SYM_INFOBYNAMEEX,
                                             0,
                                             static_cast<unsigned long>(buffer.size()),
                                             buffer.data(),
                                             name_size,
                                             const_cast<char *>(name),
                                             &bytes_read);
    if (error != ADSERR_NOERR)
        return error;
    if (bytes_read < sizeof(AdsSymbolEntry))
        return ADSERR_DEVICE_INVALIDSIZE;

    const auto *entry = reinterpret_cast<const AdsSymbolEntry *>(buffer.data());
    result.index_group = entry->iGroup;
    result.index_offset = entry->iOffs;
    result.size = entry->size;
    result.type = PADSSYMBOLTYPE(entry);
    return ADSERR_NOERR;
}

/**
 * @brief 将读取到的字节缓冲区格式化为最多 16 字节的十六进制预览。
 * @details 保持输出紧凑，超出预览范围时追加省略号，便于逐项探测结果人工核对。
 */
std::string Preview(const std::vector<std::uint8_t> &bytes)
{
    std::ostringstream stream;
    const std::size_t count = (std::min)(bytes.size(), std::size_t{16});
    for (std::size_t i = 0; i < count; ++i) {
        if (i != 0)
            stream << ' ';
        stream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
               << static_cast<unsigned int>(bytes[i]);
    }
    if (bytes.size() > count)
        stream << " ...";
    return stream.str();
}

} // namespace

/**
 * @brief Beckhoff 接口探测工具入口。
 * @details 依次解析参数、打开 ADS 端口、读取目标状态、逐项查询/读取 symbol，并按访问策略执行可选安全归零写入。
 */
int main(int argc, char **argv)
{
    // 阶段一：解析目标和本地 ADS 参数，建立目标地址并打开客户端端口。
    Options options;
    try {
        if (!ParseOptions(argc, argv, options)) {
            std::cerr << "Usage: BeckhoffInterfaceProbe.exe [--net-id A.B.C.D.E.F]"
                         " [--port 851] [--write-safe-zero]\n";
            return 2;
        }
    } catch (const std::exception &error) {
        std::cerr << "Invalid arguments: " << error.what() << '\n';
        return 2;
    }

    AmsAddr address{};
    if (!ParseNetId(options.net_id, address.netId)) {
        std::cerr << "Invalid AMS Net ID: " << options.net_id << '\n';
        return 2;
    }
    address.port = options.port;

    const long local_port = AdsPortOpen();
    if (local_port <= 0) {
        std::cerr << "AdsPortOpen failed\n";
        return 3;
    }

    USHORT ads_state = 0;
    USHORT device_state = 0;
    const long state_error = AdsSyncReadStateReq(&address, &ads_state, &device_state);
    std::cout << "Target " << options.net_id << ':' << options.port
              << " ADS state read=" << AdsHex(state_error) << '(' << ErrorName(state_error) << ')'
              << " ads_state=" << ads_state << " device_state=" << device_state << "\n\n";
    if (state_error != ADSERR_NOERR) {
        AdsPortClose();
        return 4;
    }

    // 阶段二：逐项核对 symbol 的在线类型和长度，必要时只写入预先声明的安全零值。
    std::size_t passed = 0;
    std::size_t failed = 0;
    std::size_t written = 0;
    std::cout << "RESULT | SYMBOL | EXPECTED | ONLINE | READ | WRITE | VALUE[0..15]\n";

    for (const auto &spec : kSymbols) {
        OnlineSymbol online;
        const long query_error = QuerySymbol(address, spec.name.c_str(), online);
        long read_error = ADSERR_NOERR;
        long write_error = ADSERR_NOERR;
        std::vector<std::uint8_t> bytes;
        bool ok = query_error == ADSERR_NOERR;

        if (ok) {
            bytes.resize(online.size);
            read_error = AdsSyncReadReq(&address,
                                        online.index_group,
                                        online.index_offset,
                                        online.size,
                                        bytes.data());
            ok = read_error == ADSERR_NOERR && online.size == spec.expected_size;
        }

        const bool should_write = options.write_safe_zero && spec.access == Access::SafeZeroWrite;
        if (should_write && query_error == ADSERR_NOERR) {
            if (online.size != spec.expected_size) {
                write_error = ADSERR_DEVICE_INVALIDSIZE;
            } else {
                std::vector<std::uint8_t> zeros(online.size, 0);
                write_error = AdsSyncWriteReq(&address,
                                              online.index_group,
                                              online.index_offset,
                                              online.size,
                                              zeros.data());
                if (write_error == ADSERR_NOERR)
                    ++written;
                ok = ok && write_error == ADSERR_NOERR;
            }
        }

        if (ok)
            ++passed;
        else if (spec.required)
            ++failed;

        std::cout << (ok ? "PASS" : (spec.required ? "FAIL" : "INFO")) << " | " << spec.name
                  << " | " << spec.expected_type << '/' << spec.expected_size << " | ";
        if (query_error == ADSERR_NOERR) {
            std::cout << online.type << '/' << online.size;
        } else {
            std::cout << AdsHex(query_error) << '(' << ErrorName(query_error) << ')';
        }
        std::cout << " | " << (query_error == ADSERR_NOERR ? AdsHex(read_error) : "-") << " | ";
        if (should_write && query_error != ADSERR_NOERR)
            std::cout << "NOT-FOUND";
        else if (should_write)
            std::cout << AdsHex(write_error);
        else if (spec.access == Access::DangerousAction)
            std::cout << "SKIP-DANGEROUS";
        else
            std::cout << "SKIP";
        std::cout << " | " << Preview(bytes) << '\n';
    }

    // 阶段三：关闭 ADS 端口并输出必需字段失败数，作为命令行退出码。
    AdsPortClose();
    std::cout << "\nSUMMARY total=" << kSymbols.size() << " pass=" << passed
              << " fail=" << failed << " safe_zero_writes=" << written << '\n';
    return failed == 0 ? 0 : 1;
}

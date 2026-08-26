#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

#include "TcAdsDef.h"
#include "fake_ads_server.hpp"

namespace ercp::timing_probe {
namespace {

constexpr std::size_t kTcpHeaderSize = 6;
constexpr std::size_t kAmsHeaderSize = 32;
constexpr std::uint16_t kAdsCommandRead = 2;
constexpr std::uint16_t kAdsCommandWrite = 3;
constexpr std::uint16_t kAdsCommandReadState = 4;
constexpr std::uint16_t kAdsCommandWriteControl = 5;
constexpr std::uint16_t kAdsCommandReadWrite = 9;
constexpr std::uint32_t kSymbolHandleByName = ADSIGRP_SYM_HNDBYNAME;
constexpr std::uint32_t kSymbolValueByHandle = ADSIGRP_SYM_VALBYHND;
constexpr std::uint32_t kSymbolReleaseHandle = ADSIGRP_SYM_RELEASEHND;
constexpr std::uint32_t kSumRead = ADSIGRP_SUMUP_READ;

std::uint64_t UnixNowNs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::uint16_t ReadU16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(data[1]) << 8;
}

std::uint32_t ReadU32(const std::uint8_t *data)
{
    return static_cast<std::uint32_t>(data[0]) |
           static_cast<std::uint32_t>(data[1]) << 8 |
           static_cast<std::uint32_t>(data[2]) << 16 |
           static_cast<std::uint32_t>(data[3]) << 24;
}

void WriteU16(std::vector<std::uint8_t> &data, std::size_t offset, std::uint16_t value)
{
    data[offset] = static_cast<std::uint8_t>(value);
    data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

void WriteU32(std::vector<std::uint8_t> &data, std::size_t offset, std::uint32_t value)
{
    for (unsigned shift = 0; shift < 32; shift += 8)
        data[offset + shift / 8] = static_cast<std::uint8_t>(value >> shift);
}

void AppendU16(std::vector<std::uint8_t> &data, std::uint16_t value)
{
    data.push_back(static_cast<std::uint8_t>(value));
    data.push_back(static_cast<std::uint8_t>(value >> 8));
}

void AppendU32(std::vector<std::uint8_t> &data, std::uint32_t value)
{
    for (unsigned shift = 0; shift < 32; shift += 8)
        data.push_back(static_cast<std::uint8_t>(value >> shift));
}

bool ReceiveAll(SOCKET socket, std::uint8_t *data, std::size_t length)
{
    while (length != 0) {
        const int chunk = static_cast<int>((std::min)(
            length, static_cast<std::size_t>((std::numeric_limits<int>::max)())));
        const int received = recv(socket, reinterpret_cast<char *>(data), chunk, 0);
        if (received <= 0)
            return false;
        data += received;
        length -= static_cast<std::size_t>(received);
    }
    return true;
}

bool SendAll(SOCKET socket, const std::uint8_t *data, std::size_t length)
{
    while (length != 0) {
        const int chunk = static_cast<int>((std::min)(
            length, static_cast<std::size_t>((std::numeric_limits<int>::max)())));
        const int sent = send(socket, reinterpret_cast<const char *>(data), chunk, 0);
        if (sent <= 0)
            return false;
        data += sent;
        length -= static_cast<std::size_t>(sent);
    }
    return true;
}

std::string BoundedString(const std::uint8_t *data, std::size_t length)
{
    const auto *begin = reinterpret_cast<const char *>(data);
    const auto *end = begin + length;
    const auto *terminator = std::find(begin, end, '\0');
    return std::string(begin, terminator);
}

} // namespace

struct FakeAdsServer::Impl {
    FakeAdsOptions options;
    SOCKET listen_socket = INVALID_SOCKET;
    SOCKET client_socket = INVALID_SOCKET;
    std::uint16_t bound_port = 0;
    bool winsock_started = false;
    std::atomic<bool> running{false};
    std::thread worker;

    mutable std::mutex mutex;
    std::unordered_map<std::string, std::uint32_t> handles_by_symbol;
    std::unordered_map<std::uint32_t, std::string> symbols_by_handle;
    std::uint32_t next_handle = 1;
    std::vector<AdsRequestRecord> records;

    bool Start(const FakeAdsOptions &requested, std::string *error)
    {
        if (running.load(std::memory_order_acquire))
            return true;

        options = requested;
        if (options.response_delay_ms < 0)
            options.response_delay_ms = 0;

        WSADATA winsock{};
        if (WSAStartup(MAKEWORD(2, 2), &winsock) != 0) {
            if (error != nullptr)
                *error = "WSAStartup failed";
            return false;
        }
        winsock_started = true;

        listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_socket == INVALID_SOCKET) {
            if (error != nullptr)
                *error = "socket failed";
            Stop();
            return false;
        }

        BOOL reuse = TRUE;
        setsockopt(listen_socket,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   reinterpret_cast<const char *>(&reuse),
                   sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(options.listen_port);
        if (bind(listen_socket,
                 reinterpret_cast<const sockaddr *>(&address),
                 sizeof(address)) == SOCKET_ERROR ||
            listen(listen_socket, SOMAXCONN) == SOCKET_ERROR) {
            if (error != nullptr)
                *error = "bind/listen failed";
            Stop();
            return false;
        }

        sockaddr_in actual{};
        int actual_size = sizeof(actual);
        if (getsockname(listen_socket,
                        reinterpret_cast<sockaddr *>(&actual),
                        &actual_size) == SOCKET_ERROR) {
            if (error != nullptr)
                *error = "getsockname failed";
            Stop();
            return false;
        }
        bound_port = ntohs(actual.sin_port);
        running.store(true, std::memory_order_release);
        worker = std::thread([this]() { Run(); });
        return true;
    }

    void Stop()
    {
        running.store(false, std::memory_order_release);

        SOCKET listener = INVALID_SOCKET;
        SOCKET client = INVALID_SOCKET;
        {
            std::lock_guard<std::mutex> lock(mutex);
            listener = listen_socket;
            listen_socket = INVALID_SOCKET;
            client = client_socket;
        }

        if (listener != INVALID_SOCKET) {
            shutdown(listener, SD_BOTH);
            closesocket(listener);
        }
        // shutdown wakes the blocking recv; ClientLoop owns the close and
        // clears client_socket before the worker exits.
        if (client != INVALID_SOCKET)
            shutdown(client, SD_BOTH);

        if (worker.joinable())
            worker.join();

        {
            std::lock_guard<std::mutex> lock(mutex);
            if (client_socket != INVALID_SOCKET) {
                closesocket(client_socket);
                client_socket = INVALID_SOCKET;
            }
            bound_port = 0;
        }
        if (winsock_started) {
            WSACleanup();
            winsock_started = false;
        }
    }

    bool ShouldDelay(std::uint16_t command, std::uint32_t group) const
    {
        if (options.response_delay_ms <= 0 || options.delay_scope == AdsDelayScope::None)
            return false;
        if (options.delay_scope == AdsDelayScope::All)
            return true;
        if (options.delay_scope == AdsDelayScope::Writes)
            return command == kAdsCommandWrite || command == kAdsCommandWriteControl;
        return command == kAdsCommandRead || command == kAdsCommandReadState ||
               command == kAdsCommandReadWrite || group == kSumRead;
    }

    std::uint32_t HandleFor(const std::string &symbol)
    {
        const auto found = handles_by_symbol.find(symbol);
        if (found != handles_by_symbol.end())
            return found->second;

        const auto handle = next_handle++;
        handles_by_symbol.emplace(symbol, handle);
        symbols_by_handle.emplace(handle, symbol);
        return handle;
    }

    std::string SymbolFor(std::uint32_t handle) const
    {
        const auto found = symbols_by_handle.find(handle);
        return found == symbols_by_handle.end() ? std::string{} : found->second;
    }

    std::vector<std::uint8_t> ReadValue(const std::string &symbol, std::uint32_t length) const
    {
        // All values are deterministic and safe zeroes. This server does not
        // model a motor or a PLC task; it only supplies enough feedback for
        // RobotSystem's real polling/control code to run.
        std::vector<std::uint8_t> value(length, 0);
        if (symbol == "MAIN.Status_Feedback_ToMaster" && length >= 4)
            WriteU32(value, 0, 0); // BAMS_START: no arm transition is requested.
        return value;
    }

    void AddRecord(AdsRequestRecord record)
    {
        std::lock_guard<std::mutex> lock(mutex);
        records.push_back(std::move(record));
    }

    std::vector<std::uint8_t> MakeResponse(const std::vector<std::uint8_t> &request,
                                           std::uint16_t command,
                                           std::uint32_t invoke,
                                           std::uint32_t ams_error,
                                           const std::vector<std::uint8_t> &payload) const
    {
        std::vector<std::uint8_t> ams(kAmsHeaderSize, 0);
        if (request.size() >= 16)
            std::copy(request.begin(), request.begin() + 16, ams.begin());
        WriteU16(ams, 16, command);
        WriteU16(ams, 18, 4);
        WriteU32(ams, 20, static_cast<std::uint32_t>(payload.size()));
        WriteU32(ams, 24, ams_error);
        WriteU32(ams, 28, invoke);
        ams.insert(ams.end(), payload.begin(), payload.end());

        std::vector<std::uint8_t> frame(kTcpHeaderSize, 0);
        WriteU16(frame, 0, 0);
        WriteU32(frame, 2, static_cast<std::uint32_t>(ams.size()));
        frame.insert(frame.end(), ams.begin(), ams.end());
        return frame;
    }

    void Run()
    {
        while (running.load(std::memory_order_acquire)) {
            SOCKET listener = INVALID_SOCKET;
            {
                std::lock_guard<std::mutex> lock(mutex);
                listener = listen_socket;
            }
            if (listener == INVALID_SOCKET)
                break;

            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(listener, &read_set);
            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;
            const int ready = select(0, &read_set, nullptr, nullptr, &timeout);
            if (ready <= 0)
                continue;

            sockaddr_in peer{};
            int peer_size = sizeof(peer);
            const SOCKET client = accept(listener,
                                         reinterpret_cast<sockaddr *>(&peer),
                                         &peer_size);
            if (client == INVALID_SOCKET)
                continue;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (!running.load(std::memory_order_acquire)) {
                    closesocket(client);
                    break;
                }
                client_socket = client;
            }
            ClientLoop(client);
        }
    }

    void ClientLoop(SOCKET client)
    {
        while (running.load(std::memory_order_acquire)) {
            std::array<std::uint8_t, kTcpHeaderSize> tcp_header{};
            if (!ReceiveAll(client, tcp_header.data(), tcp_header.size()))
                break;
            if (ReadU16(tcp_header.data()) != 0)
                break;
            const auto ams_size = ReadU32(tcp_header.data() + 2);
            if (ams_size < kAmsHeaderSize || ams_size > 1024 * 1024)
                break;

            std::vector<std::uint8_t> request(ams_size);
            if (!ReceiveAll(client, request.data(), request.size()))
                break;
            const auto command = ReadU16(request.data() + 16);
            const auto invoke = ReadU32(request.data() + 28);
            const auto data_size = ReadU32(request.data() + 20);
            if (data_size > request.size() - kAmsHeaderSize)
                break;

            const auto *data = request.data() + kAmsHeaderSize;
            std::uint32_t group = 0;
            std::uint32_t offset = 0;
            std::uint32_t length = 0;
            std::uint32_t write_length = 0;
            std::string symbol;
            std::uint32_t ams_error = ADSERR_NOERR;
            std::vector<std::uint8_t> payload;
            bool is_write = false;

            if (command == kAdsCommandReadState) {
                AppendU32(payload, ADSERR_NOERR);
                AppendU16(payload, 5); // ADS_STATE_RUN; only a read check.
                AppendU16(payload, 0);
            } else if (command == kAdsCommandReadWrite && data_size >= 16) {
                group = ReadU32(data);
                offset = ReadU32(data + 4);
                length = ReadU32(data + 8);
                write_length = ReadU32(data + 12);
                if (16ull + write_length > data_size)
                    break;

                if (group == kSymbolHandleByName) {
                    symbol = BoundedString(data + 16, write_length);
                    // Make the optional ERCP interface absent. That keeps
                    // this probe focused on the common dial/control path and
                    // avoids extra optional writes in the timing baseline.
                    if (symbol == "POU_Ercp_CycleExecute.Ercp_Ready_State") {
                        ams_error = ADSERR_DEVICE_SYMBOLNOTFOUND;
                    } else if (length == sizeof(std::uint32_t)) {
                        const auto handle = HandleFor(symbol);
                        AppendU32(payload, ADSERR_NOERR);
                        AppendU32(payload, sizeof(handle));
                        AppendU32(payload, handle);
                    } else {
                        ams_error = ADSERR_DEVICE_INVALIDSIZE;
                    }
                } else if (group == kSumRead) {
                    const auto count = offset;
                    const auto expected_request_size = 16ull + count * 12ull;
                    if (expected_request_size > data_size) {
                        ams_error = ADSERR_DEVICE_INVALIDSIZE;
                    } else {
                        AppendU32(payload, ADSERR_NOERR);
                        AppendU32(payload, length);
                        std::size_t cursor = 16;
                        std::uint32_t produced = 0;
                        for (std::uint32_t index = 0; index < count; ++index) {
                            const auto item_group = ReadU32(data + cursor);
                            const auto item_handle = ReadU32(data + cursor + 4);
                            const auto item_length = ReadU32(data + cursor + 8);
                            cursor += 12;
                            (void)item_group;
                            AppendU32(payload, ADSERR_NOERR);
                            const auto value = ReadValue(SymbolFor(item_handle), item_length);
                            payload.insert(payload.end(), value.begin(), value.end());
                            produced += item_length;
                        }
                        if (produced + count * sizeof(std::uint32_t) != length)
                            ams_error = ADSERR_DEVICE_INVALIDSIZE;
                    }
                } else {
                    ams_error = ADSERR_DEVICE_INVALIDGRP;
                }
            } else if (command == kAdsCommandRead && data_size >= 12) {
                group = ReadU32(data);
                offset = ReadU32(data + 4);
                length = ReadU32(data + 8);
                const auto value = ReadValue(SymbolFor(offset), length);
                AppendU32(payload, ADSERR_NOERR);
                AppendU32(payload, length);
                payload.insert(payload.end(), value.begin(), value.end());
            } else if (command == kAdsCommandWrite && data_size >= 12) {
                group = ReadU32(data);
                offset = ReadU32(data + 4);
                length = ReadU32(data + 8);
                is_write = true;
                if (12ull + length > data_size) {
                    ams_error = ADSERR_DEVICE_INVALIDSIZE;
                } else {
                    symbol = SymbolFor(offset);
                    AppendU32(payload, ADSERR_NOERR);
                }
            } else if (command == kAdsCommandWriteControl && data_size >= 8) {
                length = ReadU32(data + 4);
                is_write = true;
                if (8ull + length > data_size)
                    ams_error = ADSERR_DEVICE_INVALIDSIZE;
                else
                    AppendU32(payload, ADSERR_NOERR);
            } else {
                ams_error = ADSERR_DEVICE_INVALIDPARM;
            }

            const auto received = UnixNowNs();
            if (ShouldDelay(command, group))
                std::this_thread::sleep_for(std::chrono::milliseconds(options.response_delay_ms));
            const auto responded = UnixNowNs();

            AddRecord({received,
                       responded,
                       command,
                       group,
                       offset,
                       length,
                       ams_error,
                       is_write,
                       symbol});

            const auto response = MakeResponse(request, command, invoke, ams_error, payload);
            if (!SendAll(client, response.data(), response.size()))
                break;
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            if (client_socket == client)
                client_socket = INVALID_SOCKET;
        }
        shutdown(client, SD_BOTH);
        closesocket(client);
    }
};

FakeAdsServer::FakeAdsServer()
    : impl_(std::make_unique<Impl>())
{
}

FakeAdsServer::~FakeAdsServer()
{
    Stop();
}

bool FakeAdsServer::Start(const FakeAdsOptions &options, std::string *error)
{
    return impl_->Start(options, error);
}

void FakeAdsServer::Stop()
{
    impl_->Stop();
}

std::uint16_t FakeAdsServer::port() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->bound_port;
}

std::vector<AdsRequestRecord> FakeAdsServer::Records() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->records;
}

} // namespace ercp::timing_probe

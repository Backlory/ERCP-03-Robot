#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

#include "direct_ads_transport.hpp"

namespace device { namespace beckhoff {
namespace {

constexpr std::uint16_t kAdsCommandRead = 2;
constexpr std::uint16_t kAdsCommandWrite = 3;
constexpr std::uint16_t kAdsCommandReadState = 4;
constexpr std::uint16_t kAdsCommandWriteControl = 5;
constexpr std::uint16_t kAdsCommandReadWrite = 9;
constexpr std::uint16_t kAmsCommandRequest = 0x0004;
constexpr std::size_t kAmsHeaderSize = 32;
constexpr std::uint32_t kMaximumFrameSize = 1024 * 1024;

void AppendU16(std::vector<std::uint8_t> &buffer, std::uint16_t value)
{
    buffer.push_back(static_cast<std::uint8_t>(value & 0xff));
    buffer.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
}

void AppendU32(std::vector<std::uint8_t> &buffer, std::uint32_t value)
{
    for (unsigned shift = 0; shift < 32; shift += 8)
        buffer.push_back(static_cast<std::uint8_t>((value >> shift) & 0xff));
}

std::uint16_t ReadU16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0])
        | static_cast<std::uint16_t>(data[1]) << 8;
}

std::uint32_t ReadU32(const std::uint8_t *data)
{
    return static_cast<std::uint32_t>(data[0])
        | static_cast<std::uint32_t>(data[1]) << 8
        | static_cast<std::uint32_t>(data[2]) << 16
        | static_cast<std::uint32_t>(data[3]) << 24;
}

bool SendAll(SOCKET socket, const std::uint8_t *data, std::size_t size)
{
    while (size != 0) {
        const auto chunk = static_cast<int>((std::min)(
            size, static_cast<std::size_t>((std::numeric_limits<int>::max)())));
        const int sent = send(socket, reinterpret_cast<const char *>(data), chunk, 0);
        if (sent == SOCKET_ERROR || sent == 0) return false;
        data += sent;
        size -= static_cast<std::size_t>(sent);
    }
    return true;
}

bool ReceiveAll(SOCKET socket, std::uint8_t *data, std::size_t size)
{
    while (size != 0) {
        const auto chunk = static_cast<int>((std::min)(
            size, static_cast<std::size_t>((std::numeric_limits<int>::max)())));
        const int received = recv(socket, reinterpret_cast<char *>(data), chunk, 0);
        if (received == SOCKET_ERROR || received == 0) return false;
        data += received;
        size -= static_cast<std::size_t>(received);
    }
    return true;
}

} // namespace

struct DirectAdsTransport::Impl {
    SOCKET socket = INVALID_SOCKET;
    bool winsockStarted = false;
    AmsAddr target{};
    std::uint32_t invokeId = 0;

    std::uint32_t Exchange(std::uint16_t command, const std::vector<std::uint8_t> &request,
        std::vector<std::uint8_t> &response)
    {
        if (socket == INVALID_SOCKET) return ADSERR_CLIENT_PORTNOTOPEN;

        const std::uint32_t invoke = ++invokeId;
        std::vector<std::uint8_t> frame;
        frame.reserve(6 + kAmsHeaderSize + request.size());
        AppendU16(frame, 0);
        AppendU32(frame, static_cast<std::uint32_t>(kAmsHeaderSize + request.size()));
        frame.insert(frame.end(), target.netId.b, target.netId.b + 6);
        AppendU16(frame, target.port);
        const std::array<std::uint8_t, 6> source{{127, 0, 0, 1, 1, 2}};
        frame.insert(frame.end(), source.begin(), source.end());
        AppendU16(frame, 32905);
        AppendU16(frame, command);
        AppendU16(frame, kAmsCommandRequest);
        AppendU32(frame, static_cast<std::uint32_t>(request.size()));
        AppendU32(frame, 0);
        AppendU32(frame, invoke);
        frame.insert(frame.end(), request.begin(), request.end());

        if (!SendAll(socket, frame.data(), frame.size())) {
            CloseSocket();
            return ADSERR_CLIENT_W32ERROR;
        }

        std::array<std::uint8_t, 6> tcpHeader{};
        if (!ReceiveAll(socket, tcpHeader.data(), tcpHeader.size())) {
            CloseSocket();
            return ADSERR_CLIENT_SYNCTIMEOUT;
        }
        const auto reserved = ReadU16(tcpHeader.data());
        const auto length = ReadU32(tcpHeader.data() + 2);
        if (reserved != 0 || length < kAmsHeaderSize || length > kMaximumFrameSize)
            return ADSERR_CLIENT_SYNCRESINVALID;

        std::vector<std::uint8_t> ams(length);
        if (!ReceiveAll(socket, ams.data(), ams.size())) {
            CloseSocket();
            return ADSERR_CLIENT_SYNCTIMEOUT;
        }
        if (ReadU16(ams.data() + 16) != command || ReadU32(ams.data() + 28) != invoke)
            return ADSERR_CLIENT_SYNCRESINVALID;
        const auto amsError = ReadU32(ams.data() + 24);
        if (amsError != 0) return amsError;
        const auto payloadLength = ReadU32(ams.data() + 20);
        if (payloadLength != ams.size() - kAmsHeaderSize)
            return ADSERR_CLIENT_SYNCRESINVALID;
        response.assign(ams.begin() + kAmsHeaderSize, ams.end());
        return ADSERR_NOERR;
    }

    void CloseSocket()
    {
        if (socket != INVALID_SOCKET) {
            shutdown(socket, SD_BOTH);
            closesocket(socket);
            socket = INVALID_SOCKET;
        }
    }
};

DirectAdsTransport::DirectAdsTransport() : impl_(new Impl) {}

DirectAdsTransport::~DirectAdsTransport()
{
    Close();
}

bool DirectAdsTransport::Connect(
    const std::string &host, std::uint16_t tcpPort, const AmsAddr &target)
{
    Close();
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return false;
    impl_->winsockStarted = true;

    impl_->socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (impl_->socket == INVALID_SOCKET) {
        Close();
        return false;
    }

    DWORD timeoutMs = 2000;
    setsockopt(impl_->socket, SOL_SOCKET, SO_RCVTIMEO,
        reinterpret_cast<const char *>(&timeoutMs), sizeof(timeoutMs));
    setsockopt(impl_->socket, SOL_SOCKET, SO_SNDTIMEO,
        reinterpret_cast<const char *>(&timeoutMs), sizeof(timeoutMs));
    BOOL noDelay = TRUE;
    setsockopt(impl_->socket, IPPROTO_TCP, TCP_NODELAY,
        reinterpret_cast<const char *>(&noDelay), sizeof(noDelay));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(tcpPort);
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1
        || connect(impl_->socket, reinterpret_cast<const sockaddr *>(&address),
               sizeof(address))
            == SOCKET_ERROR) {
        Close();
        return false;
    }
    impl_->target = target;
    impl_->invokeId = 0;
    return true;
}

void DirectAdsTransport::Close()
{
    impl_->CloseSocket();
    if (impl_->winsockStarted) {
        WSACleanup();
        impl_->winsockStarted = false;
    }
}

bool DirectAdsTransport::IsConnected() const
{
    return impl_->socket != INVALID_SOCKET;
}

std::uint32_t DirectAdsTransport::ReadState(
    std::uint16_t &adsState, std::uint16_t &deviceState)
{
    std::vector<std::uint8_t> response;
    const auto exchangeResult = impl_->Exchange(kAdsCommandReadState, {}, response);
    if (exchangeResult != ADSERR_NOERR) return exchangeResult;
    if (response.size() < 8) return ADSERR_CLIENT_SYNCRESINVALID;
    const auto result = ReadU32(response.data());
    if (result == ADSERR_NOERR) {
        adsState = ReadU16(response.data() + 4);
        deviceState = ReadU16(response.data() + 6);
    }
    return result;
}

std::uint32_t DirectAdsTransport::WriteControl(std::uint16_t adsState,
    std::uint16_t deviceState, std::uint32_t length, const void *data)
{
    if (length != 0 && data == nullptr) return ADSERR_CLIENT_INVALIDPARM;
    std::vector<std::uint8_t> request;
    AppendU16(request, adsState);
    AppendU16(request, deviceState);
    AppendU32(request, length);
    if (length != 0) {
        const auto *bytes = static_cast<const std::uint8_t *>(data);
        request.insert(request.end(), bytes, bytes + length);
    }
    std::vector<std::uint8_t> response;
    const auto exchangeResult = impl_->Exchange(kAdsCommandWriteControl, request, response);
    if (exchangeResult != ADSERR_NOERR) return exchangeResult;
    return response.size() >= 4 ? ReadU32(response.data()) : ADSERR_CLIENT_SYNCRESINVALID;
}

std::uint32_t DirectAdsTransport::Read(std::uint32_t indexGroup,
    std::uint32_t indexOffset, std::uint32_t length, void *data)
{
    if (length != 0 && data == nullptr) return ADSERR_CLIENT_INVALIDPARM;
    std::vector<std::uint8_t> request;
    AppendU32(request, indexGroup);
    AppendU32(request, indexOffset);
    AppendU32(request, length);
    std::vector<std::uint8_t> response;
    const auto exchangeResult = impl_->Exchange(kAdsCommandRead, request, response);
    if (exchangeResult != ADSERR_NOERR) return exchangeResult;
    if (response.size() < 8) return ADSERR_CLIENT_SYNCRESINVALID;
    const auto result = ReadU32(response.data());
    const auto bytesRead = ReadU32(response.data() + 4);
    if (result == ADSERR_NOERR) {
        if (bytesRead > length || response.size() != 8 + bytesRead)
            return ADSERR_CLIENT_SYNCRESINVALID;
        if (bytesRead != 0) std::memcpy(data, response.data() + 8, bytesRead);
    }
    return result;
}

std::uint32_t DirectAdsTransport::Write(std::uint32_t indexGroup,
    std::uint32_t indexOffset, std::uint32_t length, const void *data)
{
    if (length != 0 && data == nullptr) return ADSERR_CLIENT_INVALIDPARM;
    std::vector<std::uint8_t> request;
    AppendU32(request, indexGroup);
    AppendU32(request, indexOffset);
    AppendU32(request, length);
    if (length != 0) {
        const auto *bytes = static_cast<const std::uint8_t *>(data);
        request.insert(request.end(), bytes, bytes + length);
    }
    std::vector<std::uint8_t> response;
    const auto exchangeResult = impl_->Exchange(kAdsCommandWrite, request, response);
    if (exchangeResult != ADSERR_NOERR) return exchangeResult;
    return response.size() >= 4 ? ReadU32(response.data()) : ADSERR_CLIENT_SYNCRESINVALID;
}

std::uint32_t DirectAdsTransport::ReadWrite(std::uint32_t indexGroup,
    std::uint32_t indexOffset, std::uint32_t readLength, void *readData,
    std::uint32_t writeLength, const void *writeData, std::uint32_t *bytesRead)
{
    if ((readLength != 0 && readData == nullptr)
        || (writeLength != 0 && writeData == nullptr))
        return ADSERR_CLIENT_INVALIDPARM;
    std::vector<std::uint8_t> request;
    AppendU32(request, indexGroup);
    AppendU32(request, indexOffset);
    AppendU32(request, readLength);
    AppendU32(request, writeLength);
    if (writeLength != 0) {
        const auto *bytes = static_cast<const std::uint8_t *>(writeData);
        request.insert(request.end(), bytes, bytes + writeLength);
    }
    std::vector<std::uint8_t> response;
    const auto exchangeResult = impl_->Exchange(kAdsCommandReadWrite, request, response);
    if (exchangeResult != ADSERR_NOERR) return exchangeResult;
    if (response.size() < 8) return ADSERR_CLIENT_SYNCRESINVALID;
    const auto result = ReadU32(response.data());
    const auto actual = ReadU32(response.data() + 4);
    if (bytesRead != nullptr) *bytesRead = actual;
    if (result == ADSERR_NOERR) {
        if (actual > readLength || response.size() != 8 + actual)
            return ADSERR_CLIENT_SYNCRESINVALID;
        if (actual != 0) std::memcpy(readData, response.data() + 8, actual);
    }
    return result;
}

}} // namespace device::beckhoff

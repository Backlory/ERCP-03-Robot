#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "TcAdsDef.h"

namespace device {
namespace beckhoff {

class DirectAdsTransport {
public:
    DirectAdsTransport();
    ~DirectAdsTransport();

    DirectAdsTransport(const DirectAdsTransport &) = delete;
    DirectAdsTransport &operator=(const DirectAdsTransport &) = delete;

    bool Connect(const std::string &host, std::uint16_t tcpPort, const AmsAddr &target);
    void Close();
    bool IsConnected() const;

    std::uint32_t ReadState(std::uint16_t &adsState, std::uint16_t &deviceState);
    std::uint32_t WriteControl(std::uint16_t adsState,
                               std::uint16_t deviceState,
                               std::uint32_t length,
                               const void *data);
    std::uint32_t
    Read(std::uint32_t indexGroup, std::uint32_t indexOffset, std::uint32_t length, void *data);
    std::uint32_t Write(std::uint32_t indexGroup,
                        std::uint32_t indexOffset,
                        std::uint32_t length,
                        const void *data);
    std::uint32_t ReadWrite(std::uint32_t indexGroup,
                            std::uint32_t indexOffset,
                            std::uint32_t readLength,
                            void *readData,
                            std::uint32_t writeLength,
                            const void *writeData,
                            std::uint32_t *bytesRead = nullptr);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}
} // namespace device::beckhoff

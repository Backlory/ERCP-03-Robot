#pragma once

// Compatibility entry for the historical V2 include path.
// New code includes robot_udp_v3.hpp and uses ercp::protocol::v3.
#include "robot_udp_v3.hpp"

namespace ercp::protocol::v3 {
inline constexpr int kRobotUdpV2SyncVersion = kRobotUdpV3SyncVersion;
}

namespace ercp::protocol {
namespace v2 = v3;
}

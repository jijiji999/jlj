#include "dm_motor/socketcan_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace dm_motor
{

SocketCanTransport::SocketCanTransport(const std::string & interface_name)
: interface_name_(interface_name)
{
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw std::runtime_error(
            "Failed to create CAN socket for " + interface_name_ + ": " + std::strerror(errno));
  }

  int enable_can_fd = 1;
  if (::setsockopt(
        socket_fd_,
        SOL_CAN_RAW,
        CAN_RAW_FD_FRAMES,
        &enable_can_fd,
        sizeof(enable_can_fd)) < 0)
  {
    const auto message = "Failed to enable CAN FD on " + interface_name_ + ": " + std::strerror(errno);
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error(message);
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    const auto message =
      "Failed to query interface index for " + interface_name_ + ": " + std::strerror(errno);
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error(message);
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    const auto message = "Failed to bind CAN socket for " + interface_name_ + ": " + std::strerror(errno);
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error(message);
  }
}

SocketCanTransport::~SocketCanTransport()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

const std::string & SocketCanTransport::interface_name() const
{
  return interface_name_;
}

void SocketCanTransport::send(const CanFrame & frame)
{
  std::lock_guard<std::mutex> lock(io_mutex_);

  struct canfd_frame native {};
  native.can_id = frame.is_extended_id ? (frame.arbitration_id | CAN_EFF_FLAG) : frame.arbitration_id;
  if (frame.bitrate_switch) {
    native.flags |= CANFD_BRS;
  }

  native.len = static_cast<__u8>(frame.size);
  std::copy(frame.data.begin(), frame.data.begin() + frame.size, native.data);

  const auto written = ::write(socket_fd_, &native, sizeof(native));
  if (written != static_cast<ssize_t>(sizeof(native))) {
    throw std::runtime_error(
            "Failed to send CAN frame on " + interface_name_ + ": " + std::strerror(errno));
  }
}

std::optional<CanFrame> SocketCanTransport::receive(const std::chrono::milliseconds timeout)
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (!wait_readable(timeout)) {
    return std::nullopt;
  }
  return read_frame_unlocked();
}

std::vector<CanFrame> SocketCanTransport::receive_available(
  const std::chrono::milliseconds first_timeout,
  const size_t max_frames)
{
  std::vector<CanFrame> frames;
  std::lock_guard<std::mutex> lock(io_mutex_);

  if (!wait_readable(first_timeout)) {
    return frames;
  }

  frames.emplace_back(read_frame_unlocked());
  while (frames.size() < max_frames && wait_readable(std::chrono::milliseconds(0))) {
    frames.emplace_back(read_frame_unlocked());
  }

  return frames;
}

bool SocketCanTransport::wait_readable(const std::chrono::milliseconds timeout) const
{
  struct pollfd pfd {};
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  const int timeout_ms =
    timeout.count() > static_cast<long long>(std::numeric_limits<int>::max()) ?
    std::numeric_limits<int>::max() :
    static_cast<int>(timeout.count());

  const int rc = ::poll(&pfd, 1, timeout_ms);
  if (rc < 0) {
    throw std::runtime_error(
            "poll() failed on " + interface_name_ + ": " + std::strerror(errno));
  }

  return rc > 0 && (pfd.revents & POLLIN) != 0;
}

CanFrame SocketCanTransport::read_frame_unlocked() const
{
  struct canfd_frame native {};
  const auto bytes_read = ::read(socket_fd_, &native, sizeof(native));
  if (bytes_read < 0) {
    throw std::runtime_error(
            "Failed to read CAN frame on " + interface_name_ + ": " + std::strerror(errno));
  }

  CanFrame frame;
  frame.arbitration_id = native.can_id & CAN_EFF_MASK;
  frame.is_extended_id = (native.can_id & CAN_EFF_FLAG) != 0;
  frame.is_fd = bytes_read == static_cast<ssize_t>(sizeof(struct canfd_frame));
  frame.bitrate_switch = (native.flags & CANFD_BRS) != 0;
  frame.size = native.len;
  std::copy(native.data, native.data + native.len, frame.data.begin());
  return frame;
}

}  // namespace dm_motor

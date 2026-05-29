#ifndef DM_MOTOR__SOCKETCAN_TRANSPORT_HPP_
#define DM_MOTOR__SOCKETCAN_TRANSPORT_HPP_

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "dm_motor/types.hpp"

namespace dm_motor
{

class SocketCanTransport
{
public:
  explicit SocketCanTransport(const std::string & interface_name);
  ~SocketCanTransport();

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  const std::string & interface_name() const;
  void send(const CanFrame & frame);
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout);
  std::vector<CanFrame> receive_available(
    std::chrono::milliseconds first_timeout,
    size_t max_frames = 128U);

private:
  bool wait_readable(std::chrono::milliseconds timeout) const;
  CanFrame read_frame_unlocked() const;

  int socket_fd_ {-1};
  std::string interface_name_;
  mutable std::mutex io_mutex_;
};

}  // namespace dm_motor

#endif  // DM_MOTOR__SOCKETCAN_TRANSPORT_HPP_

#pragma once

// Cross-platform TCP client/listener with all platform socket code confined to
// tcp_connection.cpp — this header exposes no <winsock2.h>/<sys/socket.h>, so
// consumers depend only on the standard library. The socket handle is stored as
// an intptr_t (fits a POSIX fd and a Windows SOCKET); -1 means "not open".
//
// Line framing: the protocol is newline-delimited JSON, so the connection reads
// and writes whole '\n'-terminated lines and buffers any partial trailing line
// between reads.

#include <cstdint>
#include <string>
#include <vector>

namespace cavr::adapters::generic_tcp_robot {

class TcpListener;  // friend: hands a freshly accepted socket to a TcpConnection

// A TCP client connection. Blocking operations take a millisecond timeout;
// drain_lines() is non-blocking and suits per-tick telemetry polling.
class TcpConnection final {
 public:
  TcpConnection() = default;
  ~TcpConnection();

  TcpConnection(const TcpConnection&) = delete;
  TcpConnection& operator=(const TcpConnection&) = delete;
  TcpConnection(TcpConnection&& other) noexcept;
  TcpConnection& operator=(TcpConnection&& other) noexcept;

  // Connects to host:port, waiting up to timeout_ms. Returns an empty string on
  // success, otherwise a human-readable error.
  [[nodiscard]] std::string connect(const std::string& host, std::uint16_t port, int timeout_ms = 3000);

  [[nodiscard]] bool is_open() const noexcept { return sock_ >= 0; }
  void close() noexcept;

  // Sends `line` followed by '\n'. Returns an empty string on success.
  [[nodiscard]] std::string send_line(const std::string& line);

  // Blocks until a complete '\n'-terminated line arrives or timeout_ms elapses.
  // On success sets `out` (newline stripped) and returns true. On timeout sets
  // `timed_out` and returns false; a closed/errored peer returns false with
  // `timed_out` false.
  [[nodiscard]] bool read_line(int timeout_ms, std::string& out, bool& timed_out);

  // Non-blocking: drains all bytes currently available and appends every
  // complete line to `out`. Returns false if the peer has closed the connection
  // (after appending any lines that had already arrived).
  [[nodiscard]] bool drain_lines(std::vector<std::string>& out);

 private:
  friend class TcpListener;
  explicit TcpConnection(std::intptr_t sock) : sock_(sock) {}

  std::intptr_t sock_{-1};
  std::string rx_;  // buffered bytes not yet split into a complete line
};

// A listening socket bound to 127.0.0.1, used by tests and reference servers so
// that all platform socket code stays in this one translation unit.
class TcpListener final {
 public:
  TcpListener() = default;
  ~TcpListener();

  TcpListener(const TcpListener&) = delete;
  TcpListener& operator=(const TcpListener&) = delete;

  // Binds to 127.0.0.1:port (port 0 picks an ephemeral port, readable via
  // port()) and starts listening. Returns an empty string on success.
  [[nodiscard]] std::string listen(std::uint16_t port);

  [[nodiscard]] std::uint16_t port() const noexcept { return port_; }
  [[nodiscard]] bool is_open() const noexcept { return sock_ >= 0; }
  void close() noexcept;

  // Accepts one connection, waiting up to timeout_ms. The returned connection is
  // open on success (check is_open()).
  [[nodiscard]] TcpConnection accept(int timeout_ms = 3000);

 private:
  std::intptr_t sock_{-1};
  std::uint16_t port_{0};
};

}  // namespace cavr::adapters::generic_tcp_robot

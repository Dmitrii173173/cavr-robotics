// The single translation unit that touches the platform socket API. Winsock on
// Windows, BSD sockets elsewhere; the differences are bridged by a handful of
// aliases so the body reads the same on both.

#include <cavr/adapters/generic_tcp_robot/tcp_connection.hpp>

#include <cstring>
#include <string>
#include <utility>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
using socklen_t = int;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace cavr::adapters::generic_tcp_robot {

namespace {

#if defined(_WIN32)
constexpr std::intptr_t kInvalid = -1;  // INVALID_SOCKET == (SOCKET)(~0) == -1 as intptr_t

// Winsock needs one-time startup. A function-local static runs it once and
// schedules cleanup at exit; harmless if a process also inits Winsock elsewhere.
struct WsaInit {
  WsaInit() {
    WSADATA data;
    WSAStartup(MAKEWORD(2, 2), &data);
  }
  ~WsaInit() { WSACleanup(); }
};
void ensure_winsock() { static WsaInit init; }

int last_error() { return WSAGetLastError(); }
bool would_block(int err) { return err == WSAEWOULDBLOCK; }
void close_socket(std::intptr_t s) { closesocket(static_cast<SOCKET>(s)); }
constexpr int kSendFlags = 0;
#else
constexpr std::intptr_t kInvalid = -1;
void ensure_winsock() {}
int last_error() { return errno; }
bool would_block(int err) { return err == EWOULDBLOCK || err == EAGAIN; }
void close_socket(std::intptr_t s) { ::close(static_cast<int>(s)); }
#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;  // don't raise SIGPIPE on a closed peer (Linux)
#else
constexpr int kSendFlags = 0;             // macOS/BSD use SO_NOSIGPIPE below instead
#endif
#endif

// Sets non-blocking mode; returns true on success.
bool set_nonblocking(std::intptr_t s, bool on) {
#if defined(_WIN32)
  u_long mode = on ? 1 : 0;
  return ioctlsocket(static_cast<SOCKET>(s), FIONBIO, &mode) == 0;
#else
  const int flags = fcntl(static_cast<int>(s), F_GETFL, 0);
  if (flags < 0) return false;
  const int updated = on ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
  return fcntl(static_cast<int>(s), F_SETFL, updated) == 0;
#endif
}

// Suppresses SIGPIPE per-socket where the send flag isn't available (macOS/BSD).
void suppress_sigpipe(std::intptr_t s) {
#if defined(SO_NOSIGPIPE)
  int on = 1;
  setsockopt(static_cast<int>(s), SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#else
  (void)s;
#endif
}

// Waits until `s` is readable (want_write=false) or writable (want_write=true),
// up to timeout_ms. Returns 1 ready, 0 timeout, -1 error.
int wait_ready(std::intptr_t s, bool want_write, int timeout_ms) {
  fd_set set;
  FD_ZERO(&set);
  FD_SET(static_cast<int>(s), &set);
  timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  const int nfds = static_cast<int>(s) + 1;
  if (want_write) return ::select(nfds, nullptr, &set, nullptr, &tv);
  return ::select(nfds, &set, nullptr, nullptr, &tv);
}

// Moves every complete '\n'-terminated line out of `buffer` into `out`.
void extract_lines(std::string& buffer, std::vector<std::string>& out) {
  std::size_t start = 0;
  std::size_t nl;
  while ((nl = buffer.find('\n', start)) != std::string::npos) {
    std::string line = buffer.substr(start, nl - start);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    out.push_back(std::move(line));
    start = nl + 1;
  }
  buffer.erase(0, start);
}

}  // namespace

// ------------------------------------------------------------------ TcpConnection

TcpConnection::~TcpConnection() { close(); }

TcpConnection::TcpConnection(TcpConnection&& other) noexcept
    : sock_(other.sock_), rx_(std::move(other.rx_)) {
  other.sock_ = kInvalid;
}

TcpConnection& TcpConnection::operator=(TcpConnection&& other) noexcept {
  if (this != &other) {
    close();
    sock_ = other.sock_;
    rx_ = std::move(other.rx_);
    other.sock_ = kInvalid;
  }
  return *this;
}

void TcpConnection::close() noexcept {
  if (sock_ >= 0) {
    close_socket(sock_);
    sock_ = kInvalid;
  }
  rx_.clear();
}

std::string TcpConnection::connect(const std::string& host, std::uint16_t port, int timeout_ms) {
  ensure_winsock();
  close();

  addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  addrinfo* result = nullptr;
  const std::string port_str = std::to_string(port);
  if (::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &result) != 0 || result == nullptr) {
    return "Failed to resolve " + host + ":" + port_str;
  }

  const std::intptr_t s = static_cast<std::intptr_t>(
      ::socket(result->ai_family, result->ai_socktype, result->ai_protocol));
  if (s < 0) {
    ::freeaddrinfo(result);
    return "Failed to create socket";
  }
  suppress_sigpipe(s);

  // Non-blocking connect with a select() timeout so a dead host doesn't hang.
  set_nonblocking(s, true);
  const int rc = ::connect(s, result->ai_addr, static_cast<socklen_t>(result->ai_addrlen));
  ::freeaddrinfo(result);

  if (rc != 0) {
    const int err = last_error();
#if defined(_WIN32)
    const bool in_progress = would_block(err) || err == WSAEINPROGRESS;
#else
    const bool in_progress = (err == EINPROGRESS);
#endif
    if (!in_progress) {
      close_socket(s);
      return "Failed to connect to " + host + ":" + port_str;
    }
    if (wait_ready(s, /*want_write=*/true, timeout_ms) != 1) {
      close_socket(s);
      return "Connection to " + host + ":" + port_str + " timed out";
    }
    int so_error = 0;
    socklen_t len = sizeof(so_error);
    ::getsockopt(static_cast<int>(s), SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len);
    if (so_error != 0) {
      close_socket(s);
      return "Failed to connect to " + host + ":" + port_str;
    }
  }

  set_nonblocking(s, false);
  {
    int one = 1;
    ::setsockopt(static_cast<int>(s), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&one),
                 sizeof(one));
  }
  sock_ = s;
  return {};
}

std::string TcpConnection::send_line(const std::string& line) {
  if (sock_ < 0) return "Connection is not open";
  const std::string payload = line + "\n";
  std::size_t sent = 0;
  while (sent < payload.size()) {
    const auto n = ::send(static_cast<int>(sock_), payload.data() + sent,
                          static_cast<int>(payload.size() - sent), kSendFlags);
    if (n <= 0) {
      if (n < 0 && would_block(last_error())) {
        if (wait_ready(sock_, /*want_write=*/true, 1000) == 1) continue;
      }
      return "Failed to send on socket";
    }
    sent += static_cast<std::size_t>(n);
  }
  return {};
}

bool TcpConnection::read_line(int timeout_ms, std::string& out, bool& timed_out) {
  timed_out = false;
  if (sock_ < 0) return false;

  // Serve a line already buffered from a previous read.
  {
    std::vector<std::string> lines;
    extract_lines(rx_, lines);
    if (!lines.empty()) {
      out = std::move(lines.front());
      // push any extra lines back onto the buffer, in order
      for (std::size_t i = lines.size(); i-- > 1;) rx_.insert(0, lines[i] + "\n");
      return true;
    }
  }

  char buffer[4096];
  for (;;) {
    const int ready = wait_ready(sock_, /*want_write=*/false, timeout_ms);
    if (ready == 0) {
      timed_out = true;
      return false;
    }
    if (ready < 0) return false;

    const auto n = ::recv(static_cast<int>(sock_), buffer, sizeof(buffer), 0);
    if (n == 0) return false;  // peer closed
    if (n < 0) {
      if (would_block(last_error())) continue;
      return false;
    }
    rx_.append(buffer, static_cast<std::size_t>(n));

    std::vector<std::string> lines;
    extract_lines(rx_, lines);
    if (!lines.empty()) {
      out = std::move(lines.front());
      for (std::size_t i = lines.size(); i-- > 1;) rx_.insert(0, lines[i] + "\n");
      return true;
    }
  }
}

bool TcpConnection::drain_lines(std::vector<std::string>& out) {
  if (sock_ < 0) return false;

  bool peer_open = true;
  char buffer[4096];
  for (;;) {
    const int ready = wait_ready(sock_, /*want_write=*/false, 0);  // poll, no wait
    if (ready <= 0) break;                                          // nothing more available

    const auto n = ::recv(static_cast<int>(sock_), buffer, sizeof(buffer), 0);
    if (n == 0) {
      peer_open = false;  // peer closed
      break;
    }
    if (n < 0) {
      if (would_block(last_error())) break;
      peer_open = false;
      break;
    }
    rx_.append(buffer, static_cast<std::size_t>(n));
  }

  extract_lines(rx_, out);
  return peer_open;
}

// ------------------------------------------------------------------ TcpListener

TcpListener::~TcpListener() { close(); }

void TcpListener::close() noexcept {
  if (sock_ >= 0) {
    close_socket(sock_);
    sock_ = kInvalid;
  }
}

std::string TcpListener::listen(std::uint16_t port) {
  ensure_winsock();
  close();

  const std::intptr_t s = static_cast<std::intptr_t>(::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
  if (s < 0) return "Failed to create listening socket";

  int reuse = 1;
  ::setsockopt(static_cast<int>(s), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&reuse),
               sizeof(reuse));

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);
  if (::bind(static_cast<int>(s), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close_socket(s);
    return "Failed to bind listening socket";
  }
  if (::listen(static_cast<int>(s), 1) != 0) {
    close_socket(s);
    return "Failed to listen";
  }

  // Read back the bound port (relevant when port 0 requested an ephemeral one).
  sockaddr_in bound;
  socklen_t len = sizeof(bound);
  if (::getsockname(static_cast<int>(s), reinterpret_cast<sockaddr*>(&bound), &len) == 0) {
    port_ = ntohs(bound.sin_port);
  } else {
    port_ = port;
  }
  sock_ = s;
  return {};
}

TcpConnection TcpListener::accept(int timeout_ms) {
  if (sock_ < 0) return TcpConnection(kInvalid);
  if (wait_ready(sock_, /*want_write=*/false, timeout_ms) != 1) return TcpConnection(kInvalid);

  const std::intptr_t client = static_cast<std::intptr_t>(::accept(static_cast<int>(sock_), nullptr, nullptr));
  if (client < 0) return TcpConnection(kInvalid);
  suppress_sigpipe(client);
  {
    int one = 1;
    ::setsockopt(static_cast<int>(client), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&one),
                 sizeof(one));
  }
  return TcpConnection(client);
}

}  // namespace cavr::adapters::generic_tcp_robot

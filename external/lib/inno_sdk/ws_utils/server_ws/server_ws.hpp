/**
   The MIT License (MIT)

   Copyright (c) 2014-2018 Ole Christian Eidheim

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#ifndef WS_UTILS_SERVER_WS_SERVER_WS_HPP_
#define WS_UTILS_SERVER_WS_SERVER_WS_HPP_

#include <array>
#include <atomic>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>   // NOLINT
#include <string>
#include <thread>  // NOLINT
#include <unordered_set>
#include <utility>
#include <vector>

#include "ws_utils/server_ws/crypto.hpp"
#include "ws_utils/server_ws/utility.hpp"
#include "src/utils/inno_lidar_log.h"
#include "sdk_common/inno_lidar_packet_utils.h"

#ifdef USE_STANDALONE_ASIO

#include <asio.hpp>
#include <asio/steady_timer.hpp>

namespace SimpleWeb {
using error_code = std::error_code;
using errc = std::errc;
namespace make_error_code = std;
}  // namespace SimpleWeb

#else  // USE_STANDALONE_ASIO

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
namespace SimpleWeb {
namespace asio = boost::asio;
using error_code = boost::system::error_code;
namespace errc = boost::system::errc;
namespace make_error_code = boost::system::errc;
}  // namespace SimpleWeb

#endif  // USE_STANDALONE_ASIO

#if BOOST_VERSION >= 107000
#define GET_IO_SERVICE(s) \
  ((boost::asio::io_context&)(s)->get_executor().context())
#else
#define GET_IO_SERVICE(s) ((s->get_io_service()))
#endif

// Late 2017 TODO: remove the following checks and always use std::regex
#ifdef USE_BOOST_REGEX
#include <boost/regex.hpp>
namespace SimpleWeb {
namespace regex = boost;
}
#else
#include <regex>  // NOLINT
namespace SimpleWeb {
namespace regex = std;
}
#endif

namespace SimpleWeb {
using WS = asio::ip::tcp::socket;
template <class socket_type>
class SocketServer;
class Endpoint;

/****************
 SocketServerBase
****************/
template <class socket_type>
class SocketServerBase {
 public:
  class Message : public std::istream {
    friend class SocketServerBase<socket_type>;

   public:
    unsigned char fin_rsv_opcode;
    std::size_t size() noexcept {
      return length_;
    }

    /// Convenience function to return std::string. The stream buffer is consumed.
    std::string string() noexcept {
      try {
        std::string str;
        auto size = streambuf_.size();
        str.resize(size);
        read(&str[0], static_cast<std::streamsize>(size));
        return str;
      }
      catch(...) {
        return std::string();
      }
    }

   private:
    Message() noexcept
    : std::istream(&streambuf_)
        , length_(0) {
    }

    Message(unsigned char fin_rsv_opcode,
            std::size_t length) noexcept
        : std::istream(&streambuf_)
        , fin_rsv_opcode(fin_rsv_opcode)
        , length_(length) {
    }

    std::size_t length_;
    asio::streambuf streambuf_;
  };  // Message

  /// The buffer is not consumed during send operations.
  /// Do not alter while sending.
  /****************
    SocketServerBase::SendStream
  ****************/
  class SendStream : public std::ostream {
    friend class SocketServerBase<socket_type>;

   private:
    asio::streambuf streambuf_;

   public:
    SendStream() noexcept
    : std::ostream(&streambuf_) {
    }

    /// Returns the size of the buffer
    std::size_t size() const noexcept {
      return streambuf_.size();
    }
  };  // SendStream

  class Connection : public std::enable_shared_from_this<Connection> {
    friend class SocketServerBase<socket_type>;
    friend class SocketServer<socket_type>;

   public:
    std::string method, path, query_string, http_version;
    std::vector<int> topics_subscribed;
    void *ctx;
    CaseInsensitiveMultimap header;
    std::string recv_buffer;
    std::string url;
    regex::smatch path_match;
    asio::ip::tcp::endpoint remote_endpoint;

    std::string remote_endpoint_address() noexcept {
      try {
        return remote_endpoint.address().to_string();
      }
      catch(...) {
        return std::string();
      }
    }

    uint16_t remote_endpoint_port() noexcept {
      return remote_endpoint.port();
    }

    std::string get_source_ip_string() {
      return socket_->remote_endpoint().address().to_string();
    }

    std::string get_recv_buffer() {
      return recv_buffer;
    }

    std::string get_url() {
      return url;
    }

    void close() noexcept {
      error_code ec;
      std::unique_lock<std::mutex> lock(socket_close_mutex_);
      // The following operations seems to be needed to run sequentially
      socket_->lowest_layer().shutdown(
          asio::ip::tcp::socket::shutdown_both, ec);
      socket_->lowest_layer().close(ec);
    }

   private:
    template <typename... Args>
    Connection(void *context,
               std::shared_ptr<ScopeRunner> handler_runner,
               int64_t timeout_idle,
               std::weak_ptr<SocketServerBase<WS>> sockserver,
               Args &&... args) noexcept
        : ctx(context)
        , handler_runner_(std::move(handler_runner))
        , socket_(new socket_type(std::forward<Args>(args)...))
        , timeout_idle_(timeout_idle)
        , strand_(GET_IO_SERVICE(socket_))
        , closed_(false)
        , is_sp_(false) {
      write_buffer_stream_ = std::make_shared<SendStream>();
      bad_ = false;
      timer_set_flag_ = false;
      websock_server_ = sockserver;
    }

   private:
    std::shared_ptr<ScopeRunner> handler_runner_;
    // Socket must be unique_ptr since
    //  asio::ssl::stream<asio::ip::tcp::socket> is not movable
    std::unique_ptr<socket_type> socket_;
    std::mutex socket_close_mutex_;

    asio::streambuf read_buffer_;
    std::shared_ptr<Message> fragmented_message_;

    int64_t timeout_idle_;
    std::unique_ptr<asio::steady_timer> timer_;
    std::unique_ptr<asio::steady_timer> timer_send_timeout_;
    std::mutex timer_mutex_;
    std::shared_ptr<SendStream> write_buffer_stream_;
    bool bad_;
    bool timer_set_flag_;
    std::weak_ptr<SocketServerBase<WS>> websock_server_;

    void set_bad_() {
      bad_ = true;
    }

    bool is_bad_() {
      return  bad_;
    }

    void react_on_timeout_(const std::shared_ptr<Connection> &connection,
                           int timeout, const char* ec_msg) const {
      if (auto conn = websock_server_.lock()) {
        for (auto& ep : conn->endpoint) {
          if (regex::regex_match(connection->path_match.str(), ep.first)) {
            conn->connection_timeout_(connection, timeout,
                                      &ep.second, ec_msg);
            return;
          }
        }
        inno_log_error("no matching endpoint found for conn %p",
                       connection.get());
        inno_log_panic("enforced shutdown!!!");
      }
    }

    void set_send_timeout_(int64_t seconds = 2) noexcept {
      std::unique_lock<std::mutex> lock(timer_mutex_);
      if (seconds == 0) {
        timer_set_flag_ = true;
        timer_send_timeout_ = nullptr;
        return;
      }
      if (timer_set_flag_) {
        return;
      }
      if (timer_send_timeout_ == NULL) {
        timer_send_timeout_ = std::unique_ptr<asio::steady_timer>
                              (new asio::steady_timer(GET_IO_SERVICE(socket_)));
      }
      timer_send_timeout_->expires_from_now(std::chrono::seconds(seconds));
      auto self = this->shared_from_this();
      std::weak_ptr<Connection> connection_weak(self);
      timer_send_timeout_->async_wait(
          [connection_weak, seconds](const error_code &ec) {
            if (!ec) {
              if (auto connection = connection_weak.lock()) {
                connection->react_on_timeout_(connection,
                                              seconds,
                                              "Send time out");
              }
            }
          });
    }

    void set_timeout_(int64_t seconds = -1) {
      bool use_timeout_idle = false;
      if (seconds == -1) {
        use_timeout_idle = true;
        seconds = timeout_idle_;
      }

      std::unique_lock<std::mutex> lock(timer_mutex_);

      if (seconds == 0) {
        timer_ = nullptr;
        return;
      }

      timer_ = std::unique_ptr<asio::steady_timer>
               (new asio::steady_timer(GET_IO_SERVICE(socket_)));
      timer_->expires_from_now(std::chrono::seconds(seconds));
      // To avoid keeping Connection instance alive longer than needed
      std::weak_ptr<Connection> connection_weak(this->shared_from_this());
      timer_->async_wait(
          [connection_weak, use_timeout_idle](const error_code &ec) {
            if (!ec) {
              if (auto connection = connection_weak.lock()) {
                if (use_timeout_idle) {
                  // 1000=normal closure
                  connection->send_close(1000, "idle timeout");
                } else {
                  connection->close();
                }
              }
            }
          });
    }

    void cancel_timeout_() noexcept {
      std::unique_lock<std::mutex> lock(timer_mutex_);
      if (timer_) {
        error_code ec;
        timer_->cancel(ec);
      }
    }

    void cancel_send_timeout_() noexcept {
      std::unique_lock<std::mutex> lock(timer_mutex_);
      if (timer_send_timeout_) {
        error_code ec;
        timer_send_timeout_->cancel(ec);
        if (!ec) {
          timer_set_flag_ = false;
        }
      }
    }

    bool generate_handshake_(const std::shared_ptr<asio::streambuf>
                             &write_buffer) {
      std::ostream handshake(write_buffer.get());

      auto header_it = header.find("Sec-WebSocket-Key");
      if (header_it == header.end()) {
        return false;
      }

      static auto ws_magic_string = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
      auto sha1 = Crypto::sha1(header_it->second + ws_magic_string);

      handshake << "HTTP/1.1 101 Web Socket Protocol Handshake\r\n";
      handshake << "Upgrade: websocket\r\n";
      handshake << "Connection: Upgrade\r\n";
      handshake << "Sec-WebSocket-Accept: "
                << Crypto::Base64::encode(sha1) << "\r\n";
      handshake << "\r\n";

      return true;
    }

    asio::io_service::strand strand_;

    /****************
       SocketServerBase::Connection::SendData
    ****************/
    class SendData {
     public:
      SendData(std::shared_ptr<SendStream> header_stream,
               std::shared_ptr<SendStream> message_stream,
               std::function<void(const error_code)> &&callback) noexcept
          : header_stream(std::move(header_stream))
          , message_stream(std::move(message_stream))
          , callback(std::move(callback)) {
      }
      std::shared_ptr<SendStream> header_stream;
      std::shared_ptr<SendStream> message_stream;
      std::function<void(const error_code)> callback;
    };  // SendData

    std::list<SendData> send_queue_;

    void send_from_queue_() {
      auto self = this->shared_from_this();
      strand_.post(
          [self]() {
            self->set_send_timeout_(2);
            asio::async_write(
                *self->socket_,
                self->send_queue_.begin()->header_stream->streambuf_,
                self->strand_.wrap(
                    [self]
                    (const error_code &ec,
                     std::size_t /*bytes_transferred*/) {
                      auto lock = self->handler_runner_->continue_lock();
                      if (!lock) {
                        // cancel_send_timeout_();
                        // shall we call cancel_send_timeout_() //
                        // before return? i think not, queue may still increase
                        return;
                      }
                      if (!ec) {
                        asio::async_write(
                            *self->socket_,
                            self->send_queue_.begin()->message_stream->streambuf_.data(),
                            self->strand_.wrap(
                                [self]
                                (const error_code &ec,
                                 std::size_t /*bytes_transferred*/) {
                                  auto lock = self->handler_runner_->continue_lock();
                                  if (!lock) {
                                    return;
                                  }
                                  auto send_queued = self->send_queue_.begin();
                                  if (send_queued->callback) {
                                    send_queued->callback(ec);
                                  }
                                  if (!ec) {
                                    self->send_queue_.erase(send_queued);
                                    if (self->send_queue_.size() > 0) {
                                      self->send_from_queue_();
                                    }
                                  } else {
                                    self->send_queue_.clear();
                                  }
                                  self->cancel_send_timeout_();
                                }));
                      } else {
                        auto send_queued = self->send_queue_.begin();
                        if (send_queued->callback) {
                          send_queued->callback(ec);
                        }
                        self->send_queue_.clear();
                      }
                    }));
          });
    }  // send_from_queue_

    std::atomic<bool> closed_;
    bool is_sp_;  // true means not real websocket

    void read_remote_endpoint_() noexcept {
      try {
        remote_endpoint = socket_->lowest_layer().remote_endpoint();
      } catch (...) {
        inno_log_error("read_remote_endpoint_ error, quit current connection!");
        return;
      }
    }

   public:
    /// fin_rsv_opcode: 129=one fragment, text, 130=one fragment, binary, 136=close connection.
    /// See http://tools.ietf.org/html/rfc6455#section-5.2 for more information
    bool is_sp_conn() const {
      return is_sp_;
    }

    ssize_t write_to_buffer(const char *buf, ssize_t input_len) {
      write_buffer_stream_->write(buf, input_len);
      return is_bad_() ? -1: input_len;
    }

    int flush_buffer() {
      send(write_buffer_stream_, nullptr, 130);
      write_buffer_stream_ = std::make_shared<SendStream>();
      return is_bad_() ? -1 : 0;
    }

    void send(const std::shared_ptr<SendStream> &send_stream,
              const std::function<void(const error_code &)> &callback = nullptr,
              unsigned char fin_rsv_opcode = 129) {
      cancel_timeout_();
      set_timeout_();

      auto header_stream = std::make_shared<SendStream>();

      std::size_t length = send_stream->size();

      if (is_sp_) {
        // no header_stream for sp connection
      } else {
        header_stream->put(static_cast<char>(fin_rsv_opcode));
        // Unmasked (first length byte<128)
        if (length >= 126) {
          std::size_t num_bytes;
          if (length > 0xffff) {
            num_bytes = 8;
            header_stream->put(127);
          } else {
            num_bytes = 2;
            header_stream->put(126);
          }

          for (std::size_t c = num_bytes - 1;
               c != static_cast<std::size_t>(-1);
               c--) {
            header_stream->put((static_cast<uint64_t>(length) >> (8 * c)) % 256);
          }
        } else {
          header_stream->put(static_cast<char>(length));
        }
      }

      auto self = this->shared_from_this();
      strand_.post(
          [self, header_stream, send_stream, callback]
          () {
            self->send_queue_.emplace_back(header_stream, send_stream, callback);
            if (self->send_queue_.size() == 1) {
              self->send_from_queue_();
            }
          });
    }  // send

    void send_close(int status,
                    const std::string &reason = "",
                    const std::function<void(const error_code &)> &callback = nullptr) {
      // Send close only once (in case close is initiated by server)
      if (closed_) {
        return;
      }
      closed_ = true;

      auto send_stream = std::make_shared<SendStream>();

      send_stream->put(status >> 8);
      send_stream->put(status % 256);

      *send_stream << reason;

      // fin_rsv_opcode=136: message close
      send(send_stream, callback, 136);
    }
  };  // Connection

  /****************
      SocketServerBase::Endpoint
  ****************/
  class Endpoint {
    friend class SocketServerBase<socket_type>;
   public:
    Endpoint()
        : is_sp_(false)
        , connections_mutex_() {
    }

    ~Endpoint() {
    }

   private:
    std::unordered_set<std::shared_ptr<Connection>> connections_;
    bool is_sp_;  // true means not real websocket
    std::mutex connections_mutex_;

   public:
    std::function<void(std::shared_ptr<Connection>, const std::string &path)> on_open;
    std::function<void(std::shared_ptr<Connection>, std::shared_ptr<Message>)> on_message;
    std::function<void(std::shared_ptr<Connection>, int, const std::string &)> on_close;
    // std::function<void(std::shared_ptr<Connection>, const error_code &)> on_error;
    std::function<void(std::shared_ptr<Connection>, const char*)> on_error;
    std::function<void(std::shared_ptr<Connection>, int timeout, const char*)> on_send_timeout;
    std::function<void(std::shared_ptr<Connection>)> on_ping;
    std::function<void(std::shared_ptr<Connection>)> on_pong;

    std::unordered_set<std::shared_ptr<Connection>> get_connections() noexcept {
      std::unique_lock<std::mutex> lock(connections_mutex_);
      auto copy = connections_;
      return copy;
    }

    void set_sp(bool sp) {
      is_sp_ = sp;
    }

    bool is_sp_conn() const {
      return is_sp_;
    }
  };  // Endpoint

  /****************
      SocketServerBase::Config
  ****************/
  class Config {
    friend class SocketServerBase<socket_type>;

   private:
    explicit Config(uint16_t port) noexcept
        : port(port) {
    }

   public:
    //  Port number to use. Defaults to 80 for HTTP
    //   and 443 for HTTPS. Set to 0 get an assigned port.
    uint16_t port;
    /// If io_service is not set, number of threads
    //  that the server will use when start() is called.
    //  Defaults to 1 thread.
    std::size_t thread_pool_size = 1;
    /// Timeout on request handling. Defaults to 5 seconds.
    int64_t timeout_request = 5;
    /// Idle timeout. Defaults to no timeout.
    int64_t timeout_idle = 0;
    /// Maximum size of incoming messages. Defaults to architecture maximum.
    /// Exceeding this limit will result in a message_size
    // error code and the connection will be closed.
    std::size_t max_message_size = std::numeric_limits<std::size_t>::max();
    /// IPv4 address in dotted decimal form or IPv6
    // address in hexadecimal notation.
    /// If empty, the address will be any address.
    std::string address;
    /// Set to false to avoid binding the socket to
    // an address that is already in use. Defaults to true.
    bool reuse_address = true;
  };  // Config
  /// Set before calling start().
  Config config;

 private:
  /****************
      SocketServerBase::Config::regex_orderable
  ****************/
  class regex_orderable : public regex::regex {
   private:
    std::string str_;

   public:
    regex_orderable(const char *regex_cstr)  // NOLINT
        : regex::regex(regex_cstr)
        , str_(regex_cstr) {
    }

    regex_orderable(const std::string &regex_str) // NOLINT
        : regex::regex(regex_str)
        , str_(regex_str) {
    }

    bool operator<(const regex_orderable &rhs) const noexcept {
      return str_ < rhs.str_;
    }

    const std::string &get_str() const {
      return str_;
    }
  };  // regex_orderable

 public:
  /// Warning: do not add or remove endpoints after start() is called
  std::map<regex_orderable, Endpoint> endpoint;

  /// If you know the server port in advance, use start() instead.
  /// Returns assigned port. If io_service is not set, an internal io_service is created instead.
  /// Call before accept_and_run().
  uint16_t bind() {
    asio::ip::tcp::endpoint endpoint;
    if (config.address.size() > 0) {
      endpoint = asio::ip::tcp::endpoint(
          asio::ip::address::from_string(config.address), config.port);
    } else {
      endpoint = asio::ip::tcp::endpoint(asio::ip::tcp::v4(), config.port);
    }
    if (!io_service) {
      io_service = std::make_shared<asio::io_service>();
      internal_io_service_ = true;
    }

    if (!acceptor_) {
      acceptor_ = std::unique_ptr<asio::ip::tcp::acceptor>
                  (new asio::ip::tcp::acceptor(*io_service));
    }
    acceptor_->open(endpoint.protocol());
    acceptor_->set_option(asio::socket_base::reuse_address(config.reuse_address));
    acceptor_->bind(endpoint);

    after_bind_();

    return acceptor_->local_endpoint().port();
  }

  /// If you know the server port in advance, use start() instead.
  /// Accept requests, and if io_service was not set
  /// before calling bind(), run the internal io_service instead.
  /// Call after bind().
  void accept_and_run() {
    acceptor_->listen();
    accept_();

    if (internal_io_service_) {
      if (io_service->stopped()) {
        io_service->reset();
      }

      // If thread_pool_size>1, start m_io_service.run()
      // in (thread_pool_size-1) threads for thread-pooling
      threads_.clear();
      for (std::size_t c = 1; c < config.thread_pool_size; c++) {
        threads_.emplace_back(
            [this]() {
              this->io_service->run();
            });
      }

      // Main thread
      if (config.thread_pool_size > 0) {
        io_service->run();
      }

      // Wait for the rest of the threads, if any, to finish as well
      for (auto &t : threads_) {
        t.join();
      }
    }
  }

  /// Start the server by calling bind() and accept_and_run()
  void start() {
    bind();
    accept_and_run();
  }

  /// Stop accepting new connections, and close current connections
  void stop() noexcept {
    while (!acceptor_) {
      sleep(0.01);
    }
    if (acceptor_) {
      error_code ec;
      acceptor_->close(ec);

      for (auto &pair : endpoint) {
        std::unique_lock<std::mutex> lock(pair.second.connections_mutex_);
        for (auto &connection : pair.second.connections_) {
          connection->close();
        }
        pair.second.connections_.clear();
      }

      if (internal_io_service_) {
        io_service->stop();
      }
    }
  }

  /// Stop accepting new connections
  void stop_accept() noexcept {
    if (acceptor_) {
      error_code ec;
      acceptor_->close(ec);
    }
  }

  virtual ~SocketServerBase() noexcept {
  }

  std::unordered_set<std::shared_ptr<Connection>> get_connections() noexcept {
    std::unordered_set<std::shared_ptr<Connection>> all_connections;
    for (auto &e : endpoint) {
      std::unique_lock<std::mutex> lock(e.second.connections_mutex_);
      all_connections.insert(e.second.connections_.begin(), e.second.connections_.end());
    }
    return all_connections;
  }

  /**
   * Upgrades a request, from for instance Simple-Web-Server, to a WebSocket connection.
   * The parameters are moved to the Connection object.
   * See also Server::on_upgrade in the Simple-Web-Server project.
   * The socket's io_service is used, thus running start() is not needed.
   *
   * Example use:
   * server.on_upgrade=[&socket_server] (auto socket, auto request) {
   *   auto connection=std::make_shared<SimpleWeb::SocketServer<SimpleWeb::WS>::Connection>(std::move(socket));
   *   connection->method=std::move(request->method);
   *   connection->path=std::move(request->path);
   *   connection->query_string=std::move(request->query_string);
   *   connection->http_version=std::move(request->http_version);
   *   connection->header=std::move(request->header);
   *   connection->remote_endpoint=std::move(*request->remote_endpoint);
   *   socket_server.upgrade(connection);
   * }
   */
  void upgrade(const std::shared_ptr<Connection> &connection) {
    connection->handler_runner_ = handler_runner_;
    connection->timeout_idle_ = config.timeout_idle;
    write_handshake_(connection);
  }

  /// If you have your own asio::io_service, store its pointer here before running start().
  std::shared_ptr<asio::io_service> io_service;

 protected:
  bool internal_io_service_ = false;
  std::unique_ptr<asio::ip::tcp::acceptor> acceptor_;
  std::vector<std::thread> threads_;
  std::shared_ptr<ScopeRunner> handler_runner_;

  explicit SocketServerBase(uint16_t port) noexcept
      : config(port)
      , handler_runner_(new ScopeRunner()) {
    //  BOOST_VERSION % 100 is the patch level
    //  BOOST_VERSION / 100 % 1000 is the minor version
    //  BOOST_VERSION / 100000 is the major version
    int patch_lv = BOOST_VERSION % 100;
    int minor_version = BOOST_VERSION / 100 % 1000;
    int major_version = BOOST_VERSION / 100000;
    std::string version;
    version += std::to_string(major_version);
    version += ".";
    version += std::to_string(minor_version);
    version += ".";
    version += std::to_string(patch_lv);
    inno_log_info("current boost version: %s", version.c_str());
  }

  virtual void after_bind_() {
  }
  virtual void accept_() = 0;

  void read_handshake2_(const std::shared_ptr<Connection> &connection) {
    connection->set_timeout_(config.timeout_request);
    asio::async_read_until(
        *connection->socket_,
        connection->read_buffer_,
        "\r\n\r\n",
        [this, connection]
        (const error_code &ec,
         std::size_t /*bytes_transferred*/) {
          connection->cancel_timeout_();
          auto lock = connection->handler_runner_->continue_lock();
          if (!lock) {
            return;
          }
          if (!ec) {
            // std::vector<char> target(connection->read_buffer_.size());
            // buffer_copy(boost::asio::buffer(target), connection->read_buffer_.data());
            // std::string s(target.begin(), target.end());
            // fprintf(stderr, "######################################## %s\n", s.c_str());

            std::istream stream(&connection->read_buffer_);
            if (RequestMessage::parse(stream, connection->method,
                                      connection->path,
                                      connection->query_string,
                                      connection->http_version,
                                      connection->url, connection->header))
              write_handshake_(connection);
          }
        });
  }

  void read_handshake_(const std::shared_ptr<Connection> &connection) {
    connection->read_remote_endpoint_();

    connection->set_timeout_(config.timeout_request);
    asio::async_read_until(
        *connection->socket_,
        connection->read_buffer_,
        "\r\n\r\n",
        [this, connection]
        (const error_code &ec, std::size_t /*bytes_transferred*/) {
          connection->cancel_timeout_();
          auto lock = connection->handler_runner_->continue_lock();
          if (!lock) {
            return;
          }
          if (!ec) {
            std::vector<char> target(connection->read_buffer_.size());
            buffer_copy(boost::asio::buffer(target), connection->read_buffer_.data());
            std::string s(target.begin(), target.end());
            connection->recv_buffer = s;
            if (s.find("GET ") == 0) {
              read_handshake2_(connection);
            } else {
              // remove the first line
              std::istream stream(&connection->read_buffer_);
              std::string line, path_str, query_str;
              getline(stream, line);
              // find the last "/" or first "?"
              size_t idx_slash = line.rfind("/");
              size_t idx_q = line.find("?");
              if (idx_slash != std::string::npos) {
                path_str = line.substr(0, idx_slash);
              } else {
                if (idx_q != std::string::npos) {
                  path_str = line.substr(0, idx_q);
                } else {
                  path_str = line;
                }
              }
              if (idx_q != std::string::npos) {
                query_str = line.substr(idx_q + 1);
              }
              auto lock = connection->handler_runner_->continue_lock();
              if (!lock) {
                return;
              }
              for (auto &regex_endpoint : endpoint) {
                regex::smatch path_match;
                // std::cout << path_str << " "
                //           << regex_endpoint.second.is_sp
                //           << regex_endpoint.first.get_str() << std::endl;
                if (regex_endpoint.second.is_sp_ &&
                    regex::regex_match(path_str,
                                       path_match,
                                       regex_endpoint.first)) {
                  connection->path = path_str;
                  connection->path_match = std::move(path_match);
                  connection->is_sp_ = true;
                  // if (s.find("start/?") == 0) {
                  // finished handshake, ready to start streaming
                  connection->query_string = query_str;
                  connection->is_sp_ = true;
                  connection_open_(connection, &regex_endpoint.second,
                                   regex_endpoint.first.get_str());
                  read_message_(connection, &regex_endpoint.second);
                  return;
                }
              }
              // connection_close_(connection, regex_endpoint.second, 1002, "response to stop");
            }
          }
        });
  }

  void write_handshake_(const std::shared_ptr<Connection> &connection) {
    for (auto &regex_endpoint : endpoint) {
      regex::smatch path_match;
      // std::cout << connection->path << "  " << connection->query_string << " "
      //           << regex_endpoint.first.get_str() << std::endl;
      if (regex::regex_match(connection->path,
                             path_match,
                             regex_endpoint.first)) {
        auto write_buffer = std::make_shared<asio::streambuf>();
        if (regex_endpoint.second.is_sp_) {
          auto lock = connection->handler_runner_->continue_lock();
          if (!lock) {
            return;
          }
          connection->path_match = std::move(path_match);
          connection->is_sp_ = true;
          connection_open_(connection, &regex_endpoint.second,
                           regex_endpoint.first.get_str());
          read_message_(connection, &regex_endpoint.second);
        } else if (connection->generate_handshake_(write_buffer)) {
          connection->path_match = std::move(path_match);
          connection->set_timeout_(config.timeout_request);
          asio::async_write(
              *connection->socket_,
              *write_buffer,
              [this, connection, write_buffer, &regex_endpoint]
              (const error_code &ec, std::size_t /*bytes_transferred*/) {
                connection->cancel_timeout_();
                const char* err_msg = ec.message().c_str();
                auto lock = connection->handler_runner_->continue_lock();
                if (!lock) {
                  return;
                }
                if (!ec) {
                  connection_open_(connection, &regex_endpoint.second,
                                   regex_endpoint.first.get_str());
                  read_message_(connection, &regex_endpoint.second);
                } else {
                  connection_error_(connection, &regex_endpoint.second,
                                    err_msg);
                }
              });
        }
        return;
      } else {
        // std::cout << "not matched" << std::endl;
      }
    }
  }

  void read_message_(const std::shared_ptr<Connection> &connection,
                     Endpoint *endpoint) const {
    asio::async_read(
        *connection->socket_,
        connection->read_buffer_,
        asio::transfer_exactly(2),
        [this, connection, endpoint]
        (const error_code &ec,
         std::size_t bytes_transferred) {
          auto lock = connection->handler_runner_->continue_lock();
          if (!lock) {
            return;
          }
          if (!ec) {
            if (bytes_transferred == 0) {
              // xxx todo: why does this happen sometimes?
              read_message_(connection, endpoint);
              return;
            }
            std::istream stream(&connection->read_buffer_);

            std::array<unsigned char, 2> first_bytes;
            stream.read(reinterpret_cast<char *>(&first_bytes[0]), 2);

            unsigned char fin_rsv_opcode = first_bytes[0];
            // Close connection if unmasked message from client (protocol error)
            if (first_bytes[1] < 128) {
              // fprintf(stderr, "first bytes %c\n", first_bytes[1]);
              const std::string reason("message from client not masked");
              connection->send_close(1002, reason);
              connection_close_(connection, endpoint, 1002, reason);
              return;
            }

            std::size_t length = (first_bytes[1] & 127);

            if (length == 126) {
              // 2 next bytes is the size of content
              asio::async_read(
                  *connection->socket_,
                  connection->read_buffer_,
                  asio::transfer_exactly(2),
                  [this, connection, endpoint, fin_rsv_opcode]
                  (const error_code &ec,
                   std::size_t /*bytes_transferred*/) {
                    auto lock = connection->handler_runner_->continue_lock();
                    if (!lock) {
                      return;
                    }
                    if (!ec) {
                      std::istream stream(&connection->read_buffer_);

                      std::array<unsigned char, 2> length_bytes;
                      stream.read(reinterpret_cast<char *>(&length_bytes[0]),
                                  2);

                      std::size_t length = 0;
                      std::size_t num_bytes = 2;
                      for (std::size_t c = 0; c < num_bytes; c++) {
                        length += static_cast<std::size_t>
                                  (length_bytes[c]) << (8 * (num_bytes - 1 - c));
                      }

                      read_message_content_(connection, length,
                                            endpoint, fin_rsv_opcode);
                    } else {
                      const char* err_msg = ec.message().c_str();
                      connection_error_(connection, endpoint, err_msg);
                    }
                  });
            } else if (length == 127) {
              // 8 next bytes is the size of content
              asio::async_read(
                  *connection->socket_,
                  connection->read_buffer_,
                  asio::transfer_exactly(8),
                  [this, connection, endpoint, fin_rsv_opcode]
                  (const error_code &ec,
                   std::size_t /*bytes_transferred*/) {
                    auto lock = connection->handler_runner_->continue_lock();
                    if (!lock) {
                      return;
                    }
                    if (!ec) {
                      std::istream stream(&connection->read_buffer_);

                      std::array<unsigned char, 8> length_bytes;
                      stream.read(reinterpret_cast<char *>(&length_bytes[0]),
                                  8);

                      std::size_t length = 0;
                      std::size_t num_bytes = 8;
                      for (std::size_t c = 0; c < num_bytes; c++) {
                        length += static_cast<std::size_t>
                                  (length_bytes[c]) << (8 * (num_bytes - 1 - c));
                      }

                      read_message_content_(connection, length,
                                            endpoint, fin_rsv_opcode);
                    } else {
                      const char* err_msg = ec.message().c_str();
                      connection_error_(connection, endpoint, err_msg);
                    }
                  });
            } else {
              read_message_content_(connection, length,
                                    endpoint, fin_rsv_opcode);
            }
          } else if (ec == boost::asio::error::operation_aborted) {
            // do nothing for cancel operation
          } else {
            const char* err_msg = ec.message().c_str();
            connection_error_(connection, endpoint, err_msg);
          }
        });
  }

  void read_message_content_(const std::shared_ptr<Connection> &connection,
                             std::size_t length,
                             Endpoint *endpoint,
                             unsigned char fin_rsv_opcode) const {
    if (length + (connection->fragmented_message_
                  ? connection->fragmented_message_->size()
                  : 0)
        > config.max_message_size) {
      connection_error_(connection, endpoint, "EMSGSIZE");
      const int status = 1009;
      const std::string reason = "message too big";
      connection->send_close(status, reason);
      connection_close_(connection, endpoint, status, reason);
      return;
    }
    asio::async_read(
        *connection->socket_,
        connection->read_buffer_,
        asio::transfer_exactly(4 + length),
        [this, connection, length, endpoint, fin_rsv_opcode]
        (const error_code &ec,
         std::size_t /*bytes_transferred*/) {
          auto lock = connection->handler_runner_->continue_lock();
          if (!lock) {
            return;
          }
          if (!ec) {
            std::istream istream(&connection->read_buffer_);

            // Read mask
            std::array<unsigned char, 4> mask;
            istream.read(reinterpret_cast<char *>(&mask[0]), 4);

            std::shared_ptr<Message> message;

            // If fragmented message
            if ((fin_rsv_opcode & 0x80) == 0 || (fin_rsv_opcode & 0x0f) == 0) {
              if (!connection->fragmented_message_) {
                connection->fragmented_message_ =
                    std::shared_ptr<Message>(new Message(fin_rsv_opcode,
                                                         length));
                connection->fragmented_message_->fin_rsv_opcode |= 0x80;
              } else {
                connection->fragmented_message_->length_ += length;
              }
              message = connection->fragmented_message_;
            } else {
              message = std::shared_ptr<Message>(
                  new Message(fin_rsv_opcode, length));
            }
            std::ostream ostream(&message->streambuf_);
            for (std::size_t c = 0; c < length; c++) {
              ostream.put(istream.get() ^ mask[c % 4]);
            }
            // If connection close
            if ((fin_rsv_opcode & 0x0f) == 8) {
              connection->cancel_timeout_();
              connection->set_timeout_();

              int status = 0;
              if (length >= 2) {
                unsigned char byte1 = message->get();
                unsigned char byte2 = message->get();
                status = (static_cast<int>(byte1) << 8) + byte2;
              }

              auto reason = message->string();
              connection->send_close(status, reason);
              this->connection_close_(connection, endpoint, status, reason);
            } else if ((fin_rsv_opcode & 0x0f) == 9) {
              // If ping
              connection->cancel_timeout_();
              connection->set_timeout_();

              // Send pong
              auto empty_send_stream = std::make_shared<SendStream>();
              connection->send(empty_send_stream, nullptr, fin_rsv_opcode + 1);

              if (endpoint->on_ping) {
                endpoint->on_ping(connection);
              }

              // Next message
              this->read_message_(connection, endpoint);
            } else if ((fin_rsv_opcode & 0x0f) == 10) {
              // If pong
              connection->cancel_timeout_();
              connection->set_timeout_();

              if (endpoint->on_pong) {
                endpoint->on_pong(connection);
              }

              // Next message
              this->read_message_(connection, endpoint);
            } else if ((fin_rsv_opcode & 0x80) == 0) {
              // If fragmented message and not final fragment
              // Next message
              this->read_message_(connection, endpoint);
            } else {
              connection->cancel_timeout_();
              connection->set_timeout_();

              if (endpoint->on_message) {
                endpoint->on_message(connection, message);
              }

              // Next message
              // Only reset fragmented_message_ for
              // non-control frames (control frames can
              // be in between a fragmented message)
              connection->fragmented_message_ = nullptr;
              this->read_message_(connection, endpoint);
            }
          } else {
            const char* err_msg = ec.message().c_str();
            this->connection_error_(connection, endpoint, err_msg);
          }
        });
  }

  void connection_open_(const std::shared_ptr<Connection> &connection,
                        Endpoint *endpoint,
                        const std::string &path) const {
    connection->cancel_timeout_();
    connection->set_timeout_();

    {
      std::unique_lock<std::mutex> lock(endpoint->connections_mutex_);
      endpoint->connections_.insert(connection);
    }

    if (endpoint->on_open) {
      endpoint->on_open(connection, path);
    }
  }

  void connection_close_(const std::shared_ptr<Connection> &connection,
                         Endpoint *endpoint, int status,
                         const std::string &reason) const {
    connection->cancel_timeout_();
    connection->set_timeout_();

    {
      std::unique_lock<std::mutex> lock(endpoint->connections_mutex_);
      endpoint->connections_.erase(connection);
    }
    connection->set_bad_();
    if (endpoint->on_close) {
      endpoint->on_close(connection, status, reason);
    }
  }

  void connection_error_(const std::shared_ptr<Connection> &connection,
                         Endpoint *endpoint,
                         const char* ec) const {
    connection->cancel_timeout_();
    connection->set_timeout_();

    {
      std::unique_lock<std::mutex> lock(endpoint->connections_mutex_);
      endpoint->connections_.erase(connection);
    }
    connection->set_bad_();
    if (endpoint->on_error) {
      endpoint->on_error(connection, ec);
    }
  }

  void connection_timeout_(const std::shared_ptr<Connection> &connection,
                           int timeout,
                           Endpoint *endpoint,
                           const char* ec_msg) const {
    connection->socket_->cancel();
    connection->cancel_send_timeout_();
    connection->cancel_timeout_();
    {
      std::unique_lock<std::mutex> lock(endpoint->connections_mutex_);
      endpoint->connections_.erase(connection);
    }
    connection->set_bad_();
    if (endpoint->on_send_timeout) {
      endpoint->on_send_timeout(connection, timeout, ec_msg);
    }
  }
};  // SocketServerBase<>

/****************
   SocketServer<WS>
****************/
template <class socket_type>
class SocketServer : public SocketServerBase<socket_type> {};

/****************
   SocketServer<WS> Template specialization
****************/
template <>
class SocketServer<WS> : public SocketServerBase<WS>
                       , public std::enable_shared_from_this<SocketServer<WS>> {
 public:
  explicit SocketServer(void *ctx) noexcept
      : SocketServerBase<WS>(80)
      , ctx_(ctx) {
  }

  ~SocketServer() {
  }

 protected:
  void accept_() override {
    std::weak_ptr<SocketServerBase<WS>> websock_server(
        this->shared_from_this());
    if (auto server = websock_server.lock()) {
      std::shared_ptr<Connection> connection(
          new Connection(ctx_, handler_runner_,
                         config.timeout_idle,
                         websock_server, *io_service));
      acceptor_->async_accept(
          *connection->socket_,
          [this, connection]
          (const error_code &ec) {
            auto lock = connection->handler_runner_->continue_lock();
            if (!lock) {
              return;
            }
            // Immediately start accepting a new connection
            // (if io_service hasn't been stopped)
            if (ec != asio::error::operation_aborted) {
              accept_();
            }

            if (!ec) {
              asio::ip::tcp::no_delay option(true);
              connection->socket_->set_option(option);
              read_handshake_(connection);
            }
          });
    }
  }

 private:
  void *ctx_;
};  // SocketServer <WS>
}  // namespace SimpleWeb

#endif  // WS_UTILS_SERVER_WS_SERVER_WS_HPP_

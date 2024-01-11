#include "client.h"

#include <chrono>
#include <iostream>

#include <websocketpp/close.hpp>
#include <websocketpp/common/connection_hdl.hpp>

WebSocketClient::WebSocketClient()
    : thread_ptr_(nullptr),
      conn_ptr_(nullptr) {
  endpoint_.clear_access_channels(websocketpp::log::alevel::all);
  endpoint_.clear_error_channels(websocketpp::log::elevel::all);
  endpoint_.init_asio();
  endpoint_.start_perpetual();
  thread_ptr_.reset(new websocketpp::lib::thread(&WSClient::run, &endpoint_));
}

WebSocketClient::~WebSocketClient() {
  endpoint_.stop_perpetual();

  if (conn_ptr_ != nullptr) {
    if (conn_ptr_->get_state() == websocketpp::session::state::open ||
        conn_ptr_->get_state() == websocketpp::session::state::connecting) {
      websocketpp::lib::error_code ec;
      endpoint_.close(conn_ptr_->get_handle(), websocketpp::close::status::going_away, "", ec);
      if (ec)
        std::cerr << "Failed to close websocket: " << ec.message() << '\n';
    }
  }

  if (thread_ptr_ != nullptr) {
    thread_ptr_->join();
    thread_ptr_.reset();
    thread_ptr_ = nullptr;
  }
}

bool WebSocketClient::isConnected() {
  return (conn_ptr_ == nullptr) ? false : conn_ptr_->get_state() == websocketpp::session::state::connecting;
}

bool WebSocketClient::connect(const std::string& url) {
  websocketpp::lib::error_code ec;
  WSClient::connection_ptr ptr = endpoint_.get_connection(url, ec);
  if (ec) {
    std::cerr << "Failed to connect to websocket server: " << ec.message() << '\n';
    return false;
  }

  conn_ptr_ = ptr;

  ptr->set_socket_init_handler(websocketpp::lib::bind(&WebSocketClient::onSocketInit, this, websocketpp::lib::placeholders::_1));
  ptr->set_open_handler(websocketpp::lib::bind(&WebSocketClient::onOpen, this, websocketpp::lib::placeholders::_1));
  ptr->set_fail_handler(websocketpp::lib::bind(&WebSocketClient::onFail, this, websocketpp::lib::placeholders::_1));
  ptr->set_close_handler(websocketpp::lib::bind(&WebSocketClient::onClose, this, websocketpp::lib::placeholders::_1));
  ptr->set_message_handler(websocketpp::lib::bind(&WebSocketClient::onMessage, this, websocketpp::lib::placeholders::_1, websocketpp::lib::placeholders::_2));

  endpoint_.connect(ptr);
  // Wait for 1 seconds is necessary, otherwise the first send will fail.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return true;
}

bool WebSocketClient::close(const std::string& reason) {
  if (conn_ptr_ == nullptr)
    return false;

  websocketpp::lib::error_code ec;
  endpoint_.close(conn_ptr_->get_handle(), websocketpp::close::status::normal, reason, ec);
  if (ec) {
    std::cerr << "Failed to close websocket: " << ec.message() << '\n';
    return false;
  }

  return true;
}

bool WebSocketClient::sendTextData(const std::string& msg) {
  if (conn_ptr_ == nullptr)
    return false;

  if (conn_ptr_->get_state() == websocketpp::session::state::open) {
    websocketpp::lib::error_code ec;
    endpoint_.send(conn_ptr_->get_handle(), msg, websocketpp::frame::opcode::text, ec);
    if (ec) {
      std::cerr << "Failed to send text message to websocket server: " << ec.message() << '\n';
      return false;
    }
  } else {
    std::cerr << "Failed to send text message to websocket server as the session is not open\n";
    return false;
  }

  return true;
}

bool WebSocketClient::sendBinaryData(void *data, uint64_t len) {
  if (conn_ptr_ == nullptr)
    return false;

  if (conn_ptr_->get_state() == websocketpp::session::state::open) {
    websocketpp::lib::error_code ec;
    endpoint_.send(conn_ptr_->get_handle(), data, len, websocketpp::frame::opcode::binary, ec);
    if (ec) {
      std::cerr << "Failed to send binary message to websocket server: " << ec.message() << '\n';
      return false;
    }
  } else {
    std::cerr << "Failed to send binary message to websocket server as the session is not open\n";
    return false;
  }

  return true;
}

bool WebSocketClient::sendTextDataTillSuccess(const std::string& msg) {
  bool state = false;

  do {
    state = sendTextData(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } while (!state);

  return state;
}

bool WebSocketClient::sendBinaryDataTillSuccess(void *data, uint64_t len) {
  bool state = false;

  do {
    state = sendBinaryData(data, len);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  } while (!state);

  return state;
}

bool WebSocketClient::ping() {
  if (conn_ptr_ != nullptr) {
    websocketpp::lib::error_code ec;
    endpoint_.ping(conn_ptr_->get_handle(), "ping", ec);
    if (ec) {
      std::cerr << "Websocket ping failed:" << ec.message() << '\n';
      return false;
    }
  }

  return true;
}

void WebSocketClient::onSocketInit(websocketpp::connection_hdl hdl) {}

void WebSocketClient::onOpen(websocketpp::connection_hdl hdl) {
  std::cout << "Websocket is open\n";
}

void WebSocketClient::onMessage(websocketpp::connection_hdl hdl, WSClient::message_ptr msg) {
  if (custom_msg_handler_ != nullptr)
    custom_msg_handler_(msg->get_payload());
  else
    std::cout << "Message received: " << msg->get_payload() << '\n';
}

void WebSocketClient::onFail(websocketpp::connection_hdl hdl) {
  WSClient::connection_ptr conn = endpoint_.get_con_from_hdl(hdl);
  int local_close_code = conn->get_local_close_code();
  std::string local_close_reason = conn->get_local_close_reason();
  int remote_close_code = conn->get_remote_close_code();
  std::string remote_close_reason = conn->get_remote_close_reason();
  std::string errmsg = conn->get_ec().message();
  int errcode = conn->get_ec().value();

  std::cerr << "error code: " << remote_close_code << " error message: " << errmsg << '\n';
}

void WebSocketClient::onClose(websocketpp::connection_hdl hdl) {
  WSClient::connection_ptr conn = endpoint_.get_con_from_hdl(hdl);
  int close_code = conn->get_remote_close_code();
  std::string close_reason = conn->get_remote_close_reason();

  // std::cerr << "close code: " << close_code << " close message: " << close_reason << '\n';
}

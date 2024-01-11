#ifndef AI_ROBOT_WEBSOCKET_CLIENT_H_
#define AI_ROBOT_WEBSOCKET_CLIENT_H_

#include <cstdint>
#include <functional>
#include <string>

#include <websocketpp/common/connection_hdl.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

using WSClient = websocketpp::client<websocketpp::config::asio_client>;

class WebSocketClient {
public:
  WebSocketClient();
  virtual ~WebSocketClient();

public:
  bool isConnected();
  bool connect(const std::string& url);
  bool close(const std::string& reason = "");
  bool sendTextData(const std::string& msg);
  bool sendBinaryData(void *data, uint64_t len);
  bool sendTextDataTillSuccess(const std::string& msg);
  bool sendBinaryDataTillSuccess(void *data, uint64_t len);
  bool ping();
  void setCustomMessageHandler(std::function<void(const std::string& msg)> func) { custom_msg_handler_ = func; }

private:
  void onSocketInit(websocketpp::connection_hdl hdl);
  void onOpen(websocketpp::connection_hdl hdl);
  void onMessage(websocketpp::connection_hdl hdl, WSClient::message_ptr msg);
  void onFail(websocketpp::connection_hdl hdl);
  void onClose(websocketpp::connection_hdl hdl);

private:
  WSClient endpoint_;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> thread_ptr_;
  WSClient::connection_ptr conn_ptr_;
  std::function<void(const std::string& msg)> custom_msg_handler_;
};

#endif // !AI_ROBOT_WEBSOCKET_CLIENT_H_

#include <iostream>
#include <string>

#include "../websocket/client.h"

void printMessage(const std::string& msg) {
  std::cout << msg << '\n';
}

int main(int argc, char *argv[]) {
  WebSocketClient c;
  std::string url = "ws://139.196.197.176:30110";

  if (argc == 2)
    url = argv[1];

  std::string request =
   "{"
      "\"points\": ["
      "{"
          "\"x\": 13519977.939568073,"
          "\"y\": 3614361.787713769,"
          "\"level\": 1"
      "},"
      "{"
          "\"x\": 13519955.121249475,"
          "\"y\": 3614354.0630268487,"
          "\"level\": 1"
      "}"
    "]"
   "}";

  c.connect(url);
  c.sendTextDataTillSuccess(request);
}

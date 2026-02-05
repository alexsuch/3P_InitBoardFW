// clang-format off
#ifndef PsychicWebSocket_h
#define PsychicWebSocket_h

#include "PsychicCore.h"
#include "PsychicRequest.h"

#if PSYCHIC_WEBSOCKET_ENABLED

class PsychicWebSocketRequest;
class PsychicWebSocketClient;

//callback function definitions
typedef std::function<void(PsychicWebSocketClient *client)> PsychicWebSocketClientCallback;
typedef std::function<esp_err_t(PsychicWebSocketRequest *request, httpd_ws_frame_t *frame)> PsychicWebSocketFrameCallback;

class PsychicWebSocketClient : public PsychicClient
{
  public:
    PsychicWebSocketClient(PsychicClient *client);
    ~PsychicWebSocketClient();
    int send_failures = 0;
    bool is_closing = false;  // Flag to prevent race conditions during cleanup
    int current_msg_priority = 2;  // Default to HIGH priority (most critical)
    esp_err_t sendMessage(httpd_ws_frame_t * ws_pkt);
    esp_err_t sendMessage(httpd_ws_type_t op, const void *data, size_t len);
    esp_err_t sendMessage(const char *buf);
};

class PsychicWebSocketRequest : public PsychicRequest
{
  private:
    PsychicWebSocketClient _client;

  public:
    PsychicWebSocketRequest(PsychicRequest *req);
    virtual ~PsychicWebSocketRequest();

    PsychicWebSocketClient * client() override;

    esp_err_t reply(httpd_ws_frame_t * ws_pkt);
    esp_err_t reply(httpd_ws_type_t op, const void *data, size_t len);
    esp_err_t reply(const char *buf);
};

class PsychicWebSocketHandler : public PsychicHandler {
  protected:
    PsychicWebSocketClientCallback _onOpen;
    PsychicWebSocketFrameCallback _onFrame;
    PsychicWebSocketClientCallback _onClose;
    SemaphoreHandle_t _mutex;
  public:
    PsychicWebSocketHandler();
    ~PsychicWebSocketHandler();

    PsychicWebSocketClient * getClient(int socket) override;
    PsychicWebSocketClient * getClient(PsychicClient *client) override;
    void addClient(PsychicClient *client) override;
    void removeClient(PsychicClient *client) override;
    void openCallback(PsychicClient *client) override;
    void closeCallback(PsychicClient *client) override;

    bool isWebSocket() override final;
    esp_err_t handleRequest(PsychicRequest *request) override;

    PsychicWebSocketHandler *onOpen(PsychicWebSocketClientCallback fn);
    PsychicWebSocketHandler *onFrame(PsychicWebSocketFrameCallback fn);
    PsychicWebSocketHandler *onClose(PsychicWebSocketClientCallback fn);

    void sendAll(httpd_ws_frame_t * ws_pkt);
    void sendAll(httpd_ws_type_t op, const void *data, size_t len);
    void sendAll(const char *buf);
};

#endif // PSYCHIC_WEBSOCKET_ENABLED

#endif // PsychicWebSocket_h
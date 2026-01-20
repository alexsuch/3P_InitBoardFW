// clang-format off
#include <esp_http_server.h>
#include "PsychicWebSocket.h"

#if PSYCHIC_WEBSOCKET_ENABLED



/*************************************/
/*  PsychicWebSocketRequest      */
/*************************************/

PsychicWebSocketRequest::PsychicWebSocketRequest(PsychicRequest *req) :
  PsychicRequest(req->server(), req->request()),
  _client(req->client())
{
}

PsychicWebSocketRequest::~PsychicWebSocketRequest()
{
}

PsychicWebSocketClient * PsychicWebSocketRequest::client() {
  return &_client;
}

esp_err_t PsychicWebSocketRequest::reply(httpd_ws_frame_t * ws_pkt)
{
  return httpd_ws_send_frame(this->_req, ws_pkt);
} 

esp_err_t PsychicWebSocketRequest::reply(httpd_ws_type_t op, const void *data, size_t len)
{
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

  ws_pkt.payload = (uint8_t*)data;
  ws_pkt.len = len;
  ws_pkt.type = op;

  return this->reply(&ws_pkt);
}

esp_err_t PsychicWebSocketRequest::reply(const char *buf)
{
  return this->reply(HTTPD_WS_TYPE_TEXT, buf, strlen(buf));
}

/*************************************/
/*  PsychicWebSocketClient   */
/*************************************/

PsychicWebSocketClient::PsychicWebSocketClient(PsychicClient *client)
  : PsychicClient(client->server(), client->socket())
{
}

PsychicWebSocketClient::~PsychicWebSocketClient() {
}



esp_err_t PsychicWebSocketClient::sendMessage(httpd_ws_frame_t * ws_pkt)
{
    // Don't send to clients that are being closed
    if (is_closing) {
        return ESP_FAIL;
    }

    httpd_ws_client_info_t info = httpd_ws_get_fd_info(this->server(), this->socket());
    if (info != HTTPD_WS_CLIENT_WEBSOCKET) {
        ESP_LOGW(PH_TAG, "Client %d is not a valid WS session", this->socket());
        // The main loop will handle closing via ws_socket_tracker
        return ESP_ERR_INVALID_STATE;
    }

    return httpd_ws_send_frame_async(this->server(), this->socket(), ws_pkt);
}
 

esp_err_t PsychicWebSocketClient::sendMessage(httpd_ws_type_t op, const void *data, size_t len)
{
  // Don't send to clients that are being closed
  if (is_closing) {
    return ESP_FAIL;
  }

  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

  ws_pkt.payload = (uint8_t*)data;
  ws_pkt.len = len;
  ws_pkt.type = op;

  return this->sendMessage(&ws_pkt);
}

esp_err_t PsychicWebSocketClient::sendMessage(const char *buf)
{
  // Don't send to clients that are being closed
  if (is_closing) {
    return ESP_FAIL;
  }

  return this->sendMessage(HTTPD_WS_TYPE_TEXT, buf, strlen(buf));
}

PsychicWebSocketHandler::PsychicWebSocketHandler() :
  PsychicHandler(),
  _onOpen(NULL),
  _onFrame(NULL),
  _onClose(NULL)
  {
    _mutex = xSemaphoreCreateMutex(); 
  }

PsychicWebSocketHandler::~PsychicWebSocketHandler() {
}

PsychicWebSocketClient * PsychicWebSocketHandler::getClient(int socket)
{
  PsychicClient *client = PsychicHandler::getClient(socket);
  if (client == NULL)
    return NULL;

  if (client->_friend == NULL)
  {
    return NULL;
  }

  return (PsychicWebSocketClient *)client->_friend;
}

PsychicWebSocketClient * PsychicWebSocketHandler::getClient(PsychicClient *client) {
  return getClient(client->socket());
}

void PsychicWebSocketHandler::addClient(PsychicClient *client) {  
  client->_friend = new PsychicWebSocketClient(client);
  
  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    PsychicHandler::addClient(client);
    xSemaphoreGive(_mutex);
  }
}

void PsychicWebSocketHandler::removeClient(PsychicClient *client) {

  if (client && client->_friend != NULL) {
    delete (PsychicWebSocketClient*)client->_friend;
    client->_friend = NULL;
  }

  PsychicHandler::removeClient(client);
}

void PsychicWebSocketHandler::openCallback(PsychicClient *client) {
  PsychicWebSocketClient *buddy = getClient(client);
  if (buddy == NULL)
  {
    return;
  }

  if (_onOpen != NULL)
    _onOpen(getClient(buddy));
}

void PsychicWebSocketHandler::closeCallback(PsychicClient *client) {
  ESP_LOGI(PH_TAG, "WebSocket client %d disconnected", client->socket());

  if (client == nullptr) {
    return;
  }

  PsychicWebSocketClient *ws_client = nullptr;
  if (client->_friend != NULL) {
    ws_client = (PsychicWebSocketClient *)client->_friend;
    ws_client->is_closing = true;
  }

  if (_onClose != NULL && ws_client != nullptr) {
    _onClose(ws_client);
  }

  if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
    removeClient(client);
    xSemaphoreGive(_mutex);
  }
}

bool PsychicWebSocketHandler::isWebSocket() { return true; }

esp_err_t PsychicWebSocketHandler::handleRequest(PsychicRequest *request)
{
  //lookup our client
  PsychicClient *client = checkForNewClient(request->client());

  // beginning of the ws URI handler and our onConnect hook
  if (request->method() == HTTP_GET)
  {
    if (client->isNew)
      openCallback(client);

    return ESP_OK;
  }

  //prep our request
  PsychicWebSocketRequest wsRequest(request);

  //init our memory for storing the packet
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  uint8_t *buf = NULL;

  /* Set max_len = 0 to get the frame len */
  esp_err_t ret = httpd_ws_recv_frame(wsRequest.request(), &ws_pkt, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(PH_TAG, "httpd_ws_recv_frame failed to get frame len with %s", esp_err_to_name(ret));
    return ret;
  }

  //okay, now try to load the packet
  //ESP_LOGD(PH_TAG, "frame len is %d", ws_pkt.len);
  if (ws_pkt.len) {
    /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
    buf = (uint8_t*) calloc(1, ws_pkt.len + 1);
    if (buf == NULL) {
      ESP_LOGE(PH_TAG, "Failed to calloc memory for buf");
      return ESP_ERR_NO_MEM;
    }
    ws_pkt.payload = buf;
    /* Set max_len = ws_pkt.len to get the frame payload */
    ret = httpd_ws_recv_frame(wsRequest.request(), &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
      ESP_LOGE(PH_TAG, "httpd_ws_recv_frame failed with %s", esp_err_to_name(ret));
      free(buf);
      return ret;
    }
    //ESP_LOGD(PH_TAG, "Got packet with message: %s", ws_pkt.payload);
  }

  // Text messages are our payload.
  if (ws_pkt.type == HTTPD_WS_TYPE_TEXT || ws_pkt.type == HTTPD_WS_TYPE_BINARY)
  {
    if (this->_onFrame != NULL)
      ret = this->_onFrame(&wsRequest, &ws_pkt);
  }

  //logging housekeeping
  if (ret != ESP_OK)
    ESP_LOGE(PH_TAG, "httpd_ws_send_frame failed with %s", esp_err_to_name(ret));
    // ESP_LOGD(PH_TAG, "ws_handler: httpd_handle_t=%p, sockfd=%d, client_info:%d", 
    //   request->server(),
    //   httpd_req_to_sockfd(request->request()),
    //   httpd_ws_get_fd_info(request->server()->server, httpd_req_to_sockfd(request->request())));

  //dont forget to release our buffer memory
  free(buf);

  return ret;
}

PsychicWebSocketHandler * PsychicWebSocketHandler::onOpen(PsychicWebSocketClientCallback fn) {
  _onOpen = fn;
  return this;
}

PsychicWebSocketHandler * PsychicWebSocketHandler::onFrame(PsychicWebSocketFrameCallback fn) {
  _onFrame = fn;
  return this;
}

PsychicWebSocketHandler * PsychicWebSocketHandler::onClose(PsychicWebSocketClientCallback fn) {
  _onClose = fn;
  return this;
}


void PsychicWebSocketHandler::sendAll(httpd_ws_frame_t * ws_pkt)
{
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        for (auto it = _clients.begin(); it != _clients.end(); ++it)
        {
            PsychicClient *client = *it;
            if (client) {
                PsychicWebSocketClient *wsClient = (PsychicWebSocketClient *)client->_friend;
                if (wsClient && !wsClient->is_closing) {
                    // The result is not checked here. The caller is responsible for broadcast statistics.
                    // The ws_socket_tracker will handle dead clients.
                    wsClient->sendMessage(ws_pkt);
                }
            }
        }
        xSemaphoreGive(_mutex);
    }
}

void PsychicWebSocketHandler::sendAll(httpd_ws_type_t op, const void *data, size_t len)
{
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

  ws_pkt.payload = (uint8_t*)data;
  ws_pkt.len = len;
  ws_pkt.type = op;

  this->sendAll(&ws_pkt);
}

void PsychicWebSocketHandler::sendAll(const char *buf)
{
  this->sendAll(HTTPD_WS_TYPE_TEXT, buf, strlen(buf));
}

#endif // PSYCHIC_WEBSOCKET_ENABLED

#include <dirent.h>
#include <errno.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_spiffs.h>
#include <esp_timer.h>
#include <esp_vfs.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <list>
#include <map>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "PsychiHttp/ChunkPrinter.h"
#include "PsychiHttp/PsychicHttp.h"
#include "PsychiHttp/PsychicUploadHandler.h"
#include "PsychiHttp/PsychicWebParameter.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"
#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "target.h"

#if defined(USE_WEB_FILE_SEREVER)

#include "web_server.h"
#include "wifi/file_upload.h"
#include "wifi/upload_common.h"

static const char* TAG = "WEB_SERVER_CPP";

static constexpr const char* kWpadResponse = "function FindProxyForURL(url, host) { return 'DIRECT'; }\n";

namespace {

std::atomic<size_t> g_download_chunk_size{WEB_SERVER_DOWNLOAD_CHUNK_DEFAULT};
constexpr size_t kDownloadBufferSize = 8 * 1024;
static uint8_t* s_download_buffer = nullptr;
static std::atomic<bool> s_download_in_progress{false};

struct DownloadSessionGuard {
    std::atomic<bool>& flag;
    explicit DownloadSessionGuard(std::atomic<bool>& guard_flag) : flag(guard_flag) {}
    ~DownloadSessionGuard() { flag.store(false, std::memory_order_release); }
    DownloadSessionGuard(const DownloadSessionGuard&) = delete;
    DownloadSessionGuard& operator=(const DownloadSessionGuard&) = delete;
};

struct CaptivePortalState {
    bool registered = false;
    std::atomic<bool> enabled{false};
};

std::map<PsychicHttpServer*, CaptivePortalState>& captive_portal_states();

esp_err_t send_text_response(PsychicRequest* request, const char* body, const char* content_type, bool is_head);
esp_err_t send_meta_refresh(PsychicRequest* request, bool is_head);
esp_err_t send_redirect_to_root(PsychicRequest* request);
esp_err_t send_empty_response(PsychicRequest* request, const char* content_type);
esp_err_t send_head_ok(PsychicRequest* request);
std::string sanitize_path(const std::string& input);
std::string build_fs_path(const std::string& relative);
std::string html_escape(const std::string& value);
std::string url_encode(const std::string& path);
std::string join_paths(const std::string& base, const std::string& name);
std::string parent_path(const std::string& path);
std::string safe_filename(const std::string& name);
std::string generate_csrf_token();
std::string get_cookie_value(const std::string& header, const std::string& name);
void set_csrf_cookie(PsychicResponse& response, const std::string& token);
bool is_within_root(const std::string& fs_path);

static constexpr size_t kUploadWorkerStackSizeBytes = 4096;
static constexpr size_t kUploadWorkerQueueLength = 4;
static constexpr TickType_t kUploadWorkerQueueSendTimeout = pdMS_TO_TICKS(1000);
static constexpr TickType_t kUploadWorkerShutdownTimeout = pdMS_TO_TICKS(5000);

struct UploadWorkerMessage {
    uint8_t* data;
    size_t length;
    bool final_chunk;
};

struct UploadContext {
    std::string directory_relative;
    std::string directory_fs;
    std::string file_relative;
    std::string file_fs;
    FILE* file = nullptr;
    size_t total_bytes = 0;
    esp_err_t status = ESP_OK;
    bool initialized = false;
    bool validation_done = false;
    uint64_t start_time = 0;
    QueueHandle_t worker_queue = nullptr;
    TaskHandle_t worker_task = nullptr;
    SemaphoreHandle_t worker_done = nullptr;
    bool worker_active = false;
    esp_err_t worker_status = ESP_OK;
    enum class Error {
        kNone,
        kMissingFilename,
        kFileExists,
        kFileOpenFailed,
        kWriteFailed,
        kTooLarge,
        kMissingPath,
        kMissingToken,
        kInvalidCSRF,
        kInvalidPath,
        kDirectoryNotFound,
        kTimeout
    } error = Error::kNone;
};

static bool upload_worker_start(UploadContext* context);
static bool upload_worker_enqueue_chunk(UploadContext* context, const uint8_t* data, size_t len, bool final_chunk);
static bool upload_worker_wait_for_completion(UploadContext* context);
static void upload_worker_abort(UploadContext* context);
static void upload_worker_cleanup(UploadContext* context);
static void upload_worker_task(void* arg);

}  // namespace

// Enhanced connection management with memory-aware throttling
static void web_server_manage_connections(PsychicRequest* request, bool is_download = false) {
    if (!request) return;
    PsychicHttpServer* server = request->server();
    if (!server || !server->server) return;
    httpd_handle_t hd = server->server;
    int current_fd = httpd_req_to_sockfd(request->request());

    size_t free_heap = esp_get_free_heap_size();
    const std::list<PsychicClient*>& clients = server->getClientList();
    size_t active_connections = clients.size();

    ESP_LOGD(TAG, "Connection management: %zu active connections, %zu bytes free heap", active_connections, free_heap);

    // Aggressive connection management for downloads or low memory
    bool aggressive_cleanup = is_download || (free_heap < 5000);                     // Only when critically low
    size_t max_allowed = aggressive_cleanup ? 4 : (WEB_SERVER_MAX_CONNECTIONS - 1);  // Allow 4 during downloads

    if (active_connections > max_allowed) {
        ESP_LOGW(TAG, "Too many connections (%zu > %zu), closing oldest sessions", active_connections, max_allowed);

        // Collect sockets to close (keep current and most recent)
        std::vector<int> fds_to_close;
        size_t to_close = active_connections - max_allowed;

        for (auto it = clients.begin(); it != clients.end() && fds_to_close.size() < to_close; ++it) {
            PsychicClient* client = *it;
            if (!client) continue;
            int fd = client->socket();
            if (fd >= 0 && fd != current_fd) {
                fds_to_close.push_back(fd);
            }
        }

        for (int fd : fds_to_close) {
            ESP_LOGW(TAG, "Closing session fd=%d to free resources (current=%d)", fd, current_fd);
            httpd_sess_trigger_close(hd, fd);
        }
    }

    // Keep current session active in LRU
    if (current_fd >= 0) {
        httpd_sess_update_lru_counter(hd, current_fd);
    }

    // Log memory impact
    if (aggressive_cleanup) {
        ESP_LOGI(TAG, "Connection cleanup: %zu -> %zu connections, heap: %zu bytes", active_connections, clients.size(), esp_get_free_heap_size());
    }
}

// Legacy wrapper for backward compatibility
static void web_server_close_other_sessions(PsychicRequest* request) { web_server_manage_connections(request, false); }

// Memory-aware chunk sizing to prevent heap exhaustion during downloads
size_t get_safe_download_chunk_size() {
    size_t free_heap = esp_get_free_heap_size();
    size_t base_chunk = g_download_chunk_size.load(std::memory_order_relaxed);

    // Emergency mode: very low memory
    if (free_heap < 5000) {
        ESP_LOGW(TAG, "Emergency mode: heap=%zu, using 1024-byte chunks", free_heap);
        return 1024;
    }
    // Conservative mode: low memory
    if (free_heap < 10000) {
        size_t safe_size = std::min(base_chunk / 2, static_cast<size_t>(4096));
        safe_size = std::max<size_t>(safe_size, 1024);
        ESP_LOGW(TAG, "Conservative mode: heap=%zu, using %zu-byte chunks", free_heap, safe_size);
        return safe_size;
    }
    // Normal mode
    return base_chunk;
}

// C++ implementation functions that are called from C code
extern "C" {

void web_server_set_download_chunk_size(size_t bytes) {
    size_t clamped = bytes;
    if (clamped < WEB_SERVER_DOWNLOAD_CHUNK_DEFAULT) {
        clamped = WEB_SERVER_DOWNLOAD_CHUNK_DEFAULT;
    }
    if (clamped > WEB_SERVER_DOWNLOAD_CHUNK_MAX) {
        clamped = WEB_SERVER_DOWNLOAD_CHUNK_MAX;
    }
    g_download_chunk_size.store(clamped, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Download chunk size set to %zu bytes", clamped);
}

size_t web_server_get_download_chunk_size(void) { return g_download_chunk_size.load(std::memory_order_relaxed); }

size_t web_server_get_default_download_chunk_size(void) { return WEB_SERVER_DOWNLOAD_CHUNK_DEFAULT; }

void* web_server_create_server() {
    // Create server with minimal custom configuration
    // Use mostly defaults to avoid configuration issues
    httpd_config_t custom_config = HTTPD_DEFAULT_CONFIG();

    // Memory-optimized settings to prevent heap exhaustion
    custom_config.max_uri_handlers = 32;  // Enough for captive portal + app endpoints
    custom_config.max_open_sockets = WEB_SERVER_MAX_CONNECTIONS;
    custom_config.max_resp_headers = 8;  // Reduce from default to save memory
    custom_config.lru_purge_enable = true;

    // Dynamic stack size based on available memory - INCREASED for multipart parsing
    size_t free_heap = esp_get_free_heap_size();
    size_t optimal_stack_size;

    ESP_LOGI(TAG, "=== HEAP ANALYSIS ===");
    ESP_LOGI(TAG, "Current free heap: %d bytes (%.1f KB)", free_heap, free_heap / 1024.0);
    ESP_LOGI(TAG, "Minimum free heap: %d bytes", esp_get_minimum_free_heap_size());

    // CRITICAL: Multipart parsing has recursive calls that need larger stack
    optimal_stack_size = 16 * 1024;      // Minimum 16KB for multipart parsing
    if (free_heap > 120000) {            // > 120KB free heap
        optimal_stack_size = 24 * 1024;  // 24KB for large multipart files
        ESP_LOGI(TAG, "High memory available, using 24KB stack for multipart parsing");
    } else if (free_heap > 100000) {     // > 100KB free heap
        optimal_stack_size = 20 * 1024;  // 20KB
        ESP_LOGI(TAG, "Good memory available, using 20KB stack for multipart parsing");
    } else if (free_heap > 80000) {      // > 80KB free heap
        optimal_stack_size = 16 * 1024;  // 16KB minimum for multipart
        ESP_LOGI(TAG, "Medium memory available, using 16KB stack for multipart parsing");
    } else {
        optimal_stack_size = 16 * 1024;  // 16KB minimum - multipart parsing requires this
        ESP_LOGW(TAG, "Low memory available, using minimum 16KB stack for multipart parsing");
        ESP_LOGW(TAG, "WARNING: Multipart parsing may fail with less than 16KB stack");
    }

    custom_config.stack_size = optimal_stack_size;

    // VERY IMPORTANT: keep httpd send wait short so sendChunk() returns quickly on EAGAIN
    custom_config.send_wait_timeout = 1;  // seconds
    custom_config.recv_wait_timeout = 30;

    ESP_LOGI(TAG, "Creating HTTP server with custom config:");
    ESP_LOGI(TAG, "  - Stack size: %d bytes", custom_config.stack_size);
    ESP_LOGI(TAG, "  - Max URI handlers: %d", custom_config.max_uri_handlers);
    ESP_LOGI(TAG, "  - Max open sockets: %d", custom_config.max_open_sockets);
    ESP_LOGI(TAG, "  - LRU purge: %s", custom_config.lru_purge_enable ? "enabled" : "disabled");

    // Check available heap before creating server (reuse existing free_heap variable)
    ESP_LOGI(TAG, "Available heap before server creation: %d bytes", free_heap);

    // Check if we have enough memory for the stack size - UPDATED for larger stack
    size_t required_heap = custom_config.stack_size + 30000;  // Need extra 30KB for multipart parsing
    if (free_heap < required_heap) {
        ESP_LOGW(TAG, "Low heap memory for server creation: %d bytes (need ~%d bytes)", free_heap, required_heap);
        if (custom_config.stack_size > 16 * 1024) {
            ESP_LOGW(TAG, "Reducing stack size to 16KB minimum due to memory constraints");
            custom_config.stack_size = 16 * 1024;
        }
    }

    return new PsychicHttpServer(&custom_config);
}

void web_server_destroy_server(void* server) {
    if (server) {
        PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);
        captive_portal_states().erase(http_server);
        http_server->stop();
        delete http_server;
    }

    // Free download buffer if still allocated (safety check)
    if (s_download_buffer != nullptr) {
        heap_caps_free(s_download_buffer);
        s_download_buffer = nullptr;
        ESP_LOGI(TAG, "Freed download buffer in destroy_server");
    }
}

esp_err_t web_server_start_server(void* server, uint16_t port) {
    if (!server) return ESP_ERR_INVALID_ARG;

    PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);

    // Allocate download buffer on server start
    if (s_download_buffer == nullptr) {
        s_download_buffer = static_cast<uint8_t*>(heap_caps_malloc(kDownloadBufferSize, MALLOC_CAP_8BIT));
        if (s_download_buffer == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate download buffer (%zu bytes)", kDownloadBufferSize);
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Allocated download buffer: %zu bytes", kDownloadBufferSize);
    }

    // Configuration is already set during server creation
    // Just ensure the port is set and start the server
    http_server->config.server_port = port;
    http_server->config.close_fn = PsychicHttpServer::closeCallback;
    http_server->config.uri_match_fn = httpd_uri_match_wildcard;

    // Add detailed debugging information
    ESP_LOGI(TAG, "Starting HTTP server on port %d", port);
    ESP_LOGI(TAG, "Server config: max_open_sockets=%d, max_uri_handlers=%d, stack_size=%d", http_server->config.max_open_sockets,
             http_server->config.max_uri_handlers, http_server->config.stack_size);
    ESP_LOGI(TAG, "HTTPD started: max_open_sockets=%d, send_wait_timeout=%ds, recv_wait_timeout=%ds", http_server->config.max_open_sockets,
             http_server->config.send_wait_timeout, http_server->config.recv_wait_timeout);

    // Check available heap before starting server
    ESP_LOGI(TAG, "Available heap before server start: %d bytes", esp_get_free_heap_size());

    size_t current_heap = esp_get_free_heap_size();
    if (current_heap < 50000) {
        ESP_LOGW(TAG, "Low heap memory: %d bytes (recommended: >50KB)", current_heap);
    }

    // Check if we have enough memory for the stack size - UPDATED for multipart parsing
    if (current_heap < http_server->config.stack_size + 15000) {
        ESP_LOGE(TAG, "Insufficient heap for HTTP server task with multipart parsing. Need %d bytes, have %d bytes", http_server->config.stack_size + 15000,
                 current_heap);
        return ESP_ERR_NO_MEM;
    }

    // Log task creation details
    ESP_LOGI(TAG, "Creating HTTP server task with stack size: %d bytes", http_server->config.stack_size);
    ESP_LOGI(TAG, "Task priority: %d, core: %d", http_server->config.task_priority, http_server->config.core_id);

    esp_err_t result = http_server->listen(port);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server started successfully on port %d", port);

        // Log server status after start
        ESP_LOGI(TAG, "Server handle: %p", http_server->server);
        ESP_LOGI(TAG, "Available heap after server start: %d bytes", esp_get_free_heap_size());

        // Test if server is actually responding
        ESP_LOGI(TAG, "Server configuration validation:");
        ESP_LOGI(TAG, "  - Port: %d", http_server->config.server_port);
        ESP_LOGI(TAG, "  - Max URI handlers: %d", http_server->config.max_uri_handlers);
        ESP_LOGI(TAG, "  - Max open sockets: %d", http_server->config.max_open_sockets);
        ESP_LOGI(TAG, "  - Stack size: %d bytes", http_server->config.stack_size);
        ESP_LOGI(TAG, "  - LRU purge enabled: %s", http_server->config.lru_purge_enable ? "yes" : "no");

        // Verify server is actually running by checking the handle
        if (http_server->server == NULL) {
            ESP_LOGE(TAG, "Server handle is NULL after successful start - this indicates a problem");
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server on port %d: %s", port, esp_err_to_name(result));

        // Provide specific error details
        switch (result) {
            case ESP_ERR_HTTPD_TASK:
                ESP_LOGE(TAG, "HTTPD_TASK error: Failed to create HTTP server task. Possible causes:");
                ESP_LOGE(TAG, "  - Insufficient heap memory (need ~%d bytes)", http_server->config.stack_size + 10000);
                ESP_LOGE(TAG, "  - Stack size too large: %d bytes", http_server->config.stack_size);
                ESP_LOGE(TAG, "  - Task priority conflict");
                ESP_LOGE(TAG, "  - Port %d already in use", port);
                ESP_LOGE(TAG, "  - System task limit reached");
                break;
            case ESP_ERR_HTTPD_INVALID_REQ:
                ESP_LOGE(TAG, "INVALID_REQ error: Invalid HTTP server configuration");
                ESP_LOGE(TAG, "  - Check stack_size: %d", http_server->config.stack_size);
                ESP_LOGE(TAG, "  - Check max_open_sockets: %d", http_server->config.max_open_sockets);
                ESP_LOGE(TAG, "  - Check max_uri_handlers: %d", http_server->config.max_uri_handlers);
                break;
            case ESP_ERR_HTTPD_HANDLERS_FULL:
                ESP_LOGE(TAG, "HANDLERS_FULL error: Too many URI handlers registered");
                break;
            case ESP_ERR_HTTPD_HANDLER_EXISTS:
                ESP_LOGE(TAG, "HANDLER_EXISTS error: Handler already exists for this URI");
                break;
            case ESP_ERR_INVALID_ARG:
                ESP_LOGE(TAG, "INVALID_ARG error: Invalid server configuration parameters");
                break;
            case ESP_ERR_NO_MEM:
                ESP_LOGE(TAG, "NO_MEM error: Insufficient memory for server task");
                ESP_LOGE(TAG, "  - Required: %d bytes", http_server->config.stack_size + 10000);
                ESP_LOGE(TAG, "  - Available: %d bytes", current_heap);
                break;
            default:
                ESP_LOGE(TAG, "Unknown error: %s (0x%x)", esp_err_to_name(result), result);
                break;
        }
    }

    return result;
}

void web_server_stop_server(void* server) {
    if (server) {
        PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);
        http_server->stop();
    }

    // Free download buffer on server stop
    if (s_download_buffer != nullptr) {
        heap_caps_free(s_download_buffer);
        s_download_buffer = nullptr;
        ESP_LOGI(TAG, "Freed download buffer");
    }
}

esp_err_t web_server_add_endpoint(void* server, const char* uri, esp_err_t (*callback)(void*)) {
    if (!server || !uri || !callback) return ESP_ERR_INVALID_ARG;

    PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);

    PsychicHttpRequestCallback wrapper = [callback](PsychicRequest* request) -> esp_err_t {
        ESP_LOGI(TAG, "Endpoint handler called for URI: %s", request->uri().c_str());
        esp_err_t result = callback(request);
        ESP_LOGI(TAG, "Endpoint handler result: %s", esp_err_to_name(result));
        return result;
    };

    ESP_LOGI(TAG, "Registering GET endpoint: %s", uri);
    ESP_LOGI(TAG, "Server handle: %p", http_server->server);
    ESP_LOGI(TAG, "Available heap before endpoint registration: %d bytes", esp_get_free_heap_size());

    PsychicEndpoint* endpoint = http_server->on(uri, HTTP_GET, wrapper);
    if (endpoint == nullptr) {
        esp_err_t error = http_server->lastEndpointError();
        ESP_LOGE(TAG, "Failed to register endpoint %s: %s", uri, esp_err_to_name(error));
        ESP_LOGE(TAG, "Server handle: %p", http_server->server);
        ESP_LOGE(TAG, "Available heap: %d bytes", esp_get_free_heap_size());
        return error;
    }
    ESP_LOGI(TAG, "Successfully registered endpoint: %s", uri);
    ESP_LOGI(TAG, "Available heap after endpoint registration: %d bytes", esp_get_free_heap_size());
    return ESP_OK;
}

// Forward declaration
static esp_err_t upload_handler_native(PsychicRequest* psychic_request);

// Custom upload handler that doesn't consume the request body
class NativeUploadHandler : public PsychicHandler {
   public:
    NativeUploadHandler() : PsychicHandler() {}

    bool canHandle(PsychicRequest* request) override { return request->method() == HTTP_POST; }

    esp_err_t handleRequest(PsychicRequest* request) override {
        // CRITICAL: Do NOT call parent's handleRequest() or loadBody()
        // because it consumes the request body, leaving nothing for multipart parsing
        ESP_LOGI(TAG, "NativeUploadHandler::handleRequest - preserving request body for multipart parsing");

        // Call our native upload handler directly
        return upload_handler_native(request);
    }
};

esp_err_t web_server_add_post_endpoint(void* server, const char* uri, esp_err_t (*callback)(void*)) {
    if (!server || !uri || !callback) {
        return ESP_ERR_INVALID_ARG;
    }

    PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);

    // For upload endpoints, use our custom handler that doesn't consume the request body
    if (strcmp(uri, "/upload") == 0) {
        ESP_LOGI(TAG, "Registering POST endpoint with custom upload handler: %s", uri);
        NativeUploadHandler* handler = new (std::nothrow) NativeUploadHandler();
        if (handler == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate NativeUploadHandler for %s", uri);
            return ESP_ERR_NO_MEM;
        }

        PsychicEndpoint* endpoint = http_server->on(uri, HTTP_POST, handler);
        if (endpoint == nullptr) {
            delete handler;
            esp_err_t error = http_server->lastEndpointError();
            ESP_LOGE(TAG, "Failed to register POST endpoint %s: %s", uri, esp_err_to_name(error));
            return error;
        }
        ESP_LOGI(TAG, "Successfully registered POST endpoint with custom handler: %s", uri);
        return ESP_OK;
    }

    // For other POST endpoints, use the standard handler
    PsychicHttpRequestCallback wrapper = [callback](PsychicRequest* request) -> esp_err_t { return callback(request); };

    ESP_LOGI(TAG, "Registering POST endpoint: %s", uri);
    PsychicEndpoint* endpoint = http_server->on(uri, HTTP_POST, wrapper);
    if (endpoint == nullptr) {
        esp_err_t error = http_server->lastEndpointError();
        ESP_LOGE(TAG, "Failed to register POST endpoint %s: %s", uri, esp_err_to_name(error));
        return error;
    }
    ESP_LOGI(TAG, "Successfully registered POST endpoint: %s", uri);
    return ESP_OK;
}

void* web_server_setup_static_files(void* server, const char* uri, const char* path, const char* cache_control) {
    if (!server) return nullptr;

    PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);
    // Create a file system instance for SPIFFS
    static fs::FS spiffs_fs;
    return http_server->serveStatic(uri, spiffs_fs, path, cache_control);
}

void web_server_set_default_file(void* static_handler, const char* filename) {
    if (static_handler && filename) {
        PsychicStaticFileHandler* handler = static_cast<PsychicStaticFileHandler*>(static_handler);
        handler->setDefaultFile(filename);
    }
}

void web_server_configure_captive_portal(void* server, bool enabled) {
    if (!server) {
        return;
    }

    PsychicHttpServer* http_server = static_cast<PsychicHttpServer*>(server);
    auto& state_map = captive_portal_states();
    CaptivePortalState& state = state_map[http_server];

    ESP_LOGI(TAG, "Configuring captive portal: enabled=%s, already_registered=%s, server=%p", enabled ? "true" : "false", state.registered ? "true" : "false",
             http_server);

    state.enabled.store(enabled, std::memory_order_release);

    if (!state.registered && enabled) {
        ESP_LOGI(TAG, "Registering captive portal endpoints for the first time");
        CaptivePortalState* state_ptr = &state;

        auto meta_handler = [state_ptr](PsychicRequest* request) -> esp_err_t {
            if (!state_ptr->enabled.load(std::memory_order_acquire)) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            http_method method = request->method();
            if (method != HTTP_GET && method != HTTP_HEAD) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            bool is_head = method == HTTP_HEAD;
            return send_meta_refresh(request, is_head);
        };

        http_server->on("/hotspot-detect.html", HTTP_GET, meta_handler);
        http_server->on("/hotspot-detect.html", HTTP_HEAD, meta_handler);
        http_server->on("/ncsi.txt", HTTP_GET, meta_handler);
        http_server->on("/ncsi.txt", HTTP_HEAD, meta_handler);
        http_server->on("/connecttest.txt", HTTP_GET, meta_handler);
        http_server->on("/connecttest.txt", HTTP_HEAD, meta_handler);
        http_server->on("/redirect", HTTP_GET, meta_handler);
        http_server->on("/redirect", HTTP_HEAD, meta_handler);

        auto redirect_handler = [state_ptr](PsychicRequest* request) -> esp_err_t {
            if (!state_ptr->enabled.load(std::memory_order_acquire)) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            http_method method = request->method();
            if (method != HTTP_GET && method != HTTP_HEAD) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            return send_redirect_to_root(request);
        };

        http_server->on("/generate_204", HTTP_GET, redirect_handler);
        http_server->on("/generate_204", HTTP_HEAD, redirect_handler);
        http_server->on("/gen_204", HTTP_GET, redirect_handler);
        http_server->on("/gen_204", HTTP_HEAD, redirect_handler);

        auto wpad_handler = [state_ptr](PsychicRequest* request) -> esp_err_t {
            if (!state_ptr->enabled.load(std::memory_order_acquire)) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            http_method method = request->method();
            if (method != HTTP_GET && method != HTTP_HEAD) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            bool is_head = method == HTTP_HEAD;
            return send_text_response(request, kWpadResponse, "application/x-ns-proxy-autoconfig", is_head);
        };

        http_server->on("/wpad.dat", HTTP_GET, wpad_handler);
        http_server->on("/wpad.dat", HTTP_HEAD, wpad_handler);

        auto crl_handler = [state_ptr](PsychicRequest* request) -> esp_err_t {
            if (!state_ptr->enabled.load(std::memory_order_acquire)) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            http_method method = request->method();
            if (method != HTTP_GET && method != HTTP_HEAD) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            return send_empty_response(request, "application/pkix-crl");
        };

        http_server->on("/DigiCertTrustedRootG4.crl", HTTP_GET, crl_handler);
        http_server->on("/DigiCertTrustedRootG4.crl", HTTP_HEAD, crl_handler);

        auto head_handler = [state_ptr](PsychicRequest* request) -> esp_err_t {
            if (!state_ptr->enabled.load(std::memory_order_acquire)) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            http_method method = request->method();
            if (method != HTTP_HEAD && method != HTTP_GET) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            return send_head_ok(request);
        };

        http_server->on("/", HTTP_HEAD, head_handler);
        http_server->on("/index.html", HTTP_HEAD, head_handler);

        http_server->onNotFound([state_ptr](PsychicRequest* request) -> esp_err_t {
            if (!state_ptr->enabled.load(std::memory_order_acquire)) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            http_method method = request->method();
            if (method != HTTP_GET && method != HTTP_HEAD) {
                return PsychicHttpServer::defaultNotFoundHandler(request);
            }
            bool is_head = method == HTTP_HEAD;
            return send_meta_refresh(request, is_head);
        });

        state.registered = true;
        ESP_LOGI(TAG, "Captive portal endpoints registered successfully");
    } else if (state.registered) {
        ESP_LOGI(TAG, "Captive portal endpoints already registered, skipping registration");
    } else {
        ESP_LOGI(TAG, "Captive portal disabled, not registering endpoints");
    }

    ESP_LOGI(TAG, "Captive portal HTTP state: %s", enabled ? "enabled" : "disabled");
}

}  // extern "C"

namespace {

constexpr const char* kSdRoot = "/sdcard";
constexpr size_t kChunkBufferSize = 2048;  // Reduced to 2KB for better reliability on large files

// ---- Diagnostics helpers ----------------------------------------------------
static void log_heap_diag(const char* where) {
    size_t free8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t min8 = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "[%s] HEAP: free8=%u min8=%u", where, (unsigned)free8, (unsigned)min8);
}

// --- Socket back-pressure helper --------------------------------------------
static bool wait_socket_writable(int fd, uint32_t timeout_ms) {
    if (fd < 0) return false;
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int r = select(fd + 1, NULL, &wfds, NULL, &tv);
    return (r > 0) && FD_ISSET(fd, &wfds);
}

static void wait_heap_recovers(size_t min_safe_bytes, uint32_t step_sleep_ms, uint32_t max_wait_ms) {
    uint32_t waited = 0;
    while (heap_caps_get_free_size(MALLOC_CAP_8BIT) < min_safe_bytes && waited < max_wait_ms) {
        vTaskDelay(pdMS_TO_TICKS(step_sleep_ms));
        waited += step_sleep_ms;
    }
}

std::map<PsychicHttpServer*, CaptivePortalState>& captive_portal_states() {
    static std::map<PsychicHttpServer*, CaptivePortalState> states;
    return states;
}

static constexpr const char* kMetaRefreshHtml =
    "<!doctype html><meta http-equiv=\"refresh\" content='0;url=/'>"
    "<title>Opening…</title><p>Opening <a href='/'>portal</a>…";

// WARNING: body parameter must have guaranteed lifetime (e.g., string literal or static storage)
esp_err_t send_text_response(PsychicRequest* request, const char* body, const char* content_type, bool is_head) {
    PsychicResponse response(request);
    response.setCode(200);
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response.addHeader("Content-Type", content_type);

    if (is_head) {
        char length_buf[16];
        snprintf(length_buf, sizeof(length_buf), "%u", static_cast<unsigned int>(strlen(body)));
        response.addHeader("Content-Length", length_buf);
    } else {
        response.setContent(body);
    }

    return response.send();
}

esp_err_t send_meta_refresh(PsychicRequest* request, bool is_head) {
    return send_text_response(request, kMetaRefreshHtml, "text/html; charset=utf-8", is_head);
}

esp_err_t send_redirect_to_root(PsychicRequest* request) {
    PsychicResponse response(request);
    response.setCode(302);
    response.addHeader("Location", "/");
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response.setContentLength(0);
    return response.send();
}

esp_err_t send_empty_response(PsychicRequest* request, const char* content_type) {
    PsychicResponse response(request);
    response.setCode(200);
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response.addHeader("Content-Type", content_type);
    response.setContentLength(0);
    return response.send();
}

esp_err_t send_head_ok(PsychicRequest* request) {
    PsychicResponse response(request);
    response.setCode(200);
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response.addHeader("Content-Type", "text/html; charset=utf-8");
    response.setContentLength(0);
    return response.send();
}

std::string sanitize_path(const std::string& input) {
    if (input.empty()) {
        return "/";
    }

    std::string normalized;
    normalized.reserve(input.size() + 1);
    if (input.front() != '/') {
        normalized.push_back('/');
    }
    for (char c : input) {
        normalized.push_back(c == '\\' ? '/' : c);
    }

    std::vector<std::string> segments;
    size_t index = 0;
    const size_t len = normalized.size();
    while (index < len) {
        while (index < len && normalized[index] == '/') {
            ++index;
        }
        size_t start = index;
        while (index < len && normalized[index] != '/') {
            ++index;
        }
        if (index > start) {
            std::string segment = normalized.substr(start, index - start);
            if (segment == ".") {
                continue;
            }
            if (segment == "..") {
                if (!segments.empty()) {
                    segments.pop_back();
                }
            } else {
                segments.push_back(segment);
            }
        }
    }

    if (segments.empty()) {
        return "/";
    }

    std::string sanitized = "/";
    for (size_t i = 0; i < segments.size(); ++i) {
        sanitized += segments[i];
        if (i + 1 < segments.size()) {
            sanitized += "/";
        }
    }
    return sanitized;
}

std::string build_fs_path(const std::string& relative) {
    std::string fs_path(kSdRoot);
    if (relative.size() > 1) {
        fs_path += relative;
    } else {
        fs_path += "/";
    }
    return fs_path;
}

std::string html_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (char c : value) {
        switch (c) {
            case '&':
                escaped += "&amp;";
                break;
            case '<':
                escaped += "&lt;";
                break;
            case '>':
                escaped += "&gt;";
                break;
            case '"':
                escaped += "&quot;";
                break;
            case '\'':
                escaped += "&#39;";
                break;
            default:
                escaped.push_back(c);
                break;
        }
    }
    return escaped;
}

std::string url_encode(const std::string& path) {
    String encoded = PsychicHttp::urlEncode(path.c_str());
    return std::string(encoded.c_str());
}

std::string join_paths(const std::string& base, const std::string& name) {
    std::string combined = base;
    if (combined.size() > 1 && combined.back() == '/') {
        combined.pop_back();
    }
    if (combined.empty()) {
        combined = "/";
    }
    if (combined.back() != '/') {
        combined += "/";
    }
    combined += name;
    return sanitize_path(combined);
}

std::string parent_path(const std::string& path) {
    if (path.empty() || path == "/") {
        return "/";
    }

    std::string trimmed = path;
    if (trimmed.size() > 1 && trimmed.back() == '/') {
        trimmed.pop_back();
    }

    size_t slash = trimmed.find_last_of('/');
    if (slash == std::string::npos || slash == 0) {
        return "/";
    }
    return trimmed.substr(0, slash);
}

std::string safe_filename(const std::string& name) {
    if (name.empty()) {
        return "download.bin";
    }

    std::string safe = name;
    for (char& c : safe) {
        unsigned char uc = static_cast<unsigned char>(c);
        if (uc < 0x20 || uc == 0x7F || c == '"' || c == '\\' || c == '\r' || c == '\n' || c == ';' || c == '/') {
            c = '_';
        }
    }
    return safe;
}

std::string format_file_size(size_t bytes) {
    if (bytes == 0) {
        return "0 B";
    }

    const char* units[] = {"B", "KB", "MB", "GB", "TB"};
    int unit_index = 0;
    double size = static_cast<double>(bytes);

    while (size >= 1024.0 && unit_index < 4) {
        size /= 1024.0;
        unit_index++;
    }

    char buffer[32];
    if (unit_index == 0) {
        // For bytes, show as integer
        snprintf(buffer, sizeof(buffer), "%zu %s", bytes, units[unit_index]);
    } else if (size >= 100.0) {
        // For larger sizes, show 0 decimal places
        snprintf(buffer, sizeof(buffer), "%.0f %s", size, units[unit_index]);
    } else if (size >= 10.0) {
        // For medium sizes, show 1 decimal place
        snprintf(buffer, sizeof(buffer), "%.1f %s", size, units[unit_index]);
    } else {
        // For small sizes, show 2 decimal places
        snprintf(buffer, sizeof(buffer), "%.2f %s", size, units[unit_index]);
    }

    return std::string(buffer);
}

std::string generate_csrf_token() {
    uint8_t random_bytes[16];
    for (size_t i = 0; i < sizeof(random_bytes); i += sizeof(uint32_t)) {
        uint32_t value = esp_random();
        size_t remaining = std::min(sizeof(random_bytes) - i, sizeof(uint32_t));
        memcpy(&random_bytes[i], &value, remaining);
    }

    char hex[sizeof(random_bytes) * 2 + 1];
    static const char* kHex = "0123456789abcdef";
    for (size_t i = 0; i < sizeof(random_bytes); ++i) {
        hex[i * 2] = kHex[(random_bytes[i] >> 4) & 0x0F];
        hex[i * 2 + 1] = kHex[random_bytes[i] & 0x0F];
    }
    hex[sizeof(random_bytes) * 2] = '\0';
    return std::string(hex);
}

std::string get_cookie_value(const std::string& header, const std::string& name) {
    if (header.empty()) {
        return std::string();
    }

    std::string needle = name + "=";
    size_t pos = 0;
    while (pos < header.size()) {
        size_t end = header.find(';', pos);
        std::string segment = header.substr(pos, end == std::string::npos ? std::string::npos : end - pos);

        // Trim leading spaces
        size_t first = segment.find_first_not_of(" \t");
        if (first != std::string::npos) {
            segment.erase(0, first);
        } else {
            segment.clear();
        }

        if (!segment.empty() && segment.rfind(needle, 0) == 0) {
            return segment.substr(needle.size());
        }

        if (end == std::string::npos) {
            break;
        }
        pos = end + 1;
    }
    return std::string();
}

static bool upload_worker_start(UploadContext* context) {
    if (!context) return false;

    if (!context->worker_queue) {
        context->worker_queue = xQueueCreate(kUploadWorkerQueueLength, sizeof(UploadWorkerMessage));
        if (!context->worker_queue) {
            ESP_LOGE(TAG, "Failed to create upload worker queue");
            return false;
        }
    }

    if (!context->worker_done) {
        context->worker_done = xSemaphoreCreateBinary();
        if (!context->worker_done) {
            ESP_LOGE(TAG, "Failed to create upload worker semaphore");
            vQueueDelete(context->worker_queue);
            context->worker_queue = nullptr;
            return false;
        }
    }

    UBaseType_t worker_priority = TASK_PRIORITY_WEB_SERVER;
    if (worker_priority > 1) {
        worker_priority -= 1;
    }

    BaseType_t task_status = xTaskCreatePinnedToCore(upload_worker_task, "UPLOAD_WRITER", kUploadWorkerStackSizeBytes / sizeof(StackType_t), context,
                                                     worker_priority, &context->worker_task, 0);
    if (task_status != pdPASS) {
        ESP_LOGE(TAG, "Failed to create upload worker task");
        upload_worker_cleanup(context);
        return false;
    }

    context->worker_active = true;
    context->worker_status = ESP_OK;
    return true;
}

static void upload_worker_cleanup(UploadContext* context) {
    if (!context) return;
    if (context->worker_queue) {
        vQueueDelete(context->worker_queue);
        context->worker_queue = nullptr;
    }
    if (context->worker_done) {
        vSemaphoreDelete(context->worker_done);
        context->worker_done = nullptr;
    }
    context->worker_task = nullptr;
}

static bool upload_worker_enqueue_chunk(UploadContext* context, const uint8_t* data, size_t len, bool final_chunk) {
    if (!context) return false;

    if (!context->worker_active) {
        if (!upload_worker_start(context)) {
            return false;
        }
    }

    uint8_t* buffer = nullptr;
    if (len > 0) {
        buffer = static_cast<uint8_t*>(malloc(len));
        if (!buffer) {
            ESP_LOGE(TAG, "Failed to allocate %zu bytes for upload chunk", len);
            return false;
        }
        memcpy(buffer, data, len);
    }

    UploadWorkerMessage message{
        .data = buffer,
        .length = len,
        .final_chunk = final_chunk,
    };

    if (xQueueSend(context->worker_queue, &message, kUploadWorkerQueueSendTimeout) != pdTRUE) {
        ESP_LOGE(TAG, "Upload worker queue full");
        if (buffer) free(buffer);
        return false;
    }

    return true;
}

static bool upload_worker_wait_for_completion(UploadContext* context) {
    if (!context) return false;
    if (!context->worker_active) {
        upload_worker_cleanup(context);
        return true;
    }

    if (!context->worker_done) {
        ESP_LOGE(TAG, "Upload worker semaphore missing");
        return false;
    }

    if (xSemaphoreTake(context->worker_done, kUploadWorkerShutdownTimeout) != pdTRUE) {
        ESP_LOGE(TAG, "Upload worker shutdown timed out");
        return false;
    }

    context->worker_active = false;
    bool ok = (context->worker_status == ESP_OK);
    upload_worker_cleanup(context);
    return ok;
}

static void upload_worker_abort(UploadContext* context) {
    if (!context || !context->worker_queue) {
        upload_worker_cleanup(context);
        return;
    }

    UploadWorkerMessage message{
        .data = nullptr,
        .length = 0,
        .final_chunk = true,
    };

    if (xQueueSend(context->worker_queue, &message, kUploadWorkerQueueSendTimeout) != pdTRUE) {
        xQueueReset(context->worker_queue);
        xQueueSend(context->worker_queue, &message, kUploadWorkerQueueSendTimeout);
    }
    upload_worker_wait_for_completion(context);
}

static void upload_worker_task(void* arg) {
    auto* context = static_cast<UploadContext*>(arg);
    UploadWorkerMessage message{};
    esp_err_t status = ESP_OK;

    while (xQueueReceive(context->worker_queue, &message, portMAX_DELAY) == pdTRUE) {
        if (message.data == nullptr && message.length == 0 && message.final_chunk) {
            break;
        }

        if (message.length > 0 && context->file && status == ESP_OK) {
            size_t written = fwrite(message.data, 1, message.length, context->file);
            if (written != message.length) {
                ESP_LOGE(TAG, "Upload worker short write: %zu/%zu bytes", written, message.length);
                status = ESP_FAIL;
                if (context->status == ESP_OK) {
                    context->status = ESP_FAIL;
                    context->error = UploadContext::Error::kWriteFailed;
                }
            }
        }

        if (message.data) {
            free(message.data);
        }

        if (status != ESP_OK || message.final_chunk) {
            break;
        }
    }

    if (context->file) {
        fflush(context->file);
        fsync(fileno(context->file));
    }

    context->worker_status = status;
    context->worker_active = false;
    if (context->worker_done) {
        xSemaphoreGive(context->worker_done);
    }
    context->worker_task = nullptr;
    vTaskDelete(NULL);
}

void set_csrf_cookie(PsychicResponse& response, const std::string& token) {
    std::string cookie = "csrf_token=" + token + "; Path=/; HttpOnly; SameSite=Strict; Max-Age=3600";
    response.addHeader("Set-Cookie", cookie.c_str());
}

bool is_within_root(const std::string& fs_path) {
    const std::string root(kSdRoot);
    if (fs_path == root) {
        return true;
    }
    if (fs_path.size() < root.size() + 1) {
        return false;
    }
    return fs_path.compare(0, root.size(), root) == 0 && fs_path[root.size()] == '/';
}

// Recursively delete directory and all its contents
esp_err_t delete_directory_recursive(const char* dir_path) {
    if (!dir_path) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if path is within allowed root
    std::string fs_path_str(dir_path);
    if (!is_within_root(fs_path_str)) {
        ESP_LOGE(TAG, "Attempted to delete outside root: %s", dir_path);
        return ESP_ERR_INVALID_ARG;
    }

    DIR* dir = opendir(dir_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s: %s", dir_path, strerror(errno));
        return ESP_FAIL;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        const char* entry_name = entry->d_name;
        if (!entry_name) {
            continue;
        }
        // Skip . and .. entries
        if (strcmp(entry_name, ".") == 0 || strcmp(entry_name, "..") == 0) {
            continue;
        }

        // Build full path for the entry
        std::string full_path = std::string(dir_path);
        if (full_path.back() != '/') {
            full_path += "/";
        }
        full_path += entry_name;

        struct stat st = {0};
        if (stat(full_path.c_str(), &st) != 0) {
            ESP_LOGW(TAG, "Failed to stat %s: %s", full_path.c_str(), strerror(errno));
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            // Recursively delete subdirectory
            esp_err_t result = delete_directory_recursive(full_path.c_str());
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to delete subdirectory %s", full_path.c_str());
                closedir(dir);
                return result;
            }
        } else {
            // Delete file
            if (unlink(full_path.c_str()) != 0) {
                ESP_LOGE(TAG, "Failed to delete file %s: %s", full_path.c_str(), strerror(errno));
                closedir(dir);
                return ESP_FAIL;
            }
            ESP_LOGD(TAG, "Deleted file: %s", full_path.c_str());
        }
    }

    closedir(dir);

    // Remove the empty directory
    if (rmdir(dir_path) != 0) {
        ESP_LOGE(TAG, "Failed to remove directory %s: %s", dir_path, strerror(errno));
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Deleted directory: %s", dir_path);
    return ESP_OK;
}

// Delete all files and folders except configuration.ini
esp_err_t delete_all_except_config(const char* root_path) {
    if (!root_path) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if path is within allowed root
    std::string fs_path_str(root_path);
    if (!is_within_root(fs_path_str)) {
        ESP_LOGE(TAG, "Attempted to delete outside root: %s", root_path);
        return ESP_ERR_INVALID_ARG;
    }

    DIR* dir = opendir(root_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open root directory %s: %s", root_path, strerror(errno));
        return ESP_FAIL;
    }

    struct dirent* entry;
    int files_deleted = 0;
    int dirs_deleted = 0;
    while ((entry = readdir(dir)) != nullptr) {
        const char* entry_name = entry->d_name;
        if (!entry_name) {
            continue;
        }
        // Skip . and .. entries
        if (strcmp(entry_name, ".") == 0 || strcmp(entry_name, "..") == 0) {
            continue;
        }

        // Skip configuration.ini file and other critical system files
        if (strcmp(entry_name, "configuration.ini") == 0) {
            ESP_LOGI(TAG, "Preserving configuration.ini file");
            continue;
        }

        // Additional safety: skip any hidden files that might be system files
        if (entry_name[0] == '.') {
            ESP_LOGI(TAG, "Skipping hidden file: %s", entry_name);
            continue;
        }

        // Build full path for the entry
        std::string full_path = std::string(root_path);
        if (full_path.back() != '/') {
            full_path += "/";
        }
        full_path += entry_name;

        struct stat st = {0};
        if (stat(full_path.c_str(), &st) != 0) {
            ESP_LOGW(TAG, "Failed to stat %s: %s", full_path.c_str(), strerror(errno));
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            // Recursively delete subdirectory
            ESP_LOGI(TAG, "Deleting directory: %s", full_path.c_str());
            esp_err_t result = delete_directory_recursive(full_path.c_str());
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to delete directory %s", full_path.c_str());
                closedir(dir);
                return result;
            }
            dirs_deleted++;
        } else {
            // Delete file
            ESP_LOGI(TAG, "Deleting file: %s", full_path.c_str());
            if (unlink(full_path.c_str()) != 0) {
                ESP_LOGE(TAG, "Failed to delete file %s: %s", full_path.c_str(), strerror(errno));
                closedir(dir);
                return ESP_FAIL;
            }
            files_deleted++;
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Delete all operation completed successfully - deleted %d files and %d directories", files_deleted, dirs_deleted);
    return ESP_OK;
}

}  // namespace

extern "C" void web_server_notify_request_result(bool success, size_t bytes_served);

// Directory entry structure for sorting
struct DirectoryEntry {
    std::string name;
    bool is_directory;
    size_t file_size;

    DirectoryEntry(const std::string& n, bool is_dir, size_t size) : name(n), is_directory(is_dir), file_size(size) {}
};

// Custom comparator for sorting directory entries
// Files first (alphabetically), then folders in descending order
static bool compare_directory_entries(const DirectoryEntry& a, const DirectoryEntry& b) {
    // Files come before folders
    if (!a.is_directory && b.is_directory) return true;
    if (a.is_directory && !b.is_directory) return false;

    // If both are files: alphabetical ascending
    if (!a.is_directory && !b.is_directory) {
        return a.name < b.name;
    }

    // If both are folders: descending order
    // Try numeric comparison first for pure numbers, fall back to string comparison
    bool a_is_numeric = !a.name.empty() && std::all_of(a.name.begin(), a.name.end(), ::isdigit);
    bool b_is_numeric = !b.name.empty() && std::all_of(b.name.begin(), b.name.end(), ::isdigit);

    if (a_is_numeric && b_is_numeric) {
        // Both are numeric - compare as numbers (descending)
        int num_a = std::atoi(a.name.c_str());
        int num_b = std::atoi(b.name.c_str());
        return num_a > num_b;
    }

    // At least one is not numeric - use string comparison (descending)
    return a.name > b.name;
}

// PsychicHttp upload implementation (original)
static esp_err_t upload_handler_psychic(PsychicRequest* psychic_request) {
    ESP_LOGI(TAG, "=== PSYCHIC UPLOAD HANDLER START ===");
    ESP_LOGI(TAG, "Upload handler called - method: %d, content-length: %d", psychic_request->method(), psychic_request->contentLength());

    if (psychic_request->method() != HTTP_POST) {
        ESP_LOGW(TAG, "Upload handler called with non-POST method: %d", psychic_request->method());
        return psychic_request->reply(405, "text/plain", "Method Not Allowed");
    }

    // Proactively manage connections to free resources
    ESP_LOGI(TAG, "Managing connections for upload");
    web_server_manage_connections(psychic_request, false);

    // Note: For multipart uploads, parameters are extracted during multipart parsing
    // We cannot load them early as they're embedded in the multipart stream
    ESP_LOGI(TAG, "Content-Type: %s", psychic_request->header("Content-Type").c_str());
    ESP_LOGI(TAG, "Is multipart: %s", psychic_request->isMultipart() ? "true" : "false");

    PsychicUploadHandler uploader;
    ESP_LOGI(TAG, "PsychicUploadHandler created successfully");

    static constexpr size_t kMaxUploadBytes = 10 * 1024 * 1024;  // 10 MiB limit
    const uint64_t upload_start_time = esp_timer_get_time();
    const uint64_t max_upload_time_us = 300000000;  // 5 minutes timeout

    UploadContext context;
    context.start_time = upload_start_time;

    // Set up redirect response callback for successful uploads
    ESP_LOGI(TAG, "Setting up onRequest callback");
    uploader.onRequest([&context](PsychicRequest* req) -> esp_err_t {
        ESP_LOGI(TAG, "onRequest callback called - status: %s", esp_err_to_name(context.status));

        if (context.status != ESP_OK) {
            ESP_LOGE(TAG, "Upload failed with error type: %d", static_cast<int>(context.error));
            // Handle validation errors
            switch (context.error) {
                case UploadContext::Error::kMissingPath:
                    ESP_LOGE(TAG, "Error: Missing path parameter");
                    return req->reply(400, "text/plain", "Missing path parameter");
                case UploadContext::Error::kMissingToken:
                    ESP_LOGE(TAG, "Error: CSRF token missing");
                    return req->reply(403, "text/plain", "CSRF token missing");
                case UploadContext::Error::kInvalidCSRF:
                    ESP_LOGE(TAG, "Error: CSRF validation failed");
                    return req->reply(403, "text/plain", "CSRF validation failed");
                case UploadContext::Error::kInvalidPath:
                    ESP_LOGE(TAG, "Error: Invalid path parameter");
                    return req->reply(400, "text/plain", "Invalid path parameter");
                case UploadContext::Error::kDirectoryNotFound:
                    ESP_LOGE(TAG, "Error: Directory not found");
                    return req->reply(400, "text/plain", "Directory not found");
                case UploadContext::Error::kMissingFilename:
                    ESP_LOGE(TAG, "Error: Missing filename");
                    return req->reply(400, "text/plain", "Missing filename");
                case UploadContext::Error::kFileExists:
                    ESP_LOGE(TAG, "Error: File already exists");
                    return req->reply(409, "text/plain", "File already exists");
                case UploadContext::Error::kTooLarge:
                    ESP_LOGE(TAG, "Error: Uploaded file is too large");
                    return req->reply(413, "text/plain", "Uploaded file is too large");
                case UploadContext::Error::kTimeout:
                    ESP_LOGE(TAG, "Error: Upload timeout");
                    return req->reply(408, "text/plain", "Upload timeout");
                case UploadContext::Error::kFileOpenFailed:
                case UploadContext::Error::kWriteFailed:
                    ESP_LOGE(TAG, "Error: Failed to store uploaded file");
                    return req->reply(500, "text/plain", "Failed to store uploaded file");
                default:
                    ESP_LOGE(TAG, "Error: Unknown upload failure");
                    return req->reply(500, "text/plain", "Upload failed");
            }
        }

        // Success - redirect to the directory
        ESP_LOGI(TAG, "Upload successful, redirecting to: %s", context.directory_relative.c_str());
        std::string redirect_path = "/?path=" + url_encode(context.directory_relative);
        PsychicResponse response(req);
        response.setCode(303);
        response.addHeader("Location", redirect_path.c_str());
        response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
        response.setContent("");
        return response.send();
    });

    auto finalize_file = [&]() {
        upload_worker_abort(&context);
        if (context.file) {
            fclose(context.file);
            context.file = nullptr;
        }
    };

    ESP_LOGI(TAG, "Setting up onUpload callback");
    uploader.onUpload([&](PsychicRequest* req, const String& filename, uint64_t /*index*/, uint8_t* data, size_t len, bool final) {
        ESP_LOGD(TAG, "onUpload callback - filename: %s, len: %zu, final: %s", filename.c_str(), len, final ? "true" : "false");

        // Check timeout first
        uint64_t elapsed_us = esp_timer_get_time() - context.start_time;
        if (elapsed_us > max_upload_time_us) {
            ESP_LOGE(TAG, "Upload timeout after %llu seconds", elapsed_us / 1000000);
            context.error = UploadContext::Error::kTimeout;
            context.status = ESP_ERR_TIMEOUT;
            return ESP_ERR_TIMEOUT;
        }

        if (context.status != ESP_OK) {
            ESP_LOGW(TAG, "Upload already failed, skipping data processing");
            return context.status;
        }

        // Perform validation on first call (when parameters are available)
        if (!context.validation_done) {
            ESP_LOGI(TAG, "Performing upload validation");
            context.validation_done = true;

            // Extract parameters from the request (they should be available now)
            PsychicWebParameter* path_param = req->getParam("path");
            PsychicWebParameter* token_param = req->getParam("csrf_token");

            ESP_LOGI(TAG, "Parameters extracted during validation - path: %s, token: %s", path_param ? "present" : "missing",
                     token_param ? "present" : "missing");

            if (!path_param) {
                ESP_LOGE(TAG, "Validation failed: Missing path parameter");
                context.error = UploadContext::Error::kMissingPath;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            if (!token_param) {
                ESP_LOGE(TAG, "Validation failed: Missing CSRF token");
                context.error = UploadContext::Error::kMissingToken;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            // Validate CSRF token
            std::string cookie_header = std::string(req->header("Cookie").c_str());
            std::string cookie_token = get_cookie_value(cookie_header, "csrf_token");
            std::string form_token = std::string(token_param->value().c_str());
            ESP_LOGD(TAG, "CSRF validation - cookie: %s, form: %s", cookie_token.c_str(), form_token.c_str());
            if (cookie_token.empty() || cookie_token != form_token) {
                ESP_LOGE(TAG, "Validation failed: CSRF token mismatch");
                context.error = UploadContext::Error::kInvalidCSRF;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            // Validate and sanitize path
            std::string directory_relative = sanitize_path(std::string(path_param->value().c_str()));
            std::string directory_fs = build_fs_path(directory_relative);
            ESP_LOGI(TAG, "Path validation - relative: %s, fs: %s", directory_relative.c_str(), directory_fs.c_str());
            if (!is_within_root(directory_fs)) {
                ESP_LOGE(TAG, "Validation failed: Path outside root");
                context.error = UploadContext::Error::kInvalidPath;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            // Verify directory exists
            struct stat dir_stat = {0};
            if (stat(directory_fs.c_str(), &dir_stat) != 0 || !S_ISDIR(dir_stat.st_mode)) {
                ESP_LOGE(TAG, "Validation failed: Directory not found: %s", directory_fs.c_str());
                context.error = UploadContext::Error::kDirectoryNotFound;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            // Initialize context with validated parameters
            context.directory_relative = directory_relative;
            context.directory_fs = directory_fs;
            ESP_LOGI(TAG, "Upload validation complete - directory: %s", directory_relative.c_str());
        }

        if (!context.initialized) {
            ESP_LOGI(TAG, "Initializing file upload");
            std::string original_filename = std::string(filename.c_str());
            ESP_LOGI(TAG, "Original filename: %s", original_filename.c_str());

            if (original_filename.empty()) {
                ESP_LOGE(TAG, "File initialization failed: Missing filename");
                context.error = UploadContext::Error::kMissingFilename;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            std::string sanitized_filename = safe_filename(original_filename);
            if (sanitized_filename.empty()) {
                sanitized_filename = "upload.bin";
            }
            ESP_LOGI(TAG, "Sanitized filename: %s", sanitized_filename.c_str());

            context.file_relative = join_paths(context.directory_relative, sanitized_filename);
            context.file_fs = build_fs_path(context.file_relative);
            ESP_LOGI(TAG, "File paths - relative: %s, fs: %s", context.file_relative.c_str(), context.file_fs.c_str());

            if (!is_within_root(context.file_fs)) {
                ESP_LOGE(TAG, "File initialization failed: Path outside root");
                context.error = UploadContext::Error::kFileOpenFailed;
                context.status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            struct stat existing = {0};
            if (stat(context.file_fs.c_str(), &existing) == 0) {
                ESP_LOGE(TAG, "File initialization failed: File already exists");
                context.error = UploadContext::Error::kFileExists;
                context.status = ESP_ERR_INVALID_STATE;
                return ESP_FAIL;
            }

            ESP_LOGI(TAG, "Opening file for writing: %s", context.file_fs.c_str());
            int fd = open(context.file_fs.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
            if (fd < 0) {
                ESP_LOGE(TAG, "Failed to open %s for writing: %s (errno: %d)", context.file_fs.c_str(), strerror(errno), errno);
                context.error = UploadContext::Error::kFileOpenFailed;
                context.status = ESP_FAIL;
                return ESP_FAIL;
            }
            context.file = fdopen(fd, "wb");
            if (!context.file) {
                ESP_LOGE(TAG, "fdopen failed for %s: %s (errno: %d)", context.file_fs.c_str(), strerror(errno), errno);
                close(fd);
                context.error = UploadContext::Error::kFileOpenFailed;
                context.status = ESP_FAIL;
                return ESP_FAIL;
            }

            context.initialized = true;
            ESP_LOGI(TAG, "File initialization complete");
        }

        if (len > 0) {
            if (context.total_bytes >= kMaxUploadBytes || len > (kMaxUploadBytes - context.total_bytes)) {
                ESP_LOGW(TAG, "Upload exceeds max size (%zu bytes)", kMaxUploadBytes);
                context.error = UploadContext::Error::kTooLarge;
                context.status = ESP_ERR_INVALID_SIZE;
                finalize_file();
                return ESP_FAIL;
            }
        }

        if (len > 0 || final) {
            ESP_LOGD(TAG, "Queueing %zu bytes for upload worker (total so far: %zu)", len, context.total_bytes);
            if (!upload_worker_enqueue_chunk(&context, data, len, final)) {
                ESP_LOGE(TAG, "Failed to enqueue upload chunk");
                context.error = UploadContext::Error::kWriteFailed;
                context.status = ESP_FAIL;
                finalize_file();
                return ESP_FAIL;
            }
            context.total_bytes += len;

            if (len > 0 && ((context.total_bytes % (64 * 1024)) == 0 || context.total_bytes < 1024)) {
                ESP_LOGI(TAG, "Upload progress: %zu bytes (%.1f%% of %zu MB limit)", context.total_bytes, (context.total_bytes * 100.0) / kMaxUploadBytes,
                         kMaxUploadBytes / (1024 * 1024));
            }
        }

        if (final) {
            ESP_LOGI(TAG, "Upload finalization started - total bytes: %zu", context.total_bytes);
            if (!upload_worker_wait_for_completion(&context)) {
                ESP_LOGE(TAG, "Upload worker failed to flush data");
                context.error = UploadContext::Error::kWriteFailed;
                context.status = ESP_FAIL;
                finalize_file();
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Upload finalization complete - total bytes: %zu", context.total_bytes);
            finalize_file();
        }

        return ESP_OK;
    });

    ESP_LOGI(TAG, "Starting upload processing with handleRequest");
    esp_err_t upload_result = uploader.handleRequest(psychic_request);
    ESP_LOGI(TAG, "handleRequest completed with result: %s", esp_err_to_name(upload_result));

    finalize_file();

    if (upload_result != ESP_OK || context.status != ESP_OK) {
        ESP_LOGE(TAG, "Upload failed - handleRequest: %s, context.status: %s", esp_err_to_name(upload_result), esp_err_to_name(context.status));

        if (!context.file_fs.empty()) {
            ESP_LOGI(TAG, "Cleaning up failed upload file: %s", context.file_fs.c_str());
            unlink(context.file_fs.c_str());
        }

        // Response is handled by the onRequest callback, just return the result
        web_server_notify_request_result(false, context.total_bytes);
        return upload_result;
    }

    ESP_LOGI(TAG, "Upload completed successfully - %zu bytes uploaded", context.total_bytes);
    web_server_notify_request_result(true, context.total_bytes);

    // Response is handled by the onRequest callback, just return success
    return ESP_OK;
}

// Native ESP upload implementation using file_upload.c
static esp_err_t upload_handler_native(PsychicRequest* psychic_request) {
    ESP_LOGI(TAG, "=== NATIVE UPLOAD HANDLER START ===");
    ESP_LOGI(TAG, "Upload handler called - method: %d, content-length: %d", psychic_request->method(), psychic_request->contentLength());

    if (psychic_request->method() != HTTP_POST) {
        ESP_LOGW(TAG, "Upload handler called with non-POST method: %d", psychic_request->method());
        return psychic_request->reply(405, "text/plain", "Method Not Allowed");
    }

    // Proactively manage connections to free resources
    ESP_LOGI(TAG, "Managing connections for upload");
    web_server_manage_connections(psychic_request, false);

    // Extract httpd_req_t from PsychicRequest
    httpd_req_t* httpd_req = psychic_request->request();
    if (!httpd_req) {
        ESP_LOGE(TAG, "Failed to get httpd_req_t from PsychicRequest");
        return ESP_ERR_INVALID_ARG;
    }

    // Create FileUploadRequest bridge adapter
    struct NativeUploadContext {
        PsychicRequest* psychic_req;
        httpd_req_t* httpd_req;
        FILE* file;
        char directory_relative[512];  // Increased buffer sizes to prevent truncation
        char directory_fs[512];
        char file_relative[512];
        char file_fs[512];
        char filename[256];
        size_t total_bytes;
        bool validation_done;
        bool file_initialized;
        esp_err_t status;
        std::map<std::string, std::string> headers;      // Persistent storage for header values
        std::map<std::string, std::string> form_fields;  // Storage for multipart form fields
    };

    static NativeUploadContext context = {0};
    context.psychic_req = psychic_request;
    context.httpd_req = httpd_req;
    context.status = ESP_OK;
    context.total_bytes = 0;
    context.validation_done = false;
    context.file_initialized = false;

    // FileUploadRequest operations implementation
    FileUploadRequestOps ops = {.content_length = [](void* ctx) -> size_t {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    size_t len = ctx_ptr->psychic_req->contentLength();
                                    ESP_LOGI(TAG, "Content-Length from PsychicRequest: %zu", len);
                                    return len;
                                },
                                .max_upload_size = [](void* ctx) -> size_t {
                                    return 10 * 1024 * 1024;  // 10 MiB limit
                                },
                                .load_params = [](void* ctx) -> void {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    // CRITICAL: Do NOT call loadParams() for multipart requests
                                    // as it consumes the request body, leaving nothing for multipart parsing
                                    ESP_LOGI(TAG, "Skipping loadParams() to preserve request body for multipart parsing");
                                },
                                .is_multipart = [](void* ctx) -> bool {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    return ctx_ptr->psychic_req->isMultipart();
                                },
                                .get_filename = [](void* ctx) -> const char* {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    return ctx_ptr->filename;
                                },
                                .get_header = [](void* ctx, const char* name) -> const char* {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    if (!ctx_ptr || !name) return NULL;

                                    // Check if we already have this header cached
                                    std::string header_name(name);
                                    auto it = ctx_ptr->headers.find(header_name);
                                    if (it != ctx_ptr->headers.end()) {
                                        return it->second.c_str();
                                    }

                                    // Fetch header from PsychicRequest and store persistently
                                    std::string header_value = ctx_ptr->psychic_req->header(name).c_str();
                                    if (header_value.empty()) {
                                        return NULL;  // Header doesn't exist
                                    }

                                    // Store in persistent map and return pointer to stored string
                                    ctx_ptr->headers[header_name] = header_value;
                                    return ctx_ptr->headers[header_name].c_str();
                                },
                                .get_httpd_req = [](void* ctx) -> httpd_req_t* {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    return ctx_ptr->httpd_req;
                                },
                                .reply = [](void* ctx, int code, const char* content_type, const char* content) -> esp_err_t {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    return ctx_ptr->psychic_req->reply(code, content_type, content);
                                },
                                .reply_text = [](void* ctx, const char* content) -> esp_err_t {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    return ctx_ptr->psychic_req->reply(200, "text/plain", content);
                                },
                                .add_form_field = [](void* ctx, const char* name, const char* value) -> esp_err_t {
                                    auto* ctx_ptr = static_cast<NativeUploadContext*>(ctx);
                                    if (ctx_ptr && name && value) {
                                        ctx_ptr->form_fields[std::string(name)] = std::string(value);
                                        ESP_LOGI(TAG, "Stored form field: %s = %s", name, value);

                                        // CRITICAL: Validate path immediately when received
                                        if (strcmp(name, "path") == 0) {
                                            esp_err_t path_err = upload_validate_path(value, ctx_ptr->directory_relative, ctx_ptr->directory_fs);
                                            if (path_err != ESP_OK) {
                                                ESP_LOGE(TAG, "Path validation failed: %s", value);
                                                ctx_ptr->status = path_err;
                                                return ESP_ERR_INVALID_ARG;
                                            }
                                            ESP_LOGI(TAG, "Path validated early - dir: %s", ctx_ptr->directory_relative);
                                        }
                                    }
                                    return ESP_OK;
                                },
                                .add_file_field = [](void* ctx, const char* name, const char* filename, size_t size) -> esp_err_t {
                                    // Not used in native implementation
                                    return ESP_OK;
                                }};

    FileUploadRequest request = {.ctx = &context, .ops = &ops, .current_file_size_hint = 0, .current_file_size_hint_valid = false};

    // Initialize FileUploadHandler
    FileUploadHandler handler;
    file_upload_handler_init(&handler);

    // Set up upload callback
    handler.upload_cb = [](FileUploadRequest* req, const char* filename, uint64_t index, const uint8_t* data, size_t len, bool final) -> esp_err_t {
        auto* ctx_ptr = static_cast<NativeUploadContext*>(req->ctx);

        ESP_LOGD(TAG, "Native upload callback - filename: %s, len: %zu, final: %s", filename, len, final ? "true" : "false");

        // Initialize file on first data chunk
        if (!ctx_ptr->file_initialized && len > 0) {
            ESP_LOGI(TAG, "Initializing native file upload");

            if (strlen(filename) == 0) {
                ESP_LOGE(TAG, "File initialization failed: Missing filename");
                ctx_ptr->status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            // Sanitize filename using shared helper
            char safe_filename[256];
            esp_err_t filename_err = upload_sanitize_filename(filename, safe_filename);
            if (filename_err != ESP_OK) {
                ESP_LOGE(TAG, "File initialization failed: Invalid filename");
                ctx_ptr->status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            strcpy(ctx_ptr->filename, safe_filename);

            // Build file paths with bounds checking
            int result1 = snprintf(ctx_ptr->file_relative, sizeof(ctx_ptr->file_relative), "%s/%s", ctx_ptr->directory_relative, safe_filename);
            if (result1 >= (int)sizeof(ctx_ptr->file_relative)) {
                ESP_LOGE(TAG, "File relative path too long");
                ctx_ptr->status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            int result2 = snprintf(ctx_ptr->file_fs, sizeof(ctx_ptr->file_fs), "/sdcard%s", ctx_ptr->file_relative);
            if (result2 >= (int)sizeof(ctx_ptr->file_fs)) {
                ESP_LOGE(TAG, "File filesystem path too long");
                ctx_ptr->status = ESP_ERR_INVALID_ARG;
                return ESP_FAIL;
            }

            // Check if file already exists using shared helper
            if (upload_check_file_exists(ctx_ptr->file_fs)) {
                ESP_LOGE(TAG, "File initialization failed: File already exists");
                ctx_ptr->status = ESP_ERR_INVALID_STATE;
                return ESP_FAIL;
            }

            ESP_LOGI(TAG, "Opening file for writing: %s", ctx_ptr->file_fs);
            ctx_ptr->file = fopen(ctx_ptr->file_fs, "wb");
            if (!ctx_ptr->file) {
                ESP_LOGE(TAG, "Failed to open %s for writing: %s", ctx_ptr->file_fs, strerror(errno));
                ctx_ptr->status = ESP_FAIL;
                return ESP_FAIL;
            }

            ctx_ptr->file_initialized = true;
            ESP_LOGI(TAG, "Native file initialization complete");
        }

        // Write data to file
        if (len > 0 && ctx_ptr->file) {
            size_t written = fwrite(data, 1, len, ctx_ptr->file);
            if (written != len) {
                ESP_LOGE(TAG, "Failed to write %zu bytes to file (wrote %zu)", len, written);
                ctx_ptr->status = ESP_FAIL;
                return ESP_FAIL;
            }
            ctx_ptr->total_bytes += len;

            if ((ctx_ptr->total_bytes % (64 * 1024)) == 0 || ctx_ptr->total_bytes < 1024) {
                ESP_LOGI(TAG, "Native upload progress: %zu bytes", ctx_ptr->total_bytes);
            }
        }

        // Finalize file
        if (final && ctx_ptr->file) {
            ESP_LOGI(TAG, "Native upload finalization - total bytes: %zu", ctx_ptr->total_bytes);
            fflush(ctx_ptr->file);
            fsync(fileno(ctx_ptr->file));
            fclose(ctx_ptr->file);
            ctx_ptr->file = nullptr;
        }

        return ESP_OK;
    };

    // Set up request callback for response
    handler.request_cb = [](FileUploadRequest* req) -> esp_err_t {
        auto* ctx_ptr = static_cast<NativeUploadContext*>(req->ctx);

        // Perform validation after multipart parsing is complete
        ESP_LOGI(TAG, "Performing native upload validation after multipart parsing");

        // Extract form fields from multipart data
        auto path_it = ctx_ptr->form_fields.find("path");
        auto token_it = ctx_ptr->form_fields.find("csrf_token");

        if (path_it == ctx_ptr->form_fields.end()) {
            ESP_LOGE(TAG, "Validation failed: Missing path parameter");
            return ctx_ptr->psychic_req->reply(400, "text/plain", "Missing path parameter");
        }

        if (token_it == ctx_ptr->form_fields.end()) {
            ESP_LOGE(TAG, "Validation failed: Missing CSRF token");
            return ctx_ptr->psychic_req->reply(400, "text/plain", "Missing CSRF token");
        }

        // Validate CSRF token using shared helper
        std::string cookie_header = std::string(ctx_ptr->psychic_req->header("Cookie").c_str());
        if (!upload_validate_csrf_token(cookie_header.c_str(), token_it->second.c_str())) {
            ESP_LOGE(TAG, "Validation failed: CSRF token mismatch");
            return ctx_ptr->psychic_req->reply(400, "text/plain", "CSRF token mismatch");
        }

        // Path validation was already done in add_form_field callback
        // Just verify directory exists using shared helper
        esp_err_t dir_err = upload_validate_directory(ctx_ptr->directory_fs);
        if (dir_err != ESP_OK) {
            ESP_LOGE(TAG, "Validation failed: Directory not found: %s", ctx_ptr->directory_fs);
            return ctx_ptr->psychic_req->reply(400, "text/plain", "Directory not found");
        }

        ESP_LOGI(TAG, "Native upload validation complete - directory: %s", ctx_ptr->directory_relative);

        if (ctx_ptr->status != ESP_OK) {
            ESP_LOGE(TAG, "Native upload failed with status: %s", esp_err_to_name(ctx_ptr->status));
            return ctx_ptr->psychic_req->reply(500, "text/plain", "Upload failed");
        }

        // Success - redirect to the directory
        ESP_LOGI(TAG, "Native upload successful, redirecting to: %s", ctx_ptr->directory_relative);
        std::string redirect_path = "/?path=" + url_encode(ctx_ptr->directory_relative);
        PsychicResponse response(ctx_ptr->psychic_req);
        response.setCode(303);
        response.addHeader("Location", redirect_path.c_str());
        response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
        response.setContent("");
        return response.send();
    };

    ESP_LOGI(TAG, "Starting native upload processing");
    esp_err_t upload_result = file_upload_handler_handle_request(&handler, &request);
    ESP_LOGI(TAG, "Native upload processing completed with result: %s", esp_err_to_name(upload_result));

    // Cleanup
    file_upload_handler_deinit(&handler);

    if (context.file) {
        fclose(context.file);
        context.file = nullptr;
    }

    if (upload_result != ESP_OK || context.status != ESP_OK) {
        ESP_LOGE(TAG, "Native upload failed - result: %s, status: %s", esp_err_to_name(upload_result), esp_err_to_name(context.status));

        if (strlen(context.file_fs) > 0) {
            ESP_LOGI(TAG, "Cleaning up failed upload file: %s", context.file_fs);
            unlink(context.file_fs);
        }

        web_server_notify_request_result(false, context.total_bytes);
        return upload_result;
    }

    ESP_LOGI(TAG, "Native upload completed successfully - %zu bytes uploaded", context.total_bytes);
    web_server_notify_request_result(true, context.total_bytes);

    return ESP_OK;
}

// Simple test handler for debugging
extern "C" esp_err_t web_server_test_handler(void* request) {
    if (!request) {
        return ESP_ERR_INVALID_ARG;
    }

    auto* psychic_request = static_cast<PsychicRequest*>(request);

    ESP_LOGI(TAG, "Test handler called - server is responding!");
    ESP_LOGI(TAG, "Request URI: %s", psychic_request->uri().c_str());
    ESP_LOGI(TAG, "Request method: %d", psychic_request->method());
    ESP_LOGI(TAG, "Available heap during request: %d bytes", esp_get_free_heap_size());

    const char* response =
        "<!DOCTYPE html><html><head><title>ESP32 Web Server Test</title></head>"
        "<body><h1>ESP32 Web Server is Working!</h1>"
        "<p>Server is responding correctly.</p>"
        "<p>Time: " __DATE__ " " __TIME__
        "</p>"
        "<p>Stack size: 8KB</p>"
        "</body></html>";

    esp_err_t result = psychic_request->reply(200, "text/html", response);
    ESP_LOGI(TAG, "Test handler response result: %s", esp_err_to_name(result));
    return result;
}

// Send one memory segment with short backoff, keeping session alive.
// Does NOT reallocate or change user's buffer. Returns ESP_OK on success.
static esp_err_t send_segment_with_backpressure(PsychicResponse& response, httpd_handle_t hd, httpd_req_t* raw_req, uint8_t* data, size_t len,
                                                uint32_t per_segment_time_limit_ms,  // e.g. 30000
                                                uint32_t* io_retry_counter_out       // accumulated retry counter
) {
    const int64_t t0 = esp_timer_get_time();
    size_t tries = 0;
    esp_err_t err = ESP_OK;
    int last_errno = 0;

    while ((err = response.sendChunk(data, len)) != ESP_OK) {
        last_errno = errno;
        // Only treat EAGAIN/ENOMEM as backpressure; other errors are fatal
        if (err == ESP_ERR_HTTPD_RESP_SEND && (last_errno == EAGAIN || last_errno == ENOMEM)) {
            ++tries;
            if (io_retry_counter_out) (*io_retry_counter_out)++;
            // keep the session hot in LRU while we wait
            int fd = httpd_req_to_sockfd(raw_req);
            if (hd && fd >= 0) httpd_sess_update_lru_counter(hd, fd);
            // small exponential backoff capped at 200 ms
            uint32_t backoff_ms = 8u << (tries < 5 ? tries : 5);
            if (backoff_ms > 200) backoff_ms = 200;
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));

            // time budget per segment
            int64_t dt_us = esp_timer_get_time() - t0;
            if ((uint64_t)dt_us > (uint64_t)per_segment_time_limit_ms * 1000ULL) {
                ESP_LOGE(TAG, "send_segment timeout after %zu tries (errno=%d)", tries, last_errno);
                return ESP_ERR_TIMEOUT;
            }
            continue;  // retry same data pointer/len
        }
        // fatal send error (peer reset/closed/etc.)
        ESP_LOGE(TAG, "send_segment fatal send error: errno=%d err=%s", last_errno, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

extern "C" esp_err_t web_server_file_manager_handler(void* request) {
    if (!request) {
        return ESP_ERR_INVALID_ARG;
    }

    auto* psychic_request = static_cast<PsychicRequest*>(request);
    psychic_request->loadParams();

    // Proactively free up sockets from stale clients on page load/refresh
    web_server_manage_connections(psychic_request, false);

    std::string requested_path = "/";
    if (psychic_request->hasParam("path")) {
        PsychicWebParameter* param = psychic_request->getParam("path");
        if (param) {
            requested_path = std::string(param->value().c_str());
        }
    }

    // Get pagination parameters
    int page = 1;
    if (psychic_request->hasParam("page")) {
        PsychicWebParameter* page_param = psychic_request->getParam("page");
        if (page_param) {
            const char* page_str = page_param->value().c_str();
            int parsed_page = atoi(page_str);
            if (parsed_page > 0) {
                page = parsed_page;
            }
        }
    }

    const int ITEMS_PER_PAGE = 20;

    std::string sanitized = sanitize_path(requested_path);
    std::string fs_path = build_fs_path(sanitized);

    if (!is_within_root(fs_path)) {
        ESP_LOGW(TAG, "Rejected path outside root: %s", requested_path.c_str());
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Invalid path");
    }

    DIR* dir = opendir(fs_path.c_str());
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s: %s", fs_path.c_str(), strerror(errno));
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(404, "text/plain", "Directory not found");
    }

    // Collect all directory entries into a vector for sorting
    std::vector<DirectoryEntry> entries;
    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
        const char* entry_name = entry->d_name;
        if (!entry_name) {
            continue;
        }
        if (strcmp(entry_name, ".") == 0 || strcmp(entry_name, "..") == 0) {
            continue;
        }

        // Get file stats to determine if it's a directory and get size
        std::string child_relative = join_paths(sanitized, entry_name);
        std::string child_fs_path = build_fs_path(child_relative);

        struct stat st = {0};
        if (stat(child_fs_path.c_str(), &st) != 0) {
            ESP_LOGW(TAG, "Failed to stat %s: %s", child_fs_path.c_str(), strerror(errno));
            continue;
        }

        bool is_dir = S_ISDIR(st.st_mode);
        size_t file_size = static_cast<size_t>(st.st_size);

        entries.emplace_back(entry_name, is_dir, file_size);
    }
    closedir(dir);

    // Sort entries using custom comparator
    std::sort(entries.begin(), entries.end(), compare_directory_entries);

    // Calculate pagination bounds
    int total_items = static_cast<int>(entries.size());
    int total_pages = (total_items + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    if (total_pages <= 0) {
        total_pages = 1;
    }
    if (page < 1) {
        page = 1;
    } else if (page > total_pages) {
        page = total_pages;
    }
    bool has_prev_page = (page > 1);
    bool has_next_page = (page < total_pages);

    // Simple pagination: just show items for current page
    // Skip items before current page, show ITEMS_PER_PAGE items
    int items_to_skip = (page - 1) * ITEMS_PER_PAGE;
    int items_shown = 0;

    PsychicResponse response(psychic_request);
    response.setCode(200);
    response.setContentType("text/html; charset=UTF-8");
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");

    std::string csrf_token = generate_csrf_token();
    set_csrf_cookie(response, csrf_token);

    uint8_t buffer[kChunkBufferSize];
    ChunkPrinter printer(&response, buffer, sizeof(buffer));
    response.sendHeaders();

    size_t total_bytes = 0;
    auto append_cstr = [&](const char* text) { total_bytes += printer.print(text); };
    auto append_string = [&](const std::string& text) { total_bytes += printer.print(text.c_str()); };

    append_cstr("<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>File Manager</title>");
    append_cstr(
        "<style>body{font-family:sans-serif;margin:16px;}table{width:100%;border-collapse:collapse;}th,td{padding:8px;border-bottom:1px solid #3333;}"
        "th{text-align:left;}td a{color:#0070f3;text-decoration:none;}td a:hover{text-decoration:underline;}"
        ".pagination{margin:20px 0;text-align:center;}.pagination a{margin:0 5px;padding:8px 12px;text-decoration:none;border:1px solid "
        "#ddd;border-radius:4px;}"
        ".pagination a:hover{background-color:#f5f5f5;}.pagination .current{background-color:#0070f3;color:white;border-color:#0070f3;}"
        ".pagination .disabled{color:#ccc;cursor:not-allowed;}"
        ".delete-link{color:#dc3545;text-decoration:none;margin-left:8px;}"
        ".delete-link:hover{text-decoration:underline;color:#c82333;}</style>");

    // Add JavaScript for delete confirmation
    append_cstr(
        "<script>"
        "function confirmDelete(path, isDir) {"
        "var itemType = isDir ? 'folder' : 'file';"
        "var message = 'Are you sure you want to delete this ' + itemType + '?\\n\\n' + path;"
        "if (isDir) {"
        "    message += '\\n\\nWARNING: This will delete the entire folder and all its contents!';"
        "}"
        "message += '\\n\\nThis action cannot be undone.';"
        "return confirm(message);"
        "}"
        "function confirmDeleteAll() {"
        "var message = '⚠️ DANGER ZONE ⚠️\\n\\n' +"
        "'Are you absolutely sure you want to delete ALL files and folders on the SD card?\\n\\n' +"
        "'Type YES to confirm deletion:';"
        "var userInput = prompt(message);"
        "return userInput === 'YES';"
        "}"
        "</script></head><body>");

    append_cstr("<h1>Files in ");
    append_string(html_escape(sanitized));
    append_cstr("</h1>");

    // Show pagination info with proper bounds
    append_cstr("<p>Page ");
    append_string(std::to_string(page));
    append_cstr(" of ");
    append_string(std::to_string(total_pages));
    append_cstr(" - Showing ");
    append_string(std::to_string(total_items));
    append_cstr(" total items</p>");

    append_cstr("<hr><table>");
    append_cstr("<tr><th>Name</th><th>Size</th><th>Actions</th><th>Delete</th></tr>");

    if (sanitized != "/") {
        append_cstr("<tr><td><a href=\"/?path=");
        append_string(url_encode(parent_path(sanitized)));
        append_cstr("\">.. (Up)</a></td><td></td><td></td><td></td></tr>");
    }

    // Display items for the current page from sorted vector
    for (size_t i = 0; i < entries.size(); ++i) {
        const DirectoryEntry& entry = entries[i];

        // Skip items before the current page
        if (items_to_skip > 0) {
            items_to_skip--;
            continue;
        }

        // Stop if we've shown enough items for this page
        if (items_shown >= ITEMS_PER_PAGE) {
            break;
        }

        std::string child_relative = join_paths(sanitized, entry.name);
        std::string child_fs_path = build_fs_path(child_relative);

        append_cstr("<tr><td>");
        std::string display_name = html_escape(entry.name);
        if (entry.is_directory) {
            append_cstr("<a href=\"/?path=");
            append_string(url_encode(child_relative));
            append_cstr("\">");
            append_string(display_name);
            append_cstr("/</a>");
        } else {
            append_string(display_name);
        }
        append_cstr("</td><td>");
        if (entry.is_directory) {
            append_cstr("-");
        } else {
            std::string formatted_size = format_file_size(entry.file_size);
            total_bytes += printer.print(formatted_size.c_str());
        }
        append_cstr("</td><td>");
        if (entry.is_directory) {
            append_cstr("&nbsp;");
        } else {
            append_cstr("<a href=\"/download?path=");
            append_string(url_encode(child_relative));
            append_cstr("\">Download</a>");
        }
        append_cstr("</td><td>");
        // Add delete link with confirmation
        append_cstr("<a href=\"#\" onclick=\"if(confirmDelete('");
        append_string(html_escape(entry.name));
        append_cstr("', ");
        append_cstr(entry.is_directory ? "true" : "false");
        append_cstr(")) { window.location.href='/delete?path=");
        append_string(url_encode(child_relative));
        append_cstr("&csrf_token=");
        append_string(csrf_token);
        append_cstr("'; } return false;\" class=\"delete-link\">Delete</a>");
        append_cstr("</td></tr>");

        items_shown++;
    }

    append_cstr("</table>");

    // Add pagination controls with proper bounds checking
    append_cstr("<div class=\"pagination\">");

    // Previous button - only show if there is a previous page
    if (has_prev_page) {
        append_cstr("<a href=\"/?path=");
        append_string(url_encode(sanitized));
        append_cstr("&page=");
        append_string(std::to_string(page - 1));
        append_cstr("\">&laquo; Previous</a>");
    } else {
        append_cstr("<span class=\"disabled\">&laquo; Previous</span>");
    }

    // Current page
    append_cstr("<span class=\"current\">Page ");
    append_string(std::to_string(page));
    append_cstr(" of ");
    append_string(std::to_string(total_pages));
    append_cstr("</span>");

    // Next button - only show if there is a next page
    if (has_next_page) {
        append_cstr("<a href=\"/?path=");
        append_string(url_encode(sanitized));
        append_cstr("&page=");
        append_string(std::to_string(page + 1));
        append_cstr("\">Next &raquo;</a>");
    } else {
        append_cstr("<span class=\"disabled\">Next &raquo;</span>");
    }

    append_cstr("</div>");

    append_cstr("<hr><h3>Upload File</h3>");
    append_cstr("<form method=\"POST\" action=\"/upload\" enctype=\"multipart/form-data\">");
    append_cstr("<input type=\"hidden\" name=\"path\" value=\"");
    append_string(html_escape(sanitized));
    append_cstr("\">");
    append_cstr("<input type=\"hidden\" name=\"csrf_token\" value=\"");
    append_string(csrf_token);
    append_cstr("\">");
    append_cstr("<input type=\"file\" name=\"file_to_upload\"><br>");
    append_cstr("<input type=\"submit\" value=\"Upload\">");
    append_cstr("</form>");

    // Add Delete All button (only show on root directory)
    if (sanitized == "/") {
        append_cstr("<hr><h3>Danger Zone</h3>");
        append_cstr("<p style=\"color:#dc3545;font-weight:bold;\">⚠️ WARNING: This will delete ALL files and folders on the SD card!</p>");
        append_cstr("<p>The <code>configuration.ini</code> file will be preserved.</p>");
        append_cstr("<button onclick=\"if(confirmDeleteAll()) { window.location.href='/delete-all?csrf_token=");
        append_string(csrf_token);
        append_cstr("'; }\" ");
        append_cstr("style=\"background-color:#dc3545;color:white;border:none;padding:10px 20px;border-radius:4px;cursor:pointer;font-weight:bold;\">");
        append_cstr("🗑️ DELETE ALL FILES</button>");
    }

    append_cstr("</body></html>");

    printer.flush();
    esp_err_t err = response.finishChunking();

    if (err == ESP_OK) {
        web_server_notify_request_result(true, total_bytes);
    } else {
        web_server_notify_request_result(false, total_bytes);
    }

    return err;
}

extern "C" esp_err_t web_server_upload_handler(void* request) {
    if (!request) {
        ESP_LOGE(TAG, "Upload handler called with NULL request");
        return ESP_ERR_INVALID_ARG;
    }

    auto* psychic_request = static_cast<PsychicRequest*>(request);

    // Get configuration from global module
    extern web_server_module_t g_web_server_module;
    bool use_native = g_web_server_module.use_native_upload;

    ESP_LOGI(TAG, "Upload handler dispatch - using %s upload", use_native ? "Native ESP" : "PsychicHttp");

    if (use_native) {
        return upload_handler_native(psychic_request);
    } else {
        // Alternative: PsychicHttp upload (original implementation)
        return upload_handler_psychic(psychic_request);
    }
}

extern "C" esp_err_t web_server_delete_handler(void* request) {
    if (!request) {
        return ESP_ERR_INVALID_ARG;
    }

    auto* psychic_request = static_cast<PsychicRequest*>(request);
    psychic_request->loadParams();

    // Proactively free up sockets from stale clients
    web_server_manage_connections(psychic_request, false);

    PsychicWebParameter* path_param = psychic_request->getParam("path");
    PsychicWebParameter* token_param = psychic_request->getParam("csrf_token");

    // CSRF validation
    if (!token_param) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(403, "text/plain", "CSRF token missing");
    }
    std::string cookie = std::string(psychic_request->header("Cookie").c_str());
    std::string cookie_token = get_cookie_value(cookie, "csrf_token");
    if (cookie_token.empty() || cookie_token != std::string(token_param->value().c_str())) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(403, "text/plain", "CSRF validation failed");
    }

    if (!path_param) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Missing path parameter");
    }

    std::string requested_path = std::string(path_param->value().c_str());
    std::string sanitized = sanitize_path(requested_path);
    if (sanitized == "/") {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Cannot delete root directory");
    }

    // Additional security check: prevent deletion of system directories
    if (sanitized.find("/system") == 0 || sanitized.find("/config") == 0) {
        ESP_LOGW(TAG, "Blocked attempt to delete system directory: %s", sanitized.c_str());
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(403, "text/plain", "Cannot delete system directories");
    }

    std::string fs_path = build_fs_path(sanitized);
    if (!is_within_root(fs_path)) {
        ESP_LOGW(TAG, "Rejected delete outside root: %s", requested_path.c_str());
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Invalid file path");
    }

    struct stat st = {0};
    if (stat(fs_path.c_str(), &st) != 0) {
        ESP_LOGE(TAG, "Failed to stat %s: %s", fs_path.c_str(), strerror(errno));
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(404, "text/plain", "File or directory not found");
    }

    bool is_directory = S_ISDIR(st.st_mode);
    esp_err_t delete_result = ESP_OK;

    if (is_directory) {
        // Delete directory recursively
        ESP_LOGI(TAG, "Deleting directory: %s", fs_path.c_str());
        delete_result = delete_directory_recursive(fs_path.c_str());
    } else {
        // Delete single file
        ESP_LOGI(TAG, "Deleting file: %s", fs_path.c_str());
        if (unlink(fs_path.c_str()) != 0) {
            ESP_LOGE(TAG, "Failed to delete file %s: %s", fs_path.c_str(), strerror(errno));
            delete_result = ESP_FAIL;
        }
    }

    if (delete_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete %s", fs_path.c_str());
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(500, "text/plain", "Failed to delete file or directory");
    }

    ESP_LOGI(TAG, "Successfully deleted: %s", fs_path.c_str());
    web_server_notify_request_result(true, 0);

    // Redirect back to the parent directory
    std::string parent_dir = parent_path(sanitized);
    std::string redirect_path = "/?path=" + url_encode(parent_dir);
    PsychicResponse response(psychic_request);
    response.setCode(303);
    response.addHeader("Location", redirect_path.c_str());
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response.setContent("");
    return response.send();
}

extern "C" esp_err_t web_server_delete_all_handler(void* request) {
    if (!request) {
        return ESP_ERR_INVALID_ARG;
    }

    auto* psychic_request = static_cast<PsychicRequest*>(request);
    psychic_request->loadParams();

    // Proactively free up sockets from stale clients
    web_server_manage_connections(psychic_request, false);

    // CSRF validation
    PsychicWebParameter* token_param = psychic_request->getParam("csrf_token");
    if (!token_param) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(403, "text/plain", "CSRF token missing");
    }
    std::string cookie = std::string(psychic_request->header("Cookie").c_str());
    std::string cookie_token = get_cookie_value(cookie, "csrf_token");
    if (cookie_token.empty() || cookie_token != std::string(token_param->value().c_str())) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(403, "text/plain", "CSRF validation failed");
    }

    ESP_LOGI(TAG, "Delete All request received - deleting all files except configuration.ini");
    ESP_LOGI(TAG, "This operation may take some time depending on the number of files...");

    std::string root_path = build_fs_path("/");
    esp_err_t delete_result = delete_all_except_config(root_path.c_str());

    if (delete_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete all files: %s", esp_err_to_name(delete_result));
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(500, "text/plain", "Failed to delete all files");
    }

    ESP_LOGI(TAG, "Successfully deleted all files except configuration.ini");
    web_server_notify_request_result(true, 0);

    // Redirect back to root directory
    PsychicResponse response(psychic_request);
    response.setCode(303);
    response.addHeader("Location", "/");
    response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    response.setContent("");
    return response.send();
}

extern "C" esp_err_t web_server_download_handler(void* request) {
    if (!request) {
        return ESP_ERR_INVALID_ARG;
    }

    auto* psychic_request = static_cast<PsychicRequest*>(request);
    psychic_request->loadParams();

    // Aggressively manage connections for downloads to preserve memory
    ESP_LOGI(TAG, "Download request starting - managing connections");
    web_server_manage_connections(psychic_request, true);

    PsychicWebParameter* path_param = psychic_request->getParam("path");
    if (!path_param) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Missing path parameter");
    }

    std::string requested_path = std::string(path_param->value().c_str());
    std::string sanitized = sanitize_path(requested_path);
    if (sanitized == "/") {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Invalid file path");
    }

    std::string fs_path = build_fs_path(sanitized);
    if (!is_within_root(fs_path)) {
        ESP_LOGW(TAG, "Rejected download outside root: %s", requested_path.c_str());
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Invalid file path");
    }

    struct stat st = {0};
    if (stat(fs_path.c_str(), &st) != 0) {
        ESP_LOGE(TAG, "Failed to stat %s: %s", fs_path.c_str(), strerror(errno));
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(404, "text/plain", "File not found");
    }

    if (!S_ISREG(st.st_mode)) {
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(400, "text/plain", "Requested path is not a file");
    }

    FILE* file = fopen(fs_path.c_str(), "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file %s: %s", fs_path.c_str(), strerror(errno));
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(404, "text/plain", "File not found");
    }

    PsychicResponse response(psychic_request);
    response.setCode(200);

    std::string filename;
    size_t last_slash = sanitized.find_last_of('/');
    if (last_slash == std::string::npos) {
        filename = sanitized;
    } else {
        filename = sanitized.substr(last_slash + 1);
    }

    // Set appropriate MIME type based on file extension
    std::string mime_type = "application/octet-stream";
    if (filename.find(".csv") != std::string::npos) {
        mime_type = "text/csv";
    } else if (filename.find(".txt") != std::string::npos || filename.find(".log") != std::string::npos) {
        mime_type = "text/plain";
    } else if (filename.find(".json") != std::string::npos) {
        mime_type = "application/json";
    } else if (filename.find(".xml") != std::string::npos) {
        mime_type = "application/xml";
    }
    response.setContentType(mime_type.c_str());

    // Set Content-Disposition header to force download
    std::string disposition = "attachment; filename=\"" + safe_filename(filename) + "\"";
    response.addHeader("Content-Disposition", disposition.c_str());

    // Add performance optimization headers
    response.addHeader("Cache-Control", "no-cache, no-store, must-revalidate");  // Don't cache downloads
    // Range requests are not yet implemented, so omit Accept-Ranges to avoid advertising unsupported features
    // Note: Don't set Content-Length when using chunked transfer encoding

    response.sendHeaders();
    ESP_LOGI(TAG, "Download headers sent for %s (MIME: %s, Size: %zu bytes)", filename.c_str(), mime_type.c_str(), st.st_size);

    // Small delay to ensure headers are processed by the client
    vTaskDelay(pdMS_TO_TICKS(10));
    // Get socket file descriptor for diagnostics
    int fd = httpd_req_to_sockfd(psychic_request->request());
    int64_t t0 = esp_timer_get_time();
    bool chunk_started = false;
    ESP_LOGI(TAG, "=== DOWNLOAD STARTING ===");
    ESP_LOGI(TAG, "File: %s", filename.c_str());
    ESP_LOGI(TAG, "Size: %zu bytes (%.1f MB)", st.st_size, st.st_size / (1024.0f * 1024.0f));
    ESP_LOGI(TAG, "Socket FD: %d", fd);
    log_heap_diag("download-begin");

    bool expected = false;
    if (!s_download_in_progress.compare_exchange_strong(expected, true, std::memory_order_acq_rel, std::memory_order_relaxed)) {
        ESP_LOGW(TAG, "Download already in progress - rejecting %s", filename.c_str());
        fclose(file);
        web_server_notify_request_result(false, 0);
        return psychic_request->reply(429, "text/plain", "Another download is already in progress");
    }
    DownloadSessionGuard download_guard(s_download_in_progress);

    // Initialize heap trace if enabled
    bool heap_trace_active = false;
#ifdef CONFIG_HEAP_TRACING
    // Start heap trace for this download session (if tracing is enabled)
    static heap_trace_record_t s_trace_records[128];
    if (heap_trace_init_standalone(s_trace_records, 128) == ESP_OK) {
        if (heap_trace_start(HEAP_TRACE_LEAKS) == ESP_OK) {
            heap_trace_active = true;
            ESP_LOGI(TAG, "Heap trace started for download session");
        }
    }
#endif

    size_t total_bytes = 0;
    // Use configured safe chunk size; cap to buffer limits
    size_t chunk_size = get_safe_download_chunk_size();
    if (chunk_size > kDownloadBufferSize) chunk_size = kDownloadBufferSize;
    if (chunk_size < 1024) chunk_size = 1024;

    // Check if download buffer is allocated
    if (s_download_buffer == nullptr) {
        ESP_LOGE(TAG, "Download buffer not allocated - server may not be started");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t* chunk_buffer = s_download_buffer;
    ESP_LOGI(TAG, "Using %zu-byte download chunks from dynamic buffer", chunk_size);

    esp_err_t err = ESP_OK;
    size_t file_size = st.st_size;
    size_t bytes_sent = 0;
    uint32_t consecutive_blocked_sends = 0;
    uint32_t total_retry_attempts = 0;
    const uint32_t max_total_retries = 40;
    size_t chunk_index = 0;
    uint64_t last_progress_log_us = esp_timer_get_time();
    size_t last_progress_log_bytes = 0;
    // Flow control and timeout management

    ESP_LOGI(TAG, "Starting download: %s (%zu bytes)", filename.c_str(), file_size);

    // Add timeout protection for large files - set to 1 hour
    uint64_t start_time = esp_timer_get_time();
    const uint64_t max_download_time_us = 3600000000;          // 1 hour (3600 seconds)
    bool timeout_protection = (file_size > 10 * 1024 * 1024);  // Only for files > 10MB

    while (err == ESP_OK && bytes_sent < file_size) {
        // 1) Загальний таймаут
        if (timeout_protection) {
            uint64_t elapsed = esp_timer_get_time() - start_time;
            if (elapsed > max_download_time_us) {
                ESP_LOGE(TAG, "Download timeout after %llu seconds for file %s", elapsed / 1000000, filename.c_str());
                err = ESP_ERR_TIMEOUT;
                break;
            }
        }

        // 2) Чекаємо готовність сокета (back-pressure)
        if (!wait_socket_writable(fd, /*ms*/ 1500)) {
            ++consecutive_blocked_sends;
            total_retry_attempts++;
            ESP_LOGW(TAG, "send() back-pressured (select timeout), tries=%u", consecutive_blocked_sends);
            if (total_retry_attempts > max_total_retries) {
                err = ESP_ERR_TIMEOUT;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;  // повторити перевірку
        } else {
            consecutive_blocked_sends = 0;
        }

        // 3) Якщо купа просіла — даємо стеку «видихнути»
        size_t current_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        if (current_heap < 10000) {
            ESP_LOGW(TAG, "Low heap during send (%u), waiting drain…", (unsigned)current_heap);
            wait_heap_recovers(/*min_safe*/ 12000, /*step*/ 10, /*max*/ 500);
        }

        // 4) Читаємо з диска
        size_t remaining = file_size - bytes_sent;
        size_t read_size = (remaining < chunk_size) ? remaining : chunk_size;

        // Periodic pre-send debug (lightweight): every 128 chunks
        if ((chunk_index % 128U) == 0U) {
            ESP_LOGD(TAG, "[pre-send] chunk=%zu offset=%zu read_size=%zu retries=%u", chunk_index, bytes_sent, read_size, total_retry_attempts);
        }

        size_t read_bytes = fread(chunk_buffer, 1, read_size, file);
        if (read_bytes > 0) {
            esp_err_t s = response.sendChunk(chunk_buffer, read_bytes);
            chunk_started = true;
            if (s == ESP_OK) {
                bytes_sent += read_bytes;
                total_bytes += read_bytes;
                ++chunk_index;
                // «повільне» збільшення темпу після успіхів: нічого не realoc'имо — лише скорочуємо паузу
                // (залишено простим, щоб уникнути фрагментації та повторних malloc)
            } else if (s == ESP_ERR_HTTPD_RESP_SEND) {
                ++total_retry_attempts;
                if (total_retry_attempts > max_total_retries) {
                    err = ESP_ERR_TIMEOUT;
                    break;
                }
                // backoff перед наступним select()
                vTaskDelay(pdMS_TO_TICKS(25));
            } else {
                err = s;
                break;
            }
        } else if (feof(file)) {
            break;
        } else {
            ESP_LOGE(TAG, "Failed to read from file during download");
            err = ESP_FAIL;
            break;
        }

        // Heartbeat log every ~1s even without milestones
        uint64_t now_us = esp_timer_get_time();
        if (now_us - last_progress_log_us >= 1000000ULL || bytes_sent - last_progress_log_bytes > 512 * 1024) {
            last_progress_log_us = now_us;
            last_progress_log_bytes = bytes_sent;
            ESP_LOGI(TAG, "[heartbeat] sent=%zu/%zu bytes (%.1f%%) chunks=%zu cur_chunk=%zu retries=%u | HEAP free8=%u min8=%u", bytes_sent, file_size,
                     (bytes_sent * 100.0) / file_size, chunk_index, chunk_size, total_retry_attempts, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
        }
    }

    // --- Finalize / cleanup ---------------------------------------------------
    fclose(file);
    if (chunk_started) {
        // гарантуємо коректне завершення chunked незалежно від коду помилки
        response.finishChunking();
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Download complete: %s", filename.c_str());
        web_server_notify_request_result(true, total_bytes);
    } else {
        ESP_LOGE(TAG, "Download failed: %s (%zu bytes sent before error) - Error: %s", filename.c_str(), bytes_sent, esp_err_to_name(err));
        web_server_notify_request_result(false, bytes_sent);
    }

    // Heap tracing cleanup
#ifdef CONFIG_HEAP_TRACING
    if (heap_trace_active) {
        heap_trace_stop();
        ESP_LOGW(TAG, "Heap trace dump for download session:");
        heap_trace_dump();
    }
#endif

    // CRITICAL: Restore normal connection management after download completes
    // This cleans up closed sessions and allows new connections to be accepted
    ESP_LOGI(TAG, "Download finished - restoring normal connection management");
    web_server_manage_connections(psychic_request, false);

    // Ensure function always returns
    return err;
}

#endif  // defined(USE_WEB_FILE_SEREVER)

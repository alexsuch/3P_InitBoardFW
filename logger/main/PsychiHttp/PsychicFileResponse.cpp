#include "PsychicFileResponse.h"

#include "PsychicRequest.h"
#include "PsychicResponse.h"

PsychicFileResponse::PsychicFileResponse(PsychicRequest *request, FS &fs, const String &path, const String &contentType, bool download)
    : PsychicResponse(request) {
    //_code = 200;
    String _path(path);

    if (!download && !fs.exists(_path) && fs.exists(_path + ".gz")) {
        _path = _path + ".gz";
        addHeader("Content-Encoding", "gzip");
        addHeader("Vary", "Accept-Encoding");
    }

    _content = fs.open(_path, "r");
    if (!_content) {
        _contentLength = 0;
        return;  // File open failed, will be handled in send()
    }
    _contentLength = _content.size();

    if (contentType == "")
        _setContentType(path);
    else
        setContentType(contentType.c_str());

    int filenameStart = path.lastIndexOf('/') + 1;
    const char *filename = path.c_str() + filenameStart;
    String cd;
    cd.reserve(32 + path.length() - filenameStart);
    cd += (download ? "attachment; filename=\"" : "inline; filename=\"");
    cd += filename;
    cd += "\"";
    addHeader("Content-Disposition", cd.c_str());
}

PsychicFileResponse::PsychicFileResponse(PsychicRequest *request, File content, const String &path, const String &contentType, bool download)
    : PsychicResponse(request) {
    String _path(path);

    if (!download && String(content.name()).endsWith(".gz") && !path.endsWith(".gz")) {
        addHeader("Content-Encoding", "gzip");
        addHeader("Vary", "Accept-Encoding");
    }

    _content = std::move(content);
    _contentLength = _content.size();

    if (contentType == "")
        _setContentType(path);
    else
        setContentType(contentType.c_str());

    int filenameStart = path.lastIndexOf('/') + 1;
    const char *filename = path.c_str() + filenameStart;
    String cd;
    cd.reserve(32 + path.length() - filenameStart);
    cd += (download ? "attachment; filename=\"" : "inline; filename=\"");
    cd += filename;
    cd += "\"";
    addHeader("Content-Disposition", cd.c_str());
}

PsychicFileResponse::~PsychicFileResponse() {
    if (_content) _content.close();
}

void PsychicFileResponse::_setContentType(const String &path) {
    const char *_contentType;

    if (path.endsWith(".html"))
        _contentType = "text/html";
    else if (path.endsWith(".htm"))
        _contentType = "text/html";
    else if (path.endsWith(".css"))
        _contentType = "text/css";
    else if (path.endsWith(".json"))
        _contentType = "application/json";
    else if (path.endsWith(".js"))
        _contentType = "application/javascript";
    else if (path.endsWith(".png"))
        _contentType = "image/png";
    else if (path.endsWith(".gif"))
        _contentType = "image/gif";
    else if (path.endsWith(".jpg"))
        _contentType = "image/jpeg";
    else if (path.endsWith(".ico"))
        _contentType = "image/x-icon";
    else if (path.endsWith(".svg"))
        _contentType = "image/svg+xml";
    else if (path.endsWith(".eot"))
        _contentType = "font/eot";
    else if (path.endsWith(".woff"))
        _contentType = "font/woff";
    else if (path.endsWith(".woff2"))
        _contentType = "font/woff2";
    else if (path.endsWith(".ttf"))
        _contentType = "font/ttf";
    else if (path.endsWith(".xml"))
        _contentType = "text/xml";
    else if (path.endsWith(".pdf"))
        _contentType = "application/pdf";
    else if (path.endsWith(".zip"))
        _contentType = "application/zip";
    else if (path.endsWith(".gz"))
        _contentType = "application/x-gzip";
    else
        _contentType = "text/plain";

    setContentType(_contentType);
}

esp_err_t PsychicFileResponse::send() {
    esp_err_t err = ESP_OK;

    // Check if file was successfully opened
    if (!_content) {
        httpd_resp_send_err(this->_request->request(), HTTPD_404_NOT_FOUND, "File not found.");
        return ESP_FAIL;
    }

    // just send small files directly
    size_t size = getContentLength();
    if (size == 0) {
        // No allocation needed; send headers + empty body
        err = PsychicResponse::send();
    } else if (size < FILE_CHUNK_SIZE) {
        uint8_t *buffer = (uint8_t *)malloc(size);
        if (buffer == NULL) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(this->_request->request(), HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to allocate memory.");
            return ESP_FAIL;
        }

        size_t readSize = _content.readBytes((char *)buffer, size);

        this->setContent(buffer, readSize);
        err = PsychicResponse::send();

        free(buffer);
    } else {
        /* Retrieve the pointer to scratch buffer for temporary storage */
        char *chunk = (char *)malloc(FILE_CHUNK_SIZE);
        if (chunk == NULL) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(this->_request->request(), HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to allocate memory.");
            return ESP_FAIL;
        }

        this->sendHeaders();

        size_t chunksize;
        do {
            /* Read file in chunks into the scratch buffer */
            chunksize = _content.readBytes(chunk, FILE_CHUNK_SIZE);
            if (chunksize > 0) {
                err = this->sendChunk((uint8_t *)chunk, chunksize);
                if (err != ESP_OK) break;
            }

            /* Keep looping till the whole file is sent */
        } while (chunksize != 0);

        // keep track of our memory
        free(chunk);

        if (err == ESP_OK) {
            ESP_LOGD(PH_TAG, "File sending complete");
            this->finishChunking();
        }
    }

    return err;
}

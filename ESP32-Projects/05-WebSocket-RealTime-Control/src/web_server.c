/**
 * @file web_server.c
 * @brief HTTP Web Server with WebSocket implementation for real-time control.
 *
 * This file implements the HTTP server and WebSocket handler for the
 * ESP32 motor/servo control system. It serves an embedded HTML/JS
 * single-page application, handles WebSocket connections for real-time
 * bidirectional communication, and provides REST API endpoints.
 *
 * @author EmbeddedProjects
 * @date 2026
 * @version 1.0.0
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "cJSON.h"

#include "web_server.h"
#include "motor_controller.h"

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

/** @brief Logging tag for this module. */
static const char *TAG = "web_server";

/*******************************************************************************
 * Embedded HTML/JS/CSS Page
 ******************************************************************************/

/**
 * @brief Embedded HTML page served at the root "/" URI.
 *
 * This single-page application provides:
 *  - Motor speed slider (0-100%)
 *  - Direction buttons (Forward, Reverse, Brake, Coast)
 *  - Servo angle slider (0-180 degrees)
 *  - Real-time status display (speed, direction, angle, encoder, RPM)
 *  - WebSocket connection status indicator
 *  - Emergency stop button
 */
static const char INDEX_HTML[] =
    "<!DOCTYPE html>"
    "<html lang=\"en\">"
    "<head>"
    "<meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0\">"
    "<title>ESP32 Motor Control</title>"
    "<style>"
    "*{margin:0;padding:0;box-sizing:border-box;}"
    "body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;"
    "background:#1a1a2e;color:#eee;min-height:100vh;padding:20px;}"
    ".container{max-width:800px;margin:0 auto;}"
    "h1{text-align:center;color:#e94560;margin-bottom:20px;font-size:1.8em;}"
    ".status-bar{display:flex;justify-content:space-between;align-items:center;"
    "background:#16213e;padding:10px 20px;border-radius:8px;margin-bottom:20px;}"
    ".ws-status{display:flex;align-items:center;gap:8px;}"
    ".ws-dot{width:12px;height:12px;border-radius:50%;background:#e94560;}"
    ".ws-dot.connected{background:#0f3460;background:#00d97e;}"
    ".card{background:#16213e;border-radius:12px;padding:20px;margin-bottom:16px;"
    "box-shadow:0 4px 6px rgba(0,0,0,0.3);}"
    ".card h2{color:#e94560;margin-bottom:15px;font-size:1.2em;}"
    ".slider-group{margin-bottom:15px;}"
    ".slider-group label{display:block;margin-bottom:8px;font-weight:600;}"
    ".slider-group input[type=range]{width:100%;height:8px;border-radius:4px;"
    "background:#0f3460;outline:none;-webkit-appearance:none;}"
    ".slider-group input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;"
    "width:24px;height:24px;border-radius:50%;background:#e94560;cursor:pointer;}"
    ".slider-value{text-align:center;font-size:2em;color:#e94560;font-weight:700;"
    "margin:10px 0;}"
    ".btn-group{display:flex;gap:10px;flex-wrap:wrap;justify-content:center;}"
    ".btn{padding:12px 24px;border:none;border-radius:8px;font-size:1em;"
    "font-weight:600;cursor:pointer;transition:all 0.2s;min-width:100px;}"
    ".btn-fwd{background:#0f3460;color:#fff;}"
    ".btn-rev{background:#533483;color:#fff;}"
    ".btn-brake{background:#e94560;color:#fff;}"
    ".btn-coast{background:#16213e;color:#fff;border:2px solid #0f3460;}"
    ".btn-estop{background:#ff0000;color:#fff;font-size:1.2em;padding:15px 40px;"
    "animation:pulse 2s infinite;width:100%;margin-top:10px;}"
    "@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(255,0,0,0.4);}"
    "50%{box-shadow:0 0 20px 10px rgba(255,0,0,0.2);}}"
    ".btn:hover{opacity:0.85;transform:translateY(-1px);}"
    ".btn:active{transform:translateY(1px);}"
    ".btn.active{box-shadow:0 0 0 3px #00d97e;}"
    ".telemetry{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));"
    "gap:12px;}"
    ".telem-item{background:#0f3460;padding:15px;border-radius:8px;text-align:center;}"
    ".telem-item .label{font-size:0.8em;color:#aaa;text-transform:uppercase;"
    "letter-spacing:1px;}"
    ".telem-item .value{font-size:1.8em;font-weight:700;color:#e94560;margin-top:5px;}"
    ".clients-info{text-align:center;color:#aaa;font-size:0.85em;margin-top:8px;}"
    "</style>"
    "</head>"
    "<body>"
    "<div class=\"container\">"
    "<h1>ESP32 Motor Control Panel</h1>"
    "<div class=\"status-bar\">"
    "<div class=\"ws-status\">"
    "<div class=\"ws-dot\" id=\"wsDot\"></div>"
    "<span id=\"wsStatus\">Disconnected</span>"
    "</div>"
    "<div class=\"clients-info\">Clients: <span id=\"clientCount\">0</span></div>"
    "</div>"
    "<div class=\"card\">"
    "<h2>Motor Speed</h2>"
    "<div class=\"slider-group\">"
    "<input type=\"range\" id=\"speedSlider\" min=\"0\" max=\"100\" value=\"0\">"
    "<div class=\"slider-value\"><span id=\"speedVal\">0</span>%</div>"
    "</div>"
    "<div class=\"btn-group\">"
    "<button class=\"btn btn-fwd\" id=\"btnFwd\" onclick=\"setDir('forward')\">Forward</button>"
    "<button class=\"btn btn-rev\" id=\"btnRev\" onclick=\"setDir('reverse')\">Reverse</button>"
    "<button class=\"btn btn-brake\" id=\"btnBrake\" onclick=\"setDir('brake')\">Brake</button>"
    "<button class=\"btn btn-coast\" id=\"btnCoast\" onclick=\"setDir('coast')\">Coast</button>"
    "</div>"
    "</div>"
    "<div class=\"card\">"
    "<h2>Servo Angle</h2>"
    "<div class=\"slider-group\">"
    "<input type=\"range\" id=\"servoSlider\" min=\"0\" max=\"180\" value=\"90\">"
    "<div class=\"slider-value\"><span id=\"servoVal\">90</span>&deg;</div>"
    "</div>"
    "</div>"
    "<div class=\"card\">"
    "<h2>Telemetry</h2>"
    "<div class=\"telemetry\">"
    "<div class=\"telem-item\"><div class=\"label\">Speed</div>"
    "<div class=\"value\" id=\"telemSpeed\">0%</div></div>"
    "<div class=\"telem-item\"><div class=\"label\">Direction</div>"
    "<div class=\"value\" id=\"telemDir\">--</div></div>"
    "<div class=\"telem-item\"><div class=\"label\">Servo</div>"
    "<div class=\"value\" id=\"telemServo\">90&deg;</div></div>"
    "<div class=\"telem-item\"><div class=\"label\">Encoder</div>"
    "<div class=\"value\" id=\"telemEnc\">0</div></div>"
    "<div class=\"telem-item\"><div class=\"label\">RPM</div>"
    "<div class=\"value\" id=\"telemRpm\">0</div></div>"
    "</div>"
    "</div>"
    "<button class=\"btn btn-estop\" onclick=\"emergencyStop()\">EMERGENCY STOP</button>"
    "</div>"
    "<script>"
    "let ws=null;let currentDir='coast';"
    "function connectWS(){"
    "const host=window.location.hostname||'esp32-control.local';"
    "ws=new WebSocket('ws://'+host+'/ws');"
    "ws.onopen=function(){"
    "document.getElementById('wsDot').classList.add('connected');"
    "document.getElementById('wsStatus').textContent='Connected';"
    "console.log('WebSocket connected');"
    "};"
    "ws.onclose=function(){"
    "document.getElementById('wsDot').classList.remove('connected');"
    "document.getElementById('wsStatus').textContent='Disconnected';"
    "console.log('WebSocket disconnected, reconnecting...');"
    "setTimeout(connectWS,2000);"
    "};"
    "ws.onerror=function(e){console.error('WebSocket error:',e);};"
    "ws.onmessage=function(evt){"
    "try{const d=JSON.parse(evt.data);"
    "if(d.motor_speed!==undefined)document.getElementById('telemSpeed').textContent=d.motor_speed+'%';"
    "if(d.motor_dir!==undefined)document.getElementById('telemDir').textContent=d.motor_dir;"
    "if(d.servo_angle!==undefined)document.getElementById('telemServo').innerHTML=d.servo_angle+'&deg;';"
    "if(d.encoder_count!==undefined)document.getElementById('telemEnc').textContent=d.encoder_count;"
    "if(d.rpm!==undefined)document.getElementById('telemRpm').textContent=d.rpm.toFixed(1);"
    "if(d.clients!==undefined)document.getElementById('clientCount').textContent=d.clients;"
    "}catch(e){console.error('Parse error:',e);}"
    "};"
    "}"
    "function sendCmd(cmd){"
    "if(ws&&ws.readyState===WebSocket.OPEN){"
    "ws.send(JSON.stringify(cmd));"
    "}"
    "}"
    "function setDir(dir){"
    "currentDir=dir;"
    "document.querySelectorAll('.btn-group .btn').forEach(b=>b.classList.remove('active'));"
    "const map={forward:'btnFwd',reverse:'btnRev',brake:'btnBrake',coast:'btnCoast'};"
    "document.getElementById(map[dir]).classList.add('active');"
    "sendCmd({motor_dir:dir});"
    "}"
    "function emergencyStop(){"
    "document.getElementById('speedSlider').value=0;"
    "document.getElementById('speedVal').textContent='0';"
    "sendCmd({motor_speed:0,motor_dir:'brake',servo_angle:90});"
    "document.getElementById('servoSlider').value=90;"
    "document.getElementById('servoVal').textContent='90';"
    "}"
    "document.getElementById('speedSlider').addEventListener('input',function(){"
    "document.getElementById('speedVal').textContent=this.value;"
    "sendCmd({motor_speed:parseInt(this.value)});"
    "});"
    "document.getElementById('servoSlider').addEventListener('input',function(){"
    "document.getElementById('servoVal').textContent=this.value;"
    "sendCmd({servo_angle:parseInt(this.value)});"
    "});"
    "connectWS();"
    "</script>"
    "</body>"
    "</html>";

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

/** @brief Web server context (singleton). */
static web_server_ctx_t s_server_ctx = {
    .server = NULL,
    .active_client_count = 0,
    .running = false};

/** @brief Mutex for protecting client session list access. */
static SemaphoreHandle_t s_client_mutex = NULL;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t ws_handler(httpd_req_t *req);
static esp_err_t api_status_get_handler(httpd_req_t *req);
static esp_err_t api_motor_post_handler(httpd_req_t *req);
static esp_err_t parse_ws_command(const char *json_str, ws_command_t *cmd);
static esp_err_t apply_ws_command(const ws_command_t *cmd);
static int add_client_session(int fd);
static void remove_client_session(int fd);
static void init_client_sessions(void);

/*******************************************************************************
 * URI Handler Definitions
 ******************************************************************************/

/**
 * @brief URI handler for serving the root HTML page.
 */
static const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};

/**
 * @brief URI handler for WebSocket endpoint.
 */
static const httpd_uri_t uri_ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true};

/**
 * @brief URI handler for GET /api/status.
 */
static const httpd_uri_t uri_api_status = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = api_status_get_handler,
    .user_ctx = NULL};

/**
 * @brief URI handler for POST /api/motor.
 */
static const httpd_uri_t uri_api_motor = {
    .uri = "/api/motor",
    .method = HTTP_POST,
    .handler = api_motor_post_handler,
    .user_ctx = NULL};

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Initialize all client session slots to empty.
 */
static void init_client_sessions(void)
{
    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++)
    {
        s_server_ctx.clients[i].fd = -1;
        s_server_ctx.clients[i].active = false;
        s_server_ctx.clients[i].msg_count = 0;
        s_server_ctx.clients[i].connected_at = 0;
    }
    s_server_ctx.active_client_count = 0;
}

/**
 * @brief Add a new client session to the tracking array.
 *
 * @param[in] fd    Socket file descriptor of the new client.
 *
 * @return Index of the assigned slot, or -1 if all slots are full.
 */
static int add_client_session(int fd)
{
    int slot = -1;

    if (s_client_mutex != NULL)
    {
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    }

    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++)
    {
        if (!s_server_ctx.clients[i].active)
        {
            s_server_ctx.clients[i].fd = fd;
            s_server_ctx.clients[i].active = true;
            s_server_ctx.clients[i].msg_count = 0;
            s_server_ctx.clients[i].connected_at = esp_timer_get_time();
            s_server_ctx.active_client_count++;
            slot = i;
            ESP_LOGI(TAG, "Client added: fd=%d, slot=%d, total=%d",
                     fd, i, s_server_ctx.active_client_count);
            break;
        }
    }

    if (s_client_mutex != NULL)
    {
        xSemaphoreGive(s_client_mutex);
    }

    if (slot < 0)
    {
        ESP_LOGW(TAG, "No free client slots for fd=%d", fd);
    }

    return slot;
}

/**
 * @brief Remove a client session from the tracking array.
 *
 * @param[in] fd    Socket file descriptor of the client to remove.
 */
static void remove_client_session(int fd)
{
    if (s_client_mutex != NULL)
    {
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    }

    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++)
    {
        if (s_server_ctx.clients[i].active && s_server_ctx.clients[i].fd == fd)
        {
            s_server_ctx.clients[i].fd = -1;
            s_server_ctx.clients[i].active = false;
            if (s_server_ctx.active_client_count > 0)
            {
                s_server_ctx.active_client_count--;
            }
            ESP_LOGI(TAG, "Client removed: fd=%d, slot=%d, total=%d",
                     fd, i, s_server_ctx.active_client_count);
            break;
        }
    }

    if (s_client_mutex != NULL)
    {
        xSemaphoreGive(s_client_mutex);
    }
}

/**
 * @brief Parse a JSON command string into a ws_command_t structure.
 *
 * Expected JSON format:
 * @code
 * {
 *     "motor_speed": 75,
 *     "motor_dir": "forward",
 *     "servo_angle": 90
 * }
 * @endcode
 *
 * @param[in]  json_str    Null-terminated JSON string.
 * @param[out] cmd         Parsed command structure.
 *
 * @return
 *  - ESP_OK on successful parse.
 *  - ESP_ERR_INVALID_ARG if parameters are NULL.
 *  - ESP_FAIL on JSON parse error.
 */
static esp_err_t parse_ws_command(const char *json_str, ws_command_t *cmd)
{
    if (json_str == NULL || cmd == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize command with defaults. */
    memset(cmd, 0, sizeof(ws_command_t));
    cmd->motor_dir = WS_MOTOR_DIR_UNKNOWN;

    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "JSON parse error near: %s", cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown");
        return ESP_FAIL;
    }

    /* Parse motor_speed field. */
    cJSON *speed = cJSON_GetObjectItemCaseSensitive(root, "motor_speed");
    if (cJSON_IsNumber(speed))
    {
        cmd->motor_speed = speed->valueint;
        cmd->has_motor_speed = true;
    }

    /* Parse motor_dir field. */
    cJSON *dir = cJSON_GetObjectItemCaseSensitive(root, "motor_dir");
    if (cJSON_IsString(dir) && dir->valuestring != NULL)
    {
        cmd->has_motor_dir = true;
        if (strcmp(dir->valuestring, "forward") == 0)
        {
            cmd->motor_dir = WS_MOTOR_DIR_FORWARD;
        }
        else if (strcmp(dir->valuestring, "reverse") == 0)
        {
            cmd->motor_dir = WS_MOTOR_DIR_REVERSE;
        }
        else if (strcmp(dir->valuestring, "brake") == 0)
        {
            cmd->motor_dir = WS_MOTOR_DIR_BRAKE;
        }
        else if (strcmp(dir->valuestring, "coast") == 0)
        {
            cmd->motor_dir = WS_MOTOR_DIR_COAST;
        }
        else
        {
            cmd->motor_dir = WS_MOTOR_DIR_UNKNOWN;
            ESP_LOGW(TAG, "Unknown motor direction: %s", dir->valuestring);
        }
    }

    /* Parse servo_angle field. */
    cJSON *angle = cJSON_GetObjectItemCaseSensitive(root, "servo_angle");
    if (cJSON_IsNumber(angle))
    {
        cmd->servo_angle = angle->valueint;
        cmd->has_servo_angle = true;
    }

    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * @brief Apply a parsed WebSocket command to the motor controller.
 *
 * @param[in] cmd    Pointer to the parsed command.
 *
 * @return ESP_OK on success, or an error code from the motor controller.
 */
static esp_err_t apply_ws_command(const ws_command_t *cmd)
{
    esp_err_t ret = ESP_OK;

    if (cmd == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (cmd->has_motor_speed)
    {
        ret = motor_controller_set_speed(cmd->motor_speed);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set motor speed: %s", esp_err_to_name(ret));
        }
    }

    if (cmd->has_motor_dir && cmd->motor_dir != WS_MOTOR_DIR_UNKNOWN)
    {
        motor_direction_t dir;
        switch (cmd->motor_dir)
        {
        case WS_MOTOR_DIR_FORWARD:
            dir = MOTOR_DIR_FORWARD;
            break;
        case WS_MOTOR_DIR_REVERSE:
            dir = MOTOR_DIR_REVERSE;
            break;
        case WS_MOTOR_DIR_BRAKE:
            dir = MOTOR_DIR_BRAKE;
            break;
        case WS_MOTOR_DIR_COAST:
            dir = MOTOR_DIR_COAST;
            break;
        default:
            dir = MOTOR_DIR_COAST;
            break;
        }
        ret = motor_controller_set_direction(dir);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set motor direction: %s", esp_err_to_name(ret));
        }
    }

    if (cmd->has_servo_angle)
    {
        ret = motor_controller_set_servo_angle(cmd->servo_angle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set servo angle: %s", esp_err_to_name(ret));
        }
    }

    return ret;
}

/*******************************************************************************
 * HTTP Handler Implementations
 ******************************************************************************/

/**
 * @brief Handler for GET "/" - serves the embedded HTML page.
 *
 * @param[in] req    HTTP request handle.
 *
 * @return ESP_OK on success.
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving root page to client");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

/**
 * @brief Handler for WebSocket endpoint at "/ws".
 *
 * Handles the WebSocket lifecycle:
 *  1. New connection: registers client session.
 *  2. Incoming frames: parses text frames as JSON commands.
 *  3. Close/error: removes client session.
 *
 * @param[in] req    HTTP request handle (upgraded to WebSocket).
 *
 * @return ESP_OK on success, or an error code.
 */
static esp_err_t ws_handler(httpd_req_t *req)
{
    /* Handle new WebSocket connection (handshake). */
    if (req->method == HTTP_GET)
    {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake from fd=%d", fd);
        int slot = add_client_session(fd);
        if (slot < 0)
        {
            ESP_LOGW(TAG, "Rejecting WebSocket: max clients reached");
            /* Still allow the handshake but log the warning. */
        }
        return ESP_OK;
    }

    /* Receive WebSocket frame. */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    /* First call with len=0 to get the frame length. */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame() failed to get length: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "WS frame: type=%d, len=%d", ws_pkt.type, ws_pkt.len);

    if (ws_pkt.len == 0)
    {
        /* Empty frame or control frame. */
        if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
        {
            int fd = httpd_req_to_sockfd(req);
            ESP_LOGI(TAG, "WebSocket close from fd=%d", fd);
            remove_client_session(fd);
        }
        return ESP_OK;
    }

    /* Limit frame size. */
    if (ws_pkt.len >= WEB_SERVER_MAX_WS_PAYLOAD_LEN)
    {
        ESP_LOGW(TAG, "WS frame too large: %d bytes (max %d)",
                 ws_pkt.len, WEB_SERVER_MAX_WS_PAYLOAD_LEN);
        return ESP_ERR_INVALID_SIZE;
    }

    /* Allocate buffer and receive the payload. */
    uint8_t *buf = calloc(1, ws_pkt.len + 1);
    if (buf == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate WS receive buffer");
        return ESP_ERR_NO_MEM;
    }

    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame() payload read failed: %s",
                 esp_err_to_name(ret));
        free(buf);
        return ret;
    }

    /* Update message counter for this client. */
    int fd = httpd_req_to_sockfd(req);
    if (s_client_mutex != NULL)
    {
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    }
    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++)
    {
        if (s_server_ctx.clients[i].active && s_server_ctx.clients[i].fd == fd)
        {
            s_server_ctx.clients[i].msg_count++;
            break;
        }
    }
    if (s_client_mutex != NULL)
    {
        xSemaphoreGive(s_client_mutex);
    }

    /* Process based on frame type. */
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
    {
        ESP_LOGI(TAG, "WS text from fd=%d: %s", fd, (char *)buf);

        /* Parse JSON command. */
        ws_command_t cmd;
        ret = parse_ws_command((const char *)buf, &cmd);
        if (ret == ESP_OK)
        {
            ret = apply_ws_command(&cmd);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to parse WS command from fd=%d", fd);
        }
    }
    else if (ws_pkt.type == HTTPD_WS_TYPE_BINARY)
    {
        ESP_LOGI(TAG, "WS binary from fd=%d: %d bytes", fd, ws_pkt.len);
        /* Binary frames are not used in this application. */
    }
    else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
    {
        ESP_LOGI(TAG, "WS close from fd=%d", fd);
        remove_client_session(fd);
    }
    else if (ws_pkt.type == HTTPD_WS_TYPE_PING)
    {
        /* Respond with pong. */
        httpd_ws_frame_t pong = {
            .type = HTTPD_WS_TYPE_PONG,
            .payload = ws_pkt.payload,
            .len = ws_pkt.len};
        ret = httpd_ws_send_frame(req, &pong);
        ESP_LOGD(TAG, "Sent pong to fd=%d", fd);
    }

    free(buf);
    return ret;
}

/**
 * @brief Handler for GET /api/status - returns JSON status.
 *
 * @param[in] req    HTTP request handle.
 *
 * @return ESP_OK on success.
 */
static esp_err_t api_status_get_handler(httpd_req_t *req)
{
    motor_state_t state;
    esp_err_t ret = motor_controller_get_state(&state);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (ret == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "motor_speed", state.current_speed);
        cJSON_AddStringToObject(root, "motor_dir",
                                motor_direction_to_str(state.direction));
        cJSON_AddNumberToObject(root, "servo_angle", state.servo_angle);
        cJSON_AddNumberToObject(root, "encoder_count", state.encoder_count);
        cJSON_AddNumberToObject(root, "rpm", state.rpm);
        cJSON_AddBoolToObject(root, "motor_enabled", state.motor_enabled);
    }
    else
    {
        cJSON_AddStringToObject(root, "error", "Motor controller not initialized");
    }

    cJSON_AddNumberToObject(root, "clients", s_server_ctx.active_client_count);

    const char *json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    ret = httpd_resp_send(req, json_str, strlen(json_str));

    cJSON_free((void *)json_str);
    cJSON_Delete(root);

    return ret;
}

/**
 * @brief Handler for POST /api/motor - apply motor command via REST.
 *
 * Accepts JSON body in the same format as WebSocket commands.
 *
 * @param[in] req    HTTP request handle.
 *
 * @return ESP_OK on success.
 */
static esp_err_t api_motor_post_handler(httpd_req_t *req)
{
    /* Read request body. */
    int content_len = req->content_len;
    if (content_len <= 0 || content_len >= WEB_SERVER_MAX_WS_PAYLOAD_LEN)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    char *buf = calloc(1, content_len + 1);
    if (buf == NULL)
    {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    int received = httpd_req_recv(req, buf, content_len);
    if (received <= 0)
    {
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    /* Parse and apply command. */
    ws_command_t cmd;
    esp_err_t ret = parse_ws_command(buf, &cmd);
    free(buf);

    if (ret != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    ret = apply_ws_command(&cmd);

    /* Respond with current status. */
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON_AddBoolToObject(root, "success", (ret == ESP_OK));
    cJSON_AddStringToObject(root, "message",
                            (ret == ESP_OK) ? "Command applied" : "Command failed");

    const char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_str, strlen(json_str));

    cJSON_free((void *)json_str);
    cJSON_Delete(root);

    return ret;
}

/**
 * @brief Callback invoked when an HTTP connection is closed.
 *
 * Used to detect WebSocket client disconnections and clean up sessions.
 *
 * @param[in] hd    HTTP server handle.
 * @param[in] fd    Socket file descriptor that was closed.
 */
static void on_client_disconnect(httpd_handle_t hd, int fd)
{
    ESP_LOGI(TAG, "Client disconnected: fd=%d", fd);
    remove_client_session(fd);
}

/*******************************************************************************
 * Public Function Implementations
 ******************************************************************************/

esp_err_t web_server_start(void)
{
    if (s_server_ctx.running)
    {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create client session mutex. */
    if (s_client_mutex == NULL)
    {
        s_client_mutex = xSemaphoreCreateMutex();
        if (s_client_mutex == NULL)
        {
            ESP_LOGE(TAG, "Failed to create client mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Initialize client sessions. */
    init_client_sessions();

    /* Configure HTTP server. */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_uri_handlers = WEB_SERVER_MAX_URI_HANDLERS;
    config.max_open_sockets = WEB_SERVER_MAX_WS_CLIENTS + 2; /* WS clients + HTTP */
    config.lru_purge_enable = true;
    config.close_fn = on_client_disconnect;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    esp_err_t ret = httpd_start(&s_server_ctx.server, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register URI handlers. */
    httpd_register_uri_handler(s_server_ctx.server, &uri_root);
    httpd_register_uri_handler(s_server_ctx.server, &uri_ws);
    httpd_register_uri_handler(s_server_ctx.server, &uri_api_status);
    httpd_register_uri_handler(s_server_ctx.server, &uri_api_motor);

    s_server_ctx.running = true;
    ESP_LOGI(TAG, "Web server started successfully");

    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (!s_server_ctx.running || s_server_ctx.server == NULL)
    {
        ESP_LOGW(TAG, "Web server not running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping web server...");

    /* Send close frame to all connected WebSocket clients. */
    if (s_client_mutex != NULL)
    {
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    }

    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++)
    {
        if (s_server_ctx.clients[i].active)
        {
            httpd_ws_frame_t close_frame = {
                .type = HTTPD_WS_TYPE_CLOSE,
                .payload = NULL,
                .len = 0};
            /* Attempt to send close frame (best effort). */
            httpd_ws_send_frame_async(s_server_ctx.server,
                                      s_server_ctx.clients[i].fd,
                                      &close_frame);
        }
    }

    if (s_client_mutex != NULL)
    {
        xSemaphoreGive(s_client_mutex);
    }

    /* Stop the server. */
    esp_err_t ret = httpd_stop(s_server_ctx.server);
    if (ret == ESP_OK)
    {
        s_server_ctx.server = NULL;
        s_server_ctx.running = false;
        init_client_sessions();
        ESP_LOGI(TAG, "Web server stopped");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to stop web server: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t web_server_broadcast_ws(const char *message)
{
    if (message == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_server_ctx.running || s_server_ctx.server == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_server_ctx.active_client_count == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    httpd_ws_frame_t ws_pkt = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)message,
        .len = strlen(message)};

    int sent_count = 0;

    if (s_client_mutex != NULL)
    {
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    }

    for (int i = 0; i < WEB_SERVER_MAX_WS_CLIENTS; i++)
    {
        if (s_server_ctx.clients[i].active)
        {
            esp_err_t ret = httpd_ws_send_frame_async(
                s_server_ctx.server,
                s_server_ctx.clients[i].fd,
                &ws_pkt);
            if (ret == ESP_OK)
            {
                sent_count++;
            }
            else
            {
                ESP_LOGW(TAG, "Failed to send to fd=%d: %s, removing client",
                         s_server_ctx.clients[i].fd, esp_err_to_name(ret));
                /* Mark client for removal. */
                s_server_ctx.clients[i].active = false;
                s_server_ctx.clients[i].fd = -1;
                if (s_server_ctx.active_client_count > 0)
                {
                    s_server_ctx.active_client_count--;
                }
            }
        }
    }

    if (s_client_mutex != NULL)
    {
        xSemaphoreGive(s_client_mutex);
    }

    ESP_LOGD(TAG, "Broadcast sent to %d/%d clients",
             sent_count, s_server_ctx.active_client_count);

    return (sent_count > 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t web_server_broadcast_status(int32_t motor_speed,
                                      const char *motor_dir,
                                      int32_t servo_angle,
                                      int32_t encoder_count,
                                      float rpm)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        return ESP_FAIL;
    }

    cJSON_AddNumberToObject(root, "motor_speed", motor_speed);
    cJSON_AddStringToObject(root, "motor_dir", motor_dir ? motor_dir : "unknown");
    cJSON_AddNumberToObject(root, "servo_angle", servo_angle);
    cJSON_AddNumberToObject(root, "encoder_count", encoder_count);

    /* Round RPM to 1 decimal place. */
    double rounded_rpm = ((int)(rpm * 10.0f + 0.5f)) / 10.0;
    cJSON_AddNumberToObject(root, "rpm", rounded_rpm);

    cJSON_AddNumberToObject(root, "clients", s_server_ctx.active_client_count);

    const char *json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    esp_err_t ret = web_server_broadcast_ws(json_str);

    cJSON_free((void *)json_str);
    cJSON_Delete(root);

    return ret;
}

uint8_t web_server_get_client_count(void)
{
    return s_server_ctx.active_client_count;
}

bool web_server_is_running(void)
{
    return s_server_ctx.running;
}

const web_server_ctx_t *web_server_get_context(void)
{
    if (!s_server_ctx.running)
    {
        return NULL;
    }
    return &s_server_ctx;
}

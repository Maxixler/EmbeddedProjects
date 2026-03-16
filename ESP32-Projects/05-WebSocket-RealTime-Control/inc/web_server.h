/**
 * @file web_server.h
 * @brief HTTP Web Server with WebSocket support for real-time motor/servo control.
 *
 * This module provides an HTTP server built on the ESP-IDF httpd component.
 * It serves an embedded HTML/JS control page and handles WebSocket connections
 * for real-time bidirectional communication with up to 4 simultaneous clients.
 *
 * Features:
 *  - Embedded HTML/CSS/JS single-page application served at "/"
 *  - WebSocket endpoint at "/ws" with upgrade handling
 *  - WebSocket text frame parsing (JSON commands)
 *  - Periodic sensor data push to all connected WebSocket clients
 *  - REST API endpoints: GET /api/status, POST /api/motor
 *  - Multiple client session tracking (max 4 concurrent WebSocket sessions)
 *
 * @note This module depends on the ESP-IDF HTTP Server component (esp_http_server)
 *       and the cJSON library for JSON parsing.
 *
 * @author EmbeddedProjects
 * @date 2026
 * @version 1.0.0
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

/*******************************************************************************
 * Preprocessor Definitions
 ******************************************************************************/

/** @brief Maximum number of concurrent WebSocket client sessions. */
#define WEB_SERVER_MAX_WS_CLIENTS 4

/** @brief Maximum length of a single WebSocket text frame payload (bytes). */
#define WEB_SERVER_MAX_WS_PAYLOAD_LEN 512

/** @brief HTTP server listening port. */
#define WEB_SERVER_PORT 80

/** @brief Maximum number of URI handlers registered with the HTTP server. */
#define WEB_SERVER_MAX_URI_HANDLERS 8

/** @brief Stack size for the WebSocket broadcast task (bytes). */
#define WEB_SERVER_BROADCAST_TASK_STACK 4096

/** @brief Priority of the WebSocket broadcast task. */
#define WEB_SERVER_BROADCAST_TASK_PRIO 5

/** @brief Interval between periodic sensor broadcasts (milliseconds). */
#define WEB_SERVER_BROADCAST_INTERVAL_MS 200

    /*******************************************************************************
     * Type Definitions
     ******************************************************************************/

    /**
     * @brief Motor direction command values received from WebSocket clients.
     */
    typedef enum
    {
        WS_MOTOR_DIR_FORWARD = 0, /**< Motor forward direction. */
        WS_MOTOR_DIR_REVERSE,     /**< Motor reverse direction. */
        WS_MOTOR_DIR_BRAKE,       /**< Motor active brake. */
        WS_MOTOR_DIR_COAST,       /**< Motor coast (free-running). */
        WS_MOTOR_DIR_UNKNOWN      /**< Unknown / invalid direction. */
    } ws_motor_dir_t;

    /**
     * @brief Parsed WebSocket command structure.
     *
     * This structure represents a command received from a WebSocket client
     * after JSON parsing. The JSON format expected is:
     * @code
     * {
     *     "motor_speed": 75,
     *     "motor_dir": "forward",
     *     "servo_angle": 90
     * }
     * @endcode
     */
    typedef struct
    {
        int32_t motor_speed;      /**< Motor speed percentage (0-100). */
        ws_motor_dir_t motor_dir; /**< Motor direction command. */
        int32_t servo_angle;      /**< Servo angle in degrees (0-180). */
        bool has_motor_speed;     /**< True if motor_speed field was present. */
        bool has_motor_dir;       /**< True if motor_dir field was present. */
        bool has_servo_angle;     /**< True if servo_angle field was present. */
    } ws_command_t;

    /**
     * @brief WebSocket client session descriptor.
     *
     * Tracks an active WebSocket connection including its file descriptor
     * and connection metadata.
     */
    typedef struct
    {
        int fd;               /**< Socket file descriptor (-1 if slot is free). */
        bool active;          /**< True if this session is currently active. */
        uint32_t msg_count;   /**< Number of messages received from this client. */
        int64_t connected_at; /**< Timestamp (us) when the client connected. */
    } ws_client_session_t;

    /**
     * @brief Web server context holding runtime state.
     */
    typedef struct
    {
        httpd_handle_t server;                                  /**< HTTP server handle. */
        ws_client_session_t clients[WEB_SERVER_MAX_WS_CLIENTS]; /**< Client session slots. */
        uint8_t active_client_count;                            /**< Number of active clients. */
        bool running;                                           /**< True if server is running. */
    } web_server_ctx_t;

    /*******************************************************************************
     * Public Function Prototypes
     ******************************************************************************/

    /**
     * @brief Initialize and start the HTTP web server with WebSocket support.
     *
     * This function configures the HTTP server, registers all URI handlers
     * (root page, WebSocket endpoint, REST API endpoints), and starts
     * listening on the configured port.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_HTTPD_ALLOC_MEM if memory allocation failed.
     *  - ESP_ERR_HTTPD_TASK if server task creation failed.
     *  - ESP_FAIL on other errors.
     */
    esp_err_t web_server_start(void);

    /**
     * @brief Stop the HTTP web server and close all WebSocket connections.
     *
     * Gracefully shuts down the server, sends close frames to all connected
     * WebSocket clients, and releases all associated resources.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if the server is not running.
     */
    esp_err_t web_server_stop(void);

    /**
     * @brief Send a text message to all connected WebSocket clients.
     *
     * Iterates through all active WebSocket sessions and sends the given
     * text payload as a WebSocket text frame to each client.
     *
     * @param[in] message   Null-terminated string to broadcast.
     *
     * @return
     *  - ESP_OK if the message was sent to at least one client.
     *  - ESP_ERR_INVALID_ARG if message is NULL.
     *  - ESP_ERR_INVALID_STATE if no clients are connected.
     */
    esp_err_t web_server_broadcast_ws(const char *message);

    /**
     * @brief Send a JSON-formatted sensor status update to all WebSocket clients.
     *
     * Constructs a JSON object containing current motor speed, direction,
     * servo angle, and encoder position, then broadcasts it to all connected
     * WebSocket clients.
     *
     * @param[in] motor_speed       Current motor speed (0-100%).
     * @param[in] motor_dir         Current motor direction string.
     * @param[in] servo_angle       Current servo angle (0-180 degrees).
     * @param[in] encoder_count     Current encoder pulse count.
     * @param[in] rpm               Calculated RPM from encoder.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_FAIL on JSON construction or broadcast failure.
     */
    esp_err_t web_server_broadcast_status(int32_t motor_speed,
                                          const char *motor_dir,
                                          int32_t servo_angle,
                                          int32_t encoder_count,
                                          float rpm);

    /**
     * @brief Get the current number of active WebSocket client connections.
     *
     * @return Number of active WebSocket clients (0 to WEB_SERVER_MAX_WS_CLIENTS).
     */
    uint8_t web_server_get_client_count(void);

    /**
     * @brief Check if the web server is currently running.
     *
     * @return true if the server is running, false otherwise.
     */
    bool web_server_is_running(void);

    /**
     * @brief Get a pointer to the server context (for advanced usage).
     *
     * @return Pointer to the internal web_server_ctx_t structure, or NULL
     *         if the server has not been initialized.
     */
    const web_server_ctx_t *web_server_get_context(void);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */

#include "tasks/task_serial_comms.h"
#include "shared_state.h"
#include "serial_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

/*
 * Serial comms over USB CDC (stdin/stdout = /dev/ttyACM0 on Pi).
 *
 * Two-task design:
 *   task_serial_rx  — blocks on fgetc(stdin), pushes raw bytes to s_rx_queue.
 *                     Runs on Core 1, priority 7.
 *   task_serial_comms — 100Hz loop: drains queue, parses packets, sends STATE.
 *                       Called from main, runs on Core 1, priority 8.
 *
 * ESP_LOG is silenced in app_main before this task starts so log output
 * does not corrupt the binary packet stream on stdout.
 */

static const char *TAG = "serial";
static QueueHandle_t s_rx_queue;

static void task_serial_rx(void *arg)
{
    uint8_t chunk[64];
    while (1) {
        /* fread drains all available bytes at once — avoids the 1-byte/ms
         * throughput cap that fgetc+vTaskDelay(1) imposes, which caused the
         * USB CDC RX buffer to overflow at 100Hz CMD_VEL (2200 bytes/s). */
        size_t n = fread(chunk, 1, sizeof(chunk), stdin);
        if (n == 0) { vTaskDelay(1); continue; }
        for (size_t i = 0; i < n; i++)
            xQueueSend(s_rx_queue, &chunk[i], 0);
    }
}

void task_serial_comms(void *arg)
{
    s_rx_queue = xQueueCreate(512, sizeof(uint8_t));
    xTaskCreatePinnedToCore(task_serial_rx, "serial_rx", 2048, NULL, 7, NULL, 1);

    uint8_t rx_buf[256];
    size_t  rx_len     = 0;
    uint32_t last_hb_ms = 0;

    TickType_t last = xTaskGetTickCount();

    while (1) {
        /* drain RX queue into local buffer */
        uint8_t b;
        while (rx_len < sizeof(rx_buf) && xQueueReceive(s_rx_queue, &b, 0) == pdTRUE)
            rx_buf[rx_len++] = b;

        /* parse any complete packets */
        size_t consumed;
        uint8_t type, payload[32], plen;
        while (protocol_decode(rx_buf, rx_len, &consumed, &type, payload, &plen)) {
            if (type == PROTO_TYPE_CMD_VEL && plen == sizeof(proto_cmd_vel_t)) {
                xSemaphoreTake(g_state.mutex, portMAX_DELAY);
                memcpy(&g_state.cmd_vel, payload, sizeof(proto_cmd_vel_t));
                xSemaphoreGive(g_state.mutex);
            } else if (type == PROTO_TYPE_HEARTBEAT) {
                last_hb_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
            memmove(rx_buf, rx_buf + consumed, rx_len - consumed);
            rx_len -= consumed;
        }

        /* watchdog: no heartbeat for 2s → zero setpoints + set error flag */
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool ok = (now - last_hb_ms) < 2000;

        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.watchdog_ok  = ok;
        g_state.error_flags  = ok ? (g_state.error_flags & ~0x01)
                                  : (g_state.error_flags |  0x01);
        if (!ok) memset(&g_state.cmd_vel, 0, sizeof(g_state.cmd_vel));
        /* Snapshot accumulated encoder counts and reset for next period */
        proto_state_t sc = g_state.state;
        for (int i = 0; i < 4; i++) {
            sc.enc_delta[i]      = g_state.enc_accum[i];
            g_state.enc_accum[i] = 0;
        }
        xSemaphoreGive(g_state.mutex);

        /* TX: STATE packet at 100Hz */
        uint8_t frame[32];
        int flen = protocol_encode(PROTO_TYPE_STATE, &sc, sizeof(sc), frame, sizeof(frame));
        if (flen > 0) {
            fwrite(frame, 1, flen, stdout);
            fflush(stdout);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

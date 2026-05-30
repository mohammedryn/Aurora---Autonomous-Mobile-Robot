#include "tasks/task_serial_comms.h"
#include "shared_state.h"
#include "serial_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include <string.h>

/*
 * Uses uart_read_bytes/uart_write_bytes directly — bypasses the ESP-IDF VFS
 * shared lock that deadlocks fread(stdin) vs fwrite(stdout) when called
 * concurrently or sequentially on the same console UART file descriptor.
 *
 * uart_driver_install() must be called in app_main before this task starts.
 */

#define CONSOLE_UART CONFIG_ESP_CONSOLE_UART_NUM

void task_serial_comms(void *arg)
{
    uint8_t rx_buf[256];
    size_t  rx_len    = 0;
    uint32_t last_hb_ms = 0;

    TickType_t last = xTaskGetTickCount();

    while (1) {
        /* --- RX: drain UART ring buffer --- */
        uint8_t chunk[64];
        int n = uart_read_bytes(CONSOLE_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(1));
        if (n > 0 && rx_len + (size_t)n <= sizeof(rx_buf)) {
            memcpy(rx_buf + rx_len, chunk, n);
            rx_len += n;
        }

        /* --- Parse complete packets --- */
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

        /* --- Watchdog: no heartbeat for 2s → zero setpoints --- */
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool ok = (now - last_hb_ms) < 2000;

        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.watchdog_ok = ok;
        g_state.error_flags = ok ? (g_state.error_flags & ~0x01)
                                 : (g_state.error_flags |  0x01);
        if (!ok) memset(&g_state.cmd_vel, 0, sizeof(g_state.cmd_vel));
        proto_state_t sc = g_state.state;
        for (int i = 0; i < 4; i++) {
            sc.enc_delta[i]      = g_state.enc_accum[i];
            g_state.enc_accum[i] = 0;
        }
        xSemaphoreGive(g_state.mutex);

        /* --- TX: STATE packet at 100Hz --- */
        uint8_t frame[32];
        int flen = protocol_encode(PROTO_TYPE_STATE, &sc, sizeof(sc), frame, sizeof(frame));
        if (flen > 0) {
            uart_write_bytes(CONSOLE_UART, (const char *)frame, flen);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

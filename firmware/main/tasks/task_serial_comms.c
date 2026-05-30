#include "tasks/task_serial_comms.h"
#include "shared_state.h"
#include "serial_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

/*
 * All stdin reads and stdout writes happen in THIS task — no concurrent
 * VFS access. The separate task_serial_rx caused VFS lock contention between
 * fread(stdin) and fwrite(stdout) on the shared USB CDC device, silently
 * dropping all incoming bytes.
 */

void task_serial_comms(void *arg)
{
    uint8_t rx_buf[256];
    size_t  rx_len    = 0;
    uint32_t last_hb_ms = 0;

    TickType_t last = xTaskGetTickCount();

    while (1) {
        /* --- RX: drain USB CDC input into rx_buf --- */
        uint8_t chunk[64];
        size_t n = fread(chunk, 1, sizeof(chunk), stdin);
        if (n > 0 && rx_len + n <= sizeof(rx_buf)) {
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
            fwrite(frame, 1, flen, stdout);
            fflush(stdout);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

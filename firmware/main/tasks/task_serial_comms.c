#include "tasks/task_serial_comms.h"
#include "shared_state.h"
#include "serial_protocol.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include <string.h>

#define RX_BUF 256
static uint8_t s_rx[RX_BUF]; static size_t s_rx_len=0;
static uint32_t s_last_hb_ms=0; static int s_tof_div=0;

static void tx(uint8_t type, const void *pl, uint8_t len){
    uint8_t frame[256];
    int flen=protocol_encode(type,pl,len,frame,sizeof(frame));
    if(flen>0) usb_serial_jtag_write_bytes(frame,flen,pdMS_TO_TICKS(5));
}

static void rx_process(void){
    int n=usb_serial_jtag_read_bytes(s_rx+s_rx_len,RX_BUF-s_rx_len,0);
    if(n<=0) return;
    s_rx_len+=n;
    size_t consumed; uint8_t type,payload[32],plen;
    while(protocol_decode(s_rx,s_rx_len,&consumed,&type,payload,&plen)){
        if(type==PROTO_TYPE_CMD_VEL&&plen==sizeof(proto_cmd_vel_t)){
            xSemaphoreTake(g_state.mutex,portMAX_DELAY);
            memcpy(&g_state.cmd_vel,payload,sizeof(proto_cmd_vel_t));
            xSemaphoreGive(g_state.mutex);
        } else if(type==PROTO_TYPE_HEARTBEAT){
            s_last_hb_ms=xTaskGetTickCount()*portTICK_PERIOD_MS;
        }
        memmove(s_rx,s_rx+consumed,s_rx_len-consumed);
        s_rx_len-=consumed;
    }
}

void task_serial_comms(void *arg){
    usb_serial_jtag_driver_config_t cfg={.rx_buffer_size=512,.tx_buffer_size=512};
    usb_serial_jtag_driver_install(&cfg);
    TickType_t last=xTaskGetTickCount();
    while(1){
        rx_process();
        uint32_t now=xTaskGetTickCount()*portTICK_PERIOD_MS;
        bool ok=(now-s_last_hb_ms)<2000;
        xSemaphoreTake(g_state.mutex,portMAX_DELAY);
        g_state.watchdog_ok=ok;
        g_state.error_flags=ok?g_state.error_flags&~0x01:g_state.error_flags|0x01;
        proto_state_t sc=g_state.state; proto_tof_t tc=g_state.tof;
        xSemaphoreGive(g_state.mutex);
        tx(PROTO_TYPE_STATE,&sc,sizeof(sc));
        if(++s_tof_div>=10){s_tof_div=0;tx(PROTO_TYPE_TOF_DATA,&tc,sizeof(tc));}
        vTaskDelayUntil(&last,pdMS_TO_TICKS(10));
    }
}

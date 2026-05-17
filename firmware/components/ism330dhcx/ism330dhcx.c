#include "ism330dhcx.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define PIN_MOSI 36
#define PIN_MISO 37
#define PIN_SCLK 38
#define PIN_CS   39

#define REG_WHO_AM_I 0x0F  /* expect 0x6B */
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G  0x11
#define REG_OUTX_L_G 0x22  /* gyro X low (6 bytes) */
#define REG_OUTX_L_A 0x28  /* accel X low (6 bytes) */
#define READ_FLAG    0x80

#define ACCEL_SENS (0.000122f * 9.80665f)      /* +-4g, LSB -> m/s2 */
#define GYRO_SENS  (0.070f * 3.14159265f/180.f) /* +-2000dps, LSB -> rad/s */

static spi_device_handle_t s_spi;
static float s_gyro_bias[3];

static uint8_t reg_read(uint8_t addr) {
    uint8_t tx[2] = {addr|READ_FLAG, 0}, rx[2];
    spi_transaction_t t = {.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(s_spi, &t);
    return rx[1];
}
static void reg_write(uint8_t addr, uint8_t val) {
    uint8_t tx[2] = {addr&0x7F, val};
    spi_transaction_t t = {.length=16,.tx_buffer=tx};
    spi_device_transmit(s_spi, &t);
}
static void read_6(uint8_t addr, int16_t out[3]) {
    uint8_t tx[7]={addr|READ_FLAG}, rx[7];
    spi_transaction_t t={.length=56,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(s_spi, &t);
    for(int i=0;i<3;i++) out[i]=(int16_t)((rx[2*i+2]<<8)|rx[2*i+1]);
}

bool ism330dhcx_init(void) {
    spi_bus_config_t bc={.mosi_io_num=PIN_MOSI,.miso_io_num=PIN_MISO,
        .sclk_io_num=PIN_SCLK,.quadwp_io_num=-1,.quadhd_io_num=-1};
    spi_device_interface_config_t dc={.clock_speed_hz=8000000,.mode=3,
        .spics_io_num=PIN_CS,.queue_size=1};
    spi_bus_initialize(SPI2_HOST, &bc, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &dc, &s_spi);
    if(reg_read(REG_WHO_AM_I)!=0x6B) return false;
    reg_write(REG_CTRL1_XL, 0x4A); /* 104Hz, +-4g */
    reg_write(REG_CTRL2_G,  0x4C); /* 104Hz, +-2000dps */
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

bool ism330dhcx_read(ism330dhcx_data_t *out) {
    int16_t rg[3],ra[3];
    read_6(REG_OUTX_L_G,rg); read_6(REG_OUTX_L_A,ra);
    for(int i=0;i<3;i++){
        out->gyro[i]  = rg[i]*GYRO_SENS  - s_gyro_bias[i];
        out->accel[i] = ra[i]*ACCEL_SENS;
    }
    return true;
}

void ism330dhcx_calibrate_gyro(void) {
    double sum[3]={0}; const int N=500;
    for(int n=0;n<N;n++){
        ism330dhcx_data_t d; ism330dhcx_read(&d);
        for(int i=0;i<3;i++) sum[i]+=d.gyro[i];
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for(int i=0;i<3;i++) s_gyro_bias[i]=(float)(sum[i]/N);
}

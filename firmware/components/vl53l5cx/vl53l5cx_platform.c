#include "st_uld/vl53l5cx_platform.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT  I2C_NUM_0
#define PIN_SDA   22
#define PIN_SCL   23
#define I2C_FREQ  400000

void vl53l5cx_platform_init(void) {
    i2c_config_t cfg = {.mode=I2C_MODE_MASTER,.sda_io_num=PIN_SDA,
        .scl_io_num=PIN_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=I2C_FREQ};
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t len) {
    uint8_t rb[2] = {reg>>8, reg&0xFF};
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (p->address<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write(c, rb, 2, true);
    i2c_master_start(c);
    i2c_master_write_byte(c, (p->address<<1)|I2C_MASTER_READ, true);
    i2c_master_read(c, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(c);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return r == ESP_OK ? 0 : 1;
}

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t len) {
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (p->address<<1)|I2C_MASTER_WRITE, true);
    uint8_t rb[2] = {reg>>8, reg&0xFF};
    i2c_master_write(c, rb, 2, true);
    i2c_master_write(c, buf, len, true);
    i2c_master_stop(c);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return r == ESP_OK ? 0 : 1;
}

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p, uint16_t a, uint8_t *v) {
    return VL53L5CX_RdMulti(p, a, v, 1);
}
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p, uint16_t a, uint8_t v) {
    return VL53L5CX_WrMulti(p, a, &v, 1);
}
void VL53L5CX_WaitMs(VL53L5CX_Platform *p, uint32_t ms) {
    (void)p; vTaskDelay(pdMS_TO_TICKS(ms));
}

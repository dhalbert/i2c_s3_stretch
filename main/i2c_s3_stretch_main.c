// I2C clock stretching tests
// Assumes an LC709203F sensor.

// This code is in the Public Domain (or CC0 licensed, at your option.)

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "rom/ets_sys.h"

static const char *TAG = "i2c-s3-stretch";

//#define I2C_MASTER_SCL_IO          40      // Adafruit QT Py ESP32-S3 no PSRAM SCL1 STEMMA
//#define I2C_MASTER_SDA_IO          41      // Adafruit QT Py ESP32-S3 no PSRAM SDA1 STEMMA

//#define I2C_MASTER_SCL_IO          4      // Adafruit Feather ESP32-S2
//#define I2C_MASTER_SDA_IO          3      // Adafruit Feather ESP32-S2
//#define I2C_PWR_IO                   7      // I2C Power control

//#define I2C_MASTER_SCL_IO          34      // Adafruit Metro ESP32-S2
//#define I2C_MASTER_SDA_IO          33      // Adafruit Metro ESP32-S2

#define I2C_MASTER_SCL_IO           4      // ESP32-S3-DevKit-C1
#define I2C_MASTER_SDA_IO           5      // ESP32-S3-DevKit-C1

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
//#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */

#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

#define I2C_MASTER_TIMEOUT_MS       1000
#define TIMEOUT                     (I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)

#define I2C_ADDR                 0x0b   // I2C address of LC709203F

// Setup sequence
const uint8_t set_power_mode[] =      { 0x15, 0x01, 0x00, 0x64 }; // Set power mode to Operational.
const uint8_t set_apa[] =             { 0x0b, 0x10, 0x00, 0xa8 }; // Set APA (Adjustment Pack Application).
const uint8_t set_profile[] =         { 0x12, 0x10, 0x00, 0x72 }; // Set Battery Profile.
const uint8_t set_rsoc[] =            { 0x07, 0x55, 0xaa, 0x17 }; // Set RSOC
// Repeat
const uint8_t get_cell_voltage[] =    { 0x09 };                   // Get cell voltage; read three bytes

static esp_err_t i2c_write(const uint8_t *data, size_t len, char* label) {
    esp_err_t ret;

    if (len == 0) {
        // i2c_master_write_to_device() won't do zero-length writes, so we do it by hand.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, I2C_ADDR << 1, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIMEOUT);
        i2c_cmd_link_delete(cmd);
    } else {
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, I2C_ADDR, data, len, TIMEOUT);
    }

    if (ret) {
        ESP_LOGE(TAG, "%s write: %s", label, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t i2c_write_read(const uint8_t *out_data, size_t out_len, uint8_t *in_data, size_t in_len, char* label) {
    esp_err_t ret;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, I2C_ADDR, out_data, out_len, in_data, in_len, TIMEOUT);
    if (ret) {
        ESP_LOGE(TAG, "%s write/read: %s", label, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


#define OTHER_TASK_STACK_SIZE ((3 * configMINIMAL_STACK_SIZE / 2))
StackType_t other_task_stack[OTHER_TASK_STACK_SIZE];
StaticTask_t other_task_taskdef;

// void other_task(void *param) {
//     (void)param;

//     // RTOS forever loop
//     while (1) {
//         ets_delay_us(1);
//         vTaskDelay(1);
//     }
// }

void app_main(void)
{

    // TaskHandle_t task = xTaskCreateStaticPinnedToCore(other_task,
    //     "other",
    //     OTHER_TASK_STACK_SIZE,
    //     NULL,
    //     5,  // Note high prion
    //     other_task_stack,
    //     &other_task_taskdef,
    //     xPortGetCoreID());
    // ESP_LOGI(TAG, "created other task with handle: %p", task);

    #ifdef I2C_PWR_IO
    // Turn on I2C power on boards that control it
    gpio_set_direction(I2C_PWR_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_PWR_IO, true);
    // Wait 0.5 secs for LC709203F to power up.
    ets_delay_us(500000);
    #endif

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    i2c_write(NULL, 0, "wakeup");
    ets_delay_us(1000);
    i2c_write(set_power_mode, sizeof(set_power_mode), "set_power_mode");
    i2c_write(set_apa, sizeof(set_apa), "set_apa");
    i2c_write(set_profile, sizeof(set_profile), "set_profile");
    i2c_write(set_rsoc, sizeof(set_rsoc), "set_rsoc");

    while (1) {
        uint8_t read_data[3];
        i2c_write_read(
            get_cell_voltage, sizeof(get_cell_voltage),
            read_data, sizeof(read_data),
            "get_cell_voltage");
    }
}

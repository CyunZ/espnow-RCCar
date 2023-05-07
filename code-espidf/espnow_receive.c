/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//电机信号引脚
#define output_motor1 18
#define output_motor2 19

//舵机信号引脚
#define LEDC_OUTPUT_IO (33)  
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10位分辨率
#define LEDC_FREQUENCY (50)

// 占空比
// #define LEDC_DUTY_LEFT 26  //1023*0.025
#define LEDC_DUTY_LEFT 52   // 1023*0.05
#define LEDC_DUTY_MIDDLE 75 // 1023*0.075
#define LEDC_DUTY_RIGHT 102 // 1023*0.1
// #define LEDC_DUTY_RIGHT 127  //1023*0.125

static QueueHandle_t msgQ_UD;
static QueueHandle_t msgQ_LR;

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{

    char msg = (char)data[0];
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }
    printf("get msg:%c\n", msg);
    switch (msg)
    {
    case 'U': // 前
    case 'D': // 后
        if (xQueueSend(msgQ_UD, &msg, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Send receive queue fail");
        }
        break;
    case 'L': // 左
    case 'R': // 右
        if (xQueueSend(msgQ_LR, &msg, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Send receive queue fail");
        }
        break;
    }
}

/**
 * 驱动马达任务
 */
void driveTask_UD(void *pvParameter)
{
    char msg = 0;
    BaseType_t xStatus;
    while (1)
    {
        xStatus = xQueueReceive(msgQ_UD, &msg, 0);
        if (pdPASS == xStatus)
        {
            // printf("get msg:%c\n", msg);
            switch (msg)
            {
            case 'U': // 前
                gpio_set_level(output_motor1, 0);
                gpio_set_level(output_motor2, 1);
                break;
            case 'D': // 后
                gpio_set_level(output_motor1, 1);
                gpio_set_level(output_motor2, 0);
                break;
            }
        }
        else
        {
            // 无信息，停止
            gpio_set_level(output_motor1, 0);
            gpio_set_level(output_motor2, 0);
        }
        // 根据实际测试调整接受间隔
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * 驱动舵机任务
 */
void driveTask_LR(void *pvParameter)
{
    char msg = 0;
    BaseType_t xStatus;
    while (1)
    {
        xStatus = xQueueReceive(msgQ_LR, &msg, 0);
        if (pdPASS == xStatus)
        {
            // printf("get msg:%c\n", msg);
            switch (msg)
            {
            case 'L': // 左
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_LEFT);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;
            case 'R': // 右
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_RIGHT);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                break;
            }
        }
        else
        {
            // 无信息，方向回中
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIDDLE);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            // ESP_LOGE(TAG, "no msg");
        }
        // 根据实际测试调整接受间隔
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


static esp_err_t example_espnow_init(void)
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
#endif

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    return ESP_OK;
}

void gpio_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << output_motor1) | (1ULL << output_motor2));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
    msgQ_UD = xQueueCreate(2, sizeof(char));
    msgQ_LR = xQueueCreate(2, sizeof(char));
    if (msgQ_UD == NULL || msgQ_LR == NULL)
    {
        ESP_LOGE(TAG, "Create Queue fail");
        return;
    }
    gpio_init();
    ESP_LOGE(TAG, "gpio_init");
    ledc_init();
    ESP_LOGE(TAG, "ledc_init");
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    example_wifi_init();
    example_espnow_init();
    ESP_LOGE(TAG, "example_espnow_init");
    xTaskCreate(driveTask_UD, "driveTask_UD", 1024 * 5, (void *)msgQ_UD, 4, NULL);
    xTaskCreate(driveTask_LR, "driveTask_LR", 1024 * 5, (void *)msgQ_LR, 4, NULL);
}

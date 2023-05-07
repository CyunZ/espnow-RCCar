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

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_send";

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define input_up  25
#define input_down  26
#define input_left  18
#define input_right  19

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

/**
 * 发送回调
 **/
static void sendCb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (ESP_NOW_SEND_SUCCESS == status)
    {
        ESP_LOGI(TAG, "esp_now send success");
    }
    else
    {
        ESP_LOGE(TAG, "esp_now send fail");
    }
}

static esp_err_t example_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(sendCb));
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

/**
 *  发送前后
 * */
static void sendTask_UD(void *pvParameter)
{
    QueueHandle_t msgQ_UD = (QueueHandle_t)pvParameter;
    BaseType_t xStatus;
    char msg = 0;
    while (1)
    {
        // 一直等待消息，一有就发送
        xStatus = xQueueReceive(msgQ_UD, &msg, portMAX_DELAY);
        if (pdPASS == xStatus)
        {
            ESP_LOGI(TAG, "queue receive success");
            if (esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(char)) != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_now Send error");
            }
            printf("try to send '%c' ",msg);
        }
        else
        {
            ESP_LOGE(TAG, "queue receive fail");
        }
    }
}

static void sendTask_LR(void *pvParameter)
{
    QueueHandle_t msgQ_LR = (QueueHandle_t)pvParameter;
    BaseType_t xStatus;
    char msg = 0;
    while (1)
    {
        // 一直等待消息，一有就发送
        xStatus = xQueueReceive(msgQ_LR, &msg, portMAX_DELAY);
        if (pdPASS == xStatus)
        {
            ESP_LOGI(TAG, "queue receive success");
            if (esp_now_send(broadcast_mac, (uint8_t *)&msg, sizeof(char)) != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_now Send error");
            }
            printf("try to send '%c' ",msg);
        }
        else
        {
            ESP_LOGE(TAG, "queue receive fail");
        }
    }
}

/**
 * 按钮按下
*/
void UD_Task(void *pvParameter)
{
    QueueHandle_t msgQ_UD = (QueueHandle_t)pvParameter;
    // BaseType_t xStatus;
    char msg = 'U';
    int up, down;
    while (1)
    {
        // 获取按键输入
        up = gpio_get_level(input_up);       // 前
        down = gpio_get_level(input_down);   // 后
        // 发送前后，同时按下，前优先
        if (up == 0)
        {
            msg = 'U';
            xQueueSend(msgQ_UD, &msg, 0);
        }
        else if (down == 0)
        {
            msg = 'D';
            xQueueSend(msgQ_UD, &msg, 0);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * 左右按下
*/
void LR_Task(void *pvParameter)
{
    QueueHandle_t msgQ_LR = (QueueHandle_t)pvParameter;
    // BaseType_t xStatus;
    char msg = 'L';
    int  left, right;
    while (1)
    {
        // 获取按键输入
        left = gpio_get_level(input_left);   // 左
        right = gpio_get_level(input_right); // 右

        // 发送左右，同时按下，左优先
        if (left == 0)
        {
            msg = 'L';
            xQueueSend(msgQ_LR, &msg, 0);
        }
        else if (right == 0)
        {
            msg = 'R';
            xQueueSend(msgQ_LR, &msg, 0);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void gpio_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1ULL << input_up) | (1ULL << input_down) | (1ULL << input_left) | (1ULL << input_right));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

void app_main(void)
{

    QueueHandle_t msgQ_UD = xQueueCreate(1, sizeof(char));
    QueueHandle_t msgQ_LR = xQueueCreate(1, sizeof(char));
    if (msgQ_UD == NULL || msgQ_LR ==  NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

    gpio_init();

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

    xTaskCreate(sendTask_UD, "sendTask_UD", 2048, (void *)msgQ_UD, 4, NULL);
    xTaskCreate(sendTask_LR, "sendTask_LR", 2048, (void *)msgQ_LR, 4, NULL);
    xTaskCreate(UD_Task, "UD_Task", 2048, (void *)msgQ_UD, 4, NULL);
    xTaskCreate(LR_Task, "LR_Task", 2048, (void *)msgQ_LR, 4, NULL);
}

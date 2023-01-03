/*	This is modified code from:
*	https://github.com/zhaochenyuangit/ArduCAM-ESP32/blob/master/examples/2_low-resolution/main/network_wifi.c
*/
#include "network_wifi.h"



static const char *TAG = "wifi";
static EventGroupHandle_t wifi_event_group;
static int wifi_retry = 0;

static void my_handler(void *handler_arg, esp_event_base_t base,
                       int32_t id, void *data)
{
    if (base == WIFI_EVENT)
    {
        switch (id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "first time?");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "wifi connected!");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_retry++;
            if (wifi_retry > 3)
            {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                esp_restart();
            }
            else
            {
                ESP_LOGI(TAG, "trying to reconnect......");
                esp_wifi_connect();
            }
            break;
        default:
            ESP_LOGI(TAG, "unhandled wifi event: %d", id);
        }
    }
    else if (base == IP_EVENT)
    {
        switch (id)
        {
        case IP_EVENT_STA_GOT_IP:;// empty statement needed, because label cannot be followed by declaration
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        default:
            ESP_LOGI(TAG, "unhandled ip event: %d", id);
            break;
        }
    }
}

esp_err_t start_wifi(void)
{
    wifi_event_group = xEventGroupCreate();
    /* 0. Initialize NVS*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 1. lwip initialize*/
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &my_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &my_handler, NULL, NULL));

    /*2. wifi config*/
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = MYSSID,
            .password = MYPWD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
    ESP_LOGI(TAG, "init finished");
    /*3. wifi start*/
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bit = xEventGroupWaitBits(wifi_event_group, WIFI_FAIL_BIT | WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bit & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "wifi connection failed");
        vTaskDelay(portMAX_DELAY);
    }
    vEventGroupDelete(wifi_event_group);
    return (ESP_OK);
}

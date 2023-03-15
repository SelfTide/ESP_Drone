#ifndef NETWORK_WIFI
#define NETWORK_WIFI

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MYSSID "SSID"
#define MYPWD "PASSWORRD"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

/**
 * @brief start wifi, reconnect when disconnect
 * @return ESP_OK when success
 */
esp_err_t start_wifi(void);

#ifdef __cplusplus
}
#endif
#endif

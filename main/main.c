#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "esp_http_client.h"

#include "sensors.h"

#define EXAMPLE_WIFI_SSID "aaa"//"A"
#define EXAMPLE_WIFI_PASS "alaa1111"//"joelle123"
const int CONNECTED_BIT = BIT0;

//float temperature_degC;


/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
static const char *TAG = "example";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}


static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}





static void http_rest_with_url()
{
	esp_err_t err;
	esp_http_client_config_t config = {
	    .url = "http://192.168.137.1:8888/api/v1/KfuDRV45yHwOMNymd6Ep/telemetry",
	};
	esp_http_client_handle_t client = esp_http_client_init(&config);
	// first request
	err = esp_http_client_perform(client);

	 char post_data[500];// = "{\"temperature\": }";  //"{\"field1\":\"value1\"}";
	 sprintf((char *)post_data,"{\"temperature\": %.2f}",temperature_degC);
	// sprintf((char *)post_data,"{\"temperature\": %.3f,\"pression\": %4.2f,\"humidite\": %.2f}",temperature_degC,pressure_hPa,humidity_perc);
	    esp_http_client_set_url(client,"http://192.168.137.1:8888/api/v1/KfuDRV45yHwOMNymd6Ep/telemetry");
	    esp_http_client_set_method(client, HTTP_METHOD_POST);
	    esp_http_client_set_header(client, "Content-Type", "application/json");
	    esp_http_client_set_post_field(client, post_data, strlen(post_data));
	    err = esp_http_client_perform(client);

	esp_http_client_cleanup(client);


}
void app_main(void)
{
 nvs_flash_init();
 initialise_wifi();

 ESP_ERROR_CHECK(i2c_master_init());
 init_stts751();
 lps22hh_init();
 hts221_init();

 xTaskCreate(get_temp_task, "task", 10000, NULL, 1, NULL);
 xTaskCreate(get_press, "task2", 10000, NULL, 1, NULL);
 xTaskCreate(get_hum, "task3", 10000, NULL, 1, NULL);
 xTaskCreate(http_rest_with_url, "task4", 10000, NULL, 1, NULL);
while(1)
 {
 http_rest_with_url();
 }
}



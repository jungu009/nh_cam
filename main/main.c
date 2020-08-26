#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_loop.h"

#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_wpa2.h"
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "camera.h"
#include "bitmap.h"

#include "led.h"

static const char* TAG = "nh_camera_main";

/** socket config **/
#ifdef CONFIG_IPV4
#define HOST_IP_ADDR CONFIG_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_IPV6_ADDR
#endif

#define PORT CONFIG_PORT

/** camera config **/
#define CAMERA_PIXEL_FORMAT CAMERA_PF_JPEG
#define CAMERA_FRAME_SIZE CAMERA_FS_UXGA

size_t pic_size;
uint8_t* buffer;

static camera_pixelformat_t s_pixel_format;

/** smart_config **/
static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;


static esp_err_t init_camera()
{
	camera_config_t camera_config = {
	        .ledc_channel = LEDC_CHANNEL_0,
	        .ledc_timer = LEDC_TIMER_0,
	        .pin_d0 = CONFIG_D0,
	        .pin_d1 = CONFIG_D1,
	        .pin_d2 = CONFIG_D2,
	        .pin_d3 = CONFIG_D3,
	        .pin_d4 = CONFIG_D4,
	        .pin_d5 = CONFIG_D5,
	        .pin_d6 = CONFIG_D6,
	        .pin_d7 = CONFIG_D7,
	        .pin_xclk = CONFIG_XCLK,
	        .pin_pclk = CONFIG_PCLK,
	        .pin_vsync = CONFIG_VSYNC,
	        .pin_href = CONFIG_HREF,
	        .pin_sscb_sda = CONFIG_SDA,
	        .pin_sscb_scl = CONFIG_SCL,
	        .pin_reset = CONFIG_RESET,
	        .xclk_freq_hz = CONFIG_XCLK_FREQ,
	    };

	camera_model_t camera_model;
	esp_err_t err = camera_probe(&camera_config, &camera_model);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
		return ESP_FAIL;
	}

	if (camera_model == CAMERA_OV7725) {
		s_pixel_format = CAMERA_PIXEL_FORMAT;
		camera_config.frame_size = CAMERA_FRAME_SIZE;
		ESP_LOGI(TAG, "Detected OV7725 camera, using %s bitmap format",
				CAMERA_PIXEL_FORMAT == CAMERA_PF_GRAYSCALE ?
						"grayscale" : "RGB565");
	} else if (camera_model == CAMERA_OV2640) {
		ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
		s_pixel_format = CAMERA_PIXEL_FORMAT;
		camera_config.frame_size = CAMERA_FRAME_SIZE;
		if (s_pixel_format == CAMERA_PF_JPEG)
		camera_config.jpeg_quality = 15;
	} else {
		ESP_LOGE(TAG, "Camera not supported");
		return ESP_FAIL;
	}

	camera_config.pixel_format = s_pixel_format;
	err = camera_init(&camera_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
		return ESP_FAIL;
	}

	return ESP_OK;
}

static void smartconfig_task(void * parm);

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", HOST_IP_ADDR, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
            int err = send(sock, buffer, pic_size, 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void smartconfig_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");

            //TODO ������Ƭ��������
            led_open();
            esp_err_t err = camera_run();
            if (err != ESP_OK) {
                ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                return;
            }
            led_close();

            pic_size = camera_get_data_size();
            buffer = camera_get_fb();

            ESP_LOGI(TAG, "send picture, size width = %d, height = %d", camera_get_fb_width(), camera_get_fb_height());


            xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);


        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}


void app_main(void)
{
	esp_err_t err = nvs_flash_init();
	if (err != ESP_OK) {
		ESP_ERROR_CHECK( nvs_flash_erase() );
		ESP_ERROR_CHECK( nvs_flash_init() );
	}

	led_init();

	init_camera();

	initialise_wifi();


}


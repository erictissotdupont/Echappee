
/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_sleep.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include <esp_http_server.h>
#include <esp_timer.h>

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <driver/adc.h>


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;
static unsigned int g_getCounter = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
                                                        
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .listen_interval = 1,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}


/* An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    
    g_getCounter++;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        //ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

char g_home_page[4096];
char g_sensors[1024];

#define PUT_CALIBRATE 0x80000000
#define PUT_RESET     0x40000000
uint32_t g_PutEvents = 0;

static const httpd_uri_t homePage = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = g_home_page
};

static const httpd_uri_t sensors = {
    .uri       = "/sensors",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = g_sensors
};


/* An HTTP POST handler */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};

/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /calibrate, the /homePage and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /homePage or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /homePage). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/homePage", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/homePage URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

/* An HTTP PUT handler. This demonstrates realtime
 * registration and deregistration of URI handlers
 */
static esp_err_t calibrate_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    ESP_LOGI( TAG, "CALIBRATE");

    g_PutEvents |= PUT_CALIBRATE;

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t reset_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    ESP_LOGI( TAG, "RESET");

    g_PutEvents |= PUT_RESET;

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t calibrate = {
    .uri       = "/calibrate",
    .method    = HTTP_PUT,
    .handler   = calibrate_put_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t reset = {
    .uri       = "/reset",
    .method    = HTTP_PUT,
    .handler   = reset_put_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &homePage);
        httpd_register_uri_handler(server, &sensors);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &calibrate);
        httpd_register_uri_handler(server, &reset);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

#define ADC_AVG_SAMPLE      64

typedef struct {
    int ADC_id;
    int idx;
    bool cal;
    int offset;
    int sample[ADC_AVG_SAMPLE];
    int sum;
    float coef;
    float value;
    bool readPending;
} tADC_Reading;


uint64_t GetSaved_u64( char* name, uint64_t defaultValue )
{
    esp_err_t err;
    uint64_t ret = defaultValue;
    nvs_handle_t handle;
    
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        uint64_t tmp;
        err = nvs_get_u64(handle, name, &tmp);
        switch (err)
        {
        case ESP_OK:
            ret = tmp;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            break;
        default :
            ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }
        // Close
        nvs_close(handle);
    }
    return ret;
}

bool SetSaved_u64( char* name, uint64_t value )
{
    esp_err_t err;
    nvs_handle_t handle;
    bool ret = false;
    
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        err = nvs_set_u64(handle, name, value );
        if( err != ESP_OK )
        {
            ESP_LOGE(TAG, "Error (%s) setting NVS value %s", esp_err_to_name(err), name );
        }
        else
        {
            err = nvs_commit(handle);
            if( err != ESP_OK )
            {
                ESP_LOGE(TAG, "Error (%s) commiting to NVS", esp_err_to_name(err));
            }
            else
            {
                nvs_close(handle);
                ret = true;
            }
        }
    }
    return ret;
}

bool SetSaved_i32( char* name, int value )
{
    uint64_t u64 = value;
    return SetSaved_u64( name, u64 );
}

int GetSaved_i32( char * name, int defaultValue )
{
    return GetSaved_u64( name, defaultValue );
}

bool SetSaved_float( char* name, float value )
{
    uint64_t u64 = value;
    memcpy( &u64, &value, sizeof(value));
    return SetSaved_u64( name, u64 );
}

float GetSaved_float( char * name, float defaultValue )
{
    float ret;
    uint64_t defaultu64;
    memcpy( &defaultu64, &defaultValue, sizeof(defaultValue));
    uint64_t u64 = GetSaved_u64( name, defaultu64 );
    memcpy( &ret, &u64, sizeof(ret));
    return ret;
}

void sample_ADC( tADC_Reading* pADC, bool calibrate )
{
    // Index rollolver
    if( pADC->idx >= ADC_AVG_SAMPLE )
    {
        ESP_LOGI( TAG, "ADC[%d], %d ", pADC->ADC_id, pADC->sum / ADC_AVG_SAMPLE );
        pADC->idx = 0;
    } 
    // Sample and remove the offset
    pADC->sample[pADC->idx] = adc1_get_raw(pADC->ADC_id);
    // Update the sum of samples
    pADC->sum += pADC->sample[pADC->idx] - pADC->sample[ pADC->idx < ( ADC_AVG_SAMPLE - 1 ) ? pADC->idx + 1 : 0 ];
    if( calibrate && pADC->cal )
    {
        pADC->offset = pADC->sum;
        if( pADC->offset == 0 ) pADC->offset = 1;
    }
    pADC->value = (( pADC->sum - pADC->offset ) / ADC_AVG_SAMPLE ) * pADC->coef;
    pADC->idx++;
}


#define PORT 8080

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

#define LEDC_LS_CH0_GPIO       (18)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0

void app_main(void)
{   
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    /* Configure ADC */
    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(ADC1_CHANNEL_8,ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_0);
    
    printf("ADC configured\n");
        
#define GPIO_SENS_PWR_EN      33
#define GPIO_SENS_PWR_EN_SEL  (1ULL<<GPIO_SENS_PWR_EN)
    
    /* Configure GPIO */
    gpio_config_t io_conf;
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = GPIO_SENS_PWR_EN_SEL;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
    gpio_config(&io_conf);
    
    printf("IO configured\n");
    
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty ( 0 - 8191 )
        .freq_hz = 200,                       // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
        .timer_num = LEDC_TIMER_1,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_LS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    };
    
    ledc_channel_config( &ledc_channel );
    
    printf("LED configured\n");

    /* Connect to WiFi */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    wifi_init_sta();
    
    // Enable sensor power
    gpio_set_level( GPIO_SENS_PWR_EN, 1 );
        
    //ledc_fade_func_install(NULL);
    
    start_webserver();
    
    //xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

    // console.log(data);\n

    const char* JAVASCRIPT = "\
<script>$(function() {\n\
    setInterval(requestData, 1000);\n\
    function requestData() {\n\
        $.get('/sensors')\n\
        .done(function(data) {\n\
            var json = JSON.parse(data);\n\
            console.log(data);\n\
            if (data) {\n\
                $('#bat').text(json.bat);\n\
                $('#gen').text(json.gen);\n\
                $('#soc').text(json.soc);\n\
            } else {\n\
                $('#gen').text('?');\n\
                $('#bat').text('?');\n\
                $('#soc').text('?');\n\
            }\n\
        }).fail(function() {\n\
            console.log('The was a problem retrieving the data.');\n\
        });\n\
    }\n\
});\n\
function calibrate() {\n\
    $.ajax({\n\
        url: '/calibrate',\n\
        type: 'PUT',\n\
        data: 'param1=Value&param2=Value',\n\
        success: function(data) {console.log('Calibrate');}\n\
    });\n\
}\n\
function reset() {\n\
    $.ajax({\n\
        url: '/reset',\n\
        type: 'PUT',\n\
        data: 'param1=Value&param2=Value',\n\
        success: function(data) {console.log('Reset');}\n\
    });\n\
}\n\
</script>";

    strcpy( g_home_page, "<html><head>" );
    
    // For JQuery
                        //<script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.10.2/jquery.min.js\"></script>
    strcat( g_home_page, "<script src=\"https://code.jquery.com/jquery-3.2.1.min.js\"></script>" );
    // For refreshing data
    strcat( g_home_page, JAVASCRIPT );
    strcat( g_home_page, "</head><body>" );
    strcat( g_home_page, "<p>Generator :&nbsp;<span id='gen'></p><p>Battery :&nbsp;<span id='bat'></span></p><p>SOC :&nbsp;<span id='soc'></span></p>" );
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Calibrate' onClick='calibrate()'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Reset' onClick='reset()'>");
    strcat( g_home_page, "</body></html>");

#define GEN_CUR 0
#define BAT_VOL 1
#define BAT_CUR 2
#define MAX_ADC 3

tADC_Reading ADC[MAX_ADC] = {
    { .ADC_id = ADC1_CHANNEL_6, .idx = 0, .offset = 4143 * ADC_AVG_SAMPLE, .cal = true,  .sample = {0}, .sum = 0, .coef = 0.008118 },
    { .ADC_id = ADC1_CHANNEL_4, .idx = 0, .offset = -232 * ADC_AVG_SAMPLE, .cal = false, .sample = {0}, .sum = 0, .coef = 0.001823 },
    { .ADC_id = ADC1_CHANNEL_8, .idx = 0, .offset = 4143 * ADC_AVG_SAMPLE, .cal = true,  .sample = {0}, .sum = 0, .coef = 0.008118 }};

#define BAT_FULL 20.0

    uint32_t count = 0;
    float batCap = 20.0; // BAT_FULL;
    float energy;
    uint64_t lastEnergyTime = esp_timer_get_time( );
    uint64_t nextPIDeval = esp_timer_get_time( ) + 10000;
    int pwmDuty = 0;
    
    ADC[GEN_CUR].offset = GetSaved_i32( "Gen_offset", 4143 * ADC_AVG_SAMPLE );
    ADC[BAT_CUR].offset = GetSaved_i32( "Bat_offset", 4143 * ADC_AVG_SAMPLE );
    
    batCap = GetSaved_float( "BatCap", 20.0f );
    energy = GetSaved_float( "Energy", 0.0f );
    
    printf("Starting main loop\n");

    while( 1 ) // count < 10 )
    {
        count++;
        for( int i=0; i<MAX_ADC; i++)
        {
            sample_ADC( &ADC[i], (g_PutEvents & PUT_CALIBRATE ) == PUT_CALIBRATE );
        }
        
        if( g_PutEvents & PUT_RESET )
        {
            energy = 0.0f;
            g_PutEvents &= ~PUT_RESET;
        }
        
        if( g_PutEvents & PUT_CALIBRATE )
        {
            SetSaved_i32( "Gen_offset", ADC[GEN_CUR].offset );
            SetSaved_i32( "Bat_offset", ADC[BAT_CUR].offset );
            g_PutEvents &= ~PUT_CALIBRATE;
        }
        
        // Wait for having enough samples average to process data
        if( count > ADC_AVG_SAMPLE )
        {
            sprintf( g_sensors, "{\"gen\":\"%.1f A | %d W | %.1f J\",\"bat\":\"%.2f V | %.1f A | %d W\",\"soc\":\"%d %% | %.2f AH | %d %% %u\"}",
                ADC[GEN_CUR].value,
                (int)(ADC[GEN_CUR].value * ADC[BAT_VOL].value),
                energy,
                ADC[BAT_VOL].value, 
                ADC[BAT_CUR].value,
                (int)(ADC[BAT_CUR].value * ADC[BAT_VOL].value),
                (int)(100.0 * batCap / BAT_FULL),
                batCap,
                (int)((100 * pwmDuty) / 8192 ),
                g_getCounter );
            
            uint64_t now = esp_timer_get_time( );
            if( now > nextPIDeval )
            {
                float p,d;
                static float i = 0.0;
                static float pE = 0.0;
                float e = ADC[BAT_VOL].value - 14.1;
                float dT = ( now - lastEnergyTime ) / 1000000.0; // Timer is in uS
                              
                // Joules is W x T (in sec)
                energy += ( ADC[GEN_CUR].value * ADC[BAT_VOL].value * dT );
                
                // Battery capacity is A x T (in hours)
                dT = dT / 3600.0;
                batCap += ( ADC[BAT_CUR].value * dT );
                
                // ESP_LOGI( TAG, "%lld", ( now - lastEnergyTime ));
                lastEnergyTime = now;
                
                p = e * 1.0;
                i += e * 0.1;
                d = ( e - pE ) / 0.1;
                
                if( i < -0.1 ) i = -0.1;
                        
                pwmDuty = 500.0 * p + ( 2000.0 * i ) + ( 1500.0 * d );
                
                if( pwmDuty >= 8192 ) pwmDuty = 8191;
                else if( pwmDuty < 0 ) pwmDuty = 0;
                if( ADC[GEN_CUR].value < 0.5 )
                {
                    pwmDuty = 0;
                }
            
                ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pwmDuty );
                ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
                
                nextPIDeval = nextPIDeval + 100000;
                pE = e;
            }
        }

        vTaskDelay( 10 / portTICK_PERIOD_MS);
    }

}





#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <sys/param.h>

#include "WebServer.h"
#include "PowerManager.h"

static const char *TAG = "Web";

uint32_t g_getCounter = 0;
uint32_t g_PutEvents = 0;

float g_energyGenerated = 0.0;
float g_energySpent = 0.0;
float g_energyStored = 0.0;
float g_distance = 0.0;

int g_bReload = 0;
char g_home_page[4096];
char g_sensors[1024];

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
                $('#speed').text(json.speed);\n\
                $('#energy').text(json.energy);\n\
                if( json.rld == 1 ) {\n\
                  window.location.reload();\n\
                }\n\
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
function onButton(action,param) {\n\
    $.ajax({\n\
        url: '/onButton',\n\
        type: 'PUT',\n\
        data: 'action=' + action + '&param=' + param ,\n\
        success: function(data) {console.log('command');}\n\
    });\n\
}\n\
</script>";

void set_home_page( int flavor )
{
  switch( flavor )
  {
  // ELECTRIC
  case 0 :
    strcpy( g_home_page, "<html><head>" );
    
    // For JQuery
                        //<script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.10.2/jquery.min.js\"></script>
    strcat( g_home_page, "<script src=\"https://code.jquery.com/jquery-3.2.1.min.js\"></script>" );
    // For refreshing data
    strcat( g_home_page, JAVASCRIPT );
    strcat( g_home_page, "</head><body>" );
    strcat( g_home_page, "<p>Generator :&nbsp;<span id='gen'></p><p>Battery :&nbsp;<span id='bat'></span></p><p>SOC :&nbsp;<span id='soc'></span></p>" );
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Calibrate' onClick='onButton(1)'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Reset' onClick='onButton(2)'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Bicycle' onClick='onButton(3)'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Level' onClick='onButton(5)'>");
    strcat( g_home_page, "</body></html>");
    break;
    
  // BICYCLE
  case 1 :
    strcpy( g_home_page, "<html><head>" );
    
    // For JQuery
                        //<script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.10.2/jquery.min.js\"></script>
    strcat( g_home_page, "<script src=\"https://code.jquery.com/jquery-3.2.1.min.js\"></script>" );
    // For refreshing data
    strcat( g_home_page, JAVASCRIPT );
    strcat( g_home_page, "</head><body>" );
    strcat( g_home_page, "<p>Speed :&nbsp;<span id='speed'></p><p>Energy :&nbsp;<span id='energy'></span></p><p>SOC :&nbsp;<span id='soc'></span></p>" );
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Calibrate' onClick='onButton(1)'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Reset' onClick='onButton(2)'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Electric' onClick='onButton(4)'>");
    strcat( g_home_page, "<input id=lfp type='button' style='background-color:lightgray' value='Level' onClick='onButton(5)'>");
    strcat( g_home_page, "</body></html>");
    break;
  }
  
  ESP_LOGI( TAG, "Home page flavor %d is %d bytes long (max %d).", 
    flavor, 
    strlen( g_home_page ),
    sizeof(g_home_page));
  
  g_bReload = 1;
}

/* An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    
    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGD(TAG, "Found header => Host: %s", buf);            
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

    g_bReload = 0;
    g_getCounter++;
    
    return ESP_OK;
}

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

static esp_err_t onButton_put_handler(httpd_req_t *req)
{
    int ret;
    char szReq[100];

    ESP_LOGI( TAG, "RESET");
    
    if( req )
    {
      int action;
      char* pt;
      int remaining = MIN(req->content_len,sizeof(szReq)-1);
      ret = httpd_req_recv(req, szReq, remaining);
      szReq[remaining] = 0;
      
      ESP_LOGI( TAG, "URI:'%s' %d,%d rcv:'%s'", req->uri, remaining, ret, szReq );

      // action=3&param=undefined  
      //
      if(( pt = strstr( szReq, "action=" )) == NULL )
      {
        ESP_LOGE( TAG, "OnButton missing action" );
      }
      else
      {
        if( sscanf( pt + 7, "%d", &action ) != 1 )
        {
          ESP_LOGE( TAG, "Invalid action format" );
        }
        else
        {
          switch( action )
          {
          default:
            ESP_LOGE( TAG, "Unknown button action" );
            break;
          case 1:
            ESP_LOGW( TAG, "CALIBRATE");
            g_PutEvents |= PUT_CALIBRATE;
            break;
          case 2:
            ESP_LOGW( TAG, "RESET");
            g_PutEvents |= PUT_RESET;
            g_energyGenerated = 0.0;
            g_energySpent = 0.0;
            g_distance = 0.0;
            break;
          case 3:
            set_home_page( 1 );
            break;
          case 4:
            set_home_page( 0 );
            break;   
          case 5:
            change_level( );
            break;
          }
        }
      }
    }
    
    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t homePage = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    .user_ctx  = g_home_page
};

static const httpd_uri_t sensors = {
    .uri       = "/sensors",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    .user_ctx  = g_sensors
};

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t onButton = {
    .uri       = "/onButton",
    .method    = HTTP_PUT,
    .handler   = onButton_put_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
      ESP_LOGI(TAG, "Error starting server!");
      return NULL;
    }
    
    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &homePage);
    httpd_register_uri_handler(server, &sensors);
    httpd_register_uri_handler(server, &echo);
    httpd_register_uri_handler(server, &onButton);
    
    set_home_page( 0 );

    return server;
}

#define MAX_VOLTAGE_CHANGE_FOR_SOC  (0.02f)
#define VBAT_FULL (12.8f)
#define VBAT_EMPTY (10.7f)
#define BATT_CAPACITY (12.0 * 50.0f * 3600.0f)

void UpdateSensor( 
  float generatedCurrent,
  float batteryCurrent,
  float batterVoltage,
  int pwmDuty )
{
  uint64_t now = esp_timer_get_time( );
  static uint64_t lastUpdateTime = 0;
  static uint64_t timeToResetSOC = 0;
  float dT = ( now - lastUpdateTime ) / 1000000.0; // Timer is in uS
  // Instant generated power in W
  float genPower = generatedCurrent * batterVoltage;
  
  if( lastUpdateTime != 0 )
  {  
    if( generatedCurrent >= 0.1 )
    {
      g_energyGenerated += ( genPower * dT );
    }
    g_energyStored += ( genPower * dT );
  }
  
  if( abs(generatedCurrent) < 0.5 && 
      abs(batteryCurrent) < 0.5 )
  {
    static float voltageStart;
    
    if( timeToResetSOC == 0 ) 
    {
      int timeToReset = g_energyStored == 0.0 ? 30 : 300;
      timeToResetSOC = now + (timeToReset * 1000000LL);
      voltageStart = batterVoltage;
    }
    else if( now > timeToResetSOC )
    {
      if( abs(voltageStart-batterVoltage) < MAX_VOLTAGE_CHANGE_FOR_SOC )
      {
        if( batterVoltage >= VBAT_FULL ) g_energyStored = BATT_CAPACITY;
        else if( batterVoltage <= VBAT_EMPTY ) g_energyStored = 0.0f;
        else
        {
          float oldEnergy = g_energyStored;
          g_energyStored = BATT_CAPACITY * (batterVoltage-VBAT_EMPTY) / (VBAT_FULL-VBAT_EMPTY);
          
          ESP_LOGW( TAG, "Battery calibrated with %.2fv to %.1f %% SOC (from %.1f)", 
            batterVoltage,
            100.0 * g_energyStored / BATT_CAPACITY,
            100.0 * oldEnergy / BATT_CAPACITY ); 
        }      
      }
      timeToResetSOC = 0;      
    }
  }
  else
  {
    timeToResetSOC = 0;
  }
  
  float SOC = ( 100.0 * g_energyStored ) / BATT_CAPACITY;
   
  char szGen[64];
  char szBat[64];
  char szSOC[64];
  char szSpeed[64];
  char szEnergy[64];
  
  sprintf( szGen, "\"gen\":\"%.1f A | %.0f W | %.1f Wh (%.0f J)\"",
    generatedCurrent,
    genPower,
    g_energyGenerated / 3600.0,
    g_energyGenerated );
  
  sprintf( szBat, "\"bat\":\"%.2f V | %.1f A | %d W\"",
    batterVoltage, 
    batteryCurrent,
    (int)(batteryCurrent * batterVoltage));
  
  sprintf( szSOC, "\"soc\":\"%.0f %% | %.1f Wh | %d %% Level:%u\"",
    SOC,
    g_energyStored / 3600.0,
    (int)((100.0 * pwmDuty) / 8192 ),
    get_level( ));
    
  const struct { 
    float power;
    float speed;
  }PowerToSpeed[] =
   {{  0.0,  0.0 },
    { 25.0, 13.1 },
    { 50.0, 18.8 },
    {100.0, 25.4 },
    {150.0, 29.9 },
    {200.0, 33.5 },
    {250.0, 36.5 },
    {300.0, 39.0 },
    {400.0, 43.3 },
    {600.0, 50.1 },
    {900.0, 57.8 }};
    
  #define HUMAN_EFFICIENCY   (0.25f)  // Affects the calories
  
  #define FUDGE_FACTOR       (1.0f + 0.13f)
  
  // The efficiency of the machine depends on how many coils are
  // connected. 1, 2 or 3  
  const float LevelEfficiency[] = { 2.0, 1.63, 1.5 };
  
  float mechPower = 0.0;
  float speed = 0.0;
  
  if( genPower > 0.1 )
  { 
    mechPower = 15.0 + genPower * LevelEfficiency[get_level( )] * FUDGE_FACTOR;
    
    for( int i=1; i< sizeof(PowerToSpeed) / sizeof(PowerToSpeed[0]); i++ )
    {
      if( PowerToSpeed[i].power >= mechPower )
      {
        float dP = PowerToSpeed[i].power - PowerToSpeed[i-1].power;
        float dS = PowerToSpeed[i].speed - PowerToSpeed[i-1].speed;      
        speed = PowerToSpeed[i-1].speed;
        speed += dS * (( mechPower - PowerToSpeed[i-1].power ) / dP );
        break;
      }
    }  
    g_energySpent += mechPower * dT;
    
    // Accumulated distance in meters
    g_distance += (speed / 3.6) * dT;
  }
    
  sprintf( szSpeed, "\"speed\":\"%.0f km/h | %.0f W (%.0f W)\"",
    speed,
    mechPower,
    genPower    );

  float distanceInKm = g_distance / 1000.0;
  float caloriesBurn = g_energySpent / ( 4184.0 * HUMAN_EFFICIENCY );
    
  if( distanceInKm >= 10.0 )
  {    
    sprintf( szEnergy, "\"energy\":\"%.1f Cal | %.1f km\"",
      caloriesBurn,
      distanceInKm );
  }
  else
  {
    sprintf( szEnergy, "\"energy\":\"%.0f Cal | %.3f km\"",
      caloriesBurn,
      distanceInKm );
  }
  
  sprintf( g_sensors, "{%s,%s,%s,%s,%s,\"rld\":%d}",
    szGen,
    szBat,
    szSOC,   
    szSpeed,
    szEnergy,
    g_bReload );
    
    //ESP_LOGI( TAG, "%s", g_sensors );
    
    lastUpdateTime = now;
}


void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}
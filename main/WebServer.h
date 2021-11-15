
#include "esp_http_server.h"

#define PUT_CALIBRATE 0x80000000
#define PUT_RESET     0x40000000

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);
void UpdateSensor( 
  float generatedCurrent,
  float batteryCurrent,
  float batterVoltage,
  int pwmDuty );
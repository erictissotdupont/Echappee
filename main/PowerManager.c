
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <driver/adc.h>

#include "PowerManager.h"
#include "WebServer.h"
#include "IO_config.h"

static const char *TAG = "Power";

#define GEN_CUR 0
#define BAT_VOL 1
#define BAT_CUR 2
#define MAX_ADC 3

#define ADC_AVG_SAMPLE      32

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

// ----------------------------------------------------------------------------
//   Battery voltage measurment calibration procedure
// ----------------------------------------------------------------------------
// Disconnect from battery and power with regulated variable power supply.
// Rebuild the code with ADC_AVG_SAMPLE set to 256 or more.
// Run program with UART console connected and voltage betweem 5.5 and 6.0v
// Measure exact voltage and raw average ADC reading. Set values below for
// VBAT_CAL1_V and xxx_A respectively. Set voltage to between 14.5 and 15v
// Repeat measurement and set the values for VBAT_CAL2_V and xxx_A. Restore
// ADC_AVG_SAMPLE to previous value.

#define VBAT_CAL1_V 9.07
#define VBAT_CAL1_A 1922
#define VBAT_CAL2_V 15.61
#define VBAT_CAL2_A 8040

#define VBAT_COEF   ((VBAT_CAL2_V-VBAT_CAL1_V) / (VBAT_CAL2_A - VBAT_CAL1_A))
#define VBAT_OFFSET (((VBAT_CAL2_A * VBAT_COEF ) - VBAT_CAL2_V ) / VBAT_COEF )

// ----------------------------------------------------------------------------

tADC_Reading ADC[MAX_ADC] = {
  
    // Generator current
    { .ADC_id = ADC_GENERATOR_CURRENT, 
      .idx = 0, 
      .offset = 7620 * ADC_AVG_SAMPLE, 
      .cal = true,  
      .sample = {0}, 
      .coef = -0.0045 },
      
    // Battery voltage
    { .ADC_id = ADC_BATTERY_VOLTAGE, 
      .idx = 0, .offset = VBAT_OFFSET /*-273*/ * ADC_AVG_SAMPLE, 
      .cal = false, 
      .sample = {0}, 
    .coef = VBAT_COEF },
      //.coef = 0.00180553 },
      
    // Battery current
    { .ADC_id = ADC_BATTERY_CURRENT, 
      .idx = 0, 
      .offset = 7560 * ADC_AVG_SAMPLE, 
      .cal = true, 
      .sample = {0}, 
      .coef = -0.02330 }};
      
      
ledc_channel_config_t ledc_channel = {
      .channel    = LEDC_LS_CH0_CHANNEL,
      .duty       = 0,
      .gpio_num   = LEDC_LS_CH0_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .hpoint     = 0,
      .timer_sel  = LEDC_TIMER_1
  };

static void sample_ADC( tADC_Reading* pADC, bool calibrate )
{
    // Index rollolver
    if( pADC->idx >= ADC_AVG_SAMPLE )
    {
        pADC->idx = 0;
    } 
    // Sample and remove the offset
    pADC->sample[pADC->idx] = adc1_get_raw(pADC->ADC_id);
  
    // Update the sum of samples
    pADC->sum = 0;
    for( int i=0; i<ADC_AVG_SAMPLE; i++ )
    {
      pADC->sum += pADC->sample[i];
    }
    
    if( calibrate && pADC->cal )
    {
        pADC->offset = pADC->sum;
        if( pADC->offset == 0 ) pADC->offset = 1;
    }
    pADC->value = (( pADC->sum - pADC->offset ) / ADC_AVG_SAMPLE ) * pADC->coef;
    pADC->idx++;
    
    if( pADC->idx == ADC_AVG_SAMPLE ) 
    {
      ESP_LOGW( TAG, "ADC[%d] = %d", pADC->ADC_id, pADC->sum / ADC_AVG_SAMPLE );
    }
}

extern uint32_t g_PutEvents;
    
static void adc_read_task(void *pvParameters)
{
  uint32_t count = 0;
  float energy = 0.0;
  uint64_t nextPIDeval = 0;
  uint64_t lastEnergyTime = 0;
  float batCap = 20.0; // BAT_FULL;
  int pwmDuty = 0;
  
  while (1) {
    // ESP_LOGI( TAG, "ADC Read task" );
    
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
        //SetSaved_i32( "Gen_offset", ADC[GEN_CUR].offset );
        //SetSaved_i32( "Bat_offset", ADC[BAT_CUR].offset );
        g_PutEvents &= ~PUT_CALIBRATE;
    }

    // Wait for having enough samples average to process data
    if( count >= ADC_AVG_SAMPLE )
    {
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
        else if( pwmDuty < 819 ) pwmDuty = 0;
        if( ADC[GEN_CUR].value < 0.5 )
        {
            pwmDuty = 0;
        }
        
        //pwmDuty = 4000;
    
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pwmDuty );
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        
        nextPIDeval = nextPIDeval + 100000;
        pE = e;
          
        UpdateSensor(
          ADC[GEN_CUR].value,
          ADC[BAT_CUR].value,
          ADC[BAT_VOL].value,
          pwmDuty );
      }        
    }
    vTaskDelay( 20 / portTICK_PERIOD_MS);    
  }
}

void power_off( )
{
  // Disabled power OFF
  gpio_set_level( GPIO_POWER_OFF, 1 );
}

void start_PowerManager( )
{
  /*
   * Prepare and set configuration of timers
   * that will be used by LED Controller
   */
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty ( 0 - 8191 )
      .freq_hz = 50,                        // frequency of PWM signal
      .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
      .timer_num = LEDC_TIMER_1,            // timer index
      .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
  };
  
  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);
  ledc_channel_config( &ledc_channel );
  ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0 );
  ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
  
  /* Configure ADC */
  adc1_config_width(ADC_WIDTH_BIT_13);
  for( int i=0; i<MAX_ADC; i++)
  {
    adc1_config_channel_atten( ADC[i].ADC_id,ADC_ATTEN_DB_0);
  }
  
  // Configure the pins around the ADC
  gpio_config_t gnd_conf = {
    .intr_type = GPIO_INTR_DISABLE, //disable interrupt
    .mode = GPIO_MODE_OUTPUT, //set as input mode
    .pin_bit_mask = ALL_UNUSED_SEL,
    .pull_down_en = 0, // enable pull-down mode
    .pull_up_en = 0, //disable pull-up mode
  };
  gpio_config(&gnd_conf);  
    
  /* Configure GPIO */
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_DISABLE, //disable interrupt
    .mode = GPIO_MODE_OUTPUT, //set as output mode
    .pin_bit_mask = GPIO_SENS_PWR_EN_SEL | GPIO_POWER_OFF_EN_SEL | GPIO_LEVEL_1_SEL | GPIO_LEVEL_2_SEL,
    .pull_down_en = 0, //disable pull-down mode
    .pull_up_en = 0, //disable pull-up mode
  };
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  // Disabled power OFF
  gpio_set_level( GPIO_POWER_OFF, 0 );
  // Enable sensor power
  gpio_set_level( GPIO_SENS_PWR_EN, 1 );

  xTaskCreate(adc_read_task, "adc_read_task", 4096, NULL, 1, NULL);
  
}
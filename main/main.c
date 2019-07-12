/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include "esp32/ulp.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_event.h"
#include "sdkconfig.h"
#include "nmea_parser.h"
#include "dustsensor_parser.h"
#include "ultrasonic.h"


#define TIME_ZONE (+8)   //Singapore Time
#define YEAR_BASE (2000) //date in GPS starts from 2000


#if defined(CONFIG_DUSTSENSOR_UART_PORT_1)
#define DUSTSENSOR_UART_PORT UART_NUM_1
#elif defined(CONFIG_DUSTSENSOR_UART_PORT_2)
#define DUSTSENSOR_UART_PORT UART_NUM_2
#else
#error Please select the UART Port used by the dust sensor!
#endif

#if defined(CONFIG_GPS_UART_PORT_1)
#define GPS_UART_PORT UART_NUM_1
#elif defined(CONFIG_GPS_UART_PORT_2)
#define GPS_UART_PORT UART_NUM_2
#else
#error Please select the UART Port used by the GPS receiver!
#endif

#define MAIN_TASK_PRIO  10

typedef enum
{
    EV_GPS_UPDATE,
    EV_DUST_DATA_UPDATE,
    EV_WATER_LEVEL_UPDATE,
    EV_TEMP_HUMIDITY_UPDATE,
} main_task_event_t;

typedef struct 
{
    main_task_event_t event;
    union {
        gps_t gps_data;
        dustsensor_t dust_data;
        ultrasonicsensor_t water_level_data;
        unsigned char lora_data[58];
    } data;
} main_task_message_t;

static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR int boot_count = 0;
static nmea_parser_handle_t nmea_hdl;
static dustsensor_parser_handle_t dustsensor_hdl;
static ultrasonicsensor_handle_t floodsensor_hdl;
static QueueHandle_t main_task_queue;
static SemaphoreHandle_t shutdown_sem;

static const char *TAG = "MAIN";

/* Forware declarations of the event handlers */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void dustsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void
*event_data);
static void floodsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void
*event_data);


static void gps_init(void)
{
    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* Set UART and the RX pin */
    config.uart.uart_port = (uart_port_t) GPS_UART_PORT;
    config.uart.rx_pin = CONFIG_GPS_UART_RX_PIN;
    /* init NMEA parser library */
    nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
}

static void gps_deinit(void)
{
    if ( nmea_parser_deinit(nmea_hdl) != ESP_OK)
        ESP_LOGE(TAG, "GPS de-initialization error!\r\n");
}

static void dustsensor_init(void)
{
    /* NMEA parser configuration */
    dustsensor_parser_config_t config = DUSTSENSOR_PARSER_CONFIG_DEFAULT();
    config.uart.uart_port = (uart_port_t) DUSTSENSOR_UART_PORT;
    config.uart.rx_pin = CONFIG_DUSTSENSOR_UART_RX_PIN;
    /* init NMEA parser library */
    dustsensor_hdl = dustsensor_parser_init(&config);
    /* register event handler for NMEA parser library */
    dustsensor_parser_add_handler(dustsensor_hdl, dustsensor_event_handler, NULL);
}

static void dustsensor_deinit(void)
{
   if (dustsensor_parser_deinit( dustsensor_hdl ) != ESP_OK)
       ESP_LOGE(TAG, "Dustsensor de-initialization error!\r\n");
}

static void floodsensor_init(void)
{
    /* NMEA parser configuration */
    ultrasonicsensor_config_t config = ULTRASONICSENSOR_CONFIG_DEFAULT();
    config.rmt.trigger_pin = CONFIG_ULTRASONIC_SENSOR_TRIGGER_PIN;
    config.rmt.echo_pin = CONFIG_ULTRASONIC_SENSOR_ECHO_PIN;
    config.read_interval = CONFIG_ULTRASONIC_SENSOR_READ_INTERVAL;
    /* init NMEA parser library */
    floodsensor_hdl = ultrasonicsensor_init(&config);
    /* register event handler for NMEA parser library */
    ultrasonicsensor_add_handler(floodsensor_hdl, floodsensor_event_handler, NULL);
}

static void floodsensor_deinit(void)
{
   if (ultrasonicsensor_deinit( floodsensor_hdl ) != ESP_OK)
       ESP_LOGE(TAG, "Flood sensor de-initialization error!\r\n");
}


static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
   gps_t *gps = NULL;
   static main_task_message_t msg;
   main_task_message_t *pmsg;
   switch (event_id) {
   case GPS_UPDATE:
       gps = (gps_t *)event_data;
       /* print information parsed from GPS statements */
       ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
        "\tlatitude   = %.05f°N\r\n"
        "\tlongtitude = %.05f°E\r\n"
        "\taltitude   = %.02fm\r\n"
        "\tspeed      = %fm/s",
        gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
        gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
        gps->latitude, gps->longitude, gps->altitude, gps->speed);

        /* Post a message to the main thread */
        msg.event = EV_GPS_UPDATE;
        msg.data.gps_data.date.year = gps->date.year + YEAR_BASE;
        msg.data.gps_data.date.month = gps->date.month;
        msg.data.gps_data.date.day = gps->date.day;
        msg.data.gps_data.tim.hour = gps->tim.hour + TIME_ZONE;
        msg.data.gps_data.tim.minute = gps->tim.minute;
        msg.data.gps_data.tim.second = gps->tim.second;
        msg.data.gps_data.latitude = gps->latitude;
        msg.data.gps_data.longitude = gps->longitude;
        msg.data.gps_data.altitude = gps->altitude;
        msg.data.gps_data.speed = gps->speed;

        pmsg = &msg;
        xQueueSend(main_task_queue, (void *) &pmsg, (TickType_t) 0);

       break;
   case GPS_UNKNOWN:
       /* print unknown statements */
       ESP_LOGW(TAG, "Unknown statement:%s\r\n", (char *)event_data);
       break;
   default:
       break;
   }
}

static void dustsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    dustsensor_t *sensor = NULL;
    static main_task_message_t msg;
    main_task_message_t * pmsg;

    switch (event_id) {
    case SENSOR_UPDATE:
        sensor = (dustsensor_t *)event_data;
        // Handle the data from the sensor here
        ESP_LOGI(TAG, "Concentration Unit (Standard):\r\n"
                "\tPM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum\r\n", 
                sensor->pm1,
                sensor->pm25,
                sensor->pm10);
        ESP_LOGI(TAG, "Concentration Unit (Environmental):\r\n"
                "\tPM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum\r\n", 
                sensor->pm1_atmospheric,
                sensor->pm25_atmospheric,
                sensor->pm10_atmospheric);

        msg.event = EV_DUST_DATA_UPDATE;
        msg.data.dust_data.pm1 = sensor->pm1;
        msg.data.dust_data.pm25 = sensor->pm25;
        msg.data.dust_data.pm10 = sensor->pm10;
        msg.data.dust_data.pm1_atmospheric = sensor->pm1_atmospheric;
        msg.data.dust_data.pm25_atmospheric = sensor->pm25_atmospheric;
        msg.data.dust_data.pm10_atmospheric = sensor->pm10_atmospheric;

        pmsg = &msg;
        xQueueSend(main_task_queue, (void *) &pmsg, (TickType_t) 0);
        
        break;
    case SENSOR_UNKNOWN:
        /* print unknown statements */
        ESP_LOGE(TAG, "Unknown statement:%s\r\n", (char *)event_data);
        break;
    default:
        break;
    }
}

static void floodsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ultrasonicsensor_t *sensor = NULL;
    static main_task_message_t msg;
    main_task_message_t * pmsg;

    switch (event_id) {
    case ULTRASONICSENSOR_UPDATE:
        sensor = (ultrasonicsensor_t *)event_data;
        // Handle the data from the sensor here
        ESP_LOGI(TAG, "Distance : %.2f cm\r\n", sensor->distance_cm);

        msg.event = EV_WATER_LEVEL_UPDATE;
        msg.data.water_level_data.distance_cm = sensor->distance_cm;
        
        pmsg = &msg;
        xQueueSend(main_task_queue, (void *) &pmsg, (TickType_t) 0);

        break;
    case ULTRASONICSENSOR_UNKNOWN:
        /* print unknown statements */
        ESP_LOGI(TAG, "Unknown statement:%s\r\n", (char *)event_data);
        break;
    default:
        break;
    }
}


static void main_task(void * arg)
{
    main_task_message_t * pmsg = NULL;
    int has_gps_data = 0;
    int has_dust_data = 0;
    int has_water_level_data = 0;

    ESP_LOGI(TAG,"Joining TTN ...\r\n");

    while(1)
    {
        xQueueReceive(main_task_queue, (void *) &pmsg, portMAX_DELAY);

        switch(pmsg->event)
        {
            case EV_GPS_UPDATE:
                ESP_LOGI(TAG, "GPS Data received!!\r\n");
                has_gps_data = 1;
                break;
            case EV_DUST_DATA_UPDATE:
                ESP_LOGI(TAG, "Dust sensor data recieved!\r\n");
                has_dust_data = 1;
                break;
            case EV_WATER_LEVEL_UPDATE:
                ESP_LOGI(TAG, "Water level sensor data received!\r\n");
                has_water_level_data = 1;
                break;
            default:
                ESP_LOGE(TAG, "Unknown event type!\r\n");
        }

        /* TODO: Determine if we need to shutdown! */
        if ( has_gps_data && has_dust_data && has_water_level_data )
        {
            ESP_LOGI(TAG, "All data received! ... shutting down main task!!\r\n");
            break;
        }else
            continue;
    }

    /* Done with main task, resume shutdown */
    xSemaphoreGive(shutdown_sem);
    /* Remove this task */
    vTaskDelete(NULL);
}



void app_main()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause())
    {
        case ESP_SLEEP_WAKEUP_TIMER: {
                                         ESP_LOGI(TAG,"Wake up from timer. Time spent in deep sleep: %d ms\r\n", sleep_time_ms);
                                         break;
                                     }
        default:
                                     {
                                         ESP_LOGI(TAG,"Wake up from other sources ....\r\n");
                                         break;
                                     }
    }

    gps_init();
    dustsensor_init();
    floodsensor_init();

    ESP_LOGI(TAG, "Creating main message queue\r\n");
    /* Create the main task message queue */
    main_task_queue = xQueueCreate(10, sizeof(struct main_task_queue_t *));
    if ( main_task_queue == 0)
    {
        ESP_LOGE(TAG, "Failed in creating main task queue!");
    }

    /* Create the shutdown semaphore */
    shutdown_sem = xSemaphoreCreateBinary();

    /* Create the main task */
    xTaskCreate(main_task, "main_task", 4096, NULL, MAIN_TASK_PRIO, NULL);


    /* Wait for the shutdown_sem to be signalled */
    xSemaphoreTake(shutdown_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "Shutdown initiated ....\r\n");
    ++boot_count;

    /* Task Cleanup */
    vQueueDelete(main_task_queue);
    vSemaphoreDelete(shutdown_sem);

    /* Deep Sleep Wakeup Setup */
    ESP_LOGI(TAG, "Deep sleep set for %d seconds ... \r\n", CONFIG_DEEP_SLEEP_INTERVAL);
    const int wakeup_time_sec = CONFIG_DEEP_SLEEP_INTERVAL;
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\r\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);    

    /* Driver de-init */
    gps_deinit();
    dustsensor_deinit();
    floodsensor_deinit();


    /* Deep Sleep */
    ESP_LOGI(TAG, "Entering deep sleep\r\n");
    gettimeofday(&sleep_enter_time, NULL);
    esp_deep_sleep_start();

}


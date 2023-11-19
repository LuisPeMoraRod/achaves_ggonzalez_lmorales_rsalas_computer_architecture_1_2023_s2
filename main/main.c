#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
// #include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"


#define LIGHT_INPUT_PIN ADC1_CHANNEL_0 // pin 0 used as analog input (photoresistor)
#define TEMP_INPUT_PIN ADC1_CHANNEL_1 // pin 1 used as analog input (termistor)

#define LED_0 GPIO_NUM_8 // pin 8 for first LED
#define LED_1 GPIO_NUM_1 // pin 1 for second LED

#define MOTOR_1_A GPIO_NUM_18 // pin 18 for first pin motor driver A
#define MOTOR_1_B GPIO_NUM_19 // pin 19 for second pin motor driver A
#define MOTOR_2_A GPIO_NUM_20 // pin 20 for first pin motor driver B
#define MOTOR_2_B GPIO_NUM_21 // pin 21 for second pin motor driver B

#define FAN_OUT GPIO_NUM_23 //pin 23 for fan output

#define PROXIMITY_DIG_INPUT GPIO_NUM_22 //pin 22 for digital input (infrared proximity sensor)
#define LIGHT_THRESHOLD 1500 //threshold to turn on LEDs
#define TEMP_THRESHOLD 30.0 //threshold to turn fan out



char *TAG = "BLE-Server";
uint8_t ble_addr_type;
uint8_t lockFront = 0;

void ble_app_advertise(void);

// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char * data_raw = (char *)ctxt->om->om_data; //this pointer to string data is not null-terminated
    //It is needed to add null termination to string

    int data_len = ctxt->om->om_len;
    char * data = malloc(data_len + 1);

    if (data == NULL) {
        // Handle allocation failure
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }

    // Copy data to the new string and add null terminator
    memcpy(data, data_raw, data_len);
    data[data_len] = '\0';

    //control motors by reading data sent from client
    if ((strcmp(data, (char *)"1010\0")==0) && (lockFront == 0))
    {
        printf("Move forward\n");
        gpio_set_level(MOTOR_1_A, 1);
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_1_B, 1);
        gpio_set_level(MOTOR_2_B, 0);

    }
    else if (strcmp(data, (char *)"0101\0")==0)
    {
        printf("Move backwards\n");
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_2_A, 1);
        gpio_set_level(MOTOR_1_B, 0);
        gpio_set_level(MOTOR_2_B, 1);
    }
    else if ((strcmp(data, (char *)"1000\0")==0)  && (lockFront == 0))
    {
        printf("Move right\n");
        gpio_set_level(MOTOR_1_A, 1);
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_1_B, 0);
        gpio_set_level(MOTOR_2_B, 0);
    }
    else if ((strcmp(data, (char *)"0010\0")==0)  && (lockFront == 0))
    {
        printf("Move left\n");
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_1_B, 1);
        gpio_set_level(MOTOR_2_B, 0);
    }
    else if (strcmp(data, (char *)"0000\0")==0)
    {
        printf("Stop\n");
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_1_B, 0);
        gpio_set_level(MOTOR_2_B, 0);
    }
    else{
        printf("Unrecognized command: %s\n", data);
    }
    
    //deallocate memory
    free(data);

    return 0;
}

// Read data from ESP32 defined as server
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

// Control LEDs by reading photoresistor analog voltage input
void leds_control(){
    int lightVoltage;
    // Take an ADC1 reading on a single channel (ADC1_CHANNEL_0)
    // 11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 0 - 3.9V
    // 4053 ~ 3.86V
    lightVoltage = adc1_get_raw(LIGHT_INPUT_PIN); 
    printf("Current light: %d\n", lightVoltage);

    if (lightVoltage <= LIGHT_THRESHOLD){
        gpio_set_level(LED_0, 1); //turn LED 0 on
        gpio_set_level(LED_1, 1); //turn LED 1 on
    }else{
        gpio_set_level(LED_0, 0); //turn LED 0 off
        gpio_set_level(LED_1, 0); //turn LED 1 off
    }

}

// Control motors when car aproaches obstacle
void proximity_control(){
    if (gpio_get_level(PROXIMITY_DIG_INPUT)){
        printf("Obstacle approaching...");
        lockFront = 1;
    } else{
        lockFront = 0;
    }
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void temperature_control(){
    /*
    Takes the adc1 channel 1 
    */
    const float Rc = 10000.0; //valor de la resistencia
    const float Vcc = 5.0;
    //const int SensorPIN = A1;

    float A = 1.11492089e-3;
    float B = 2.372075385e-4;
    float C = 6.954079529e-8;

    float K = 2.5; //factor de disipacion en mW/C

    float tempVoltage = (float)adc1_get_raw(TEMP_INPUT_PIN); 
    float tempCorrected = mapFloat(tempVoltage, 0.0, 5000.0, 0.0, 1023.0);
    float V =  tempCorrected / 1024.0 * Vcc;

    float R = (Rc * V ) / (Vcc - V);
  

    float logR  = log(R);
    float R_th = 1.0 / (A + B * logR + C * logR * logR * logR );

    float kelvin = R_th - V*V/(K * R)*1000.0;
    float celsius = kelvin - 277.15;  

    printf("Current temperature: %d\n", celsius);
    
    if (celsius >= TEMP_THRESHOLD){
        gpio_set_level(FAN_OUT, 1); //turn LED 0 on
    }else{
        gpio_set_level(FAN_OUT, 0); //turn LED 0 on
    }

}

void app_main()
{

    //set digital GPIO modes
    gpio_set_direction(LED_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(FAN_OUT, GPIO_MODE_OUTPUT);
    gpio_set_direction(PROXIMITY_DIG_INPUT, GPIO_MODE_INPUT);
    gpio_set_direction(LIGHT_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(TEMP_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(MOTOR_1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_B, GPIO_MODE_OUTPUT);

    //set analog input (light sensor)
    adc1_config_width(ADC_WIDTH_BIT_12); //Configure ADC1 capture width: 12 bit decimal value from 0 to 4095
    adc1_config_channel_atten(LIGHT_INPUT_PIN, ADC_ATTEN_DB_11); //Configure the ADC1 channel (ADC1_CHANNEL_0), and setting attenuation (ADC_ATTEN_DB_11)
    adc1_config_channel_atten(TEMP_INPUT_PIN, ADC_ATTEN_DB_11); //Configure the ADC1 channel (ADC1_CHANNEL_1), and setting attenuation (ADC_ATTEN_DB_11)

    //set BLE connection
    nvs_flash_init();                          // Initialize NVS flash using
    nimble_port_init();                        // Initialize the host stack
    ble_svc_gap_device_name_set("BLE-Server"); // Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // Initialize application
    nimble_port_freertos_init(host_task);      // Run the thread

    // sensors' loop
    while(1)
    {
        
        leds_control();
        proximity_control();
        temperature_control();

        vTaskDelay(500 / portTICK_PERIOD_MS); //100 ms delay
    }
}

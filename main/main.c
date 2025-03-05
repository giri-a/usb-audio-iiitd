#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_private/usb_phy.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2s_functions.h"
#include "data_buffers.h"
#include "uad_callbacks.h"
#include "driver/gpio.h"
#include "blink.h"
#include "utilities.h"

static const char *TAG = "main";

/* entities defined in uad_callbacks.c */
extern uint32_t sampFreq;       // current sampling frequency (reqd in I2S programming)
extern uint8_t clkValid;
extern uint32_t sampleRatesList[]; // sampFreq is one of the vales in this array

extern uint32_t blink_state;

#define I2S_DATA_IN_BUFSIZ (CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX/ 2)
#define I2S_DATA_OUT_BUFSIZ (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/ 2)

// variables declared as extern in data_buffers.h

// Current resolution, update on format change
// number of bytes to be read from I2S every ms

/* data_in_buf is to temporarily store incoming data from I2S (mic).
   It is populated by I2S read function and read by the USB device callback function*/
/* tud_audio_tx_done_pre_cb() copies this data to the endpoint buffer */
size_t  data_in_buf_n_bytes  = 0;      // value is set based on sample rate etc. when the i2s is configured 
int16_t data_in_buf[I2S_DATA_IN_BUFSIZ] = {0};

volatile size_t data_out_buf_n_bytes = 0;
int16_t data_out_buf[I2S_DATA_OUT_BUFSIZ] = {0};

// end extern variables declared in data_buffers.h

TaskHandle_t usb_device_task_handle = NULL; 

void app_main()
{
    BaseType_t ret_val ;

    // Setting up GPIO_1 as input so that we can trigger a txInfodump
    assert(gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT) == ESP_OK);
    assert(gpio_pullup_en(GPIO_NUM_1) == ESP_OK);

    // Provide an initial value for the sampling frequency; later a usb callback function 
    // will set the value as controlled by the USB driver on the host.
    sampFreq = sampleRatesList[0];
    clkValid = 1;

    // Initialize I2S. I2S will start running from this point on.
    ESP_ERROR_CHECK(bsp_i2s_init(I2S_NUM_1, sampFreq));

    // Initialize the number of samples per mS for TX and RX channels
    usb_headset_init();

    // Provide the pointer to the function that the tinyusb stack will call to get I2S mic data.
    usb_get_data = &bsp_i2s_read;

    // Create a task for tinyusb device stack
    ret_val = xTaskCreatePinnedToCore(usb_device_task, "usb_device_task", 3 * 1024, NULL, 2, &usb_device_task_handle,0);
    if (ret_val != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_task");
        //return ESP_FAIL;
        return;
    }
    ESP_LOGI(TAG, "TinyUSB initialized");

    configure_led();

    blink_state = BLINK_NOT_MOUNTED;

    while(1)
    {
        drive_led();

        if(s_spk_active) {
            i2s_transmit();
        }
        else 
            vTaskDelay(pdMS_TO_TICKS(50));

    } 
}
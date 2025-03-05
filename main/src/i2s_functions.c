#include "i2s_functions.h"
#include "data_buffers.h"
#include "esp_err.h"
#include "usb_descriptors.h"
#include "esp_log.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_task_wdt.h"
#include "utilities.h"

static const char* TAG = "i2s_functions";

static i2s_chan_handle_t                tx_handle = NULL;        // I2S tx channel handler
static i2s_chan_handle_t                rx_handle = NULL;        // I2S rx channel handler

#define I2S_GPIO_DOUT    GPIO_NUM_34
#define I2S_GPIO_DIN     GPIO_NUM_36
#define I2S_GPIO_WS      GPIO_NUM_35
#define I2S_GPIO_BCLK    GPIO_NUM_37

/*raw buffer to read data from I2S dma buffers*/
static char rx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];
static size_t  rx_sample_buflen = 0;// value is set based on sample rate etc. when the i2s is configured 

static int32_t tx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/2];

extern int32_t mic_gain[2];
extern int32_t spk_gain[2];

/* For I2S on ESP32 info and how to configure it, please see the documentation at
   https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html .
   We'll use full-duplex mode of I2S.  About one-third down that page you'll find some example code. 
   The following function reuses that code with some modifications.
*/


esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate)
{
    esp_err_t ret_val = ESP_OK;

    i2s_slot_mode_t channel_fmt = I2S_SLOT_MODE_STEREO;

    // default chan_cfg : 
    // { .id = <i2s_num>, .role = <I2S_ROLE_MASTER>, .dma_desc_num = 6, .dma_frame_num = 240, .auto_clear = 0, .intr_priority = 0, }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2;
    // dma_frame_num is changed from dafult value of 240 to reduce latency
    chan_cfg.dma_frame_num = sample_rate/1000;  // number of frames in 1mS; cannot handle sample_rate like 44.1kHz
    chan_cfg.auto_clear_before_cb = true;       // this flag makes sure that only 0 is sent if no more data is provided
    
    ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        //.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, channel_fmt), //this mode seemed to work for SPH0645
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, channel_fmt), // this is needed for INMP441
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_GPIO_BCLK,
            .ws   = I2S_GPIO_WS,
            .dout = I2S_GPIO_DOUT,
            .din  = I2S_GPIO_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ret_val |= i2s_channel_init_std_mode(tx_handle, &std_cfg);
    ret_val |= i2s_channel_init_std_mode(rx_handle, &std_cfg);

    //  dma_desc_num (6) dma buffers of each dma_buffer_size = (dma_frame_num * slot_num * slot_bit_width / 8) bytes
    // read or write will block till a dma buffer i.e., dma_frame_num frames are available or transmitted

    /* raw_buffer is statically allocated for its max required size; 
       but its usable capacity is set here based on the current sample rate. 
       This is required to re-set whenever sampFreq changes.
    */
    rx_sample_buflen  = chan_cfg.dma_frame_num * std_cfg.slot_cfg.slot_mode * std_cfg.slot_cfg.data_bit_width / 8;
    assert(rx_sample_buflen <= sizeof(rx_sample_buf));
    // Even though 32 bits for each data samples are read from I2S (for the specific Mic used), only 16 bits
    // per sample is sent out over USB.
    data_in_buf_n_bytes   = chan_cfg.dma_frame_num * std_cfg.slot_cfg.slot_mode *2 ;
    ESP_LOGI(TAG,"rx_sample_buflen: %d, data_in_buf_n_bytes: %d", rx_sample_buflen, data_in_buf_n_bytes);

    ret_val |= i2s_channel_enable(tx_handle);
    ret_val |= i2s_channel_enable(rx_handle);

    return ret_val;
}

esp_err_t bsp_i2s_reconfig(uint32_t sample_rate)
{
    esp_err_t ret_val = ESP_OK;
    esp_err_t ret_val2 ;
    ret_val |= i2s_channel_disable(rx_handle);
    ret_val |= i2s_channel_disable(tx_handle);
    ret_val |= i2s_del_channel(rx_handle);
    ret_val |= i2s_del_channel(tx_handle);
    ESP_ERROR_CHECK(ret_val2 = bsp_i2s_init(I2S_NUM_1, sample_rate));
    ret_val |= ret_val2;
    //const i2s_std_clk_config_t clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    
    //ret_val |= i2s_channel_reconfig_std_clock(rx_handle, &clk_cfg);
    //ret_val |= i2s_channel_enable(rx_handle);

    // init the offset canceller filter on the read channel
    //decode_and_cancel_offset(NULL, NULL, true);
    
    return ret_val;
}


extern uint32_t sampFreq;

/*
T = 16000 // 62.5uS in .8 format for fs=16kHz
T = 10667 // 41.67uS in .8 format for fs=24kHz
T =  8000 // 31.25 in .8 format for fs=32kHz
T =  5805 // 22.68uS in .8 format for fs=44.1kHz
*/
#define getPeriod(f) (f==16000?16000:(f==24000?10667:(f==32000?8000:5805)))

#define HALF_PERIOD_441HZ 290249 /* in 0.8 format*/
#define HALF_PERIOD_220HZ 581818 /* in 0.8 format*/
#define HALF_PERIOD_100HZ 1280000 /* in 0.8 format*/
#define SIGNAL_HALF_PERIOD HALF_PERIOD_220HZ
#define SIGNAL_ON_DURATION  256000000L /* 1s or 1000000uS in 0.8 format */
#define SIGNAL_OFF_DURATION 256000000L /* 1s or 1000000uS in 0.8 format */
/*
  This function is called by tud_audio_tx_done_post_load_cb(). 
  It reads 32bits raw data for both left and right channels from I2S DMA buffers, 
  gets upper 24bits LSB aligned. After amplification, 16 bits are returned in 
  data_buf. count is the requested number of bytes. Actual number of bytes read is 
  returned.
*/

/*
  This function feeds synthetic data (square wave) to the receive channel 
  bypassing actual I2S receiver.
*/

uint16_t bsp_i2s_read(void *data_buf, uint16_t count /* bytes*/)
{
    int16_t *out_buf = (int16_t*)data_buf;

    int32_t T = getPeriod(sampFreq);
    static int16_t sig_value = 16000;
    static int n_samples_from_last_edge = 0;
    static uint32_t n_samples_from_last_on_edge = 0;
    for(int i = 0; i < count/4; i++){ /* each frame has 4 bytes*/
        if(n_samples_from_last_edge*T > SIGNAL_HALF_PERIOD) {
            sig_value = sig_value == 200 ? -200 : 200;
            n_samples_from_last_edge = 0;
        }
        else
            n_samples_from_last_edge ++;

        if(n_samples_from_last_on_edge*T > (SIGNAL_ON_DURATION + SIGNAL_OFF_DURATION)){
            n_samples_from_last_on_edge  = 0;
        }
        else if(n_samples_from_last_on_edge*T > SIGNAL_ON_DURATION){
            sig_value = 0;
            n_samples_from_last_on_edge ++;
        }
        else 
            n_samples_from_last_on_edge ++;

        *out_buf = sig_value;
        out_buf++;
        *out_buf = sig_value; /*for stereo */
        out_buf++;
    }
    return count;

}

/*
  This function formats the data (16 bits to MSB aligned 32 bits etc..) using a local buffer
  tx_sample_buf and writes to the I2S DMA buffer to be sent out over I2S.
*/
void bsp_i2s_write(void *data_buf, uint16_t n_bytes){

    /* each sample is 32bits and there are 2 channels; so an EP buffer of CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (N) 
     * bytes (each data is 16bits) will produce (N/2)*2=N 32bits total o/p samples for L+R  
     */

    int16_t *src   = (int16_t *)data_buf;
    int16_t *limit = (int16_t *)data_buf + n_bytes/2 ;
    int32_t *dst   = tx_sample_buf;

    size_t num_bytes = 0;

      while (src<limit)
      {
        //COPY *src to *dst converting 16bits to 32 bit in MSB aligned way
        *dst = (int32_t)(*src)<<16;
        dst++; src++;
      }

    // USE i2s_channel_write() to copy the buffer to TX DMA buffers
    // Total number of bytes in tx_sample_buf is n_bytes*2 since each 16bit sample in 
    // data_buf made into a 32bit value.
    if(i2s_channel_write(tx_handle, tx_sample_buf,n_bytes*2,&num_bytes,200) != ESP_OK) {
        ESP_LOGI(TAG,"Some problem writing to TX buffers");
    }
    if(num_bytes < n_bytes*2){
        // Timed out? may be.. issue a message
        ESP_LOGI(TAG,"Timed out? Some problem writing to TX buffers");
    }

}

/* The following variable is declared in tinyusb stack. Its value indicates the number of
   bytes present in 1mS frame of the audio data. Its value is set in one of the functions of 
   the tinyusb stack based on sampling frequency, number of channels (fixed at 2 now) and
   number of bytes in each audio sample (fixed at 2 now). Each call of tud_audio_read() 
   requests 's_spk_bytes_ms' bytes from the USB. 
*/
extern size_t s_spk_bytes_ms;

/* We get s_spk_bytes_ms bytes from our data generator every time; which good for 1mS
*/

void i2s_transmit() {
    // Read s_spk_bytes_ms only even if more bytes may be available
    data_out_buf_n_bytes = bsp_i2s_read(data_out_buf, s_spk_bytes_ms); // data_out_buf etc. are declared in main.c

    if(data_out_buf_n_bytes > 0)
        bsp_i2s_write(data_out_buf, data_out_buf_n_bytes);
    else
        vTaskDelay(pdMS_TO_TICKS(1));
}
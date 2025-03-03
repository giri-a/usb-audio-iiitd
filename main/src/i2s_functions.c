#include "i2s_functions.h"
#include "data_buffers.h"
#include "esp_err.h"
#include "usb_descriptors.h"
#include "esp_log.h"
#include "tusb.h"
#include "tusb_config.h"
#include "esp_task_wdt.h"
#include "utilities.h"

#define MIC_RESOLUTION      24
#define GAIN                4      // in bits i.e., 1 =>2, 2=>4, 3=>8 etc.

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
    data_in_buf_cnt   = chan_cfg.dma_frame_num * std_cfg.slot_cfg.slot_mode *2 ;
    ESP_LOGI(TAG,"rx_sample_buflen: %d, data_in_buf_cnt: %d", rx_sample_buflen, data_in_buf_cnt);

    // preload TX DMA buffer so that upon enabling it does not transmit noise
    // static int32_t tx_sample_buf [CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/2];
    size_t n_bytes = 0;
    for(;n_bytes<CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ/2;n_bytes++){
        tx_sample_buf[n_bytes] = 0;
    }
    i2s_channel_preload_data(tx_handle, (void*)tx_sample_buf, n_bytes*4, &n_bytes);

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
#ifdef SYNTHETIC
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
/*
  This function is called by tud_audio_tx_done_post_load_cb(). 
  It reads 32bits raw data for both left and right channels from I2S DMA buffers, 
  gets upper 24bits LSB aligned. After amplification, 16 bits are returned in 
  data_buf. count is the requested number of bytes. Actual number of bytes read is 
  returned.
*/

/*
  This function feeds synthetic data (sinusoid) to the receive channel 
  bypassing actual I2S receiver.
*/

uint16_t bsp_i2s_read(void *data_buf, uint16_t count /* bytes*/)
{
    int16_t *out_buf = (int16_t*)data_buf;

    int32_t T = getPeriod(sampFreq);
    static int16_t sig_value = 16000;
    static int n_samples_from_last_edge = 0;
    for(int i = 0; i < count/4; i++){ /* each frame has 4 bytes*/
        if(n_samples_from_last_edge*T > SIGNAL_HALF_PERIOD) {
            sig_value = sig_value == 1000 ? -1000 : 1000;
            n_samples_from_last_edge = 0;
        }
        else
            n_samples_from_last_edge ++;
        *out_buf = sig_value;
        out_buf++;
        *out_buf = sig_value; /*for stereo */
        out_buf++;
    }
    return count;

}
#endif

uint16_t bsp_i2s_read(void *data_buf/*16 bit samples*/, uint16_t count /* bytes*/){
    int32_t d_left, d_right;
    int16_t *out_buf = (int16_t*)data_buf;
    size_t n_bytes = 0;
    int i = 0;
    while(i < count/2){
        if(i2s_channel_read(rx_handle,rx_sample_buf,rx_sample_buflen,&n_bytes,200) == ESP_OK) {
            size_t j = 0;
            while(j < n_bytes){
                if(n_bytes - j >= 8){
                    memcpy(&d_left,rx_sample_buf+j,4);
                    memcpy(&d_right,rx_sample_buf+j+4,4);
                    j+=8;
                    out_buf[i++] = d_left >> 14;
                    out_buf[i++] = d_right >> 14;
                }
                else {
                    // we have a problem because we do not seem to have multiple of one frame's worth data
                    ESP_LOGI(TAG,"%d bytes left whereas 8 bytes are needed",(n_bytes-j));
                    break;
                }
            }
        }
        else {
            ESP_LOGI(TAG,"i2s_channel_read returned error!");
        }
    }
    if(i*2 < count){
        ESP_LOGI(TAG,"Returning %d bytes, asked for %d bytes",i*2,count);
    }
    return i*2;
}


/*
  This function formats the data (16 bits to MSB aligned 32 bits etc..) using a local buffer
  tx_sample_buf and writes to the I2S DMA buffer to be sent out over I2S.
  Each sample in tx_sample_buf is int32_t type. For an incoming (i.e. from host) stereo stream,
  tx_sample_buf needs to be n_bytes/2 * 32-bit. 
  Max val of n_bytes is CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (see uad_callbacks.c).
  This function is called by audio_consumer_func() task.
*/
void bsp_i2s_write(void *data_buf, uint16_t n_bytes){

    /* each sample is 32bits and there are 2 channels; so an EP buffer of CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (N) 
     * bytes (each data is 16bits) will produce (N/2)*2=N 32bits total o/p samples for L+R  
     */

    int16_t *src   = (int16_t *)data_buf;
    int16_t *limit = (int16_t *)data_buf + n_bytes/2 ;
    int32_t *dst   = tx_sample_buf;
    int32_t data;

    size_t num_bytes = 0;

      while (src<limit)
      {
        //COPY *src to *dst converting 16bits to 32 bit in MSB aligned way
        *dst = (int32_t)(*src)<<16;
        dst++; src++;

      }

        // USE i2s_channel_write() to copy the buffer to TX DMA buffers
        // total number of bytes in tx_sample_buf is count*2 since each 16bit sample in data_buf
        // made into a 32bit value.
    if(i2s_channel_write(tx_handle, tx_sample_buf,n_bytes*2,&num_bytes,200) != ESP_OK) {
        ESP_LOGI(TAG,"Some problem writing to TX buffers");
    }
    if(num_bytes < n_bytes*2){
        // Timed out? may be issue a message
    }

}

static void speaker_amp (int16_t *s, size_t nframes, int32_t gain[] ){
    //int16_t *t = s;
    if(nframes == 0) return;

    for(int i=0; i<nframes; i++){
        // MULTIPLY signal with gain - gain[0] for left channel, gain[1] for right channel
        // USE mul_1p31x8p24() for this OR simply use bit left shift 
        *s = mul_1p31x8p24(((int32_t)*s)<<16,gain[0]);
        s++;
        *s = mul_1p31x8p24(((int32_t)*s)<<16,gain[1]);
        s++;
    }
}

/* The following function is called by i2s_consumer_func to get data from USB
  to be sent to the I2S transmit DMA buffer.
  This function is expected to populate 'count' number of bytes in the buffer
  (pointed to by data_buf). It may occasionally send fewer bytes. The number of 
  bytes actually filled in the buffer is returned.
*/
uint16_t (*i2s_get_data)(void *data_buf, uint16_t count);

/* Each call of (*i2s_get_data)() requests 's_spk_bytes_ms' bytes from
   the USB. Value of this variable is set based on sampling frequency etc..
   in the USB related functions.
*/
extern size_t s_spk_bytes_ms;

/* This function reads USB EPOUT fifo and writes to I2S tx DMA. Before reading the EPOUT
   fifo, it is made sure that a mutiple of a frame's woth bytes (4 bytes for stereo) are
   available in the fifo. Normally, we would expect a mS worth bytes are available but it
   is not insisted on - only a warning is issued.
   Data are put through a gain stage before calling a formatting cum writing function 
   to I2S TX channel.
*/

/* Checks of there is multiple of a frame's worth data is available to be read at EPOUT fifo */
uint16_t adequate_data_at_epout(){

    // USE 'uint16_t tud_audio_available()' for checking if there are enough bytes at the 
    // USB EPOUT fifo. Enough means non-zero and also multiple of one frame's worth data
    // If this checks out, RETURN number of bytes available or 0
    uint16_t n_bytes = tud_audio_available();
    if(n_bytes == 0 ) {
        //ESP_LOGI(TAG,"0 bytes available to be read at EPOUT fifo");
        return 0;
    }
    if(n_bytes != ((n_bytes>>CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)<<CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)) {
        ESP_LOGI(TAG,"%d bytes available at EPOUT fifo (not multiples of one frame worth bytes)", n_bytes);
        //vTaskDelay(pdMS_TO_TICKS(1));
        return 0;
    }
    return n_bytes;
}

void i2s_transmit() {
    uint16_t n_bytes ;
    if((n_bytes = adequate_data_at_epout()) == 0){
        // We are likely to come here at the start of streaming when the epout buffer
        // may be short of data. So we wait here for longer to have some data buffered
        // rather than regularly running short and starve I2S DMA.
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }
    //USE 'uint16_t tud_audio_read(void *buf, uint16_t n_bytes) to read data from EPOUT buffer
    //may USE speaker_amp() to amplify the signal (i.e., volume control)
    //USE 'bsp_i2s_write(void *buf, uint16_t n_bytes)' to write to I2S TX DMA buffer

    data_out_buf_cnt = tud_audio_read(data_out_buf, n_bytes); // data_out_buf etc. are declared in main.c
    if (data_out_buf_cnt < s_spk_bytes_ms) {
            // Normally we should never land here unless the host is really busy.
            ESP_LOGI(TAG,"Only %d bytes available; expecting >= %d bytes",data_out_buf_cnt,s_spk_bytes_ms);
            vTaskDelay(pdMS_TO_TICKS(1));
    }   
    // Let us amplify signals here. spk_gain is calculated in USB stack. But we are not using USB stack yet.
    spk_gain[0] = 16777216; // 16777216 is 1.0 in 8.24 fixed point format; also 0.25 is 4194304 in the same format
    spk_gain[1] = 16777216; // 16777216 is 1.0 in 8.24 fixed point format
    speaker_amp((int16_t *)data_out_buf, data_out_buf_cnt/(2*CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX), spk_gain);

    if(data_out_buf_cnt > 0)
        bsp_i2s_write(data_out_buf, data_out_buf_cnt);
    else
        vTaskDelay(pdMS_TO_TICKS(1));
}
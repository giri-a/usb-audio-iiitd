#include <math.h>
#include "esp_log.h"
#include "FreeRTOS.h"
#include "freertos/queue.h"
#include "sys/time.h"

char *TAG = "utilities";

int32_t q31_multiply(int32_t a, int32_t b){
    // Q31 format numbers are assumed to be in 1.31 format (ranges from -1 to 0.99999)
    // Multiplication is usual operation. However the result is in 2.62 format and requires 
    // 64 bits to contain. To convert this result to 1.31 format, a left shift is performed
    // and upper 32 bits are returned. 

    int64_t t = (int64_t)a * (int64_t)b;
    t = t<<1;
    // It is a little endian machine; &t points to the LSB and (&t)+3 to the MSB.
    // We are casting &t to the pointer of a int32_t data type so that *p gives
    // lower 32 bits and *(p+1) gives us upper 32 bits.
    int32_t *p = (int32_t*) &t;
    return *++p;
}

int32_t mul_8p24x8p24(int32_t a, int32_t b){
    // Q7.24 format numbers have 8bits left to the binary point and 24 bits to the right.
    // Multiplication result has 16bits to the left of binary point and 48bits to the right
    // and requires 64 bits to be contained. To convert this back to Q7.24, we need to 
    // shift right by 24 places and pick out lower 32 bits OR shift left by 8 places and 
    // pick out the upper 32 bits. A check of the overflow is required for satuarting the
    // result.

    int64_t t = (int64_t)a * (int64_t)b;
    // Overflow check: we'll get upper 16bits and check if it is < -256 or > 255
    // It is a little endian machine; &t points to the LSB and (&t)+3 to the MSB.
    // We are casting &t to the pointer of a int16_t data type so that *p gives
    // lowest 16 bits and *(p+3) gives us highest 16 bits.
    int16_t *p = (int16_t*) &t;
    p += 3;
    if(*p < -128) return 0x80000000;
    if(*p >  127) return 0x7fffffff;

    t = t<<8;

    int32_t *q = (int32_t*) &t;

    return *++q;
}

int16_t mul_1p31x8p24(int32_t sig, int32_t gain)
{
    int64_t t = (int64_t)sig * (int64_t)gain;

    // sig is assumed to be in 1.31 and gain in 8.24 formats.
    // Therefore t must be in 9.55 fotmat. We need to return a signed 16bit
    // data in 1.15 format. So we need shift the result left by 8 positions. 
    // Before we do that we need to make sure that the result is
    // within range of a signed 16bit representation.

    int16_t *p = (int16_t*)&t;
    int16_t m = (*(p+3)) >> 7;  
    // m has the most significant 9 bits of t. We want to make sure that these
    // are all zero's or all 1's. Otherwise there is overflow or underflow.

    if(m < -1) return 0xffff; // most negative in 16bits 
    if(m >  0) return 0x7fff; // most positive in 16bits

    t = t<<8;
    return *(p+3);
}

/*=============== queue code used for debug ===============*/
struct tx_data_info
{
 int64_t timestamp;
 size_t   n_bytes;
} typedef txPacketInfo;

QueueHandle_t txInfoQ;

// Task to create a queue and post a value.
void txInfoQinit( )
{

 // Create a queue capable of containing 1024 txPacketInfo
 // These should be passed by pointer as they contain a lot of data.
 txInfoQ = xQueueCreate( 1024, sizeof( struct tx_data_info ) );
 if( txInfoQ == 0 )
 {
     ESP_LOGE(TAG,"Failed to create Queue");
 }

}
 // this keep writing to the queue discarding older items so that we
 // can get a snapshot of most recent elements in the queue
void put_txPacketInfo(txPacketInfo *data)
{
 char buf[sizeof(txPacketInfo)];
 if(xQueueSend( txInfoQ, ( void * ) data, ( TickType_t ) 0 ) == pdFALSE){
    // if a send failed, queue may be full. In that case, remove an item and try again
    xQueueReceive(txInfoQ, buf, (TickType_t)0);
    if(xQueueSend( txInfoQ, ( void * ) data, ( TickType_t ) 0 ) == pdFALSE){
        // it failed again; so bail out
        ESP_LOGE(TAG,"Failed to send data to queue");
    }

 }

}

void print_txPacketInfo(size_t n_items){
 char buf[sizeof(txPacketInfo)];
 txPacketInfo *p;
    if(n_items == 0) n_items = 1024;
    printf("==== Tx packets info =====\n");
    printf("  timestamp : n_bytes \n");
    for(size_t i=0; i<n_items; i++){
        if(xQueueReceive(txInfoQ, buf, (TickType_t)0) == pdTRUE){
            p = (txPacketInfo*)buf;
            printf("%lld : %d\n",p->timestamp,p->n_bytes);
        }
        else {
            return;
        }
    }
}


void log_txbytes(size_t n_bytes){
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    txPacketInfo s;
    s.timestamp = time_us; //esp_log_timestamp();
    s.n_bytes = n_bytes;
    put_txPacketInfo(&s);
}
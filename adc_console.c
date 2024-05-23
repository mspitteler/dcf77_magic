#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#define CAPTURE_CHANNEL 0

#define CAPTURE_DEPTH 16384

// Replace sample_buf[CAPTURE_DEPTH] with a ping and pong register
static uint16_t sample_buf_ping[CAPTURE_DEPTH];
static uint16_t sample_buf_pong[CAPTURE_DEPTH];
static int32_t bpf_output_buf[CAPTURE_DEPTH];

static uint16_t *volatile sample_buf;

static uint dma_chan;

static void generate_biquad_IIR_bpf(int16_t *const coeffs, const double fs, const double f_pass, double Q) {
    if (Q <= 0.0001)
        Q = 0.0001;

    double w0 = 2. * M_PI * f_pass / fs;
    double c = cos(w0);
    double s = sin(w0);
    double alpha = s / (2. * Q);

    double b0 = s / 2.;
    double b1 = 0.;
    double b2 = -b0;
    double a0 = 1. + alpha;
    double a1 = -2. * c;
    double a2 = 1. - alpha;

    coeffs[0] = lround(b0 / a0 * 16384.); // 2.14 fixed point.
    coeffs[1] = lround(b1 / a0 * 16384.);
    coeffs[2] = lround(b2 / a0 * 16384.);
    coeffs[3] = lround(a1 / a0 * 16384.);
    coeffs[4] = lround(a2 / a0 * 16384.);
    
    printf("Biquad coefficients = { %lf, %lf, %lf, %lf, %lf }\n",
           coeffs[0] / 16384., coeffs[1] / 16384., coeffs[2] / 16384., coeffs[3] / 16384., coeffs[4] / 16384.);
}

void filter_biquad_IIR_bpf(const int16_t *const input, int32_t *const output, const int len,
                           const int16_t *const coeffs, int32_t *const w) {
    for (int i = 0; i < len; i++) {
        int32_t d0 = (int32_t)input[i] - ((coeffs[3] * w[0] + coeffs[4] * w[1]) >> 14);
        output[i] = (coeffs[0] * d0 + coeffs[1] * w[0] + coeffs[2] * w[1]) >> 14;
        w[1] = w[0];
        w[0] = d0;
    }
}

void dma_handler() {
    static bool write_ping = true;
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan;

    if (write_ping) {
        // Ping was being written to, and has completed. Now read from ping, and write to pong.
        write_ping = false;
        sample_buf = sample_buf_ping;
        // Kick off the next transfer.
        dma_channel_set_write_addr(dma_chan, sample_buf_pong, true);
    } else {
        write_ping = true;
        sample_buf = sample_buf_pong;
        // Kick off the next transfer.
        dma_channel_set_write_addr(dma_chan, sample_buf_ping, true);
    }
    // Process what's in the read buffer.
}

int main(void) {
    int16_t bpf_coeffs[5];
    int32_t bpf_w[5] = { 0, 0 };
    
    stdio_init_all();
    sleep_ms(1000);
    
    generate_biquad_IIR_bpf(bpf_coeffs, 0.5e6, 77.5e3, 10000.); // DCF77 frequency.
    
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        true,    // Enable, so that we can check whether a sample contains the error flag
        false    // Keep all 12 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    adc_set_clkdiv(0);

    printf("Arming DMA\n");
    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        sample_buf_ping, // dst
        &adc_hw->fifo,   // src
        CAPTURE_DEPTH,   // transfer count
        false            // do not start immediately, wait for dma_handler() to be called
    );

    printf("Starting capture\n");
    adc_run(true);
    
    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, &dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Manually call the handler once, to trigger the first transfer
    dma_handler();

    uint16_t *prev_sample_buf = sample_buf_pong;
    absolute_time_t prev_time = get_absolute_time();
    while (true) {
        // dma_channel_wait_for_finish_blocking(dma_chan);
        if (sample_buf == NULL || sample_buf == prev_sample_buf)
            continue;
        absolute_time_t processing_start_time = get_absolute_time();
        filter_biquad_IIR_bpf((int16_t *)sample_buf, bpf_output_buf, sizeof(bpf_output_buf) / sizeof(*bpf_output_buf), 
                              bpf_coeffs, bpf_w);
        int64_t avg = 0ll;
        for (int i = 0; i < sizeof(bpf_output_buf) / sizeof(*bpf_output_buf); ++i) {
            avg += bpf_output_buf[i] < 0 ? -bpf_output_buf[i] : bpf_output_buf[i];
            // printf("%" PRIi32 "\n", bpf_output_buf[i]);
            // if (i % 10 == 9)
            //     printf("\n");
        }
        avg /= (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf));
        printf("%" PRIi32 "\n", (int32_t)avg);
        absolute_time_t processing_end_time = get_absolute_time();
        // printf("total us: %llu, used us: %llu\n", to_us_since_boot(processing_end_time) - to_us_since_boot(prev_time),
        //        to_us_since_boot(processing_end_time) - to_us_since_boot(processing_start_time));
        prev_time = processing_end_time;
        prev_sample_buf = sample_buf;
    }
    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();
}

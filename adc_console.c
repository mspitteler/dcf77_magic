#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#define CAPTURE_CHANNEL 0

#define CAPTURE_DEPTH 16384

#define ROUNDING_VALUE 0x7fffffffll

// Replace sample_buf[CAPTURE_DEPTH] with a ping and pong register
static uint16_t sample_buf_ping[CAPTURE_DEPTH];
static uint16_t sample_buf_pong[CAPTURE_DEPTH];
static int16_t fir_output_buf[CAPTURE_DEPTH];

static uint16_t *volatile sample_buf;

static uint dma_chan;

static inline double __attribute__((always_inline)) sinc(double x) {
    return x == 0. ? 1. : sin(M_PI * x) / (M_PI * x);
}

static double get_wind_blackman_coeff(const int i, const int len) {
    const double a0 = 0.42;
    const double a1 = 0.5;
    const double a2 = 0.08;

    double len_mult = 1. / (double)(len - 1);
    return a0 - a1 * cos(i * 2. * M_PI * len_mult) + a2 * cos(i * 4. * M_PI * len_mult);
}

static void generate_FIR_coeffs(int32_t *fir_coeffs, const uint fir_len, const double fs, const double f_pass) {
    // Even or odd length of the FIR filter
    const bool is_odd = (fir_len % 2) ? true : false;
    const double fir_order = (double)(fir_len - 1);
    
    const double w1 = 2. * (f_pass / fs - 1. / fir_len);
    const double w2 = 2. * (f_pass / fs + 1. / fir_len);
    
    printf("Generating FIR coefficients");
    
    for (int i = 0; i < fir_len; i++) {
        double t = (double)i - fir_len / 2.;
        double coeff = w1 * sinc(w1 * t) - w2 * sinc(w2 * t);
        coeff *= get_wind_blackman_coeff(i, fir_len);
        fir_coeffs[i] = coeff * 2147483648.; // 1.31 fixed point.
        
        // printf("%e, ", fir_coeffs[i] / 2147483648.);
        // if (i % 10 == 9)
        //     printf("\n");
        if (i % (fir_len / 10) == 0)
            putchar('.');
        fflush(stdout);
    }
    printf("\n");
}

void fixed_FIR_filter_s16_coeffs_1_31(const int32_t *const coeffs, const int coeffs_len, int16_t *const delay,
                                      const int16_t *const input, int16_t *const output, const int len) {
    static int pos = 0;
    int result = 0;
    int input_pos = 0;

    for (int i = 0; i < len; i++) {
        if (pos >= coeffs_len)
            pos = 0;
        delay[pos++] = input[input_pos++];

        int64_t acc = ROUNDING_VALUE;
        int coeff_pos = coeffs_len - 1;

        for (int n = pos; n < coeffs_len; n++)
            acc += (int64_t)coeffs[coeff_pos--] * (int64_t)delay[n];
        for (int n = 0; n < pos; n++)
            acc += (int64_t)coeffs[coeff_pos--] * (int64_t)delay[n];

        output[result++] = (int32_t)(acc >> 31);
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
    const int fir_len = 4096;
    int32_t fir_coeffs[fir_len];
    int16_t fir_delay[fir_len];
    
    stdio_init_all();
    sleep_ms(1000);
    
    generate_FIR_coeffs(fir_coeffs, fir_len, 0.5e6, 77.5e3); // DCF77 frequency.
    memset(fir_delay, 0, sizeof(fir_delay) / sizeof(*fir_delay));
    
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
    while (true) {
        // dma_channel_wait_for_finish_blocking(dma_chan);
        if (sample_buf == NULL || sample_buf == prev_sample_buf)
            continue;
        fixed_FIR_filter_s16_coeffs_1_31(fir_coeffs, fir_len, fir_delay, (int16_t *)sample_buf, fir_output_buf, 
                                         CAPTURE_DEPTH);
        for (int i = 0; i < CAPTURE_DEPTH; ++i) {
            printf("%1.5f, ", fir_output_buf[i] * (3.3f / 4096.f));
            if (i % 10 == 9)
                printf("\n");
        }
        prev_sample_buf = sample_buf;
    }
    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();
}

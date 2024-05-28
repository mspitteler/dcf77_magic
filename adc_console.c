#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/divider.h"
#include "pico/multicore.h"

#define CAPTURE_CHANNEL 0

#define CAPTURE_DEPTH 20000

// Replace sample_buf[CAPTURE_DEPTH] with a ping and pong register
static uint16_t sample_buf_ping[CAPTURE_DEPTH];
static uint16_t sample_buf_pong[CAPTURE_DEPTH];
static int16_t bpf_output_buf[CAPTURE_DEPTH];

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

void filter_biquad_IIR(const int16_t *const input, int16_t *const output, const int len,
                       const int16_t *const coeffs, int32_t *const w) {
    for (int i = 0; i < len; i++) {
        // We have to use 64-bit multiplies here sadly, there is not really a way around it if we want high Q,
        // but it's only two luckily, the other three coefficients can be multiplied with the coefficients downscaled
        // by 5 bits, since they only affect the output amplitude, and not the behaviour of the filter internally.
        int32_t d0 = ((int64_t)input[i] - (((int64_t)coeffs[3] * w[0] + (int64_t)coeffs[4] * w[1]) >> (14 - 5))) >> 5;
        output[i] = (coeffs[0] * d0 + coeffs[1] * w[0] + coeffs[2] * w[1]) >> 14;
        w[1] = w[0];
        w[0] = d0;
    }
}

// `conv_out' will be wrapped to have the same length as the kernel.
void conv_wrapped(const int32_t *const signal, const int sig_len, const int32_t *const kernel, const int kern_len,
                  int32_t *const conv_out) {

    const int32_t *sig = signal;
    const int32_t *kern = kernel;
    int lsig = sig_len;
    int lkern = kern_len;
    
    memset(conv_out, 0, (sig_len + kern_len - 1) * sizeof(*conv_out));

    if (sig_len < kern_len) {
        sig = kernel;
        kern = signal;
        lsig = kern_len;
        lkern = sig_len;
    }

    for (int n = 0; n < lkern; n++) {
        size_t k;

        for (k = 0; k <= n; k++)
            conv_out[n] += sig[k] * kern[n - k];
    }
    for (int n = lkern; n < lsig; n++) {
        size_t kmin, kmax, k;

        kmin = n - lkern + 1;
        kmax = n;
        for (k = kmin; k <= kmax; k++)
            conv_out[n] += sig[k] * kern[n - k];
    }

    for (int n = lsig; n < lsig + lkern - 1; n++) {
        size_t kmin, kmax, k;

        kmin = n - lkern + 1;
        kmax =  lsig - 1;

        for (k = kmin; k <= kmax; k++)
            conv_out[n] += sig[k] * kern[n - k];
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

static const int bcd_table[] = { 1, 2, 4, 8, 10, 20, 40, 80 };

void process_bit(uint64_t timestamp, int bit, bool bit_value) {
    static bool backup_antenna = false, announce_dst_switch = false, dst = false, announce_leap_second = false;
    static int minutes = -1, hours = -1, day_of_month = -1, day_of_week = -1, month_num = -1, year = -1;
    
    static bool parity = false;
    switch (bit) {
        case 0:
            if (bit_value != false)
                printf("%" PRIu32 ": Bit 0 should always be 0!!!\n", us_to_ms(timestamp));
            break;
        case 1 ... 14:
            // Meteorological data.
            break;
        case 15:
            // Whether the backup antenna is in use.
            backup_antenna = bit_value;
            break;
        case 16:
            // Is 1 during the hour before switching.
            announce_dst_switch = bit_value;
        case 17 ... 18:
            // DST or no DST.
            break;
        case 19:
            // Is 1 during the hour before the leap second is inserted.
            announce_leap_second = bit_value;
            break;
        case 20:
            if (bit_value != true)
                printf("%" PRIu32 ": Bit 20 should always be 1!!!\n", us_to_ms(timestamp));
            break;
        case 21 ... 27:
            // Minutes.
            if (bit == 21) {
                minutes = 0;
                parity = false;
            }
            minutes += (int)bit_value * bcd_table[bit - 21];
            parity ^= bit_value;
            break;
        case 28:
            // Bit 21 ... 27 even parity.
            if (bit_value != parity)
                printf("%" PRIu32 ": Parity error in bits 21 ... 27\n", us_to_ms(timestamp));
            break;
        case 29 ... 34:
            // Hours.
            if (bit == 29) {
                hours = 0;
                parity = false;
            }
            hours += (int)bit_value * bcd_table[bit - 29];
            parity ^= bit_value;
            break;
        case 35:
            // Bit 29 ... 34 even parity.
            if (bit_value != parity)
                printf("%" PRIu32 ": Parity error in bits 29 ... 34\n", us_to_ms(timestamp));
            break;
        case 36 ... 41:
            // Day of month.
            if (bit == 36) {
                day_of_month = 0;
                parity = false;
            }
            day_of_month += (int)bit_value * bcd_table[bit - 36];
            parity ^= bit_value;
            break;
        case 42 ... 44:
            // Day of week (1 = Monday, 2 = Tuesday, ..., 7 = Sunday).
            if (bit == 42)
                day_of_week = 0;
            day_of_week += (int)bit_value * bcd_table[bit - 42];
            parity ^= bit_value;
            break;
        case 45 ... 49:
            // Month number.
            if (bit == 45)
                month_num = 0;
            month_num += (int)bit_value * bcd_table[bit - 45];
            parity ^= bit_value;
            break;
        case 50 ... 57:
            if (bit == 50)
                year = 0;
            year += (int)bit_value * bcd_table[bit - 50];
            parity ^= bit_value;
            // Year.
            break;
        case 58:
            // Bit 36 ... 57 even parity.
            if (bit_value != parity)
                printf("%" PRIu32 ": Parity error in bits 36 ... 57\n", us_to_ms(timestamp));
            printf("%02d:%02d, %d, %02d-%02d-%02d\n", hours, minutes, day_of_week, day_of_month, month_num, year); 
            break;
        case 59 ... INT_MAX:
            // Invalid.
            break;
    }
}

// Kernel is the time inverted expected signal (1 period is 1s):
//
//     1 -  ________       ________
//                  |     |
//  0.75 -          |     |
//                  |     |
//   0.5 -          |__   |
//              0.43 ^ |  |
//  0.25 -             |  |
//                     |  |
//     0 -             '--'
//
static const int32_t kernel[] = {
    [0 ... (int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 0.4 - 1.)] = 7,
    [(int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 0.4) ...
        (int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 0.5 - 1.)] = 3,
    [(int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 0.5) ...
        (int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 0.6 - 1.)] = 0,
    [(int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 0.6) ...
        (int)(0.5e6 / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) / 4.) * 1. - 1.)] = 7,
};

void core1_main(void) {
    uint64_t prev_timestamp = to_us_since_boot(at_the_end_of_time);
    enum { UNPROCESSED, PROCESSED, INVALID } bit_status = INVALID;
    uint32_t prev_diff_mods[2] = { 0u, 0u };
    int32_t avg_avg = 0;
    bool prev_level = false;
    
    for (int bit = 0;;) {
        uint64_t timestamp;
        ((uint32_t *)&timestamp)[0] = multicore_fifo_pop_blocking();
        ((uint32_t *)&timestamp)[1] = multicore_fifo_pop_blocking();
        int32_t avg = (int32_t)multicore_fifo_pop_blocking();
        
        avg_avg = ((avg_avg * 511) + avg + 255) >> 9;
        // 75% of the average seems to be a good threshold.
        bool level = avg > avg_avg * 3 / 4;
        
        if (level == prev_level) {
            prev_level = level;
            continue;
        }
        prev_level = level;
        
        if (!level) { // Going down.
            uint32_t diff_mod = (timestamp - prev_timestamp) % 1000000ull;
            /**
             * Is a valid bit if it is either:
             * -    The 1st bit since boot.
             * -    Approximately a whole number of seconds after the previous valid bit.
             * -    Not a whole number of seconds after the previous valid bit, but the previous 2 invalid bits
             *      have approximately the same modulo, which means that we at some point have missed a lot of bits,
             *      or that we triggered to a dip caused by noise.
             */
            if (is_at_the_end_of_time(from_us_since_boot(prev_timestamp)) ||
                diff_mod < 50000u || 1000000u - diff_mod < 50000u || // Shouldn't be off more than 50ms.
                (labs((int32_t)prev_diff_mods[0] - (int32_t)diff_mod) < 50000l &&
                    labs((int32_t)prev_diff_mods[1] - (int32_t)diff_mod) < 50000l)) {
                if (timestamp - prev_timestamp > 1950000ull && timestamp - prev_timestamp < 2050000ull &&
                    bit_status != INVALID) {
                    // printf("%" PRIu32 ": Sync pulse!\n", us_to_ms(timestamp));
                    bit = 0;
                }
                prev_timestamp = timestamp;
                bit_status = UNPROCESSED;
            }
            prev_diff_mods[1] = prev_diff_mods[0];
            prev_diff_mods[0] = diff_mod;
        } else if (bit_status == UNPROCESSED) { // Going up.
            uint32_t diff = timestamp - prev_timestamp;
            uint32_t prev_timestamp_ms = us_to_ms(prev_timestamp);
            bool bit_value = false;
            
            if (diff > 300000u) { // Too long, should be either 200ms or 100ms.
                // printf("%" PRIu32 ": Too long!!!\n", prev_timestamp_ms);
                bit_status = INVALID;
            } else if (diff > 150000u) {
                // printf("%" PRIu32 ": Bit %d, high\n", prev_timestamp_ms, bit);
                bit_value = true;
                bit_status = PROCESSED;
            } else {
                // printf("%" PRIu32 ": Bit %d, low\n", prev_timestamp_ms, bit);
                bit_status = PROCESSED;
            }
            
            if (bit_status == PROCESSED) {
                // process_bit(prev_timestamp, bit, bit_value);
            } else {
                
            }
            bit++;
        }
    }
}

int main(void) {
    int16_t bpf_coeffs[5];
    int32_t bpf_w[5] = { 0, 0 };
    
    stdio_init_all();
    sleep_ms(1000);
    
    multicore_launch_core1(&core1_main);
    
    generate_biquad_IIR_bpf(bpf_coeffs, 0.5e6, 77.5e3, 15000.); // DCF77 frequency.
    
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
    uint32_t adc_offset = 0u;
    while (true) {
        // dma_channel_wait_for_finish_blocking(dma_chan);
        if (sample_buf == NULL || sample_buf == prev_sample_buf)
            continue;
        absolute_time_t processing_start_time = get_absolute_time();
        int32_t full_spectrum_avg = 0;
        int16_t *signed_sample_buf = (int16_t *volatile)sample_buf;
        
        /**
         * Subtract ADC offset and calculate the average amplitude of the whole spectrum.
         * We use mean of absolute values instead of RMS here because for that we would need to 
         * multiply two 32-bit numbers, which would result in a 64-bit number, for this the 
         * Cortex M0+ doesn't have hardware multiplication, so a single multiply would take
         * 16 clock cycles, which is way to slow.
         */
        for (int i = 0; i < sizeof(bpf_output_buf) / sizeof(*bpf_output_buf); ++i) {
            // Exponential moving average, based on the formula s[t] = alpha * x[t] + (1 - alpha) * s[t - 1],
            // where alpha is chosen as 1 / 2048 in this case. `sample_buf' is converted to 23.9 fixed point
            // first. Before bitshifting right by 11, we add 0.5 * 2^11 - 1 to round.
            adc_offset = ((adc_offset * 2047u) + ((uint32_t)sample_buf[i] << 9) + 1023u) >> 11;
            signed_sample_buf[i] -= ((adc_offset + 255u) >> 9); // Add 0.5 * 2^9 - 1 to round.
            full_spectrum_avg += signed_sample_buf[i] < 0 ? -signed_sample_buf[i] : signed_sample_buf[i];
            // printf("%" PRIi16 "\n", sample_buf[i]);
            // if (i % 10 == 9)
            //     printf("\n");
        }
        hw_divider_divmod_s32_start(full_spectrum_avg, sizeof(bpf_output_buf) / sizeof(*bpf_output_buf));

        filter_biquad_IIR(signed_sample_buf, bpf_output_buf, sizeof(bpf_output_buf) / sizeof(*bpf_output_buf), 
                          bpf_coeffs, bpf_w);
        
        // The division of the full spectrum average should be done by now (takes 8 clockcycles).
        full_spectrum_avg = hw_divider_s32_quotient_wait();

        for (int i = 0; i < 4; i++) {
            /**
             * Calculate average amplitude, we use mean of absolute values instead of RMS again.
             * You might say that we could also do this immediately in the filter, so that we don't
             * need to store all the samples at the output of the buffer (which takes up 32KiB of RAM),
             * but I tested it and it is not significantly slower to do it seperately, and we have enough
             * RAM anyway, so it is more elegant to do it seperately I think.
             */
            static int32_t avgs[4] = { 0, 0, 0, 0 };
            avgs[i] = 0;
            for (int j = sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) * i / 4;
                 j < sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) * (i + 1) / 4; ++j)
                avgs[i] += bpf_output_buf[j] < 0 ? -bpf_output_buf[j] : bpf_output_buf[j];
            // We're not dividing by a power of 2 anymore here, but apparently the / operator is using the
            // hardware divider anyway, so it will only take 8 cycles.
            int32_t avg = (avgs[0] + avgs[1] + avgs[2] + avgs[3]) / (sizeof(bpf_output_buf) / sizeof(*bpf_output_buf));
            /**
             * TODO: Check ratio between average after bandpass filter and average of full spectrum, so that we can
             * estimate how close the DCF77 signal is to the noise floor.
             */
            
            printf("%" PRIi32 "\n", avg);
            // This is not very elegant, but it is better than giving all 4 upsampled averages the same timestamp:
            // We just choose the time in the middle of where we are taking the average. The round brackets are the
            // previous buffer and the square brackets are the current buffer.
            //                  _____|____ -20000us
            // i = 3:          |          |
            //               _____|____ -30000us
            // i = 2:       |          |
            //            _____|____ -40000us
            // i = 1:    |          |
            //         _____|____ -50000us
            // i = 0: |          |
            //     ( )( )( )( )[ ][ ][ ][ ]
            //                            |
            //                      proc_start_time
            uint64_t timestamp = to_us_since_boot(processing_start_time) -
                // 1000000ull is the amount of microseconds in a second and 500000ull is the sample rate in Hz.
                1000000ull / 500000ull * sizeof(bpf_output_buf) / sizeof(*bpf_output_buf) * (2 + 3 - i) / 4;
            // Send to the other core to process.
            multicore_fifo_push_blocking(((uint32_t *)&timestamp)[0]);
            multicore_fifo_push_blocking(((uint32_t *)&timestamp)[1]);
            multicore_fifo_push_blocking((uint32_t)avg);
        }
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

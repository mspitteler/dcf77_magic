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
#include "hardware/rtc.h"

#define CAPTURE_CHANNEL 0

#define CAPTURE_DEPTH 20000
#define MATCHED_FILTER_N_SECONDS 300

#define N_ELEM(a) (sizeof(a) / sizeof(*(a)))
#define MAX_IDX(a, len)                                                                                               \
    ({                                                                                                                \
        typeof(*(a)) max = _Generic((a), float *: -infinityf(), double *: -infinity(), long double *: -infinityl(),   \
                                    /* We need both char and int8_t because char can both be signed and unsigned. */  \
                                    bool *: false, char *: CHAR_MIN, int8_t *: INT8_MIN, int16_t *: INT16_MIN,        \
                                    int32_t *: INT32_MIN, int64_t *: INT64_MIN, default: 0);                          \
        int max_i = 0;                                                                                                \
        for (int i = 0; i < len; i++) {                                                                               \
            if ((a)[i] > max) {                                                                                       \
                max_i = i;                                                                                            \
                max = (a)[i];                                                                                         \
            }                                                                                                         \
        }                                                                                                             \
        max_i;                                                                                                        \
    })

enum bit_value_or_sync { LOW, HIGH, SYNC };

// Replace sample_buf[CAPTURE_DEPTH] with a ping and pong register
static uint16_t sample_buf_ping[CAPTURE_DEPTH];
static uint16_t sample_buf_pong[CAPTURE_DEPTH];
static int16_t bpf_output_buf[CAPTURE_DEPTH];
static int16_t avg_buf[500000 / (N_ELEM(bpf_output_buf) / 4) * MATCHED_FILTER_N_SECONDS] = {
    // Initialize all elements to -1, they should normally never have a negative value, so this is to detect 
    // which elements haven't been set yet.
    [0 ... N_ELEM(avg_buf) - 1] = -1
};

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

    if (sig_len < kern_len) {
        sig = kernel;
        kern = signal;
        lsig = kern_len;
        lkern = sig_len;
    }

    for (int n = 0; n < lkern; n++) {
        size_t k;
        
        int n_mod_kern_len = n % kern_len;

        for (k = 0; k <= n; k++)
            conv_out[n_mod_kern_len] += sig[k] * kern[n - k];
    }
    for (int n = lkern; n < lsig; n++) {
        size_t kmin, kmax, k;
        
        hw_divider_divmod_s32_start(n, kern_len);

        kmin = n - lkern + 1;
        kmax = n;
        
        int n_mod_kern_len = hw_divider_s32_remainder_wait();
        for (k = kmin; k <= kmax; k++)
            conv_out[n_mod_kern_len] += sig[k] * kern[n - k];
    }

    for (int n = lsig; n < lsig + lkern - 1; n++) {
        size_t kmin, kmax, k;
        
        hw_divider_divmod_s32_start(n, kern_len);

        kmin = n - lkern + 1;
        kmax =  lsig - 1;

        int n_mod_kern_len = hw_divider_s32_remainder_wait();
        for (k = kmin; k <= kmax; k++)
            conv_out[n_mod_kern_len] += sig[k] * kern[n - k];
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

void process_bit(uint64_t timestamp, enum bit_value_or_sync bit_or_sync) {
    static int sync_marks_per_bit[61];
    static bool backup_antenna = false, announce_dst_switch = false, cest = false, cet = false;
    static int minutes = -1, hours = -1, day_of_month = -1, day_of_week = -1, month_num = -1, year = -1;
    static int bit = 0;
    
    static int announce_leap_second = 0;
    // When more than half the bits in the last hour had the leap second announcement bit set, we can be fairly
    // certain that we have a leap second at the last second of the hour.
    bool last_minute = false;
    int seconds_in_minute = announce_leap_second > 30 && last_minute ? 61 : 60;
    
    static bool parity = false;
    
    if (bit_or_sync == SYNC) {
        // Don't get so high that it could take more than an hour to get back in sync
        // if for some reason we got the wrong sync.
        sync_marks_per_bit[bit] += N_ELEM(sync_marks_per_bit);
        
        if (sync_marks_per_bit[bit] > 60 * 60)
            sync_marks_per_bit[bit] = 60 * 60;
        // Decrement all other elements.
        for (int i = bit + 1; i < N_ELEM(sync_marks_per_bit); i++) {
            if (sync_marks_per_bit[i] > 0)
                sync_marks_per_bit[i]--;
        }
        for (int i = 0; i < bit; i++) {
            if (sync_marks_per_bit[i] > 0)
                sync_marks_per_bit[i]--;
        }
        printf("%" PRIu32 ": Sync mark!\n", us_to_ms(timestamp));
        goto inc_bit_and_return;
    }
    int max_idx = MAX_IDX(sync_marks_per_bit, N_ELEM(sync_marks_per_bit));
    // for (int i = 0; i < N_ELEM(sync_marks_per_bit); i++)
    //     printf(sync_marks_per_bit[i] ? "%03x" : ".", sync_marks_per_bit[i]);
    // printf(", max_idx = %d\n", max_idx);
    
    bool bit_value = bit_or_sync == HIGH;
    // Use `max_idx + 1' as that is bit 0 (since `max_idx' itself is the sync mark).
    int bit_after_sync = (bit + seconds_in_minute - (max_idx + 1)) % seconds_in_minute;
    switch (bit_after_sync) {
        case 0:
            if (bit_value != false)
                printf("%" PRIu32 ": Bit 0 should always be 0!!!\n", us_to_ms(timestamp));
            datetime_t t = {
                .year  = 2000 + year,
                .month = month_num,
                .day   = day_of_month,
                .dotw  = day_of_week % 7, // Sunday is 0 instead of 7.
                .hour  = hours,
                .min   = minutes,
                .sec   = 0
            };
            rtc_set_datetime(&t);
            // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime()
            // is called. The delay is up to 3 RTC clock cycles (which is 64us with the default clock settings).
            sleep_us(64);
            rtc_get_datetime(&t);
            minutes = hours = day_of_month = day_of_week = month_num = year = -1;
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
            break;
        case 17:
            // CEST (Central European Summer Time).
            cest = bit_value;
            break;
        case 18:
            // CET (Central European Time).
            cet = bit_value;
            if (!(cest ^ cet))
                printf("%" PRIu32 ": CEST and CET cannot have the same bit value\n", us_to_ms(timestamp));
            break;
        case 19:
            // Is 1 during the hour before the leap second is inserted.
            announce_leap_second += (int)bit_value;
            break;
        case 20:
            if (bit_value != true)
                printf("%" PRIu32 ": Bit 20 should always be 1!!!\n", us_to_ms(timestamp));
            break;
        case 21:
            minutes = 0;
            parity = false;
            [[fallthrough]];
        case 22 ... 27:
            // Minutes.
            minutes += (int)bit_value * bcd_table[bit_after_sync - 21];
            parity ^= bit_value;
            break;
        case 28:
            // Bit 21 ... 27 even parity.
            if (bit_value != parity)
                printf("%" PRIu32 ": Parity error in bits 21 ... 27\n", us_to_ms(timestamp));
            break;
        case 29:
            hours = 0;
            parity = false;
            [[fallthrough]];
        case 30 ... 34:
            // Hours.
            hours += (int)bit_value * bcd_table[bit_after_sync - 29];
            parity ^= bit_value;
            break;
        case 35:
            // Bit 29 ... 34 even parity.
            if (bit_value != parity)
                printf("%" PRIu32 ": Parity error in bits 29 ... 34\n", us_to_ms(timestamp));
            break;
        case 36:
            day_of_month = 0;
            parity = false;
            [[fallthrough]];
        case 37 ... 41:
            // Day of month.
            day_of_month += (int)bit_value * bcd_table[bit_after_sync - 36];
            parity ^= bit_value;
            break;
        case 42:
            day_of_week = 0;
            [[fallthrough]];
        case 43 ... 44:
            // Day of week (1 = Monday, 2 = Tuesday, ..., 7 = Sunday).
            day_of_week += (int)bit_value * bcd_table[bit_after_sync - 42];
            parity ^= bit_value;
            break;
        case 45:
            month_num = 0;
            [[fallthrough]];
        case 46 ... 49:
            // Month number.
            month_num += (int)bit_value * bcd_table[bit_after_sync - 45];
            parity ^= bit_value;
            break;
        case 50:
            year = 0;
            [[fallthrough]];
        case 51 ... 57:
            year += (int)bit_value * bcd_table[bit_after_sync - 50];
            parity ^= bit_value;
            // Year.
            break;
        case 58:
            // Bit 36 ... 57 even parity.
            if (bit_value != parity)
                printf("%" PRIu32 ": Parity error in bits 36 ... 57\n", us_to_ms(timestamp));
            printf("%02d:%02d, %d, %02d-%02d-%02d\n", hours, minutes, day_of_week, day_of_month, month_num, year); 
            break;
        case 59:
            // Only occurs when we didn't detect a sync mark this minute (so then it is actually bit 0)
            // or when we have a leap second, in both cases it is always 0.
            if (bit_value != false)
                printf("%" PRIu32 ": Bit 59 should always be 0!!!\n", us_to_ms(timestamp));
            break;
        case 60 ... INT_MAX:
            // Invalid.
            break;
    }
inc_bit_and_return:
    bit++;
    if (bit >= seconds_in_minute)
        bit = 0;
    if (bit_after_sync == seconds_in_minute - 2 && last_minute)
        announce_leap_second = 0; // Always set back to 0 at last second of the hour.
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
    [0 ... (int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 0.4) - 1] = 7,
    [(int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 0.4) ... (int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 0.5) - 1] = 3,
    [(int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 0.5) ... (int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 0.6) - 1] = 0,
    [(int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 0.6) ... (int)(0.5e6 / (N_ELEM(bpf_output_buf) / 4.) * 1.) - 1] = 7,
};

void core1_main(void) {
    uint64_t prev_timestamp = to_us_since_boot(at_the_end_of_time);
    int32_t avg_sum = 0, avg_avg = 0, prev_avg_avgs[2] = { 0, 0 };
    
    int32_t wrapped_avg_buf[N_ELEM(avg_buf) / MATCHED_FILTER_N_SECONDS] = { 0 };
    int avg_buf_idx = 0, max_idx = 0, prev_max_idx = 0;
    
    while (true) {
        uint64_t timestamp;
        ((uint32_t *)&timestamp)[0] = multicore_fifo_pop_blocking();
        ((uint32_t *)&timestamp)[1] = multicore_fifo_pop_blocking();
        int32_t avg = (int32_t)multicore_fifo_pop_blocking();
        
        inline int idx_offs_mod_1s(int idx, int offs) { return (idx + offs) % N_ELEM(wrapped_avg_buf); };
        
        absolute_time_t tic = get_absolute_time();
        int avg_buf_idx_mod_1s = idx_offs_mod_1s(avg_buf_idx, 0);
        
        avg_buf[avg_buf_idx++] = avg;
        if (avg_buf_idx >= N_ELEM(avg_buf))
            avg_buf_idx = 0;
        
        wrapped_avg_buf[avg_buf_idx_mod_1s] = 0;
        for (int i = avg_buf_idx_mod_1s; i < N_ELEM(avg_buf); i += N_ELEM(wrapped_avg_buf)) {
            if (avg_buf[i] == -1)
                continue;
            wrapped_avg_buf[avg_buf_idx_mod_1s] += avg_buf[i];
        }
        
        int32_t conv_out[N_ELEM(kernel)] = { 0 };
        if (avg_buf_idx_mod_1s == N_ELEM(wrapped_avg_buf) - 1) {
            conv_wrapped(wrapped_avg_buf, N_ELEM(wrapped_avg_buf), kernel, N_ELEM(kernel), conv_out);

            // if (avg_buf_idx == 20000)
            //     for (int i = 0; i < N_ELEM(conv_out); i++) {
            //         printf("%" PRIi32 "\n", conv_out[i]);
            //     }
            max_idx = MAX_IDX(conv_out, N_ELEM(conv_out));
            // printf("%d\n", max_idx);
        }
        absolute_time_t toc = get_absolute_time();
        // printf("%llu\n", to_us_since_boot(toc) - to_us_since_boot(tic));
        
        //     1 -   ________       ________
        //                   |  |  |
        //  0.75 -           |  |  |
        //                   |  |  |
        //   0.5 -           |  |  |
        //                   |  |  |
        //  0.25 -           |  |  |
        //                   |  |  |
        //     0 -           '--'--'
        //       start_100ms ^  ^  ^
        //          start_200ms '  |
        //               end_200ms '
        bool start_100ms = avg_buf_idx_mod_1s == idx_offs_mod_1s(prev_max_idx, (int)(N_ELEM(conv_out) * 0.4));
        bool start_200ms = avg_buf_idx_mod_1s == idx_offs_mod_1s(prev_max_idx, (int)(N_ELEM(conv_out) * 0.5));
        bool end_200ms = avg_buf_idx_mod_1s == idx_offs_mod_1s(prev_max_idx, (int)(N_ELEM(conv_out) * 0.6));
        // Make sure that we switch to a different `max_idx' only when we are as far away from the windows as possible.
        int lower_lim = idx_offs_mod_1s(max_idx, (int)(N_ELEM(conv_out) * 0.9));
        int higher_lim = idx_offs_mod_1s(max_idx, (int)(N_ELEM(conv_out) * 0.1));
        if (lower_lim > higher_lim ? (avg_buf_idx_mod_1s > lower_lim || avg_buf_idx_mod_1s < higher_lim) :
            (avg_buf_idx_mod_1s > lower_lim && avg_buf_idx_mod_1s < higher_lim))
            prev_max_idx = max_idx;

        if (start_100ms || start_200ms || end_200ms) {
            prev_avg_avgs[1] = prev_avg_avgs[0];
            prev_avg_avgs[0] = avg_avg;
            avg_avg = avg_sum / (start_100ms ? (int)(N_ELEM(conv_out) * 0.8) : (int)(N_ELEM(conv_out) * 0.1));
            avg_sum = 0;

            if (start_100ms) {
                // 87.5% of the average during the high level seems to be a good threshold.
                bool sync_mark = prev_avg_avgs[1] > avg_avg * 7 / 8 && prev_avg_avgs[0] > avg_avg * 7 / 8;
                
                // Due to the lowpass behaviour of the high Q biquad filter, the second window of 100ms, in which
                // we would see another low level if we have a 1 bit, has a lower average than the first window,
                // so we can just check whether the average in the first window is higher than the average in
                // the second 100ms window.
                process_bit(prev_timestamp, sync_mark ? SYNC : (prev_avg_avgs[1] > prev_avg_avgs[0] ? HIGH : LOW));
                // Also we have to use the timestamp of the previous start of a 100ms window because we use averages
                // from 1 and 2 windows ago:
                //    ____     ________
                //        | | |        | | |
                //        | | |        | | |
                //        | | |        | | |
                //        '-'-'        '-'-'
                //     pt ^^ ^  aa ^ t ^
                //  paa[1] ' |
                //    paa[0] '
                //
                // Where pt is `prev_timestamp', aa is `avg_avg', t is `timestamp', paa[1] is `prev_avg_avgs[1]' and
                // paa[0] is `prev_avg_avgs[0]'.
                prev_timestamp = timestamp;
            }
        }
        // printf("%" PRIi32 ", %" PRIi32 "\n", start_100ms || end_200ms ? 0 : avg, prev_avg_avgs[1]);
        
        avg_sum += avg;
    }
}

int main(void) {
    int16_t bpf_coeffs[5];
    int32_t bpf_w[5] = { 0, 0 };
    
    stdio_init_all();
    sleep_ms(1000);
    
    // Start the RTC.
    rtc_init();

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
        for (int i = 0; i < N_ELEM(bpf_output_buf); ++i) {
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
        hw_divider_divmod_s32_start(full_spectrum_avg, N_ELEM(bpf_output_buf));

        filter_biquad_IIR(signed_sample_buf, bpf_output_buf, N_ELEM(bpf_output_buf), bpf_coeffs, bpf_w);
        
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
            for (int j = N_ELEM(bpf_output_buf) * i / 4; j < N_ELEM(bpf_output_buf) * (i + 1) / 4; ++j)
                avgs[i] += bpf_output_buf[j] < 0 ? -bpf_output_buf[j] : bpf_output_buf[j];
            // We're not dividing by a power of 2 anymore here, but apparently the / operator is using the
            // hardware divider anyway, so it will only take 8 cycles.
            int32_t avg = (avgs[0] + avgs[1] + avgs[2] + avgs[3]) / N_ELEM(bpf_output_buf);
            /**
             * TODO: Check ratio between average after bandpass filter and average of full spectrum, so that we can
             * estimate how close the DCF77 signal is to the noise floor.
             */
            
            // printf("%" PRIi32 "\n", avg);
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
                1000000ull / 500000ull * N_ELEM(bpf_output_buf) * (2 + 3 - i) / 4;
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

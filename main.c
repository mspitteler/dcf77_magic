#include <stdio.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <limits.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/divider.h"
#include "pico/multicore.h"
#include "hardware/rtc.h"
#include "pico/critical_section.h"

constexpr int CAPTURE_CHANNEL = 0;

constexpr int CAPTURE_DEPTH = 20000;
// We hava a tolerance of 30ppm for the processor clock, so that means we will drift 30ppm * 300s = 9ms in 300s,
// which is just a little under the sample time of the averages, which is good, because if the drift would be 
// significantly higher we would lose in sharpness of the peak in the convolution output buffer.
constexpr int MATCHED_FILTER_N_SECONDS = 300;

constexpr long SAMPLE_RATE = 500'000l; // Sample rate of the ADC when clkdiv is 0.

// We don't want the type of N_ELEM to be size_t because that would cause every arithmetic expression that uses this
// to turn unsigned.
#define N_ELEM(a) (ssize_t)(sizeof(a) / sizeof(*(a)))
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
// 2.14 fixed point.
#define LAGRANGE_BASIS(x, x0, x1, x2)                                                                                 \
    {                                                                                                                 \
        (int16_t)(((x) - (x1)) * ((x) - (x2)) * 16384. / (((x0) - (x1)) * ((x0) - (x2)))),                            \
        (int16_t)(((x) - (x0)) * ((x) - (x2)) * 16384. / (((x1) - (x0)) * ((x1) - (x2)))),                            \
        (int16_t)(((x) - (x0)) * ((x) - (x1)) * 16384. / (((x2) - (x0)) * ((x2) - (x1))))                             \
    }
#define LAGRANGE_BASIS_012(x) LAGRANGE_BASIS(x, 0., 1., 2.)

#define PUSH_MULTICORE_PACKET(packet)                                                                                 \
    do {                                                                                                              \
        union multicore_packet p = (packet);                                                                          \
        multicore_fifo_push_blocking(p.words[0]);                                                                     \
        multicore_fifo_push_blocking(p.words[1]);                                                                     \
    } while (false)
    
#define POP_MULTICORE_PACKET()                                                                                        \
    ({                                                                                                                \
        union multicore_packet p;                                                                                     \
        p.words[0] = multicore_fifo_pop_blocking();                                                                   \
        p.words[1] = multicore_fifo_pop_blocking();                                                                   \
        p;                                                                                                            \
    })

enum bit_value_or_sync { LOW, HIGH, SYNC };

union multicore_packet {
    uint32_t words[2];
    struct {
        uint64_t timestamp_diff : 48;
        int16_t avg;
    };
};

struct set_cmp_rtc_data {
    uint64_t timestamp;
    union bcd_date {
        uint8_t fields[6];
        struct {
            uint8_t min, hour, day, dotw, month, year;
        };
        // NOTE: The below assumes little endian!!
        struct {
            uint8_t min_lo : 4, min_hi : 4, hour_lo : 4, hour_hi : 4, day_lo : 4, day_hi : 4,
                dotw_lo : 4, dotw_hi : 4, month_lo : 4, month_hi : 4, year_lo : 4, year_hi : 4;
        };
    } bcd_date;
    bool min_parity_err, hour_parity_err, date_parity_err;
    bool cest_not_cet, unsynced;
};

enum filter_sensitivity {
    SENSITIVITY_LEVEL_0 = 5,
    SENSITIVITY_LEVEL_1 = 4,
    SENSITIVITY_LEVEL_2 = 3,
    SENSITIVITY_LEVEL_3 = 2,
    SENSITIVITY_LEVEL_4 = 1,
    SENSITIVITY_LEVEL_5 = 0,
    SENSITIVITY_LEVEL_6 = -1,
    SENSITIVITY_LEVEL_7 = -2,
    SENSITIVITY_LEVEL_8 = -3,
    SENSITIVITY_LEVEL_9 = -4,
};

constexpr enum filter_sensitivity SENSITIVITY_LEVEL = SENSITIVITY_LEVEL_3;

// Replace sample_buf[CAPTURE_DEPTH] with a ping and pong register
static uint16_t sample_buf_ping[CAPTURE_DEPTH];
static uint16_t sample_buf_pong[CAPTURE_DEPTH];
static int16_t bpf_output_buf[CAPTURE_DEPTH];
// Number of average amplitudes per second after the filter. The average is taken over all 20000 samples, but upsampled
// by a factor 4, so we get 100 amplitude averages per second.
constexpr long AVGS_PER_SECOND = (SAMPLE_RATE / (N_ELEM(bpf_output_buf) / 4l));
static int16_t avg_buf[AVGS_PER_SECOND * MATCHED_FILTER_N_SECONDS] = {
    // Initialize all elements to -1, they should normally never have a negative value, so this is to detect 
    // which elements haven't been set yet.
    [0 ... N_ELEM(avg_buf) - 1] = -1
};

static uint16_t *volatile sample_buf = nullptr;

static uint dma_chan;

static inline void set_bit [[gnu::always_inline]] (uint8_t *const field, int bit) { *field |= (1u << bit); }
static inline void clear_bit [[gnu::always_inline]] (uint8_t *const field, int bit) { *field &= ~(1u << bit); }
static inline void toggle_bit [[gnu::always_inline]] (uint8_t *const field, int bit) { *field ^= (1u << bit); }
static inline bool test_bit [[gnu::always_inline]] (uint8_t field, int bit) { return !!(field & (1u << bit)); }
static inline void assign_bit [[gnu::always_inline]] (uint8_t *const field, int bit, bool val) {
    if (val)
        set_bit(field, bit);
    else
        clear_bit(field, bit);
}

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
        // Because we want such a high Q, we have to scale down w by 5 bits to prevent overflow. As it is now,
        // we stay well within about half of the range of a 32-bit integer. This does mean that the output is scaled
        // down by 5 bits as well (divide by 32), but the behaviour of the filter is not altered.
        int32_t d0 = ((int32_t)input[i] - ((coeffs[3] * w[0] + coeffs[4] * w[1]) >> (14 - SENSITIVITY_LEVEL)));
        d0 = SENSITIVITY_LEVEL >= 0 ? d0 >> SENSITIVITY_LEVEL : d0 << -SENSITIVITY_LEVEL;
        output[i] = (coeffs[0] * d0 + coeffs[1] * w[0] + coeffs[2] * w[1]) >> 14;
        w[1] = w[0];
        w[0] = d0;
    }
}

// Circular convolution, assumes that the signal and the kernel have the same length.
void circular_conv(const int32_t *const signal, const int sig_len, const int32_t *const kernel, const int kern_len,
                   int64_t *const conv_out) {
    assert(sig_len == kern_len);
    int N = sig_len;
    
    for (int n = 0; n < N; n++) {
        // Split it up into two parts so we don't need to calculate the modulo of n - k.
        for (int k = 0; k <= n; k++)
            conv_out[n] += signal[k] * kernel[n - k];
        for (int k = n + 1; k < N; k++)
            conv_out[n] += signal[k] * kernel[n - k + N];
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

critical_section_t last_rtc_alarm_crit_sec;
volatile uint64_t last_rtc_alarm_timestamp;
volatile datetime_t last_rtc_alarm_datetime = { .year = 2000, .month = 1, .day = 1 };

void rtc_alarm_handler() {
    uint64_t timestamp = to_us_since_boot(get_absolute_time());
    datetime_t t;
    rtc_get_datetime(&t);
    // Re-enable alarm every second.
    rtc_set_alarm(&(datetime_t) { -1, -1, -1, -1, -1, -1, .sec = (t.sec + 1) % 60 }, &rtc_alarm_handler);
    
    critical_section_enter_blocking(&last_rtc_alarm_crit_sec);
    last_rtc_alarm_timestamp = timestamp;
    last_rtc_alarm_datetime = t;
    critical_section_exit(&last_rtc_alarm_crit_sec);
}

int64_t set_rtc(alarm_id_t id, void *user_data) {
    struct set_cmp_rtc_data *data = user_data;
    uint8_t flags = 0x00;
    assign_bit(&flags, 0, data->cest_not_cet);
    // assign_bit(&flags, 1, false); // Announce DST switch bit, not used in decoder.
    assign_bit(&flags, 2, data->unsynced);
    
    char s[] = "00-00-00-00-00-00-01-00\r";
    s[0] += data->bcd_date.year_hi, s[1] += data->bcd_date.year_lo, s[3] += data->bcd_date.month_hi;
    s[4] += data->bcd_date.month_lo, s[6] += data->bcd_date.day_hi, s[7] += data->bcd_date.day_lo;
    s[9] += data->bcd_date.dotw_hi, s[10] += data->bcd_date.dotw_lo, s[12] += data->bcd_date.hour_hi; 
    s[13] += data->bcd_date.hour_lo, s[15] += data->bcd_date.min_hi, s[16] += data->bcd_date.min_lo, s[22] += flags;
    
    write(1, s, N_ELEM(s) - 1);
    
    // Convert BCD to decimal.
    datetime_t datetime = {
        .sec = 0, .min = data->bcd_date.min_lo + data->bcd_date.min_hi * 10,
        .hour = data->bcd_date.hour_lo + data->bcd_date.hour_hi * 10,
        .day = data->bcd_date.day_lo + data->bcd_date.day_hi * 10,
        .dotw = data->bcd_date.dotw_lo + data->bcd_date.dotw_hi * 10,
        .month = data->bcd_date.month_lo + data->bcd_date.month_hi * 10,
        .year = 2000 + data->bcd_date.year_lo + data->bcd_date.year_hi * 10, // Year is from 0 to 99, so add 2000 here.
    };
    if (datetime.dotw == 7)
        datetime.dotw = 0; // Sunday is 0 instead of 7 for the RTC.
    
    rtc_set_datetime(&datetime);
    absolute_time_t rtc_set_datetime_time = get_absolute_time();
    
    printf("Setting %02" PRIi8 ":%02" PRIi8 ", %" PRIi8 ", %02" PRIi8 "-%02" PRIi8 "-%" PRIi16 "\n",
           datetime.hour, datetime.min, datetime.dotw, datetime.day, datetime.month, datetime.year);
    printf("Set RTC %lluus too late!\n", to_us_since_boot(rtc_set_datetime_time) - data->timestamp);
    
    // We always set the time to 0s after the minute, but the RTC immediately goes to 1s after the minute,
    // as discussed further below, so 2s after the minute is the first possible alarm we can catch.
    rtc_set_alarm(&(datetime_t) { -1, -1, -1, -1, -1, -1, .sec = 2 }, &rtc_alarm_handler);
    return 0ll;
}

bool is_valid_xor_mask(const uint8_t *const fields, const int idx, const uint8_t xor_mask) {
    uint8_t field = fields[idx];
    field ^= xor_mask;
    // Convert BDC to decimal.
    int field_dec = (field & 0x0f) + (field >> 4) * 10;
    switch (idx) {
        case offsetof(union bcd_date, min):
            return field_dec <= 59;
        case offsetof(union bcd_date, hour):
            return field_dec <= 23;
        case offsetof(union bcd_date, day):
            return field_dec >= 1 && field_dec <= 31;
        case offsetof(union bcd_date, dotw):
            return field_dec >= 1 && field_dec <= 7;
        case offsetof(union bcd_date, month):
            return field_dec >= 1 && field_dec <= 12;
        case offsetof(union bcd_date, year):
            return field_dec >= 0 && field_dec <= 99;
        default: // Should never happen.
            return false;
    }
};

int64_t compare_time(alarm_id_t id, void *user_data) {
    static int field_diffs[6][8];
    static bool prev_cest_not_cet = false;
    struct set_cmp_rtc_data *data = user_data;
    
    critical_section_enter_blocking(&last_rtc_alarm_crit_sec);
    datetime_t rtc_datetime = last_rtc_alarm_datetime;
    uint64_t rtc_timestamp = last_rtc_alarm_timestamp;
    critical_section_exit(&last_rtc_alarm_crit_sec);
    
    rtc_datetime.year -= 2000;
    if (rtc_datetime.dotw == 0)
        rtc_datetime.dotw = 7; // Sunday is 0 instead of 7 for the RTC.
    if (prev_cest_not_cet != data->cest_not_cet)
        rtc_datetime.hour += data->cest_not_cet ? 1 : -1; // We can safely assume the hour is not 23 or 00, so no carry.
        // If the hour is 00 for some reason, it could become -01, and converting this to BCD will yield 15. This is 
        // kind of weird, but if we had a DST switch at hour 00 something must have been very wrong with the time anyway 
        // and it probably won't really matter that we set the hour to some bogus value.
        // If the hour is 23 for some reason, it could become 24, and assuming we don't have any bit flips, this means
        // that we will not actually set the time since 24 is an invalid hour. This is okay as we shouldn't have a DST
        // switch at hour 23 anyway.
    prev_cest_not_cet = data->cest_not_cet;
    union bcd_date rtc_bcd_date = {
        .min_lo = rtc_datetime.min % 10, .min_hi = rtc_datetime.min / 10,
        .hour_lo = rtc_datetime.hour % 10, .hour_hi = rtc_datetime.hour / 10,
        .day_lo = rtc_datetime.day % 10, .day_hi = rtc_datetime.day / 10,
        .dotw_lo = rtc_datetime.dotw % 10, .dotw_hi = rtc_datetime.dotw / 10,
        .month_lo = rtc_datetime.month % 10, .month_hi = rtc_datetime.month / 10,
        .year_lo = rtc_datetime.year % 10, .year_hi = rtc_datetime.year / 10
    };
    if (rtc_datetime.sec != 0)
        printf("Sync mark moved!\n");
    
    if (rtc_bcd_date.min != data->bcd_date.min || rtc_bcd_date.hour != data->bcd_date.hour ||
        rtc_bcd_date.day != data->bcd_date.day || rtc_bcd_date.dotw != data->bcd_date.dotw ||
        rtc_bcd_date.month != data->bcd_date.month || rtc_bcd_date.year != data->bcd_date.year) {
        printf("Unequal date: [%06hhb]:[%07hhb], [%03hhb], [%06hhb]-[%05hhb]-[%08hhb]!\n",
               rtc_bcd_date.hour ^ data->bcd_date.hour, rtc_bcd_date.min ^ data->bcd_date.min,
               rtc_bcd_date.dotw ^ data->bcd_date.dotw, rtc_bcd_date.day ^ data->bcd_date.day,
               rtc_bcd_date.month ^ data->bcd_date.month, rtc_bcd_date.year ^ data->bcd_date.year);
    }
    
    // TODO: Do something with the detected parity errors.
    
    union bcd_date prev_rtc_bcd_date = rtc_bcd_date;
    static_assert(N_ELEM(field_diffs) == N_ELEM(rtc_bcd_date.fields));
    for (int i = 0; i < N_ELEM(field_diffs); i++) {
        uint8_t field_xor_mask = 0x00;
        for (int j = 0; j < N_ELEM(*field_diffs); j++) {
            field_diffs[i][j] += (test_bit(rtc_bcd_date.fields[i], j) ^ test_bit(data->bcd_date.fields[i], j)) ? -1 : 1;
            if (field_diffs[i][j] > 10)
                field_diffs[i][j] = 10;
            else if (field_diffs[i][j] < 0)
                set_bit(&field_xor_mask, j);
        }
        // Only toggle the bits when it will result in a valid field. If it doesn't we wait for other bits to be
        // toggled too, so that we get a valid field again.
        if (is_valid_xor_mask(rtc_bcd_date.fields, i, field_xor_mask)) {
            rtc_bcd_date.fields[i] ^= field_xor_mask;
            for (int j = 0; j < N_ELEM(*field_diffs); j++)
                if (test_bit(field_xor_mask, j))
                    field_diffs[i][j] = 10; // Assume that it is correct now.
        }
    }
    
    if (rtc_bcd_date.min != prev_rtc_bcd_date.min || rtc_bcd_date.hour != prev_rtc_bcd_date.hour ||
        rtc_bcd_date.day != prev_rtc_bcd_date.day || rtc_bcd_date.dotw != prev_rtc_bcd_date.dotw ||
        rtc_bcd_date.month != prev_rtc_bcd_date.month || rtc_bcd_date.year != prev_rtc_bcd_date.year) {
        // TODO: set data->unsynced to true here and to false again if the RTC date hasn't changed for a while.
        printf("Changing RTC date!!!\n");
    }
    
    data->bcd_date = rtc_bcd_date;
    return 0ll;
}

// volatile bool print_max_idx = true;

void process_bit(uint64_t timestamp, enum bit_value_or_sync bit_or_sync) {
    static int sync_marks_per_bit[61];
    static bool backup_antenna = false, do_dst_switch = false, cest = false, cet = false;
    static union bcd_date date = { .min = 0xff, .hour = 0xff, .day = 0xff, .dotw = 0xff, .month = 0xff, .year = 0xff };
    static bool minutes_parity_error = true, hours_parity_error = true, date_parity_error = true;
    static int bit = 0;
    
    static int announce_dst_switch = 0, dst = 0, announce_leap_second = 0;
    // When more than half the bits in the last hour had the leap second announcement bit set, we can be fairly
    // certain that we have a leap second at the last second of the hour.
    
    // TODO: Set `last_minute' to true if the date and time are stable for quite a while and it is indeed the
    // last minute of the hour.
    bool last_minute = false;
    int seconds_in_minute = announce_leap_second > 30 && last_minute ? 61 : 60;
    if (bit >= seconds_in_minute)
        bit = 0;
    if (do_dst_switch)
        dst = -dst;
    
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
    }
    int max_idx = MAX_IDX(sync_marks_per_bit, N_ELEM(sync_marks_per_bit));
    // for (int i = 0; i < N_ELEM(sync_marks_per_bit); i++)
    //     printf(sync_marks_per_bit[i] ? "%03x" : ".", sync_marks_per_bit[i]);
    // printf(", max_idx = %d\n", max_idx);
    
    // Use `max_idx + 1' as that is bit 0 (since `max_idx' itself is the sync mark).
    int bit_after_sync = (bit + seconds_in_minute - (max_idx + 1)) % seconds_in_minute;
    // Also low if it's a sync mark, which is the most likely if the sync mark was misdetected. 
    bool bit_value = bit_or_sync == HIGH;
    switch (bit_after_sync) {
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
            announce_dst_switch += (int)bit_value;
            break;
        case 17:
            // CEST (Central European Summer Time).
            cest = bit_value;
            dst += (int)cest;
            if (dst > 60) // Don't take more than an hour to switch if for some reason we had the wrong timezone.
                dst = 60;
            break;
        case 18:
            // CET (Central European Time).
            cet = bit_value;
            dst -= (int)cet;
            if (dst < -60) // Don't take more than an hour to switch if for some reason we had the wrong timezone.
                dst = -60;
            if (cest == cet)
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
            date.min = 0x00;
            minutes_parity_error = false;
            [[fallthrough]];
        case 22 ... 27:
            // Minutes.
            assign_bit(&date.min, bit_after_sync - 21, bit_value);
            [[fallthrough]];
        case 28:
            // Bit 21 ... 27 even parity.
            minutes_parity_error ^= bit_value;
            break;
        case 29:
            date.hour = 0x00;
            hours_parity_error = false;
            [[fallthrough]];
        case 30 ... 34:
            // Hours.
            assign_bit(&date.hour, bit_after_sync - 29, bit_value);
            [[fallthrough]];
        case 35:
            // Bit 29 ... 34 even parity.
            hours_parity_error ^= bit_value;
            break;
        case 36:
            date.day = 0x00;
            date_parity_error = false;
            [[fallthrough]];
        case 37 ... 41:
            // Day of month.
            assign_bit(&date.day, bit_after_sync - 36, bit_value);
            date_parity_error ^= bit_value;
            break;
        case 42:
            date.dotw = 0x00;
            [[fallthrough]];
        case 43 ... 44:
            // Day of week (1 = Monday, 2 = Tuesday, ..., 7 = Sunday).
            assign_bit(&date.dotw, bit_after_sync - 42, bit_value);
            date_parity_error ^= bit_value;
            break;
        case 45:
            date.month = 0x00;
            [[fallthrough]];
        case 46 ... 49:
            // Month number.
            assign_bit(&date.month, bit_after_sync - 45, bit_value);
            date_parity_error ^= bit_value;
            break;
        case 50:
            date.year = 0x00;
            [[fallthrough]];
        case 51 ... 57:
            // Year.
            assign_bit(&date.year, bit_after_sync - 50, bit_value);
            [[fallthrough]];
        case 58:
            // Bit 36 ... 57 even parity.
            date_parity_error ^= bit_value;
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
    if (bit_after_sync == 58) {
        // Process received date and time.
        if (minutes_parity_error || hours_parity_error || date_parity_error)
            printf("%" PRIu32 ": Parity error in bits%s%s%s.\n", us_to_ms(timestamp),
                    minutes_parity_error ? " 21 ... 27" : "",
                    hours_parity_error ? &" and 29 ... 34"[minutes_parity_error ? 0 : 4] : "",
                    date_parity_error ? &" and 36 ... 57"[hours_parity_error || minutes_parity_error ? 0 : 4] : "");
        
        // We are at second 58 now, receiving the date and time for second 0 of the minute that's about to start,
        // so we would have to add 2 seconds to the timestamp to get to second 0 for setting the new time.
        // There is one catch however, because of some weird quirk of the RP2040 RTC, if you set it,
        // the time immediately jumps to the next second, instead of waiting for one second to elapse.
        // This is why we have to actually add 3 seconds to the timestamp.
        // More information can be found here: https://forums.raspberrypi.com/viewtopic.php?t=348730
        // But we compare with the RTC time 500ms before the time would be set (so then the time should still be
        // 0 seconds after the minute). Do this because that way we make sure that if the RTC is 
        // slightly behind, we don't get for example RTC: 23:59:59 and received: 00:00:00. We don't want this because
        // that makes it very hard to determine whether it is actually still almost exactly the same time without 
        // converting back and forth to Epoch. So we can use the RTC quirk to our advantage in this case actually since
        // we can check with the same time that we're about to set (0s after the minute) and only need to subtract 500ms
        // from the timestamp we use for setting.
        
        // We don't want it on the stack because it gets used later in an interrupt.
        static struct set_cmp_rtc_data data;
        data = (struct set_cmp_rtc_data) {
            .timestamp = timestamp + 3'000'000ull, .bcd_date = date, .min_parity_err = minutes_parity_error,
            .hour_parity_err = hours_parity_error, .date_parity_err = date_parity_error, .cest_not_cet = dst > 0
        };
        if (add_alarm_at(from_us_since_boot(data.timestamp), &set_rtc, &data, true) < 0 ||
            add_alarm_at(from_us_since_boot(data.timestamp - 500'000ull), &compare_time, &data, true) < 0)
            printf("add_alarm_at: No more alarm slots available\n");
        date = (union bcd_date) { .min = 0xff, .hour = 0xff, .day = 0xff, .dotw = 0xff, .month = 0xff, .year = 0xff };
        minutes_parity_error = hours_parity_error = date_parity_error = true;
        // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime()
        // is called. The delay is up to 3 RTC clock cycles (which is 64us with the default clock settings).
        // sleep_us(64);
        // datetime_t t;
        // rtc_get_datetime(&t);
    }

    do_dst_switch = false;
    if (bit_after_sync == seconds_in_minute - 1 && last_minute) {
        do_dst_switch = announce_dst_switch > 30;
        announce_dst_switch = announce_leap_second = 0; // Always set back to 0 at last second of the hour.
    }
    bit++;
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
    [0 ... (int)(AVGS_PER_SECOND * 0.4) - 1] = 7,
    [(int)(AVGS_PER_SECOND * 0.4) ... (int)(AVGS_PER_SECOND * 0.5) - 1] = 3,
    [(int)(AVGS_PER_SECOND * 0.5) ... (int)(AVGS_PER_SECOND * 0.6) - 1] = 0,
    [(int)(AVGS_PER_SECOND * 0.6) ... (int)(AVGS_PER_SECOND * 1.) - 1] = 7,
};

// Step size after interpolating is 0.05 * 10ms = 500us, so we will get jumps of this length between timestamps,
// which is quite good.
static const int16_t lagrange_bases[][3] = {
    LAGRANGE_BASIS_012(0.),  LAGRANGE_BASIS_012(0.05), LAGRANGE_BASIS_012(0.1), LAGRANGE_BASIS_012(0.15),
    LAGRANGE_BASIS_012(0.2), LAGRANGE_BASIS_012(0.25), LAGRANGE_BASIS_012(0.3), LAGRANGE_BASIS_012(0.35),
    LAGRANGE_BASIS_012(0.4), LAGRANGE_BASIS_012(0.45), LAGRANGE_BASIS_012(0.5), LAGRANGE_BASIS_012(0.55),
    LAGRANGE_BASIS_012(0.6), LAGRANGE_BASIS_012(0.65), LAGRANGE_BASIS_012(0.7), LAGRANGE_BASIS_012(0.75),
    LAGRANGE_BASIS_012(0.8), LAGRANGE_BASIS_012(0.85), LAGRANGE_BASIS_012(0.9), LAGRANGE_BASIS_012(0.95),
    LAGRANGE_BASIS_012(1.),  LAGRANGE_BASIS_012(1.05), LAGRANGE_BASIS_012(1.1), LAGRANGE_BASIS_012(1.15),
    LAGRANGE_BASIS_012(1.2), LAGRANGE_BASIS_012(1.25), LAGRANGE_BASIS_012(1.3), LAGRANGE_BASIS_012(1.35),
    LAGRANGE_BASIS_012(1.4), LAGRANGE_BASIS_012(1.45), LAGRANGE_BASIS_012(1.5), LAGRANGE_BASIS_012(1.55),
    LAGRANGE_BASIS_012(1.6), LAGRANGE_BASIS_012(1.65), LAGRANGE_BASIS_012(1.7), LAGRANGE_BASIS_012(1.75),
    LAGRANGE_BASIS_012(1.8), LAGRANGE_BASIS_012(1.85), LAGRANGE_BASIS_012(1.9), LAGRANGE_BASIS_012(1.95),
    LAGRANGE_BASIS_012(2.),
};

void core1_main(void) {
    uint64_t prev_timestamp = to_us_since_boot(at_the_end_of_time);
    uint64_t timestamp = 0ull;
    int timestamp_offset = 0, synced_timestamp_offset = 0;
    int32_t avg_sum = 0, avg_avg = 0, prev_avg_avgs[2] = { 0, 0 };
    
    int32_t wrapped_avg_buf[AVGS_PER_SECOND] = { 0 };
    int avg_buf_idx = 0, max_idx = 0, synced_max_idx = 0;
    
    critical_section_init(&last_rtc_alarm_crit_sec);
    
    while (true) {
        union multicore_packet packet = POP_MULTICORE_PACKET();
        timestamp += packet.timestamp_diff;
        
        inline int idx_offs_mod_1s(int idx, int offs) { return (idx + offs) % N_ELEM(wrapped_avg_buf); };
        
        absolute_time_t tic = get_absolute_time();
        int avg_buf_idx_mod_1s = idx_offs_mod_1s(avg_buf_idx, 0);
        
        avg_buf[avg_buf_idx++] = packet.avg;
        if (avg_buf_idx >= N_ELEM(avg_buf))
            avg_buf_idx = 0;
        
        wrapped_avg_buf[avg_buf_idx_mod_1s] = 0;
        for (int i = avg_buf_idx_mod_1s; i < N_ELEM(avg_buf); i += N_ELEM(wrapped_avg_buf)) {
            if (avg_buf[i] == -1)
                continue;
            wrapped_avg_buf[avg_buf_idx_mod_1s] += avg_buf[i];
        }
        
        int64_t conv_out[N_ELEM(kernel)] = { 0 };
        if (avg_buf_idx_mod_1s == N_ELEM(wrapped_avg_buf) - 1) {
            circular_conv(wrapped_avg_buf, N_ELEM(wrapped_avg_buf), kernel, N_ELEM(kernel), conv_out);

            // if (avg_buf_idx == 20000)
            //     for (int i = 0; i < N_ELEM(conv_out); i++) {
            //         printf("%lld\n", conv_out[i]);
            //     }
            max_idx = MAX_IDX(conv_out, N_ELEM(conv_out));
            // Detect fractional maximum with lagrange interpolation so we don't have such large jumps of 10ms in
            // the timestamps when the maximum changes.
            int max_idx_min1 = max_idx == 0 ? N_ELEM(conv_out) - 1 : max_idx - 1,
                max_idx_plus1 = max_idx == N_ELEM(conv_out) - 1 ? 0 : max_idx + 1;
            int64_t around_max[] = {
                0ll,
                conv_out[max_idx] - conv_out[max_idx_min1],
                conv_out[max_idx_plus1] - conv_out[max_idx_min1],
            };
            static_assert(N_ELEM(*lagrange_bases) == N_ELEM(around_max));
            
            int64_t lagrange_interpolated[N_ELEM(lagrange_bases)] = { 0 };
            for (int i = 0; i < N_ELEM(lagrange_bases); i++)
                for (int j = 0; j < N_ELEM(*lagrange_bases); j++)
                    lagrange_interpolated[i] += (around_max[j] * lagrange_bases[i][j]) >> 14; // 2.14 fixed point.
            int lagrange_interpolated_max_idx = MAX_IDX(lagrange_interpolated, N_ELEM(lagrange_interpolated));
            // 0 ... 40 for `lagrange_interpolated_max_idx' maps to `max_idx' - 1 ... `max_idx' + 1.
            timestamp_offset = (lagrange_interpolated_max_idx - (N_ELEM(lagrange_bases) - 1) / 2) *
                (1'000'000l / AVGS_PER_SECOND * 2l / (N_ELEM(lagrange_bases) - 1l));
            // printf("max_idx = %d, timestamp_offset = %d\n", max_idx, timestamp_offset);
        }
        absolute_time_t toc = get_absolute_time();
        // printf("%llu\n", to_us_since_boot(toc) - to_us_since_boot(tic));
        
        // Make sure that we switch to a different `max_idx' only when we are as far away from the windows as possible.
        int lower_lim = idx_offs_mod_1s(max_idx, (int)(N_ELEM(conv_out) * 0.9));
        int higher_lim = idx_offs_mod_1s(max_idx, (int)(N_ELEM(conv_out) * 0.1));
        if (lower_lim > higher_lim ? (avg_buf_idx_mod_1s > lower_lim || avg_buf_idx_mod_1s < higher_lim) :
            (avg_buf_idx_mod_1s > lower_lim && avg_buf_idx_mod_1s < higher_lim)) {
            synced_max_idx = max_idx;
            synced_timestamp_offset = timestamp_offset;
        }
        
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
        bool start_100ms = avg_buf_idx_mod_1s == idx_offs_mod_1s(synced_max_idx, (int)(N_ELEM(conv_out) * 0.4));
        bool start_200ms = avg_buf_idx_mod_1s == idx_offs_mod_1s(synced_max_idx, (int)(N_ELEM(conv_out) * 0.5));
        bool end_200ms = avg_buf_idx_mod_1s == idx_offs_mod_1s(synced_max_idx, (int)(N_ELEM(conv_out) * 0.6));
        // if (print_max_idx) {
        //     print_max_idx = false;
        //     printf("synced_max_idx = %d, start_100ms = %d, start_200ms = %d, end_200ms = %d, lower_lim = %d, "
        //             "higher_lim = %d\n", synced_max_idx, idx_offs_mod_1s(synced_max_idx, (int)(N_ELEM(conv_out) * 0.4)),
        //             idx_offs_mod_1s(synced_max_idx, (int)(N_ELEM(conv_out) * 0.5)),
        //             idx_offs_mod_1s(synced_max_idx, (int)(N_ELEM(conv_out) * 0.6)), lower_lim, higher_lim);
        // }

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
                prev_timestamp = timestamp + synced_timestamp_offset;
            }
        }
        // printf("%" PRIi32 ", %" PRIi32 "\n", start_100ms || end_200ms ? 0 : avg, prev_avg_avgs[1]);
        
        avg_sum += packet.avg;
    }
}

int main(void) {
    int16_t bpf_coeffs[5];
    int32_t bpf_w[5] = { 0, 0 };
    int32_t avgs[4] = { 0, 0, 0, 0 };
    
    stdio_init_all();
    
    // Start the RTC.
    rtc_init();
    // Set timezone to CEST/CET.
    setenv("TZ", "CEST-1CET,M3.2.0/2:00:00,M11.1.0/2:00:00", 1);
    tzset();

    multicore_launch_core1(&core1_main);
    
    generate_biquad_IIR_bpf(bpf_coeffs, (double)SAMPLE_RATE, 77.5e3, 15000.); // DCF77 frequency.
    
    /**
     * Initialize onboard LED indicator for showing the average of the BPF output.
     */
    // Tell GPIO 25 it is allocated to the PWM.
    gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 25.
    uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);
    uint chan = pwm_gpio_to_channel(PICO_DEFAULT_LED_PIN);

    // Set period of 32768 cycles (0 to 32767 inclusive).
    pwm_set_wrap(slice_num, INT16_MAX);
    // Set channel A output high for 0 cycles, so the LED stays off.
    pwm_set_chan_level(slice_num, chan, 0);
    // Set the PWM running.
    pwm_set_enabled(slice_num, true);
    
    /**
     * Initialize ADC with a sample rate of 500kHz and DMA with ping-pong buffers.
     */
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
    uint64_t prev_timestamp = 0ull;
    uint32_t adc_offset = 0u;
    while (true) {
        // dma_channel_wait_for_finish_blocking(dma_chan);
        if (sample_buf == nullptr || sample_buf == prev_sample_buf)
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
            avgs[i] = 0;
            for (int j = N_ELEM(bpf_output_buf) * i / 4; j < N_ELEM(bpf_output_buf) * (i + 1) / 4; ++j)
                avgs[i] += bpf_output_buf[j] < 0 ? -bpf_output_buf[j] : bpf_output_buf[j];
            // We're not dividing by a power of 2 anymore here, but apparently the / operator is using the
            // hardware divider anyway, so it will only take 8 cycles.
            int16_t avg = (avgs[0] + avgs[1] + avgs[2] + avgs[3]) / N_ELEM(bpf_output_buf);
            // Update indicator LED, square `avg' for gamma correction and bitshift by 15 to the right to get it in the 
            // same scale again.
            pwm_set_chan_level(slice_num, chan, ((int32_t)avg * (int32_t)avg) >> 15);
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
            
            // 1 000 000 is the amount of microseconds in a second, so with 1 000 000 / AVGS_PER_SECOND we get
            // the amount of microseconds per avg.
            uint64_t timestamp = to_us_since_boot(processing_start_time) - 1'000'000ull / AVGS_PER_SECOND * (2 + 3 - i);
            // Send to the other core to process.
            static_assert(sizeof(union multicore_packet) == 8);
            PUSH_MULTICORE_PACKET(((union multicore_packet) { .timestamp_diff = timestamp - prev_timestamp, .avg = avg }));
            prev_timestamp = timestamp;
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

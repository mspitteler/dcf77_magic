#!/usr/bin/tcc -run

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>

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

// Taken from: https://dcf77logs.de/ereignisse/30%20-%20Schaltsekunde.zip
const uint64_t bits_leap[] = {
    0b000000000000000000000000000000000000000000000010000000000000,
    
    0b100101011001110000100110101010000000010000011111100010010001,  // So, 01.07.12 00:55:00, SZ   
    0b101100100011110100100101101010000000010000011111100010010001,  // So, 01.07.12 00:56:00, SZ   
    0b100001011011000100100111101011000000010000011111100010010001,  // So, 01.07.12 00:57:00, SZ   
    0b100101100010101100100100011011000000010000011111100010010001,  // So, 01.07.12 00:58:00, SZ   
    0b101000001011111100100110011010000000010000011111100010010001,  // So, 01.07.12 00:59:00, SZ   
    0b100010010100010100100100000000100000110000011111100010010001,  // So, 01.07.12 01:00:00, SZ   
    0b100010011001110000101110000001100000110000011111100010010001,  // So, 01.07.12 01:01:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101100101001000101101000001100000110000011111100010010001,  // So, 01.07.12 01:02:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101000001101000101111000000100000110000011111100010010001,  // So, 01.07.12 01:03:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000111001001000101100100001100000110000011111100010010001,  // So, 01.07.12 01:04:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100001111010001000101110100000100000110000011111100010010001,  // So, 01.07.12 01:05:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100001011111111000101101100000100000110000011111100010010001,  // So, 01.07.12 01:06:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100010010011110000101111100001100000110000011111100010010001,  // So, 01.07.12 01:07:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101100101100101100101100010001100000110000011111100010010001,  // So, 01.07.12 01:08:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100111011111010000101110010000100000110000011111100010010001,  // So, 01.07.12 01:09:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100110011010001100101100001001100000110000011111100010010001,  // So, 01.07.12 01:10:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101100000011111000101110001000100000110000011111100010010001,  // So, 01.07.12 01:11:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101001110010111100101101001000100000110000011111100010010001,  // So, 01.07.12 01:12:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101111010100100101111001001100000110000011111100010010001,  // So, 01.07.12 01:13:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100010001111111000101100101000100000110000011111100010010001,  // So, 01.07.12 01:14:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101011111110011100101110101001100000110000011111100010010001,  // So, 01.07.12 01:15:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000011011101100101101101001100000110000011111100010010001,  // So, 01.07.12 01:16:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100001100010010000101111101000100000110000011111100010010001,  // So, 01.07.12 01:17:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100011101000010000101100011000100000110000011111100010010001,  // So, 01.07.12 01:18:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000110001100100101110011001100000110000011111100010010001,  // So, 01.07.12 01:19:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101111111010100101100000101100000110000011111100010010001,  // So, 01.07.12 01:20:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101000110011011100101110000100100000110000011111100010010001,  // So, 01.07.12 01:21:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101100010101100101101000100100000110000011111100010010001,  // So, 01.07.12 01:22:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101010101101000100101111000101100000110000011111100010010001,  // So, 01.07.12 01:23:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101110111100101000101100100100100000110000011111100010010001,  // So, 01.07.12 01:24:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101000000001000101110100101100000110000011111100010010001,  // So, 01.07.12 01:25:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101101000000010000101101100101100000110000011111100010010001,  // So, 01.07.12 01:26:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100111011110101100101111100100100000110000011111100010010001,  // So, 01.07.12 01:27:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101011000110000101100010100100000110000011111100010010001,  // So, 01.07.12 01:28:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100010100100101000101110010101100000110000011111100010010001,  // So, 01.07.12 01:29:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100100011111011000101100001100100000110000011111100010010001,  // So, 01.07.12 01:30:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000000011111100101110001101100000110000011111100010010001,  // So, 01.07.12 01:31:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100001101011001000101101001101100000110000011111100010010001,  // So, 01.07.12 01:32:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100100111100110000101111001100100000110000011111100010010001,  // So, 01.07.12 01:33:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100100000001101100101100101101100000110000011111100010010001,  // So, 01.07.12 01:34:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101000110110010100101110101100100000110000011111100010010001,  // So, 01.07.12 01:35:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101100011001011000101101101100100000110000011111100010010001,  // So, 01.07.12 01:36:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101101011000100101111101101100000110000011111100010010001,  // So, 01.07.12 01:37:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101000011011100100101100011101100000110000011111100010010001,  // So, 01.07.12 01:38:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100011010111101100101110011100100000110000011111100010010001,  // So, 01.07.12 01:39:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000110011101100101100000011100000110000011111100010010001,  // So, 01.07.12 01:40:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000100001010000101110000010100000110000011111100010010001,  // So, 01.07.12 01:41:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101000101011110000101101000010100000110000011111100010010001,  // So, 01.07.12 01:42:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100011011001000100101111000011100000110000011111100010010001,  // So, 01.07.12 01:43:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101001101100101000101100100010100000110000011111100010010001,  // So, 01.07.12 01:44:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100000110110110100101110100011100000110000011111100010010001,  // So, 01.07.12 01:45:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100011001000111000101101100011100000110000011111100010010001,  // So, 01.07.12 01:46:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100010000101100000101111100010100000110000011111100010010001,  // So, 01.07.12 01:47:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101000101111000101100010010100000110000011111100010010001,  // So, 01.07.12 01:48:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100100100011101100101110010011100000110000011111100010010001,  // So, 01.07.12 01:49:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100010000100110100101100001010100000110000011111100010010001,  // So, 01.07.12 01:50:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101101111010110000101110001011100000110000011111100010010001,  // So, 01.07.12 01:51:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100010001000000000101101001011100000110000011111100010010001,  // So, 01.07.12 01:52:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101010000100011100101111001010100000110000011111100010010001,  // So, 01.07.12 01:53:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101011010111101100101100101011100000110000011111100010010001,  // So, 01.07.12 01:54:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100001110011100000101110101010100000110000011111100010010001,  // So, 01.07.12 01:55:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100101001110000100101101101010100000110000011111100010010001,  // So, 01.07.12 01:56:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101110000011100000101111101011100000110000011111100010010001,  // So, 01.07.12 01:57:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b100011111010000100101100011011100000110000011111100010010001,  // So, 01.07.12 01:58:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b101110100010110100101110011010100000110000011111100010010001,  // So, 01.07.12 01:59:00, SZ   Einfügen einer Schaltsekunde angekündigt
    0b1000011011111101001011000000000100001100000111111000100100010, // So, 01.07.12 02:00:00, SZ   Einfügen einer Schaltsekunde angekündigt, weiteren Impuls erhalten (Schaltsekunde)
    0b100100101011110100100110000001010000110000011111100010010001,  // So, 01.07.12 02:01:00, SZ   
    0b100100111001110100100101000001010000110000011111100010010001,  // So, 01.07.12 02:02:00, SZ   
    0b101001111101101000100111000000010000110000011111100010010001,  // So, 01.07.12 02:03:00, SZ   
    0b100111010001000100100100100001010000110000011111100010010001,  // So, 01.07.12 02:04:00, SZ   
    0b101101101110011100100110100000010000110000011111100010010001,  // So, 01.07.12 02:05:00, SZ  
};

// Taken from: https://dcf77logs.de/ereignisse/28%20-%20Jahreswechsel.zip
const uint64_t bits_year_switch[] = {
    0b101100011110011100010100001100110001110001101101001100010001,  // Sa, 31.12.11 23:30:00, NZ   
    0b100100001010001000010110001101110001110001101101001100010001,  // Sa, 31.12.11 23:31:00, NZ   
    0b101010110010010000010101001101110001110001101101001100010001,  // Sa, 31.12.11 23:32:00, NZ   
    0b100010001101010000010111001100110001110001101101001100010001,  // Sa, 31.12.11 23:33:00, NZ   
    0b100000011011100100010100101101110001110001101101001100010001,  // Sa, 31.12.11 23:34:00, NZ   
    0b101011101001110100010110101100110001110001101101001100010001,  // Sa, 31.12.11 23:35:00, NZ   
    0b100001001010000100010101101100110001110001101101001100010001,  // Sa, 31.12.11 23:36:00, NZ   
    0b100010001000001000010111101101110001110001101101001100010001,  // Sa, 31.12.11 23:37:00, NZ   
    0b101000000001110000010100011101110001110001101101001100010001,  // Sa, 31.12.11 23:38:00, NZ   
    0b100000110110000100010110011100110001110001101101001100010001,  // Sa, 31.12.11 23:39:00, NZ   
    0b100000110000010100010100000011110001110001101101001100010001,  // Sa, 31.12.11 23:40:00, NZ   
    0b101111111110101100010110000010110001110001101101001100010001,  // Sa, 31.12.11 23:41:00, NZ   
    0b100100011110000000010101000010110001110001101101001100010001,  // Sa, 31.12.11 23:42:00, NZ   
    0b100000110000000100010111000011110001110001101101001100010001,  // Sa, 31.12.11 23:43:00, NZ   
    0b101000011100010000010100100010110001110001101101001100010001,  // Sa, 31.12.11 23:44:00, NZ   
    0b101111010110011000010110100011110001110001101101001100010001,  // Sa, 31.12.11 23:45:00, NZ   
    0b100001101010011100010101100011110001110001101101001100010001,  // Sa, 31.12.11 23:46:00, NZ   
    0b100000011010111100010111100010110001110001101101001100010001,  // Sa, 31.12.11 23:47:00, NZ   
    0b100000011011001100010100010010110001110001101101001100010001,  // Sa, 31.12.11 23:48:00, NZ   
    0b100001001010010100010110010011110001110001101101001100010001,  // Sa, 31.12.11 23:49:00, NZ   
    0b101000101011000000010100001010110001110001101101001100010001,  // Sa, 31.12.11 23:50:00, NZ   
    0b100110111101100000010110001011110001110001101101001100010001,  // Sa, 31.12.11 23:51:00, NZ   
    0b100101011010000100010101001011110001110001101101001100010001,  // Sa, 31.12.11 23:52:00, NZ   
    0b100110110100111100010111001010110001110001101101001100010001,  // Sa, 31.12.11 23:53:00, NZ   
    0b100101111001001100010100101011110001110001101101001100010001,  // Sa, 31.12.11 23:54:00, NZ   
    0b100110010000001100010110101010110001110001101101001100010001,  // Sa, 31.12.11 23:55:00, NZ   
    0b101100000011001000010101101010110001110001101101001100010001,  // Sa, 31.12.11 23:56:00, NZ   
    0b100110100011111100010111101011110001110001101101001100010001,  // Sa, 31.12.11 23:57:00, NZ   
    0b100000110011000100010100011011110001110001101101001100010001,  // Sa, 31.12.11 23:58:00, NZ   
    0b101110100010100100010110011010110001110001101101001100010001,  // Sa, 31.12.11 23:59:00, NZ   
    0b101011010101000100010100000000000000010000011110000010010001,  // So, 01.01.12 00:00:00, NZ   
    0b100000000010001100010110000001000000010000011110000010010001,  // So, 01.01.12 00:01:00, NZ   
    0b100111010101101100010101000001000000010000011110000010010001,  // So, 01.01.12 00:02:00, NZ   
    0b101111010010100100010111000000000000010000011110000010010001,  // So, 01.01.12 00:03:00, NZ   
    0b100010011010001100010100100001000000010000011110000010010001,  // So, 01.01.12 00:04:00, NZ   
    0b101000001101011000010110100000000000010000011110000010010001,  // So, 01.01.12 00:05:00, NZ   
    0b101100010000111000010101100000000000010000011110000010010001,  // So, 01.01.12 00:06:00, NZ   
    0b100111100010101000010111100001000000010000011110000010010001,  // So, 01.01.12 00:07:00, NZ   
    0b101010110010111100010100010001000000010000011110000010010001,  // So, 01.01.12 00:08:00, NZ   
    0b101100111100011000010110010000000000010000011110000010010001,  // So, 01.01.12 00:09:00, NZ   
    0b100111011000010100010100001001000000010000011110000010010001,  // So, 01.01.12 00:10:00, NZ   
    0b100100100011001000010110001000000000010000011110000010010001,  // So, 01.01.12 00:11:00, NZ   
    0b100100110110100000010101001000000000010000011110000010010001,  // So, 01.01.12 00:12:00, NZ   
    0b100001001001100000010111001001000000010000011110000010010001,  // So, 01.01.12 00:13:00, NZ   
    0b101001111010001000010100101000000000010000011110000010010001,  // So, 01.01.12 00:14:00, NZ   
    0b101110011011101000010110101001000000010000011110000010010001,  // So, 01.01.12 00:15:00, NZ   
    0b100100000011111100010101101001000000010000011110000010010001,  // So, 01.01.12 00:16:00, NZ   
    0b100111100011011000010111101000000000010000011110000010010001,  // So, 01.01.12 00:17:00, NZ   
    0b100101110101101000010100011000000000010000011110000010010001,  // So, 01.01.12 00:18:00, NZ   
    0b100111100010100100010110011001000000010000011110000010010001,  // So, 01.01.12 00:19:00, NZ   
    0b101100010100001100010100000101000000010000011110000010010001,  // So, 01.01.12 00:20:00, NZ   
    0b100110101010010100010110000100000000010000011110000010010001,  // So, 01.01.12 00:21:00, NZ   
    0b100001001001011100010101000100000000010000011110000010010001,  // So, 01.01.12 00:22:00, NZ   
    0b101111010011000000010111000101000000010000011110000010010001,  // So, 01.01.12 00:23:00, NZ   
    0b101010111000100100010100100100000000010000011110000010010001,  // So, 01.01.12 00:24:00, NZ   
    0b100010010011010100010110100101000000010000011110000010010001,  // So, 01.01.12 00:25:00, NZ   
    0b100001000110010100010101100101000000010000011110000010010001,  // So, 01.01.12 00:26:00, NZ   
    0b100111001111000000010111100100000000010000011110000010010001,  // So, 01.01.12 00:27:00, NZ   
    0b100111010000010100010100010100000000010000011110000010010001,  // So, 01.01.12 00:28:00, NZ   
    0b100101011011000100010110010101000000010000011110000010010001,  // So, 01.01.12 00:29:00, NZ   
    0b101000010110110100010100001100000000010000011110000010010001,  // So, 01.01.12 00:30:00, NZ   
};

const struct { const uint64_t *const a; const int len; const char *const name; } bits[] = {
    { .a = bits_leap, .len = N_ELEM(bits_leap), .name = "bits_leap" },
    { .a = bits_year_switch, .len = N_ELEM(bits_year_switch), .name = "bits_year_switch" }
};

enum bit_value_or_sync { LOW, HIGH, SYNC };

static const int bcd_table[] = { 1, 2, 4, 8, 10, 20, 40, 80 };

static inline uint32_t us_to_ms(uint64_t us) {
    return (uint32_t)(us / 1000u);
}

void process_bit(uint64_t timestamp, enum bit_value_or_sync bit_or_sync, bool last_minute) {
    static int sync_marks_per_bit[61];
    static bool backup_antenna = false, announce_dst_switch = false, cest = false, cet = false;
    static int minutes = -1, hours = -1, day_of_month = -1, day_of_week = -1, month_num = -1, year = -1;
    static int bit = 0;
    
    static int announce_leap_second = 0;
    // When more than half the bits in the last hour had the leap second announcement bit set, we can be fairly
    // certain that we have a leap second at the last second of the hour.
    // bool last_minute = false;
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
        // printf("%" PRIu32 ": Sync mark!\n", us_to_ms(timestamp));
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
        case 37 ... 41:
            // Day of month.
            day_of_month += (int)bit_value * bcd_table[bit_after_sync - 36];
            parity ^= bit_value;
            break;
        case 42:
            day_of_week = 0;
        case 43 ... 44:
            // Day of week (1 = Monday, 2 = Tuesday, ..., 7 = Sunday).
            day_of_week += (int)bit_value * bcd_table[bit_after_sync - 42];
            parity ^= bit_value;
            break;
        case 45:
            month_num = 0;
        case 46 ... 49:
            // Month number.
            month_num += (int)bit_value * bcd_table[bit_after_sync - 45];
            parity ^= bit_value;
            break;
        case 50:
            year = 0;
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
    if (bit_after_sync == 59 && seconds_in_minute == 61)
        announce_leap_second = 0;
}

int main(void) {
    for (int i = 0; i < N_ELEM(bits); i++) {
        printf("-------%s-------\n", bits[i].name);
        for (int j = 0; j < bits[i].len; j++) {
            bool _0b1 = false;
            bool last_minute = (i == 0 && (j == 66 || j == 6)) || (i == 1 && j == 30);
            for (int bit = 63; bit >= -1; bit--) {
                enum bit_value_or_sync bit_or_sync = bit == -1 ? SYNC : (bits[i].a[j] & (1ull << bit) ? HIGH : LOW);
                if (bit_or_sync == HIGH && !_0b1) {
                    _0b1 = true;
                    continue;
                }
                if (_0b1) {
                    uint64_t timestamp = 0ull;
                    process_bit(timestamp, bit_or_sync, last_minute);
                }
            }
        }
    }
}

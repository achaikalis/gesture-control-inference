/**
 * @file main.h
 * @brief Header and LED PIN Definitions
 *
 * @note ...
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "Arduino.h"

#define LED_PIN LED_BUILTIN
#define LEDR    (22u)
#define LEDG    (23u)
#define LEDB    (24u)

#define SAMPLE_RATE            100

#define WINDOW_MS              4000
#define TOTAL_SAMPLES          ((SAMPLE_RATE * WINDOW_MS) / 1000)

#define FRAME_SIZE             32        
#define FEATURES_PER_SAMPLE    7

// #define HOP_MS                 400
// #define TIME_SHIFT_MS          960
// #define SHIFT_SAMPLES          ((SAMPLE_RATE * TIME_SHIFT_MS) / 1000)

#define EI_FEATURE_BUFFER_SIZE (TOTAL_SAMPLES * FEATURES_PER_SAMPLE)

#define BLE_ON 0

float ei_feature_buffer[EI_FEATURE_BUFFER_SIZE];

/* Buffer State */
// bool buffer_filled     = false;
// unsigned long last_hop = 0;

/* Diagnostic Logger State */
static int last_best_label         = -1;
static float last_best_conf        = 0.0f;
static unsigned long last_infer_ts = 0;

const float a_mag[3][3] = {
    {0.979, -0.032, 0.011}, {0.032, 0.960, 0.072}, {0.011, 0.072, 1.071}};

const float b_mag[3] = {31.24, -23.34, -38.60};

#endif /* MAIN_H_ */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

// Define pins and other constants
#define SD_CS 1
#define SD_MISO 7
#define ENCODER_PIN1 47
#define ENCODER_PIN2 48
#define BUTTON_PIN_01 18
#define BUTTON_PIN_02 38
#define SAMPLE_RATE 44100
#define AMPLITUDE 10000
#define I2S_BCK_PIN 15
#define I2S_WS_PIN 13
#define I2S_DATA_PIN 14
#define I2S_SAMPLE_RATE 44100
#define I2S_PORT I2S_NUM_0
#define M_PI 3.14159265358979323846

void generate_sine_wave(int16_t *buffer, int sample_rate, int frequency, int amplitude, int num_samples) {
    for (int i = 0; i < num_samples; ++i) {
        buffer[i] = (int16_t)(amplitude * sinf(2 * M_PI * frequency * i / sample_rate));
    }
}

void app_main(void) {
    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    // I2S pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DATA_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    // Install and start I2S driver
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    // Generate sine wave buffer
    const int num_samples = I2S_SAMPLE_RATE;  // 1 second of audio
    int16_t *sine_wave_buffer = (int16_t *)malloc(num_samples * sizeof(int16_t));
    generate_sine_wave(sine_wave_buffer, I2S_SAMPLE_RATE, 261, AMPLITUDE, num_samples);

    // Allocate silence buffer dynamically
    int16_t *silence_buffer = (int16_t *)calloc(num_samples, sizeof(int16_t));

    while (true) {
        // Write sine wave buffer to I2S
        size_t bytes_written;
        i2s_write(I2S_PORT, sine_wave_buffer, num_samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Write silence buffer to I2S
        i2s_write(I2S_PORT, silence_buffer, num_samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Clean up
    free(sine_wave_buffer);
    free(silence_buffer);
    i2s_driver_uninstall(I2S_PORT);
}

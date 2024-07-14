#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"

// Define pins and other constants
#define I2S_BCK_PIN 15
#define I2S_WS_PIN 13
#define I2S_DATA_PIN 14
#define I2S_SAMPLE_RATE 44100
#define I2S_PORT I2S_NUM_0
#define AMPLITUDE 10000
#define NOTE_DURATION_MS 200  // Duration of each note in milliseconds
#define WAVETABLE_SIZE 256
#define BUTTON_PIN_1 18
#define BUTTON_PIN_2 38
#define DEBOUNCE_TIME_MS 50  // Debounce time in milliseconds

// Reverb parameters
#define REVERB_DELAY 2000
#define REVERB_DECAY 0.5
#define MAX_DELAY_SAMPLES 44100

// Frequencies for the arpeggio (C3, E3, G3, B3, C4, B3, G3, E3)
float frequencies[] = {130.81, 164.81, 196.00, 246.94, 261.63, 246.94, 196.00, 164.81};
int current_note_index = 0;

// Waveform types
typedef enum {
    WAVEFORM_SINE,
    WAVEFORM_SQUARE,
    WAVEFORM_SAWTOOTH,
    WAVEFORM_TRIANGLE,
    NUM_WAVEFORMS
} waveform_t;

waveform_t current_waveform = WAVEFORM_SINE;

// Wavetable arrays
int16_t sine_wave_table[WAVETABLE_SIZE];
int16_t square_wave_table[WAVETABLE_SIZE];
int16_t sawtooth_wave_table[WAVETABLE_SIZE];
int16_t triangle_wave_table[WAVETABLE_SIZE];

// Reverb buffer
int16_t reverb_buffer[MAX_DELAY_SAMPLES];
int reverb_buffer_index = 0;

// Function to generate wavetables for different waveforms
void generate_wavetables() {
    for (int i = 0; i < WAVETABLE_SIZE; ++i) {
        sine_wave_table[i] = (int16_t)(AMPLITUDE * sinf(2 * M_PI * i / WAVETABLE_SIZE));
        square_wave_table[i] = (i < WAVETABLE_SIZE / 2) ? AMPLITUDE : -AMPLITUDE;
        sawtooth_wave_table[i] = (int16_t)(AMPLITUDE * (2.0 * i / WAVETABLE_SIZE - 1.0));
        triangle_wave_table[i] = (i < WAVETABLE_SIZE / 2) ? (int16_t)(AMPLITUDE * (4.0 * i / WAVETABLE_SIZE - 1.0)) :
                                (int16_t)(AMPLITUDE * (3.0 - 4.0 * i / WAVETABLE_SIZE));
    }
}

// Function to generate audio buffer from wavetable
void generate_audio_buffer_from_wavetable(int16_t *buffer, int sample_rate, float frequency, int amplitude, int num_samples, waveform_t waveform, float detune) {
    float wavetable_index = 0;
    float wavetable_step = (float)WAVETABLE_SIZE * frequency * pow(2.0, detune / 1200.0) / sample_rate;

    for (int i = 0; i < num_samples; ++i) {
        int16_t sample = 0;
        switch (waveform) {
            case WAVEFORM_SINE:
                sample = sine_wave_table[(int)wavetable_index];
                break;
            case WAVEFORM_SQUARE:
                sample = square_wave_table[(int)wavetable_index];
                break;
            case WAVEFORM_SAWTOOTH:
                sample = sawtooth_wave_table[(int)wavetable_index];
                break;
            case WAVEFORM_TRIANGLE:
                sample = triangle_wave_table[(int)wavetable_index];
                break;
            default:
                sample = 0;
                break;
        }
        buffer[2 * i] = sample;
        buffer[2 * i + 1] = sample;  // Both channels play the same sample for mono output

        wavetable_index += wavetable_step;
        if (wavetable_index >= WAVETABLE_SIZE) {
            wavetable_index -= WAVETABLE_SIZE;
        }
    }
}

// Apply reverb to the audio buffer
void apply_reverb(int16_t *buffer, int num_samples) {
    for (int i = 0; i < num_samples * 2; ++i) {
        int delayed_index = (reverb_buffer_index - REVERB_DELAY + MAX_DELAY_SAMPLES) % MAX_DELAY_SAMPLES;
        int16_t reverb_sample = reverb_buffer[delayed_index];

        reverb_buffer[reverb_buffer_index] = buffer[i] + reverb_sample * REVERB_DECAY;
        buffer[i] = buffer[i] + reverb_sample * REVERB_DECAY;

        reverb_buffer_index = (reverb_buffer_index + 1) % MAX_DELAY_SAMPLES;
    }
}

// Function to handle button press
void IRAM_ATTR button_isr_handler(void* arg) {
    static uint32_t last_interrupt_time = 0;
    uint32_t interrupt_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME_MS) {
        if ((int)arg == BUTTON_PIN_1) {
            // Change direction of arpeggio
            // In this case, we are not changing direction, so nothing happens here
        } else if ((int)arg == BUTTON_PIN_2) {
            current_waveform = (current_waveform + 1) % NUM_WAVEFORMS; // Change waveform
        }
    }
    last_interrupt_time = interrupt_time;
}

void app_main(void) {
    // Initialize reverb buffer
    memset(reverb_buffer, 0, sizeof(reverb_buffer));

    // Generate wavetables
    generate_wavetables();

    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Stereo output
        .communication_format = I2S_COMM_FORMAT_I2S,
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
    i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

    // Configure button pins as input
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN_1) | (1ULL << BUTTON_PIN_2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    // Install ISR service and add ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN_1, button_isr_handler, (void*)BUTTON_PIN_1);
    gpio_isr_handler_add(BUTTON_PIN_2, button_isr_handler, (void*)BUTTON_PIN_2);

    // Allocate memory for waveform buffer
    const int num_samples = I2S_SAMPLE_RATE / 8;  // 1/8 second of audio
    int16_t *waveform_buffer = (int16_t *)malloc(num_samples * 2 * sizeof(int16_t));  // Stereo buffer
    int16_t *second_voice_buffer = (int16_t *)malloc(num_samples * 2 * sizeof(int16_t));  // Stereo buffer for second voice

    if (waveform_buffer == NULL || second_voice_buffer == NULL) {
        printf("Failed to allocate memory for waveform buffer\n");
        while (1);  // Halt execution if memory allocation fails
    }

    while (true) {
        // Generate and play waveform for the current note
        float freq = frequencies[current_note_index];
                printf("Playing frequency: %.2f Hz (Note index: %d, Waveform: %d)\n", freq, current_note_index, current_waveform);
        generate_audio_buffer_from_wavetable(waveform_buffer, I2S_SAMPLE_RATE, freq, AMPLITUDE, num_samples, current_waveform, 0);

        // Generate and play second voice one octave lower with detune
        generate_audio_buffer_from_wavetable(second_voice_buffer, I2S_SAMPLE_RATE, freq / 2, AMPLITUDE, num_samples, current_waveform, -20);

        // Mix the two voices with panning
        for (int i = 0; i < num_samples; ++i) {
            int16_t left_sample = waveform_buffer[2 * i] / 2;
            int16_t right_sample = second_voice_buffer[2 * i] / 2;

            waveform_buffer[2 * i] = left_sample;         // Left channel
            waveform_buffer[2 * i + 1] = right_sample;    // Right channel
        }

        // Apply reverb to the mixed buffer
        apply_reverb(waveform_buffer, num_samples);

        size_t bytes_written;
        i2s_write(I2S_PORT, waveform_buffer, num_samples * 2 * sizeof(int16_t), &bytes_written, portMAX_DELAY);

        // Move to the next note
        current_note_index = (current_note_index + 1) % (sizeof(frequencies) / sizeof(frequencies[0]));

        vTaskDelay(pdMS_TO_TICKS(NOTE_DURATION_MS));  // Play each note for the duration of a note
    }

    // Clean up
    free(waveform_buffer);
    free(second_voice_buffer);
    i2s_driver_uninstall(I2S_PORT);
}


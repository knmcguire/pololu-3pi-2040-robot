// A simple example showing how to blink the yellow LED on the Pololu 3pi+
// 2040 Robot without using Pololu libraries.
#include <stdio.h>

#include <pico/stdlib.h>
#include "hardware/uart.h"
#include <pololu_3pi_2040_robot.h>
#include <pico/time.h>
#include <hardware/pio.h>
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"


// UART Initializations
#define UART_TX_PIN 28
#define UART_RX_PIN 29
#define UART_ID uart0
#define BAUD_RATE 9600
#define ARRAY_SIZE 6

int main() {

    // Initialize the standard I/O library for Pololu.
    stdio_init_all();
    display_init();
    motors_init();

    // Initialize the motors
    // For the hyper version the motors are reversed
    motors_flip_left(true);
    motors_flip_right(true);
    int32_t speed=1200;
    float degrees_max = 15;

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initialize display
    display_fill(0);
    display_show();

    uint32_t it = 0;

    // Start and end bytes for UART communication with array
    uint8_t start_byte = 0xAA;
    uint8_t end_byte = 0x55;
    uint8_t data[ARRAY_SIZE * sizeof(float) + 2]; // Extra space for start and end bytes
    float array[ARRAY_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Initialize encoders
    const uint sm = 0;
    PIO pio_left = pio0;
    PIO pio_right = pio1;
    pio_add_program(pio_left, &quadrature_encoder_program);
    pio_add_program(pio_right, &quadrature_encoder_program);

    quadrature_encoder_program_init(pio_left, sm, 12, 0);
    quadrature_encoder_program_init(pio_right, sm, 8, 0);

    int new_enc_value_left, delta_left, old_enc_value_left = 0;
    int new_enc_value_right, delta_right, old_enc_value_right = 0;

    while (1){
        sleep_ms(100);

        // Read the encoder value
        new_enc_value_left = quadrature_encoder_get_count(pio_left, sm);
        delta_left = new_enc_value_left - old_enc_value_left;
        old_enc_value_left = new_enc_value_left;

        new_enc_value_right = quadrature_encoder_get_count(pio_right, sm);
        delta_right = new_enc_value_right - old_enc_value_right;
        old_enc_value_right = new_enc_value_right;

        // Read the UART data
        while (uart_getc(UART_ID) != start_byte);
        for (int i = 1; i < sizeof(data) - 1; i++) { // Start at 1 to skip start byte
            data[i] = uart_getc(UART_ID);
        }
        // Check for end byte
        if (data[sizeof(data) - 1] != end_byte) {
            // Handle error
        }
        for (int i = 0; i < ARRAY_SIZE; i++) {
            memcpy(&array[i], &data[i * sizeof(float) + 1], sizeof(float)); // Offset by 1 for start byte
        }

        // Control the motors
        float left_speed = 0;
        float right_speed = 0;

        if (array[4] > degrees_max) {
            left_speed = speed;
            right_speed = speed;
        } else if(array[4] < -degrees_max){
            left_speed = -speed;
            right_speed = -speed;
        }else
        {
            left_speed = 0;
            right_speed = 0;
        }

        if (array[5] > degrees_max) {
            left_speed += speed/3;
            right_speed -= speed/3;
        } else if(array[5] < -degrees_max){
            left_speed -= speed/3;
            right_speed += speed/3;
        }else
        {
            // Nothing happens
        }

        // motors_set_speeds(left_speed, right_speed);

        // Fill strings for display
        char str1[64];
        char str2[64];
        char str3[64];
        char empty[64];
        sprintf(str1, "%.1f %.1f %.1f", array[0], array[1], array[2]);
        sprintf(str2, "%.1f %.1d %.1d", array[3], new_enc_value_left, new_enc_value_right);
        sprintf(str3, "%.1f  %.1f", array[4], array[5]);
        display_fill_rect(0, 16, DISPLAY_WIDTH, 16, 0 | DISPLAY_NOW);
        display_fill_rect(0, 32, DISPLAY_WIDTH, 16, 0 | DISPLAY_NOW);
        display_fill_rect(0, 48, DISPLAY_WIDTH, 16, 0 | DISPLAY_NOW);
        display_text(str1, 0, 16, DISPLAY_NOW | COLOR_BLACK_ON_WHITE);
        display_text(str2, 0, 32, DISPLAY_NOW | COLOR_BLACK_ON_WHITE);
        display_text(str3, 0, 48, DISPLAY_NOW | COLOR_BLACK_ON_WHITE);

        it++;
    }
}
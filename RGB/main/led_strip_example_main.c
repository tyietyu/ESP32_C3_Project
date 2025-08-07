/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

#define LED_RED     GPIO_NUM_0
#define LED_GREEN   GPIO_NUM_1
#define LED_BLUE    GPIO_NUM_2
#define KEY_INPUT   GPIO_NUM_7

#define STATE_MAX   7
#define DEBOUNCE_TIME_US 20000

volatile uint32_t key_count = 0;
static volatile uint64_t last_interrupt_time = 0;
static void IRAM_ATTR gpio_isr_handler(void* arg);

void led_init(void)
{
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_GREEN, 1);
    gpio_set_level(LED_BLUE, 1);
}

void key_init(void)
{
    gpio_set_direction(KEY_INPUT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(KEY_INPUT, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(KEY_INPUT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(KEY_INPUT, gpio_isr_handler, NULL);
}

gptimer_handle_t gptimer = NULL;
void timer_init(void)
{
    gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
    .direction = GPTIMER_COUNT_UP,      
    .resolution_hz = 1 * 1000 * 1000,   // 分辨率为 1 MHz，即 1 次滴答为 1 微秒
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void delay_us(uint64_t us)
{
    uint64_t start_time;
    gptimer_get_raw_count(gptimer, &start_time);
    while (true) {
        uint64_t current_time;
        gptimer_get_raw_count(gptimer, &current_time);
        if ((current_time - start_time) >= us) {
            break;
        }
    }
}

void set_led_color(int r, int g, int b)
{
    gpio_set_level(LED_RED, r ? 0 : 1);
    gpio_set_level(LED_GREEN, g ? 0 : 1);
    gpio_set_level(LED_BLUE, b ? 0 : 1);
}

void led_colorful_flow(void)
{
    // 1. Red
    set_led_color(1, 0, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 2. Green
    set_led_color(0, 1, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 3. Blue
    set_led_color(0, 0, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 4. Yellow (Red + Green)
    set_led_color(1, 1, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 5. Cyan (Green + Blue)
    set_led_color(0, 1, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 6. Magenta (Red + Blue)
    set_led_color(1, 0, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 7. White (Red + Green + Blue)
    set_led_color(1, 1, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Off
    set_led_color(0, 0, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void led_strobe_effect(void)
{
    for (int i = 0; i < 3; i++) {
        set_led_color(1, 0, 0); vTaskDelay(100 / portTICK_PERIOD_MS);
        set_led_color(0, 0, 0); vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    for (int i = 0; i < 3; i++) {
        set_led_color(0, 1, 0); vTaskDelay(100 / portTICK_PERIOD_MS);
        set_led_color(0, 0, 0); vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    for (int i = 0; i < 3; i++) {
        set_led_color(0, 0, 1); vTaskDelay(100 / portTICK_PERIOD_MS);
        set_led_color(0, 0, 0); vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void breathe_one_color(gpio_num_t led_pin)
{
    int brightness = 0;
    int step = 5;
    for (brightness = 0; brightness <= 255; brightness += step) {
        for (int i = 0; i < 20; i++) {
            gpio_set_level(led_pin, 0); // Turn on
            delay_us(brightness);
            gpio_set_level(led_pin, 1); // Turn off
            delay_us(255 - brightness);
        }
    }

    for (brightness = 255; brightness >= 0; brightness -= step) {
        // Loop for a short duration to make the brightness change visible
        for (int i = 0; i < brightness; i++) {
            gpio_set_level(led_pin, 0); // Turn on
            delay_us(brightness);
            gpio_set_level(led_pin, 1); // Turn off
            delay_us(255 - brightness);
        }
    }
}

void led_software_breathing_effect(void)
{
    set_led_color(0, 0, 0); 
    breathe_one_color(LED_RED);
    breathe_one_color(LED_GREEN);
    breathe_one_color(LED_BLUE);
    set_led_color(0, 0, 0);
}

void led_state_change(uint32_t counter)
{
    static uint32_t last_key_count = 0;
    if (counter == last_key_count && counter != 0) return;

    switch (counter) {
        case 1: set_led_color(1, 0, 0); break; // Red ON
        case 2: set_led_color(0, 0, 0); break; // All OFF
        case 3: set_led_color(0, 1, 0); break; // Green ON
        case 4: set_led_color(0, 0, 0); break; // All OFF
        case 5: set_led_color(0, 0, 1); break; // Blue ON
        case 6: set_led_color(0, 0, 0); break; // All OFF
        case 7: led_colorful_flow(); break;
        case 8: led_strobe_effect(); break;
        case 9: led_software_breathing_effect(); break; // 调用软件呼吸灯
        default: set_led_color(1, 1, 1); break;
    }
    last_key_count = counter;
}

static void IRAM_ATTR gpio_isr_handler(void* arg) 
{
    uint64_t count;
    gptimer_get_raw_count(gptimer, &count);
    uint64_t current_interrupt_time = count;
    if (current_interrupt_time - last_interrupt_time > DEBOUNCE_TIME_US) {
        key_count++;
    }
    if(key_count > STATE_MAX) {
        key_count = 1;
    }
    last_interrupt_time = current_interrupt_time;
}

void app_main(void)
{
   led_init();
   key_init();
   timer_init();

    while (1) {
        led_state_change(key_count);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

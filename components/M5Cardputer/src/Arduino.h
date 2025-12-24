/*
 * Arduino.h shim for ESP-IDF
 * Provides minimal Arduino compatibility layer for M5Cardputer keyboard
 */
#ifndef ARDUINO_H_SHIM
#define ARDUINO_H_SHIM

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin modes
#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3

// Interrupt modes
#define RISING          GPIO_INTR_POSEDGE
#define FALLING         GPIO_INTR_NEGEDGE
#define CHANGE          GPIO_INTR_ANYEDGE

// Arduino GPIO functions
inline void pinMode(int pin, int mode) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    switch (mode) {
        case OUTPUT:
            io_conf.mode = GPIO_MODE_OUTPUT;
            break;
        case INPUT:
            io_conf.mode = GPIO_MODE_INPUT;
            break;
        case INPUT_PULLUP:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            break;
        case INPUT_PULLDOWN:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        default:
            io_conf.mode = GPIO_MODE_INPUT;
            break;
    }
    gpio_config(&io_conf);
}

inline void digitalWrite(int pin, int value) {
    gpio_set_level((gpio_num_t)pin, value);
}

inline int digitalRead(int pin) {
    return gpio_get_level((gpio_num_t)pin);
}

inline int digitalPinToInterrupt(int pin) {
    return pin;  // On ESP32, the pin number is the interrupt number
}

// Install GPIO ISR service once
static bool _gpio_isr_service_installed = false;

inline void attachInterruptArg(int pin, void (*handler)(void*), void* arg, int mode) {
    if (!_gpio_isr_service_installed) {
        gpio_install_isr_service(0);
        _gpio_isr_service_installed = true;
    }
    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)mode);
    gpio_isr_handler_add((gpio_num_t)pin, handler, arg);
    gpio_intr_enable((gpio_num_t)pin);
}

inline void detachInterrupt(int pin) {
    gpio_isr_handler_remove((gpio_num_t)pin);
    gpio_intr_disable((gpio_num_t)pin);
}

// Timing functions
inline unsigned long millis() {
    return (unsigned long)(esp_timer_get_time() / 1000);
}

inline unsigned long micros() {
    return (unsigned long)esp_timer_get_time();
}

inline void delay(unsigned long ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

inline void delayMicroseconds(unsigned int us) {
    esp_rom_delay_us(us);
}

#endif // ARDUINO_H_SHIM

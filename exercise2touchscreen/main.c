/**
 * TSC2007 Touch Screen Controller Interface for Raspberry Pi Pico (Simplified)
 */
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>

// TSC2007 I2C address
#define TSC2007_ADDR 0x48

// Command bytes
#define TSC2007_CMD_READ_X 0xC0
#define TSC2007_CMD_READ_Y 0xD0

// Screen dimensions in mm
#define SCREEN_WIDTH_MM 165.0f
#define SCREEN_HEIGHT_MM 105.0f
#define MAX_ADC_VALUE 4095.0f

// I2C pins
#define SDA_PIN 0
#define SCL_PIN 1

void setup() {
    stdio_init_all();
    sleep_ms(2000);
    
    
    // Initialize I2C
    gpio_init(SDA_PIN);
    gpio_set_pulls(SDA_PIN, 1, 0);  // Pull-up enabled
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    
    gpio_init(SCL_PIN);
    gpio_set_pulls(SCL_PIN, 1, 0);  // Pull-up enabled
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    
    i2c_init(i2c0, 100000);  // 100 kHz
}

void loop() {
    // Read X position
    uint8_t cmd_x = TSC2007_CMD_READ_X;
    uint8_t x_data[2] = {0, 0};
    i2c_write_blocking(i2c0, TSC2007_ADDR, &cmd_x, 1, true);
    i2c_read_blocking(i2c0, TSC2007_ADDR, x_data, 2, false);
    uint16_t x_raw = ((uint16_t)x_data[0] << 4) | (x_data[1] >> 4);
    
    // Read Y position
    uint8_t cmd_y = TSC2007_CMD_READ_Y;
    uint8_t y_data[2] = {0, 0};
    i2c_write_blocking(i2c0, TSC2007_ADDR, &cmd_y, 1, true);
    i2c_read_blocking(i2c0, TSC2007_ADDR, y_data, 2, false);
    uint16_t y_raw = ((uint16_t)y_data[0] << 4) | (y_data[1] >> 4);
    
    // Convert to mm (simplified)
    float x_mm = (float)x_raw / MAX_ADC_VALUE * SCREEN_WIDTH_MM;
    float y_mm = (float)y_raw / MAX_ADC_VALUE * SCREEN_HEIGHT_MM;
    
    // Print results
    printf("Raw: X=%d, Y=%d | Physical: X=%.2f mm, Y=%.2f mm\r\n", 
           x_raw, y_raw, x_mm, y_mm);
    
    sleep_ms(200);
}

int main() {
    setup();
    
    while (true) {
        loop();
    }
}
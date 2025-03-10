/**
 * TSC2007 Touch Screen Controller Interface for Raspberry Pi Pico
 * 
 * This code reads X and Y position data from a TSC2007 touch screen controller
 * and converts the readings to physical units (millimeters) for a 7" touch screen
 * with dimensions 165mm x 105mm.
 */

 #include <stdio.h>
 #include <pico/stdlib.h>
 #include <hardware/i2c.h>
 
 // TSC2007 I2C address (assuming A1=0, A0=0)
 #define TSC2007_ADDR 0x48
 
 // Command bytes
 #define TSC2007_CMD_READ_X 0xC0  // 12-bit resolution, power down between conversions
 #define TSC2007_CMD_READ_Y 0xD0  // 12-bit resolution, power down between conversions
 #define TSC2007_CMD_PENIRQ_ON 0x00  // Power down, PENIRQ enabled
 
 // Touch screen dimensions in mm
 #define SCREEN_WIDTH_MM 165.0f
 #define SCREEN_HEIGHT_MM 105.0f
 
 // Maximum ADC value (12-bit resolution)
 #define MAX_ADC_VALUE 4095.0f
 
 // I2C pins
 #define I2C_SDA_PIN 0
 #define I2C_SCL_PIN 1
 #define I2C_INST i2c0
 
 // PENIRQ GPIO pin (optional, for detecting touch events)
 #define PENIRQ_PIN 16
 
 // Function prototypes
 bool tsc2007_read_position(uint16_t *x, uint16_t *y);
 void convert_to_mm(uint16_t x_raw, uint16_t y_raw, float *x_mm, float *y_mm);
 bool tsc2007_read_value(uint8_t command, uint16_t *value);
 void init_tsc2007(void);
 
 int main() {
     stdio_init_all();
     
     // Wait for serial console to connect
     sleep_ms(2000);
     printf("TSC2007 Touch Screen Demo for RP2040\n");
     
     // Initialize the touch screen interface
     init_tsc2007();
     
     while (true) {
         uint16_t x_raw, y_raw;
         float x_mm, y_mm;
         
         // Read the touch position
         if (tsc2007_read_position(&x_raw, &y_raw)) {
             // Convert to millimeters
             convert_to_mm(x_raw, y_raw, &x_mm, &y_mm);
             
             // Print the results
             printf("Raw: X=%d, Y=%d | Physical: X=%.2f mm, Y=%.2f mm\n", 
                    x_raw, y_raw, x_mm, y_mm);
         } else {
             printf("No touch detected or read error\n");
         }
         
         // Delay between readings
         sleep_ms(200);
     }
     
     return 0;
 }
 
 /**
  * Initialize the TSC2007 touch screen controller interface
  */
 void init_tsc2007(void) {
     // Initialize I2C
     i2c_init(I2C_INST, 400000);  // 400kHz
     gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
     gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
     gpio_pull_up(I2C_SDA_PIN);
     gpio_pull_up(I2C_SCL_PIN);
     
     // Initialize PENIRQ pin as input with pull-up if used
     #ifdef PENIRQ_PIN
     gpio_init(PENIRQ_PIN);
     gpio_set_dir(PENIRQ_PIN, GPIO_IN);
     gpio_pull_up(PENIRQ_PIN);
     #endif
     
     printf("TSC2007 interface initialized\n");
 }
 
 /**
  * Read X and Y position from the touch screen
  * 
  * @param x Pointer to store X position value
  * @param y Pointer to store Y position value
  * @return true if read successful, false otherwise
  */
 bool tsc2007_read_position(uint16_t *x, uint16_t *y) {
     bool success = true;
     
     // Read X position
     success &= tsc2007_read_value(TSC2007_CMD_READ_X, x);
     
     // Read Y position
     success &= tsc2007_read_value(TSC2007_CMD_READ_Y, y);
     
     return success;
 }
 
 /**
  * Read a value from the TSC2007 based on the provided command
  * 
  * @param command Command byte to send
  * @param value Pointer to store the read value
  * @return true if read successful, false otherwise
  */
 bool tsc2007_read_value(uint8_t command, uint16_t *value) {
     uint8_t data[2] = {0, 0};
     int bytes_read;
     
     // Send command byte
     if (i2c_write_blocking(I2C_INST, TSC2007_ADDR, &command, 1, true) != 1) {
         return false;
     }
     
     // Read the result (2 bytes)
     bytes_read = i2c_read_blocking(I2C_INST, TSC2007_ADDR, data, 2, false);
     if (bytes_read != 2) {
         return false;
     }
     
     // Combine the two bytes to form the 12-bit result
     // First byte contains the 8 most significant bits (D11-D4)
     // Second byte contains the 4 least significant bits (D3-D0) in the upper nibble
     *value = ((uint16_t)data[0] << 4) | (data[1] >> 4);
     
     return true;
 }
 
 /**
  * Convert raw touch screen values to millimeters
  * 
  * @param x_raw Raw X position from ADC
  * @param y_raw Raw Y position from ADC
  * @param x_mm Pointer to store X position in mm
  * @param y_mm Pointer to store Y position in mm
  */
 void convert_to_mm(uint16_t x_raw, uint16_t y_raw, float *x_mm, float *y_mm) {
     // Convert to millimeters
     *x_mm = (float)x_raw / MAX_ADC_VALUE * SCREEN_WIDTH_MM;
     *y_mm = (float)y_raw / MAX_ADC_VALUE * SCREEN_HEIGHT_MM;
 }
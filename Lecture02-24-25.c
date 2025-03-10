/**
 * Servo Motor Control using PWM on Raspberry Pi Pico
 * 
 * This program rotates a servo motor to 45°, waits 5 seconds,
 * then rotates to 150°, and repeats after 5 seconds.
 * 
 * Uses PWM with 20ms period (50Hz frequency)
 * - Pulse width 600μs = 0° position
 * - Pulse width 2400μs = 180° position
 */

 #include <stdio.h>
 #include <pico/stdlib.h>
 #include <hardware/pwm.h>
 #include <hardware/clocks.h>
 
 // PWM configurations
 #define SERVO_PIN 15        // GPIO pin to connect servo signal wire
 #define PWM_PERIOD 20000    // PWM period in microseconds (20ms)
 
 // Pulse width ranges for servo positions
 #define PULSE_WIDTH_MIN 600   // Pulse width for 0° (in microseconds)
 #define PULSE_WIDTH_MAX 2400  // Pulse width for 180° (in microseconds)
 
 // Function to convert angle to pulse width
 uint16_t angle_to_cc(float angle) {
     // Our calculation shows that for 20ms period with DIVi=50, TOP=53199:
     // 0° (600μs) = CC value of 1596
     // 180° (2400μs) = CC value of 6384
     if (angle < 0) angle = 0;
     if (angle > 180) angle = 180;
     
     return 1596 + (uint16_t)(angle * 26.6); // 26.6 = 4788/180 (steps per degree)
 }
 
 int main() {
     stdio_init_all();
     printf("Servo Control Program Started\n");
     
     // Initialize the GPIO pin
     gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
     
     // Work out which PWM slice is connected to GPIO
     uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
     uint chan = pwm_gpio_to_channel(SERVO_PIN);
     
     // Calculate PWM configuration (DIVi=50, TOP=53199)
     float clock_div = 50.0;
     uint16_t wrap = 53199;
     
     // Configure PWM
     pwm_set_clkdiv(slice_num, clock_div);
     pwm_set_wrap(slice_num, wrap);
     
     // Set the PWM running
     pwm_set_enabled(slice_num, true);
     
     // Main loop
     while (true) {
         // Set servo to 45 degrees
         printf("Moving to 45 degrees\n");
         pwm_set_chan_level(slice_num, chan, angle_to_cc(45.0));
         sleep_ms(5000);  // Wait 5 seconds
         
         // Set servo to 150 degrees
         printf("Moving to 150 degrees\n");
         pwm_set_chan_level(slice_num, chan, angle_to_cc(150.0));
         sleep_ms(5000);  // Wait 5 seconds
     }
     
     return 0;
 } 
 #include <stdio.h>
 #include <pico/stdlib.h>
 #include <hardware/i2c.h>
 #include <hardware/pwm.h>
 #include <hardware/clocks.h>
 
 // TSC2007 I2C address and commands
 #define TSC2007_ADDR 0x48
 #define TSC2007_CMD_READ_X 0xC0
 #define TSC2007_CMD_READ_Y 0xD0
 
 // Screen dimensions in mm
 #define SCREEN_WIDTH_MM 165.0f
 #define SCREEN_HEIGHT_MM 105.0f
 #define MAX_ADC_VALUE 4095.0f
 
 // I2C pins
 #define SDA_PIN 0
 #define SCL_PIN 1
 
 // Servo pins
 #define SERVO_X_PIN 15      // GPIO pin for X-axis servo
 #define SERVO_Y_PIN 14      // GPIO pin for Y-axis servo
 
 // Pulse width ranges for servo positions
 #define PULSE_WIDTH_MIN 600   // Pulse width for 0° (in microseconds)
 #define PULSE_WIDTH_MAX 2400  // Pulse width for 180° (in microseconds)
 
 // PID Controller constants
 #define PID_KP 0.1f       // Proportional gain
 #define PID_KD 0.1f         // Derivative gain
 #define PID_KI 0.0f        // Integral gain
 
 // Ball detection threshold
 #define BALL_DETECTION_THRESHOLD 100  // Adjust based on hardware
 
 // Task scheduling
 #define TASK1_PERIOD_MS 5    // Touchscreen read task: 5ms (200Hz)
 #define TASK2_PERIOD_MS 20   // Motor control task: 20ms (50Hz)
 
 // Global variables for ball position
 float ball_x_mm = 0.0f;
 float ball_y_mm = 0.0f;
 bool ball_detected = false;
 
 // Desired position (center of screen)
 float desired_x_mm = SCREEN_WIDTH_MM / 2;
 float desired_y_mm = SCREEN_HEIGHT_MM / 2;
 
 // PID controller variables
 float error_x_prev = 0.0f;
 float error_y_prev = 0.0f;
 float integral_x = 0.0f;
 float integral_y = 0.0f;
 
 // PWM slice numbers for each servo
 uint slice_num_x, slice_num_y;
 uint chan_x, chan_y;
 
 // Function to convert angle to PWM counter compare value
 uint16_t angle_to_cc(float angle) {
     if (angle < 0) angle = 0;
     if (angle > 180) angle = 180;
     
     return 1596 + (uint16_t)(angle * 26.6); // 26.6 = 4788/180 (steps per degree)
 }
 
 // Initialize touchscreen
 void init_touchscreen() {
     // Initialize I2C
     gpio_init(SDA_PIN);
     gpio_set_pulls(SDA_PIN, 1, 0);  // Pull-up enabled
     gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
     
     gpio_init(SCL_PIN);
     gpio_set_pulls(SCL_PIN, 1, 0);  // Pull-up enabled
     gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
     
     i2c_init(i2c0, 100000);  // 100 kHz
     
     printf("Touchscreen initialized\n");
 }
 
 // Initialize servo motors
 void init_servos() {
     // Initialize the GPIO pins
     gpio_set_function(SERVO_X_PIN, GPIO_FUNC_PWM);
     gpio_set_function(SERVO_Y_PIN, GPIO_FUNC_PWM);
     
     // Get PWM slice and channel for each servo
     slice_num_x = pwm_gpio_to_slice_num(SERVO_X_PIN);
     chan_x = pwm_gpio_to_channel(SERVO_X_PIN);
     
     slice_num_y = pwm_gpio_to_slice_num(SERVO_Y_PIN);
     chan_y = pwm_gpio_to_channel(SERVO_Y_PIN);
     
     // Configure PWM (DIVi=50, TOP=53199)
     float clock_div = 50.0;
     uint16_t wrap = 53199;
     
     // Configure PWM for X-axis servo
     pwm_set_clkdiv(slice_num_x, clock_div);
     pwm_set_wrap(slice_num_x, wrap);
     pwm_set_enabled(slice_num_x, true);
     
     // Configure PWM for Y-axis servo
     pwm_set_clkdiv(slice_num_y, clock_div);
     pwm_set_wrap(slice_num_y, wrap);
     pwm_set_enabled(slice_num_y, true);
     
     // Center both servos (90 degrees)
     pwm_set_chan_level(slice_num_x, chan_x, angle_to_cc(180.0));
     pwm_set_chan_level(slice_num_y, chan_y, angle_to_cc(180.0));
     
     printf("Servo motors initialized\n");
 }
 
 // Read touchscreen task (runs at 200Hz)
 void readTouchscreenTask() {
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
     
     // Detect if ball is present using threshold
     ball_detected = (x_raw > BALL_DETECTION_THRESHOLD && y_raw > BALL_DETECTION_THRESHOLD);
     
     if (ball_detected) {
         // Convert to mm
         ball_x_mm = (float)x_raw / MAX_ADC_VALUE * SCREEN_WIDTH_MM;
         ball_y_mm = (float)y_raw / MAX_ADC_VALUE * SCREEN_HEIGHT_MM;
         
         // Debug output (uncomment if needed)
          printf("Ball position: X=%.2f mm, Y=%.2f mm\r\n", ball_x_mm, ball_y_mm);
     }
 }
 
 // Control motors task (runs at 50Hz)
 void controlMotorsTask() {
     // Only run PID controller if ball is detected
     if (!ball_detected) {
         // Set platform to level position when no ball detected
         pwm_set_chan_level(slice_num_x, chan_x, angle_to_cc(90.0));
         pwm_set_chan_level(slice_num_y, chan_y, angle_to_cc(90.0));
         
         // Reset PID variables
         integral_x = 0.0f;
         integral_y = 0.0f;
         error_x_prev = 0.0f;
         error_y_prev = 0.0f;
         
         return;
     }
     
     // Calculate error (difference between desired and current position)
     float error_x = desired_x_mm - ball_x_mm;
     float error_y = desired_y_mm - ball_y_mm;
     
     // Calculate derivative of error (rate of change)
     float derivative_x = error_x - error_x_prev;
     float derivative_y = error_y - error_y_prev;
     
     // Update previous error for next iteration
     error_x_prev = error_x;
     error_y_prev = error_y;
     
     // Update integral term (accumulated error)
     integral_x += error_x;
     integral_y += error_y;
     
     // Apply anti-windup (limit integral term)
     if (integral_x > 100.0f) integral_x = 100.0f;
     if (integral_x < -100.0f) integral_x = -100.0f;
     if (integral_y > 100.0f) integral_y = 100.0f;
     if (integral_y < -100.0f) integral_y = -100.0f;
     
     // Calculate PID output (torque/angle adjustment)
     float tau_x = PID_KP * error_x + PID_KD * derivative_x + PID_KI * integral_x;
     float tau_y = PID_KP * error_y + PID_KD * derivative_y + PID_KI * integral_y;
     
     // Convert torque to servo angle (90° is level, +/- adjustment based on PID)
     float servo_x_angle = 90.0f + tau_x; // Invert if needed based on your setup
     float servo_y_angle = 90.0f - tau_y; // Invert if needed based on your setup
    // Apply servo angle limits
    if (servo_x_angle < 80.0f) servo_x_angle = 80.0f;
    if (servo_x_angle > 100.0f) servo_x_angle = 100.0f;
    if (servo_y_angle < 85.0f) servo_y_angle = 85.0f;
    if (servo_y_angle > 100.0f) servo_y_angle = 100.0f;

     // Set servo positions
     pwm_set_chan_level(slice_num_x, chan_x, angle_to_cc(servo_x_angle));
     pwm_set_chan_level(slice_num_y, chan_y, angle_to_cc(servo_y_angle));
     printf("Taux = %f Tauy = %f \n",tau_x,tau_y);
     printf("Angle x = %f   Angle y = %f",servo_x_angle, servo_y_angle);

 }
 
 // Simple task scheduler
 void task_scheduler() {
     static uint32_t last_task1_time = 0;
     static uint32_t last_task2_time = 0;
     uint32_t current_time = to_ms_since_boot(get_absolute_time());
     
     // Check if it's time to run task 1 (readTouchscreenTask)
     if (current_time - last_task1_time >= TASK1_PERIOD_MS) {
         readTouchscreenTask();
         last_task1_time = current_time;
     }
     
     // Check if it's time to run task 2 (controlMotorsTask)
     if (current_time - last_task2_time >= TASK2_PERIOD_MS) {
         controlMotorsTask();
         last_task2_time = current_time;
     }
 }
 
 int main() {
     stdio_init_all();
     sleep_ms(2000); // Give time for serial to initialize
     
     printf("Ball Balancing System Starting\n");
     
     // Initialize hardware
     init_touchscreen();
     init_servos();
     
     printf("System ready, entering main loop\n");
     
     // Main loop
     while (true) {
         // Run the task scheduler
         task_scheduler();
         
         // Small delay to prevent busy-waiting
         sleep_us(100);
     }
     
     return 0;
 }
//Day3
//LCD with Ultrasonic Sensor code with Linear Buzzer (CORRECTED)
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// ===== LCD CONFIGURATION =====
#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_NUM             0
#define I2C_MASTER_FREQ_HZ         40000
#define LCD_I2C_ADDRESS            0x27

// LCD Control Bits
#define LCD_ENABLE_BIT             0x04
#define LCD_BACKLIGHT_BIT          0x08
#define LCD_RS_BIT                 0x01

// ===== ULTRASONIC SENSOR (DYP-A06BLYT-V1.1) =====
#define TRIGGER_PIN                4
#define ECHO_PIN                   18
#define SOUND_SPEED_CM_PER_US      0.0343

// CRITICAL: Your sensor's valid range is 30-200cm
#define SENSOR_MIN_DISTANCE_CM     30.0    // Cannot measure below this
#define SENSOR_MAX_DISTANCE_CM     200.0   // Maximum reliable range
#define DISTANCE_THRESHOLD_CM      35.0    // "CLOSER" if distance < 35cm

// ===== BUZZER CONFIGURATION =====
#define BUZZER_PIN                 16       // GPIO pin for buzzer
#define BEEP_DURATION_MS           30       // Short beep duration (milliseconds)
#define MIN_BEEP_INTERVAL_MS       50       // Fastest beep (at 30cm - CLOSEST)
#define MAX_BEEP_INTERVAL_MS       800      // Slowest beep (at 200cm - FARTHEST)

static const char *TAG = "DYP_A06BLYT";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t lcd_device_handle = NULL;

// LCD Function Prototypes
static esp_err_t lcd_write_byte(uint8_t data);
static void lcd_pulse_enable(uint8_t data);
static void lcd_send_nibble(uint8_t nibble, uint8_t is_data);
static void lcd_send_byte(uint8_t byte, uint8_t is_data);
static void lcd_command(uint8_t cmd);
static void lcd_data(uint8_t data);
static void lcd_set_cursor(uint8_t col, uint8_t row);
static void lcd_print(const char *str);
static void lcd_clear(void);
static esp_err_t lcd_init(void);

// ===== BUZZER FUNCTIONS =====

static void buzzer_init(void)
{
    ESP_LOGI(TAG, "Initializing Buzzer on GPIO%d...", BUZZER_PIN);
    
    gpio_config_t buzzer_config = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&buzzer_config);
    
    // Ensure buzzer is off initially
    gpio_set_level(BUZZER_PIN, 0);
    
    ESP_LOGI(TAG, "Buzzer initialized successfully");
}

// Calculate beep interval based on distance (linear mapping)
// CORRECTED: Closer distance = SHORTER interval = FASTER beeps
// Farther distance = LONGER interval = SLOWER beeps
static int calculate_beep_interval(float distance)
{
    int interval;
    
    // Handle no object case (distance > max range)
    if (distance > SENSOR_MAX_DISTANCE_CM) {
        return MAX_BEEP_INTERVAL_MS;  // Slowest beeps when no object
    }
    
    // Clamp distance to sensor range
    if (distance < SENSOR_MIN_DISTANCE_CM) {
        distance = SENSOR_MIN_DISTANCE_CM;
    }
    
    // LINEAR MAPPING (CORRECTED):
    // At MIN distance (30cm) -> MIN_BEEP_INTERVAL_MS (50ms) - FASTEST beeps
    // At MAX distance (200cm) -> MAX_BEEP_INTERVAL_MS (800ms) - SLOWEST beeps
    interval = MIN_BEEP_INTERVAL_MS + 
               ((distance - SENSOR_MIN_DISTANCE_CM) * 
                (MAX_BEEP_INTERVAL_MS - MIN_BEEP_INTERVAL_MS) / 
                (SENSOR_MAX_DISTANCE_CM - SENSOR_MIN_DISTANCE_CM));
    
    // Ensure interval stays within bounds
    if (interval < MIN_BEEP_INTERVAL_MS) interval = MIN_BEEP_INTERVAL_MS;
    if (interval > MAX_BEEP_INTERVAL_MS) interval = MAX_BEEP_INTERVAL_MS;
    
    return interval;
}

static void buzzer_beep(int duration_ms)
{
    gpio_set_level(BUZZER_PIN, 1);  // Turn buzzer ON
    esp_rom_delay_us(duration_ms * 1000);  // Use microsecond delay for precision
    gpio_set_level(BUZZER_PIN, 0);  // Turn buzzer OFF
}

// ===== ULTRASONIC SENSOR FUNCTIONS =====

static void ultrasonic_init(void)
{
    ESP_LOGI(TAG, "Initializing DYP-A06BLYT-V1.1 Ultrasonic Sensor...");

    // Configure Trigger pin as output
    gpio_config_t trig_config = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_config);

    // Configure Echo pin as input
    gpio_config_t echo_config = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_config);

    // Set Trigger pin low initially
    gpio_set_level(TRIGGER_PIN, 0);

    ESP_LOGI(TAG, "Sensor initialized: TRIG=GPIO%d, ECHO=GPIO%d", TRIGGER_PIN, ECHO_PIN);
    ESP_LOGI(TAG, "Valid range: %.0fcm - %.0fcm", SENSOR_MIN_DISTANCE_CM, SENSOR_MAX_DISTANCE_CM);
    ESP_LOGI(TAG, "Threshold for 'CLOSER': < %.0fcm", DISTANCE_THRESHOLD_CM);
}

static float ultrasonic_measure_distance(void)
{
    // Send 10us pulse to trigger
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);

    // Wait for Echo pin to go high (timeout for max range ~200cm)
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - start_time) > 12000) { // ~2m max range
            ESP_LOGW(TAG, "No object detected within %.0fcm", SENSOR_MAX_DISTANCE_CM);
            return SENSOR_MAX_DISTANCE_CM + 50; // Return beyond max range
        }
    }

    // Record time when Echo goes high
    uint32_t pulse_start = esp_timer_get_time();

    // Wait for Echo pin to go low
    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - pulse_start) > 12000) {
            ESP_LOGW(TAG, "Echo pulse timeout");
            return SENSOR_MAX_DISTANCE_CM + 50;
        }
    }

    // Record time when Echo goes low
    uint32_t pulse_end = esp_timer_get_time();

    // Calculate pulse duration in microseconds
    uint32_t pulse_duration = pulse_end - pulse_start;

    // Calculate distance (duration * speed of sound / 2)
    float distance = (pulse_duration * SOUND_SPEED_CM_PER_US) / 2.0;

    // Apply sensor range constraints
    if (distance < SENSOR_MIN_DISTANCE_CM) {
        ESP_LOGW(TAG, "Object too close (%.0fcm) - below sensor's minimum range", distance);
        distance = SENSOR_MIN_DISTANCE_CM;
    }

    if (distance > SENSOR_MAX_DISTANCE_CM) {
        ESP_LOGW(TAG, "Object beyond maximum range (%.0fcm)", distance);
        distance = SENSOR_MAX_DISTANCE_CM;
    }

    return distance;
}

// ===== LCD FUNCTIONS =====

static esp_err_t lcd_write_byte(uint8_t data)
{
    if (lcd_device_handle == NULL) return ESP_FAIL;
    return i2c_master_transmit(lcd_device_handle, &data, 1, pdMS_TO_TICKS(1000));
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_write_byte(data | LCD_ENABLE_BIT);
    esp_rom_delay_us(50);   // VERY IMPORTANT (was too fast)

    lcd_write_byte(data & ~LCD_ENABLE_BIT);
    esp_rom_delay_us(50);   // VERY IMPORTANT
}

static void lcd_send_nibble(uint8_t nibble, uint8_t is_data)
{
    uint8_t data = LCD_BACKLIGHT_BIT;
    if (is_data) data |= LCD_RS_BIT;
    data |= (nibble & 0xF0);
    lcd_write_byte(data);
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t byte, uint8_t is_data)
{
    lcd_send_nibble(byte & 0xF0, is_data);
    lcd_send_nibble((byte << 4) & 0xF0, is_data);
    vTaskDelay(pdMS_TO_TICKS(2));   // ADD THIS
}

static void lcd_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_data(uint8_t data)
{
    lcd_send_byte(data, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    lcd_command(0x80 | (addr + col));
}

static void lcd_print(const char *str)
{
    while (*str) lcd_data(*str++);
}

static void lcd_clear(void)
{
    lcd_command(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static esp_err_t lcd_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD...");
    vTaskDelay(pdMS_TO_TICKS(200));

    // Reset to 4-bit mode
    for (int i = 0; i < 3; i++) {
        lcd_send_nibble(0x30, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    lcd_send_nibble(0x20, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_command(0x28);  // 2 lines, 5x8 dots
    lcd_command(0x0C);  // Display ON, Cursor OFF
    lcd_command(0x06);  // Entry mode
    lcd_clear();

    ESP_LOGI(TAG, "LCD initialized successfully");
    return ESP_OK;
}

// ===== I2C INITIALIZATION =====

static esp_err_t i2c_master_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus...");

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t lcd_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(i2c_bus_handle, &lcd_config, &lcd_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

// ===== MAIN APPLICATION =====

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "DYP-A06BLYT-V1.1 + LCD Display + Linear Buzzer");
    ESP_LOGI(TAG, "Sensor Range: 30-200cm");
    ESP_LOGI(TAG, "CORRECTED: Closer = FASTER beeps, Farther = SLOWER beeps");
    ESP_LOGI(TAG, "========================================");

    // Initialize I2C and LCD
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed! Check wiring.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (lcd_init() != ESP_OK) {
        ESP_LOGE(TAG, "LCD initialization failed! Check I2C address.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize Ultrasonic Sensor
    ultrasonic_init();
    
    // Initialize Buzzer
    buzzer_init();

    // Display startup message
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Ready!");
    lcd_set_cursor(0, 1);
    lcd_print("Linear Beep");
    vTaskDelay(pdMS_TO_TICKS(2000));

    float distance;
    char display_line1[9];
    char display_line2[9];
    int beep_interval;

    // Main loop
    while (1) {
        // Measure distance
        distance = ultrasonic_measure_distance();

        // Calculate linear beep interval based on distance
        beep_interval = calculate_beep_interval(distance);

        // Prepare display based on distance with sensor's limitations
        if (distance <= SENSOR_MIN_DISTANCE_CM) {
            // Object is at or below minimum range
            snprintf(display_line1, sizeof(display_line1), "CLOSER!");
            snprintf(display_line2, sizeof(display_line2), "<%.0fcm", SENSOR_MIN_DISTANCE_CM);
            ESP_LOGI(TAG, "Distance: <%.0fcm - VERY CLOSE! Beep interval: %dms (FAST)", SENSOR_MIN_DISTANCE_CM, beep_interval);
        }
        else if (distance < DISTANCE_THRESHOLD_CM) {
            // Object is within threshold range
            snprintf(display_line1, sizeof(display_line1), "CLOSER!");
            snprintf(display_line2, sizeof(display_line2), "%.0fcm", distance);
            ESP_LOGI(TAG, "Distance: %.1fcm - CLOSER! Beep interval: %dms", distance, beep_interval);
        }
        else if (distance <= SENSOR_MAX_DISTANCE_CM) {
            // Object is within valid range but far
            snprintf(display_line1, sizeof(display_line1), "FAR");
            snprintf(display_line2, sizeof(display_line2), "%.0fcm", distance);
            ESP_LOGI(TAG, "Distance: %.1fcm - FAR - Beep interval: %dms (SLOW)", distance, beep_interval);
        }
        else {
            // No object detected or beyond max range
            snprintf(display_line1, sizeof(display_line1), "FAR");
            snprintf(display_line2, sizeof(display_line2), "No obj");
            ESP_LOGI(TAG, "No object detected - Beep interval: %dms (SLOWEST)", beep_interval);
        }

        // Update LCD display (EXACTLY the same as your old code)
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print(display_line1);
        lcd_set_cursor(0, 1);
        lcd_print(display_line2);

        // Make the beep (30ms beep duration)
        buzzer_beep(BEEP_DURATION_MS);
        
        // Wait for the calculated interval
        vTaskDelay(pdMS_TO_TICKS(beep_interval));
    }
}
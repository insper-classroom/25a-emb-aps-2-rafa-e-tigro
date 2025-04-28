#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <math.h>

#include "Fusion.h"

//------------------------------------------------------------------
// Configuration
//------------------------------------------------------------------
#define SAMPLE_PERIOD   (0.05f)       // Adjust to real sample period

// I2C / MPU6050
#define MPU_ADDRESS     0x68
#define I2C_SDA_GPIO    4
#define I2C_SCL_GPIO    5

// Buttons (active‑low: pull‑ups enabled)
#define BUTTON0_PIN     19  // B0 – bit0
#define BUTTON1_PIN     18  // B1 – bit1
#define BUTTON2_PIN     17  // B2 – bit2
#define BUTTON3_PIN     16  // B3 – bit3

//------------------------------------------------------------------
// Data types
//------------------------------------------------------------------
typedef struct {
    float yaw;
    float roll;
    float pitch;
    float ax;
    float ay;
    float az;
} pos_t;

//------------------------------------------------------------------
// Globals
//------------------------------------------------------------------
static QueueHandle_t xQueuePos;

//------------------------------------------------------------------
// MPU6050 helpers
//------------------------------------------------------------------
static void mpu6050_reset(void) {
    const uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];

    // Acceleration 0x3B‑0x40
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Gyro 0x43‑0x48
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Temperature 0x41‑0x42
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

//------------------------------------------------------------------
// Tasks
//------------------------------------------------------------------
static void mpu6050_task(void *p) {
    // I2C setup
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    for (;;) {
        int16_t accel_raw[3], gyro_raw[3], temp_raw;
        mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);

        FusionVector g = {
            .axis.x = gyro_raw[0] / 131.0f,
            .axis.y = gyro_raw[1] / 131.0f,
            .axis.z = gyro_raw[2] / 131.0f,
        };
        FusionVector a = {
            .axis.x = accel_raw[0] / 16384.0f,
            .axis.y = accel_raw[1] / 16384.0f,
            .axis.z = accel_raw[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, g, a, SAMPLE_PERIOD);

        FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler      e = FusionQuaternionToEuler(q);
        FusionVector     l = FusionAhrsGetLinearAcceleration(&ahrs);

        pos_t pd = {
            .yaw   = e.angle.yaw,
            .roll  = e.angle.roll,
            .pitch = e.angle.pitch,
            .ax    = l.axis.x,
            .ay    = l.axis.y,
            .az    = l.axis.z,
        };
        xQueueSend(xQueuePos, &pd, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static inline uint8_t sample_buttons(void) {
    uint8_t b = 0;
    if (!gpio_get(BUTTON0_PIN)) b |= 0x01;   // pressed = 1
    if (!gpio_get(BUTTON1_PIN)) b |= 0x02;
    if (!gpio_get(BUTTON2_PIN)) b |= 0x04;
    if (!gpio_get(BUTTON3_PIN)) b |= 0x08;
    return b;
}

static void uart_task(void *p) {
    pos_t d;
    for (;;) {
        if (xQueueReceive(xQueuePos, &d, portMAX_DELAY) == pdPASS) {
            // Orientation & linear‑accel scaled ×100
            int16_t y  = (int16_t)(d.yaw   * 100);
            int16_t r  = (int16_t)(d.roll  * 100);
            int16_t p2 = (int16_t)(d.pitch * 100);
            int16_t xA = (int16_t)(d.ax    * 100);
            int16_t yA = (int16_t)(d.ay    * 100);
            int16_t zA = (int16_t)(d.az    * 100);

            uint8_t btn = sample_buttons();

            uint8_t buf[14] = {
                (y  >> 8) & 0xFF, y  & 0xFF,      // yaw
                (r  >> 8) & 0xFF, r  & 0xFF,      // roll
                (p2 >> 8) & 0xFF, p2 & 0xFF,      // pitch
                (xA >> 8) & 0xFF, xA & 0xFF,      // ax
                (yA >> 8) & 0xFF, yA & 0xFF,      // ay
                (zA >> 8) & 0xFF, zA & 0xFF,      // az
                btn,                               // 0‑3 bits = buttons 16‑19 pressed
                0xFF                               // terminator
            };
            uart_write_blocking(uart0, buf, sizeof(buf));
        }
    }
}

//------------------------------------------------------------------
// Main
//------------------------------------------------------------------
int main(void) {
    stdio_init_all();

    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // Button GPIOs
    const uint8_t btn_pins[] = {BUTTON0_PIN, BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN};
    for (size_t i = 0; i < 4; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    // Queues
    xQueuePos = xQueueCreate(10, sizeof(pos_t));
    if (xQueuePos == NULL) {
        printf("Queue creation failed\n");
        while (true);
    }

    // Tasks
    xTaskCreate(mpu6050_task, "MPU6050", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task,    "UART",    4096, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true);
}

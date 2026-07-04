/* driver_board.c  -  SPI backend for the Mini Pupper controller driver board.
 *
 * Ported from minipupper2pro/esp32 (SERVOS_BY_SPI path of mini_pupper_servos.cpp),
 * stripped down to exactly what the ESP-only gait code needs:
 *   - one-shot init (bus + 4 devices + power pin)
 *   - sync write of 12 positions + 12 current limits
 *   - cached feedback (present position + present current)
 */
#include "driver_board.h"

#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "DRVBOARD"

/* ---- pin map (same as reference board) ---- */
#define SPI_MASTER_ID    SPI2_HOST
#define SPI_MASTER_MOSI  11
#define SPI_MASTER_MISO  13
#define SPI_MASTER_CLK   12
#define SPI_MASTER_CS0   10   /* left  front */
#define SPI_MASTER_CS1   9    /* right front */
#define SPI_MASTER_CS2   14   /* left  rear  */
#define SPI_MASTER_CS3   21   /* right rear  */
#define POWER_EN_GPIO    8

/* ---- on-the-wire protocol (identical to AT32 spi_command_frame) ---- */
#define START_FIELD  0xA5A5
#define MODE_FIELD   0x0001
#define MODE_POSITION 0x0001   /* AT32: position control, "torque" field = max current mA */

#pragma pack(push,1)
typedef struct { uint16_t mode, position; int16_t torque; uint16_t kp, kd; } servo_cmd_sub_t;
typedef struct { uint16_t start, mode; servo_cmd_sub_t s1, s2, s3; uint16_t check_sum; } host_SMS_t;

typedef struct { uint16_t status, position; int16_t torque; uint32_t res; } servo_fb_sub_t;
typedef struct { uint16_t start, status; servo_fb_sub_t s1, s2, s3; uint16_t check_sum; } SMS_host_t;
#pragma pack(pop)

static spi_device_handle_t dev_left_front, dev_right_front, dev_left_rear, dev_right_rear;

/* cached feedback, index 0 == servo ID 1 */
static uint16_t fb_position[12];
static int16_t  fb_current[12];

/* board index (0..3) -> SPI device. Matches reference spi_read_write_bytes(). */
static esp_err_t spi_xfer(uint8_t board, uint8_t size, uint8_t *tx, uint8_t *rx)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length    = (size_t)size * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    switch (board) {
        case 0: return spi_device_transmit(dev_right_front, &t); /* servos 1-3  FR */
        case 1: return spi_device_transmit(dev_left_front,  &t); /* servos 4-6  FL */
        case 2: return spi_device_transmit(dev_right_rear,  &t); /* servos 7-9  RR */
        case 3: return spi_device_transmit(dev_left_rear,   &t); /* servos 10-12 RL */
        default: return ESP_FAIL;
    }
}

void driver_board_power(bool on)
{
    gpio_set_level(POWER_EN_GPIO, on ? 1 : 0);
}

void driver_board_init(void)
{
    /* power-enable pin */
    gpio_config_t io = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << POWER_EN_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    driver_board_power(false);

    /* SPI bus */
    spi_bus_config_t bus = {
        .mosi_io_num   = SPI_MASTER_MOSI,
        .miso_io_num   = SPI_MASTER_MISO,
        .sclk_io_num   = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_MASTER_ID, &bus, SPI_DMA_CH_AUTO));

    /* 4 driver-board devices (12 MHz, mode 0) */
    spi_device_interface_config_t dev = {
        .mode           = 0,
        .duty_cycle_pos = 128,
        .clock_speed_hz = 12 * 1000 * 1000,   /* 15 MHz (15 MHz max) */
        .queue_size     = 2,
    };
    dev.spics_io_num = SPI_MASTER_CS0; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_left_front));
    dev.spics_io_num = SPI_MASTER_CS1; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_right_front));
    dev.spics_io_num = SPI_MASTER_CS2; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_left_rear));
    dev.spics_io_num = SPI_MASTER_CS3; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_right_rear));

    driver_board_power(true);
    ESP_LOGI(TAG, "driver board SPI init OK (4 boards, 12 servos)");
}

void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12])
{
    host_SMS_t frame;
    SMS_host_t rx;

    for (int i = 0; i < 4; i++) {
        const int b = i * 3;   /* first servo index of this board */
        frame.start = START_FIELD;
        frame.mode  = MODE_FIELD;

        servo_cmd_sub_t *sub[3] = { &frame.s1, &frame.s2, &frame.s3 };
        for (int j = 0; j < 3; j++) {
            int idx = b + j;
            sub[j]->mode = MODE_POSITION;
            /* SCS 0..1023 -> AT32 deci-degrees 0..2700, with the same global
             * direction flip the reference uses. Position already carries the
             * gait's per-servo calibration offset (applied in servo_write). */
            sub[j]->position = (uint16_t)(2700 - (uint32_t)pos[idx] * 2700u / 1024u);
            sub[j]->torque   = (int16_t)cur_mA[idx];   /* MODE_POSITION => max current (mA) */
            sub[j]->kp = 0;
            sub[j]->kd = 0;
        }
        frame.check_sum = 0;

        if (spi_xfer((uint8_t)i, sizeof(host_SMS_t), (uint8_t *)&frame, (uint8_t *)&rx) == ESP_OK) {
            servo_fb_sub_t *fb[3] = { &rx.s1, &rx.s2, &rx.s3 };
            for (int j = 0; j < 3; j++) {
                fb_position[b + j] = (uint16_t)((uint32_t)fb[j]->position * 1024u / 2700u);
                fb_current[b + j]  = fb[j]->torque;   /* present motor current, mA */
            }
        }
    }
}

int16_t driver_board_present_current(int ch)
{
    if (ch < 1 || ch > 12) return 0;
    return fb_current[ch - 1];
}

uint16_t driver_board_present_position(int ch)
{
    if (ch < 1 || ch > 12) return 0;
    return fb_position[ch - 1];
}

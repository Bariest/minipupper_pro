/* driver_board.h
 *
 * SPI backend for the Mini Pupper "controller driver board" (4x AT32F413).
 *
 * This replaces the direct Feetech serial-bus servo control (SCServo / ftservo)
 * used previously. Instead of bit-banging the SCS bus, the ESP32 now talks to
 * four AT32F413 servo-driver boards over SPI (one board per leg, 3 servos each).
 *
 * Each SPI frame (host_SMS_t) carries, per servo: { mode, position, torque, kp, kd }.
 * IMPORTANT: there is NO "speed" field. In MODE_POSITION the AT32 firmware reads
 * the "torque" field as a MAX CURRENT LIMIT in milliamps (max_current_mA), NOT a
 * speed. So the legacy "goal_speed" value is translated into a current limit.
 *
 * Wiring (matches minipupper2pro/esp32 reference, official Mangdang board):
 *   SPI2_HOST  MOSI=11  MISO=13  CLK=12
 *   CS: FR=9  FL=10  RR=21  RL=14
 *   Power enable: GPIO 8 (high = servo bus powered)
 *
 * Servo ID -> leg mapping (unchanged from the gait code):
 *   1,2,3   = Front Right (board 0)
 *   4,5,6   = Front Left  (board 1)
 *   7,8,9   = Rear  Right (board 2)
 *   10,11,12= Rear  Left  (board 3)
 */
#ifndef DRIVER_BOARD_H
#define DRIVER_BOARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the SPI bus, the 4 driver-board devices, and the power-enable pin.
 * Leaves servo power ON (GPIO8 high). Call once at start-up. */
void driver_board_init(void);

/* Enable / disable servo bus power (GPIO 8). */
void driver_board_power(bool on);

/* Push all 12 setpoints to the four driver boards in one pass (4 SPI frames).
 *   pos[12]   : per-servo goal position, SCS scale 0..1023 (511 = centre),
 *               index 0 == servo ID 1 ... index 11 == servo ID 12.
 *   cur_mA[12]: per-servo current/torque limit in milliamps (the "torque" field).
 * Feedback (present position + present current) is captured on the same
 * transaction and cached for driver_board_present_*().
 */
void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12]);

/* Cached feedback from the last sync_write. ch = 1..12. */
int16_t  driver_board_present_current(int ch);   /* motor current, mA (signed) */
uint16_t driver_board_present_position(int ch);  /* SCS scale 0..1023          */

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BOARD_H */

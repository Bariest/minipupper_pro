/* stanford_gait.h
 *
 * C port of the Stanford Pupper trot gait controller, forward walking only.
 * Ported from mangdangroboticsclub/StanfordQuadruped (mini_pupper branch):
 *   src/Gaits.py            - phase scheduler (overlap / swing phases)
 *   src/StanceController.py - stance feet slide backward, z relaxes to ref
 *   src/SwingLegController.py - Raibert touchdown target, triangular lift
 *
 * Simplifications for this integration:
 *   - forward velocity only (vy = 0, yaw_rate = 0, roll/pitch = 0)
 *   - per-leg coordinates relative to each hip (matches the existing fRIK/
 *     fLIK/rRIK/rLIK IK in main.c), so default_stance x offset is 0
 *   - units: millimetres, output z is DOWNWARD-positive body height
 *     (same convention as the existing gait code)
 *
 * Leg index order (Stanford convention, matches the IK helpers):
 *   0 = Front Right, 1 = Front Left, 2 = Rear Right, 3 = Rear Left
 */
#ifndef STANFORD_GAIT_H
#define STANFORD_GAIT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;   /* forward offset from hip, mm (+ = forward)          */
    float z;   /* body height above foot, mm, downward-positive      */
} sg_foot_t;

/* Gait timing, ticks run at SG_DT seconds (call stanford_gait_step at
 * this rate). Values are the NATIVE Mini Pupper config from
 * mangdangroboticsclub/mini_pupper_bsp (MangDang/mini_pupper/Config.py):
 *   dt = 0.015 s, overlap_time = 0.09 s, swing_time = 0.1 s
 *   -> overlap_ticks = int(0.09/0.015) = 6
 *   -> swing_ticks   = int(0.10/0.015) = 6   (Python int() truncates)  */
#define SG_DT            0.015f
#define SG_OVERLAP_TICKS 6
#define SG_SWING_TICKS   6
#define SG_STANCE_TICKS  (2*SG_OVERLAP_TICKS + SG_SWING_TICKS)   /* 18 */
#define SG_PHASE_LENGTH  (2*SG_OVERLAP_TICKS + 2*SG_SWING_TICKS) /* 24 */

/* Native Mini Pupper walk parameters (same Config.py):
 *   default_z_ref  = -0.08 m  -> 80 mm body height
 *   z_clearance    =  0.03 m  -> 30 mm swing lift
 *   max_x_velocity =  0.20 m/s; SG_NATIVE_VX is a half-stick walk      */
#define SG_NATIVE_HEIGHT_MM     80.0f
#define SG_NATIVE_CLEARANCE_MM  30.0f
#define SG_NATIVE_VX_MM_S      100.0f

/* Duration one foot spends on the ground per cycle, seconds (0.27 s). */
float stanford_gait_stance_secs(void);

/* Reset gait state: all feet to neutral stance at height_mm, tick 0.
 * Call once when the mode is (re)activated. */
void stanford_gait_reset(float height_mm);

/* Advance the gait one tick (SG_DT seconds) and return the four foot
 * targets.
 *   vx_mm_s      : commanded forward body velocity, mm/s
 *   height_mm    : body height reference, mm (downward-positive)
 *   clearance_mm : peak swing foot lift, mm
 *   feet[4]      : output, order FR, FL, RR, RL                       */
void stanford_gait_step(float vx_mm_s, float height_mm,
                        float clearance_mm, sg_foot_t feet[4]);

#ifdef __cplusplus
}
#endif

#endif /* STANFORD_GAIT_H */

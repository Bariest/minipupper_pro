/* stanford_kinematics.h
 *
 * EXACT C port of the Mini Pupper 2 Pro inverse kinematics, faithful to the
 * reference the CM4 runs and streams to this same ESP32:
 *
 *   mangdangroboticsclub/StanfordQuadruped  (branch: mini_pupper)
 *       pupper/Kinematics.py
 *           - leg_explicit_inverse_kinematics()
 *           - four_legs_inverse_kinematics()
 *   mangdangroboticsclub/mini_pupper_2_bsp  (branch: mini_pupper_2pro_bsp)
 *       Python_Module/MangDang/mini_pupper/Config.py        (geometry)
 *       Python_Module/MangDang/mini_pupper/ServoCalibration.py
 *
 * This module is SELF-CONTAINED. It does NOT touch the existing fRIK / fLIK /
 * rRIK / rLIK helpers in main.c - those keep driving the stand / twerk / jump
 * / stretch modes exactly as before. Only the Stanford-walk path calls this.
 *
 * EXACT (byte-for-byte to the BSP): the whole leg IK - abduction offset, y-z
 *   triangle, two-link solve - with true geometry L1=50, L2=60,
 *   ABDUCTION_OFFSET=26 mm, LEG_ORIGINS x=+/-59, y=-/+23.5 mm.
 * HARDWARE-CALIBRATED (kept from your PROVEN firmware): the servo direction
 *   signs (SK_SIGN[][]) and the servo-centre reference, so the Stanford rest
 *   stance == your centred "Ini" pose. No servo re-flash, no dip at start.
 *
 * Foot input convention (matches sg_foot_t in stanford_gait.h):
 *     feet[leg].x : forward offset from hip, mm (+ forward)
 *     feet[leg].y : lateral offset from hip, mm (+ robot's LEFT)
 *     feet[leg].z : body height above foot, mm, DOWNWARD-positive
 *   Leg order: 0 = FR, 1 = FL, 2 = RR, 3 = RL.
 */
#ifndef STANFORD_KINEMATICS_H
#define STANFORD_KINEMATICS_H

#include "stanford_gait.h"   /* sg_foot_t, SG_ORIGIN_X, SG_ORIGIN_Y */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- true BSP geometry, millimetres (Config.py * 1000) ---------------- */
#define SK_L1                50.0f   /* LEG_L1 = 0.050 m                    */
#define SK_L2                60.0f   /* LEG_L2 = 0.060 m (exact, not 56)    */
#define SK_ABDUCTION_OFFSET  26.0f   /* ABDUCTION_OFFSET = 0.026 m          */
#define SK_LEG_FB            59.0f   /* LEG_FB  = 0.059 m                   */
#define SK_LEG_LR            23.5f   /* LEG_LR  = 0.0235 m                  */

/* Standing stance the servo-centre (Ini) pose maps to. This MUST equal
 * NEUTRAL_Z in main.c (your calibrated stand height) so the Stanford walk
 * rests at the SAME height as every other mode and does not dip when it
 * starts. The BSP's own reference is 80 mm (default_z_ref -0.08 m), but this
 * robot is physically calibrated so centred servos == a 70 mm stand, so we use
 * 70 here. The exact leg geometry above is unchanged. */
#define SK_NEUTRAL_HEIGHT_MM 70.0f

/* Compute all 12 servo angles (degrees, BEFORE your per-servo offset[]) for
 * the four foot targets. Output index = servo_id (1..12); servo_deg[0] unused,
 * so you call servo_write(i, servo_deg[i] + offset[i]) for i = 1..12.
 *
 * Servo id layout (same as your firmware / the BSP servo_ids):
 *     leg:   FR   FL   RR   RL
 *     abd:    1    4    7   10
 *     hip:    2    5    8   11
 *     knee:   3    6    9   12
 */
void stanford_kinematics_servo_deg(const sg_foot_t feet[4], float servo_deg[13]);

/* Lower-level: exact leg IK, returns the three JOINT angles in radians
 * [abduction, hip, knee] for one leg, given the foot position RELATIVE TO
 * THE LEG ORIGIN (body-frame minus LEG_ORIGINS), Stanford sign convention
 * (z negative below the body). leg = 0..3 (FR,FL,RR,RL). */
void stanford_leg_ik_rad(float x_mm, float y_mm, float z_mm,
                         int leg, float out_rad[3]);

#ifdef __cplusplus
}
#endif

#endif /* STANFORD_KINEMATICS_H */

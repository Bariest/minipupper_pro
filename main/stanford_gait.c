/* stanford_gait.c
 *
 * C port of the Stanford Pupper trot gait - FULL 3-axis version
 * (forward, strafe, yaw), matching the original controller:
 *   src/Gaits.py              - phase scheduler
 *   src/StanceController.py   - stance feet slide opposite to body motion,
 *                               rotate for yaw, z relaxes to reference
 *   src/SwingLegController.py - Raibert touchdown (alpha/beta), triangular
 *                               lift profile
 *
 * Coordinates are PER-HIP (mm): x forward, y left, z NEGATIVE below the
 * body internally (Stanford convention), flipped to downward-positive at
 * the output. Yaw is handled exactly like euler2mat(0,0,-yaw*dt) in the
 * Python code, applied to the foot position in the BODY frame (leg origin
 * + local offset). Default stance is (0,0) per hip - feet under the hip
 * pivots - which matches the Mini Pupper config where delta_x == LEG_FB
 * and delta_y == LEG_LR + ABDUCTION_OFFSET.
 */
#include "stanford_gait.h"
#include <math.h>

/* ---- Python config equivalents ---------------------------------- */
#define SG_ALPHA            0.5f    /* Raibert touchdown ratio        */
#define SG_BETA             0.5f    /* yaw touchdown ratio            */
#define SG_Z_TIME_CONSTANT  0.02f   /* stance z relaxation, seconds   */

/* Trot contact pattern, rows = legs (FR, FL, RR, RL), cols = phases   */
static const int contact_phases[4][4] = {
    {1, 1, 1, 0},   /* FR */
    {1, 0, 1, 1},   /* FL */
    {1, 0, 1, 1},   /* RR */
    {1, 1, 1, 0},   /* RL */
};

static const int phase_ticks[4] = {
    SG_OVERLAP_TICKS, SG_SWING_TICKS, SG_OVERLAP_TICKS, SG_SWING_TICKS
};

/* leg origins in the body frame (FR, FL, RR, RL), y+ = left */
static const float org_x[4] = { +SG_ORIGIN_X, +SG_ORIGIN_X, -SG_ORIGIN_X, -SG_ORIGIN_X };
static const float org_y[4] = { -SG_ORIGIN_Y, +SG_ORIGIN_Y, -SG_ORIGIN_Y, +SG_ORIGIN_Y };

/* ---- gait state --------------------------------------------------- */
static unsigned long sg_ticks = 0;
static float foot_x[4], foot_y[4];  /* mm, per-hip local                */
static float foot_z[4];             /* mm, Stanford sign: negative below */

float stanford_gait_stance_secs(void){
    return (float)SG_STANCE_TICKS * SG_DT;
}

void stanford_gait_reset(float height_mm){
    sg_ticks = 0;
    for(int i = 0; i < 4; i++){
        foot_x[i] = 0.0f;
        foot_y[i] = 0.0f;
        foot_z[i] = -height_mm;
    }
}

/* Gaits.py: phase_index() */
static int phase_index(unsigned long ticks){
    int phase_time = (int)(ticks % SG_PHASE_LENGTH);
    int phase_sum = 0;
    for(int i = 0; i < 4; i++){
        phase_sum += phase_ticks[i];
        if(phase_time < phase_sum) return i;
    }
    return 0; /* unreachable */
}

/* Gaits.py: subphase_ticks() */
static int subphase_ticks(unsigned long ticks){
    int phase_time = (int)(ticks % SG_PHASE_LENGTH);
    int phase_sum = 0;
    for(int i = 0; i < 4; i++){
        phase_sum += phase_ticks[i];
        if(phase_time < phase_sum)
            return phase_time - phase_sum + phase_ticks[i];
    }
    return 0; /* unreachable */
}

/* SwingLegController.py: swing_height(), triangular profile */
static float swing_height(float swing_prop, float clearance){
    if(swing_prop < 0.5f) return swing_prop / 0.5f * clearance;
    return clearance * (1.0f - (swing_prop - 0.5f) / 0.5f);
}

/* StanceController.py: next_foot_location()
 * delta_p = (-vx, -vy, (href-z)/tc) * dt;  delta_R = rotz(-wz*dt).
 * The rotation acts on the foot location in the BODY frame.             */
static void stance_next(int leg, float vx, float vy, float wz,
                        float height_ref_neg){
    float th = -wz * SG_DT;
    float c = cosf(th), s = sinf(th);
    float bx = org_x[leg] + foot_x[leg];      /* body-frame position */
    float by = org_y[leg] + foot_y[leg];
    float rx = c*bx - s*by;
    float ry = s*bx + c*by;
    foot_x[leg] = rx - org_x[leg] - vx * SG_DT;
    foot_y[leg] = ry - org_y[leg] - vy * SG_DT;
    foot_z[leg] += (height_ref_neg - foot_z[leg]) / SG_Z_TIME_CONSTANT * SG_DT;
}

/* SwingLegController.py: raibert_touchdown_location() + next_foot_location()
 * touchdown = rotz(beta*Ts*wz) @ default_stance + alpha*Ts*(vx,vy).
 * default_stance per hip is the leg origin in the body frame.           */
static void swing_next(int leg, float swing_prop, float vx, float vy,
                       float wz, float height_ref_neg, float clearance){
    float Ts = (float)SG_STANCE_TICKS * SG_DT;
    float th = SG_BETA * Ts * wz;
    float c = cosf(th), s = sinf(th);
    float tdx = (c*org_x[leg] - s*org_y[leg]) - org_x[leg] + SG_ALPHA * Ts * vx;
    float tdy = (s*org_x[leg] + c*org_y[leg]) - org_y[leg] + SG_ALPHA * Ts * vy;

    float time_left = SG_DT * (float)SG_SWING_TICKS * (1.0f - swing_prop);
    if(time_left < SG_DT) time_left = SG_DT;   /* guard, prop < 1 anyway */

    foot_x[leg] += (tdx - foot_x[leg]) / time_left * SG_DT;
    foot_y[leg] += (tdy - foot_y[leg]) / time_left * SG_DT;
    foot_z[leg]  = height_ref_neg + swing_height(swing_prop, clearance);
}

void stanford_gait_step(float vx_mm_s, float vy_mm_s, float wz_rad_s,
                        float height_mm, float clearance_mm,
                        sg_foot_t feet[4]){
    float height_ref_neg = -height_mm;   /* Stanford: z negative below body */

    int phase = phase_index(sg_ticks);
    int sub   = subphase_ticks(sg_ticks);

    for(int leg = 0; leg < 4; leg++){
        if(contact_phases[leg][phase]){
            stance_next(leg, vx_mm_s, vy_mm_s, wz_rad_s, height_ref_neg);
        }else{
            float prop = (float)sub / (float)SG_SWING_TICKS;
            swing_next(leg, prop, vx_mm_s, vy_mm_s, wz_rad_s,
                       height_ref_neg, clearance_mm);
        }
    }
    sg_ticks++;

    /* Output with safety clamps, flipped to downward-positive z. */
    for(int leg = 0; leg < 4; leg++){
        float x = foot_x[leg];
        float y = foot_y[leg];
        float z = -foot_z[leg];
        if(x >  60.0f) x =  60.0f;    /* keep IK acos/asin in domain  */
        if(x < -60.0f) x = -60.0f;    /* (leg reach L1+L2 = 106 mm)   */
        if(y >  45.0f) y =  45.0f;
        if(y < -45.0f) y = -45.0f;
        if(z <  25.0f) z =  25.0f;
        if(z > 100.0f) z = 100.0f;
        /* keep the combined target inside the leg's reach envelope
         * (L1+L2 = 106 mm, use 100 mm margin) - the joint-angle limit
         * clamp in run_robot.py serves the same purpose upstream */
        {
            float rr  = x*x + y*y;
            float lim = 100.0f*100.0f - z*z;
            if(lim < 0.0f) lim = 0.0f;
            if(rr > lim && rr > 0.0f){
                float k = sqrtf(lim / rr);
                x *= k;
                y *= k;
            }
        }
        feet[leg].x = x;
        feet[leg].y = y;
        feet[leg].z = z;
    }
}

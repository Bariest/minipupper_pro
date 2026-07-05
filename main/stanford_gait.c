/* stanford_gait.c
 *
 * C port of the Stanford Pupper trot gait (forward walk only).
 * See stanford_gait.h for source attribution and conventions.
 *
 * Internally this file keeps the Stanford sign convention: foot z is
 * NEGATIVE below the body (z = -height). It is flipped to the existing
 * downward-positive convention only at the output.
 */
#include "stanford_gait.h"

/* ---- Python config equivalents ---------------------------------- */
#define SG_ALPHA            0.5f    /* Raibert touchdown ratio        */
#define SG_Z_TIME_CONSTANT  0.02f   /* stance z relaxation, seconds   */

/* Trot contact pattern, rows = legs (FR, FL, RR, RL), cols = phases:
 *   phase 0: overlap, all four down
 *   phase 1: FL + RR swing
 *   phase 2: overlap, all four down
 *   phase 3: FR + RL swing                                            */
static const int contact_phases[4][4] = {
    {1, 1, 1, 0},   /* FR */
    {1, 0, 1, 1},   /* FL */
    {1, 0, 1, 1},   /* RR */
    {1, 1, 1, 0},   /* RL */
};

static const int phase_ticks[4] = {
    SG_OVERLAP_TICKS, SG_SWING_TICKS, SG_OVERLAP_TICKS, SG_SWING_TICKS
};

/* ---- gait state --------------------------------------------------- */
static unsigned long sg_ticks = 0;
static float foot_x[4];     /* mm, + forward                           */
static float foot_z[4];     /* mm, Stanford convention: negative below */

float stanford_gait_stance_secs(void){
    return (float)SG_STANCE_TICKS * SG_DT;
}

void stanford_gait_reset(float height_mm){
    sg_ticks = 0;
    for(int i = 0; i < 4; i++){
        foot_x[i] = 0.0f;
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
 * Stance foot slides backward at the commanded body velocity while z
 * relaxes toward the height reference with a first-order filter.       */
static void stance_next(int leg, float vx, float height_ref_neg){
    float vz = (height_ref_neg - foot_z[leg]) / SG_Z_TIME_CONSTANT;
    foot_x[leg] += -vx * SG_DT;
    foot_z[leg] +=  vz * SG_DT;
}

/* SwingLegController.py: next_foot_location()
 * Foot flies toward the Raibert touchdown point; z follows the
 * triangular lift profile added on top of the height reference.        */
static void swing_next(int leg, float swing_prop, float vx,
                       float height_ref_neg, float clearance){
    /* raibert_touchdown_location(): default stance x is 0 per-hip */
    float touchdown_x = SG_ALPHA * (float)SG_STANCE_TICKS * SG_DT * vx;

    float time_left = SG_DT * (float)SG_SWING_TICKS * (1.0f - swing_prop);
    if(time_left < SG_DT) time_left = SG_DT;   /* guard, prop < 1 anyway */

    float v = (touchdown_x - foot_x[leg]) / time_left;
    foot_x[leg] += v * SG_DT;
    foot_z[leg]  = height_ref_neg + swing_height(swing_prop, clearance);
}

void stanford_gait_step(float vx_mm_s, float height_mm,
                        float clearance_mm, sg_foot_t feet[4]){
    float height_ref_neg = -height_mm;   /* Stanford: z negative below body */

    int phase = phase_index(sg_ticks);
    int sub   = subphase_ticks(sg_ticks);

    for(int leg = 0; leg < 4; leg++){
        if(contact_phases[leg][phase]){
            stance_next(leg, vx_mm_s, height_ref_neg);
        }else{
            float prop = (float)sub / (float)SG_SWING_TICKS;
            swing_next(leg, prop, vx_mm_s, height_ref_neg, clearance_mm);
        }
    }
    sg_ticks++;

    /* Output with safety clamps, flipped to downward-positive z. */
    for(int leg = 0; leg < 4; leg++){
        float x = foot_x[leg];
        float z = -foot_z[leg];
        if(x >  60.0f) x =  60.0f;    /* keep IK acos/asin in domain  */
        if(x < -60.0f) x = -60.0f;    /* (leg reach L1+L2 = 106 mm)   */
        if(z <  25.0f) z =  25.0f;
        if(z > 100.0f) z = 100.0f;
        feet[leg].x = x;
        feet[leg].z = z;
    }
}

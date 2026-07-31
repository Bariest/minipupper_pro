// hardcode_backflip_angle.h  --  "Backflip 3"
//
// 7 hand-taught keyframes captured via teach / rec, then hardcoded here so they
// survive a reflash without needing NVS.
//
// DESIGN (calibration-safe):
//   BF3_REF[]      = reference pose (frame 0), stored as absolute SCS (0..1023)
//   BF3_DELTA[][]  = all subsequent frames as RELATIVE offsets from BF3_REF
//
//   At load time (bfload3 / bf3) the code computes:
//     rec_frames[f][id] = BF3_REF[id] + BF3_DELTA[f-1][id]
//
//   After recalibration you only need to update BF3_REF[] (re-teach the start
//   pose), and all frames automatically shift because the deltas stay the same
//   — they represent the RELATIVE joint motion, not absolute positions.
//
// Sequence: frame 0 is the start pose; frames 1..5 are the flip; frame 6
// returns to the same start pose, so it begins and ends in the same stance.
//
// Play it with:  bfload3   then  play        (or the one-shot:  bf3 )
// Tune motion with:  pspeed <ms>   (move time per pose, lower = faster)
//                    pdelay <ms>   (dwell/hold at each pose before the next)
#pragma once
#include <stdint.h>

#define BF3_FRAMES 10

// static const uint16_t BF3_SCS[BF3_FRAMES][13] = {
//     /* idx      1    2    3    4    5    6    7    8    9   10   11   12 */
//     {   0,     53,514,332,976,451,735,41,683,289,334,426,716}, /* 0 start */
//     {   0,     53,488,290,975,491,786,56,597,432,499,462,593}, /* 1 */
//     //{   0,     54,381,519,49,629,598,51,259,518,48,726,524}, /* 2 */
//     {0 ,        59,565,559,969,407,481,56,560,436,502,508,530},
//     {   0,     59,561,559,963,408,480,61,420,579,598,630,417}, /* 2 */
//     {   0,     52,481,285,964,444,791,74,387,746,785,617,241}, /* 3 */
//     {   0,     52,482,286,965,440,795,43,549,303,354,574,671}, /* 4 */
//     {   0,    69,740,131,965,262,920,46,553,304,371,571,671}, /* 5 */
//     //{   0,     51,427,520,60,593,568,62,615,714,970,527,566 }, /* 6 end (= start) */
// };

// ---- REFERENCE POSE (frame 0) ----
// Absolute SCS values for the starting stance. Update these after recalibration
// to match the new centre/neutral pose. Index [0] is unused.
static const uint16_t BF3_REF[13] = {
    /* idx  1     2    3    4    5    6    7    8    9   10   11   12 */
           1023,  642,  507, 1023,  349,  742, 1023,  695,  576,  491,  346,  800
};

// ---- DELTA FRAMES (frames 1..6, relative to BF3_REF) --------------------
// Each row is the signed offset from BF3_REF. int16_t so negative values work.
// After recalibration these do NOT change — they store the RELATIVE motion.
// Index [0] of each row is unused.
static const int16_t BF3_DELTA[BF3_FRAMES - 1][13] = {
    /* frame 0 — crouch (same as ref) */
    {0, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},

     /* frame 1 — crouch (same as ref) */
    {0,     0,  -178,   -15,     0,   194,    17,     0,   -52,   159,  -108,    48,  -160},
    
    /* frame 2 — front leg lifting */
    {0, 1023,  610,  680, 1023,  404,  527, 1023,  570,  711,  478,  391,  644},
    /* frame 3 — back leg rotating */
    {0, 0,  105,  119,    0, -141, -208,    0, -161,  243,    7,   69, -297},
    /* frame 4 — back leg pushing */
    {0, 0,  104,  117,    0, -142, -208,    0,  -58,  328,   41,   49, -398},
    /* frame 5 — retract front leg */
    {0, 0,  200, -170,    0, -145,  165,    0,  -66,  309,   37,   43, -398},
    /* frame 6 — back leg retracting and front leg landing */
    {0, 0,  197, -169,    0, -146,  164,    0,   66, -123,   -2,   20,   20},
};

// ---- PER-FRAME TIMING (edit these freely) --------------------------------
// BF3_MOVE_MS[f]  = time to MOVE into frame f from the previous pose.
// BF3_DELAY_MS[f] = time to HOLD/DWELL on frame f after arriving.
static const int BF3_MOVE_MS[BF3_FRAMES]  = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
static const int BF3_DELAY_MS[BF3_FRAMES] = { 500,  500,  500,  500,  500,  500};


// //move_ms[a]: from frame a-1 to frame a 
// static const int BF3_MOVE_MS[BF3_FRAMES]  = {  500,  250,   50,  50,  50,  100,  250};
// //delay_ms[a]: after frame executing frame a, stay ...ms before moving to the frame a+1 
// static const int BF3_DELAY_MS[BF3_FRAMES] = {  500,   1000,  5,    20,    30,   150};


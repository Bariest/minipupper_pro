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
// Sequence: frame 0 is BF3_REF (the start stance); frames 1..7 are the flip.
// Play then interp_to()s back to the Ini pose on its own, so no return frame
// is stored here.
//
// Play it with:  bfload3   then  play        (or the one-shot:  bf3 )
// Tune motion with:  pspeed <ms>   (move time per pose, lower = faster)
//                    pdelay <ms>   (dwell/hold at each pose before the next)
#pragma once
#include <stdint.h>

// Total playback frames = 1 reference pose + 7 delta keyframes.
//
// WAS 10, which was wrong: BF3_DELTA only has 7 rows and the timing tables only
// had 7 and 6 entries. C zero-fills the rest of a partially-initialized array,
// so load_bf3() (which loops f < BF3_FRAMES) fed playback three phantom frames:
//   f=7  -> the real landing keyframe, but with MOVE_MS 0 -> clamped to 1 ms
//   f=8,9-> REF + {0} = the stance, slammed into twice at 1 ms each
// Stepping with Verify </> never showed it, because Goto uses the global
// play_ms (1000 ms) and ignores frame_move_ms[] entirely. Only Play read the
// zero-padded tail. The arrays below are now unsized + _Static_assert'd so a
// short table is a compile error instead of a silent slam.
#define BF3_FRAMES 8

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
// The leading 0 is the unused index [0]. It was MISSING — only 12 values were
// listed for a [13] array, so every servo read its neighbour's value (servo 1
// got 1023->511, servo 2 got 511->684, ...) and servo 12 got the zero-fill.
// With BF3_DELTA[..][12] = -90 that made rec_frames[f][12] = -90, which the
// (uint16_t) cast in load_bf3() turns into 65446. The DELTA rows below always
// had their leading 0; only REF was short. `recdump_bf` in main.c printed it
// this way — that has been fixed too.
static const uint16_t BF3_REF[13] = {
    /* idx  1     2    3    4    5    6    7    8    9   10   11   12 */
         0,  53,  470,  482,  112,  557,  497,   25,  532,  402,  515,  310,  405
};
_Static_assert(sizeof(BF3_REF) / sizeof(BF3_REF[0]) == 13,
               "BF3_REF needs 13 entries: unused [0] + servos 1..12");

// ---- DELTA FRAMES (frames 1..6, relative to BF3_REF) --------------------
// Each row is the signed offset from BF3_REF. int16_t so negative values work.
// After recalibration these do NOT change — they store the RELATIVE motion.
// Index [0] of each row is unused.
// NOTE: left unsized on purpose — the compiler counts the rows, and the
// _Static_assert below fails the build if that count != BF3_FRAMES - 1.
// Row i here is played as frame i+1 (frame 0 is BF3_REF itself).
static const int16_t BF3_DELTA[][13] = {
    /* frame 1 — settle onto the start stance */
    {0,  0,    18,  -130,    -1,    41,   129,     1,    -5,   -13,    39,    -5,    24},

     /* frame 2 — crouch */
    {0,     0,   -58,   -70,     0,   -36,   142,     0,   132,    50,   -41,  -115,   -89},
    
    /* frame 3 — front leg lifting */
    {0, 0,   162,    75,     0,  -159,   -64,     0,   132,    33,   105,  -116,   -10},
    /* frame 4 — back leg rotating */
    {0, 0,   147,   103,     0,  -135,  -104,     0,  -104,   138,    89,    74,  -159},
    /* frame 5 — back leg pushing */
    {0, 0,   152,   106,     0,   -94,   -76,     0,   -11,   277,    88,     8,  -283},
    /* frame 6 — retract front leg */
    {0, 0,   169,  -248,     0,  -192,   254,     0,   -23,   179,   104,    19,  -167},
    /* frame 7 — back leg retracting and front leg landing */
    {0,  0,   164,  -267,     0,  -178,   284,     0,   173,  -119,   104,  -175,   127},
};

// ---- PER-FRAME TIMING (edit these freely) --------------------------------
// BF3_MOVE_MS[f]  = time to MOVE into frame f from the previous pose.
// BF3_DELAY_MS[f] = time to HOLD/DWELL on frame f after arriving.

//static const int BF3_MOVE_MS[BF3_FRAMES]  = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
//static const int BF3_DELAY_MS[BF3_FRAMES] = { 500,  500,  500,  500,  500,  500};


// move_ms[a]  : time to travel from frame a-1 into frame a.
//               move_ms[0] is the approach from the Ini pose into BF3_REF.
// delay_ms[a] : dwell on frame a after arriving, before starting frame a+1.
//
// Both tables MUST have exactly BF3_FRAMES entries. Left unsized so the
// _Static_assert below catches a short table at compile time.
//
//                                    f=  0     1    2   3   4    5    6    7
 static const int BF3_MOVE_MS[]  = {  1000,  1000,  45, 75, 45, 100, 150, 150 };
 static const int BF3_DELAY_MS[] = {   500,  500,   45, 110, 90, 150,   0, 300 };
//                                                                     ^^^  ^^^
// The last MOVE_MS (150) and last two DELAY_MS (0, 300) were never written —
// they used to fall off the end of the table and come back as 0, which
// interp_to() clamps to 1 ms. 150 matches the neighbouring airborne frames and
// 300 gives the landing time to settle before Play returns to the Ini stance.
// Tune all three on the robot.

_Static_assert(sizeof(BF3_DELTA)    / sizeof(BF3_DELTA[0])    == BF3_FRAMES - 1,
               "BF3_DELTA row count must be BF3_FRAMES - 1");
_Static_assert(sizeof(BF3_MOVE_MS)  / sizeof(BF3_MOVE_MS[0])  == BF3_FRAMES,
               "BF3_MOVE_MS must have exactly BF3_FRAMES entries");
_Static_assert(sizeof(BF3_DELAY_MS) / sizeof(BF3_DELAY_MS[0]) == BF3_FRAMES,
               "BF3_DELAY_MS must have exactly BF3_FRAMES entries");


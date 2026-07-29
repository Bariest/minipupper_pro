// hardcode_backflip_angle.h  --  "Backflip 3"
//
// 7 hand-taught keyframes captured via teach / rec, then hardcoded here so they
// survive a reflash without needing NVS. Stored as RAW servo command SCS
// (0..1023), servo id order 1..12: FR abd/hip/knee, FL, RR, RL. Index [0] of
// each row is unused (kept so the table lines up with rec_frames[f][1..12]).
//
// Unlike mp2_backflip_data.h / mp2_backflip2_data.h (which store URDF degrees
// and remap through BF_SIGN[]/BF_STAND[]/offset[]), these values are already in
// the final command convention that the trace buffer uses, so bfload3 copies
// them straight in -- no sign/stand/offset remap. That reproduces the EXACT
// physical pose that was taught, as long as the per-servo calibration is
// unchanged from when they were recorded.
//
// Sequence: frame 0 is the start pose; frames 1..5 are the flip; frame 6
// returns to the same start pose, so it begins and ends in the same stance.
//
// Play it with:  bfload3   then  play        (or the one-shot:  bf3 )
// Tune motion with:  pspeed <ms>   (move time per pose, lower = faster)
//                    pdelay <ms>   (dwell/hold at each pose before the next)
#pragma once
#include <stdint.h>

#define BF3_FRAMES 8

static const uint16_t BF3_SCS[BF3_FRAMES][13] = {
    /* idx      1    2    3    4    5    6    7    8    9   10   11   12 */
    {   0,     49,422,371,60,570,690,53,442,436,969,548,698 }, /* 0 start */
    {   0,     62,599,598,52,441,479,53,445,439,968,540,700 }, /* 1 */
    //{   0,     54,381,519,49,629,598,51,259,518,48,726,524}, /* 2 */
    {0 ,        51,405,372,60,546,672,63,607,629,980,386,584},
    {   0,     62,613,577,49,424,509,50,407,645,961,634,475}, /* 2 */
    {   0,     70,639,603,45,386,474,60,577,610,976,402,604}, /* 3 */
    {   0,     53,443,392,54,524,700,54,475,817,967,584,312}, /* 4 */
    {   0,    53,444,391,54,524,700,59,552,717,965,570,593 }, /* 5 */
    {   0,     51,427,520,60,593,568,62,615,714,970,527,566 }, /* 6 end (= start) */
};

// ---- PER-FRAME TIMING (edit these freely) --------------------------------
// One value per frame, so every transition can have its own speed and its own
// pause. Both are in milliseconds.
//   BF3_MOVE_MS[f]  = time to MOVE into frame f from the previous pose.
//                     Lower = faster snap. (Frame 0 = time to reach the start
//                     pose from the neutral stand.)
//   BF3_DELAY_MS[f] = time to HOLD/DWELL on frame f after arriving, before
//                     moving on to the next frame. 0 = no pause.
//
// Example below: ease into the start (frame 0), snap through the flip poses
// (frames 1-4) fast with no pause, brief hold at frame 5, then settle back to
// the start pose (frame 6). Tune each number to taste.
static const int BF3_MOVE_MS[BF3_FRAMES]  = {  1000,  1000,   30,  100,  50,  50,  200,  800 };
static const int BF3_DELAY_MS[BF3_FRAMES] = {  500,   400,  200,    200,    50,   300,  500,  300 };

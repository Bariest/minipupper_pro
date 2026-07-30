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

#define BF3_FRAMES 7

//good old with old calibration values
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

//Good - old config
//move_ms[a]: from frame a-1 to frame a 
// static const int BF3_MOVE_MS[BF3_FRAMES]  = {  500,  250,   50,  50,  50,  100,  250};
// //delay_ms[a]: after frame executing frame a, stay ...ms before moving to the frame a+1 
// static const int BF3_DELAY_MS[BF3_FRAMES] = {  500,   1000,  5,    20,    30,   150};

//move_ms[a]: from frame a-1 to frame a 
// static const int BF3_MOVE_MS[BF3_FRAMES]  = {  500,  250,   50,  50,  50,  100,  250};
// //delay_ms[a]: after frame executing frame a, stay ...ms before moving to the frame a+1 
// static const int BF3_DELAY_MS[BF3_FRAMES] = {  500,   1000,  10,    70,    30,   150};

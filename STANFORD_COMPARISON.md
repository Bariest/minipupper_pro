# Stanford Walk — ESP port vs. the reference robots

Line-by-line verification of the ESP-only Stanford trot (`stanford_gait.c` +
the IK in `main.c`) against the two upstream sources, and what changed.

Sources checked (live, July 2026):

- **StanfordQuadruped** — `mangdangroboticsclub/StanfordQuadruped`, branch
  `mini_pupper`: `src/Gaits.py`, `src/StanceController.py`,
  `src/SwingLegController.py`, `src/Controller.py`, `src/State.py`,
  `pupper/Kinematics.py`, `pupper/Config.py`.
- **BSP (the exact robot)** — `mangdangroboticsclub/mini_pupper_2_bsp`, branch
  `mini_pupper_2pro_bsp`: `Python_Module/MangDang/mini_pupper/Config.py`,
  `ServoCalibration.py`, `ESP32Interface.py`. This is the CM4→ESP32 setup you
  are replacing: the CM4 runs the controller and streams servo positions to the
  same ESP32 you now run standalone.

---

## 1. Verdict

For **forward walking**, your port is an algorithmically faithful reproduction
of the BSP Pro controller. Gait scheduler, stance controller, swing / Raibert
touchdown, z-relaxation, and the sign/`z` conventions all match. The thing that
made it "feel wrong" is **not** the gait math.

You also made the right call on timing: you used the **BSP Pro** numbers, not
the generic StanfordQuadruped repo numbers. They are *different robots*:

| Parameter          | Your ESP        | BSP `2pro` (exact robot) | StanfordQuadruped repo |
|--------------------|-----------------|--------------------------|------------------------|
| `dt`               | 0.015           | **0.015** ✓              | 0.01                   |
| `overlap_time`     | 0.09 → 6 ticks  | **0.09 → 6** ✓           | 0.10 → 10              |
| `swing_time`       | 0.10 → 6 ticks  | **0.10 → 6** ✓           | 0.15 → 15              |
| `stance_ticks`     | 18              | **18** ✓                 | 35                     |
| `phase_length`     | 24              | **24** ✓                 | 50                     |
| `alpha`, `beta`    | 0.5, 0.5        | **0.5, 0.5** ✓           | 0.5, 0.5               |
| `z_time_constant`  | 0.02            | **0.02** ✓               | 0.02                   |
| `contact_phases`   | identical       | **identical** ✓          | identical              |

`swing_ticks = int(0.10/0.015) = 6` — your `int()` truncation note is correct.
**Do not "fix" your timing to the StanfordQuadruped repo** — that repo still
carries the full-size Pupper's numbers.

---

## 2. Gait math — exact equivalences (forward walk)

**Phase scheduler** (`phase_index`, `subphase_ticks`) — identical integer logic
to `Gaits.py`.

**Stance** — your `stance_next()` with `wz=0`:
`x -= vx*dt`, `y -= vy*dt`, `z += (href-z)/z_tc*dt`. Same as
`StanceController.position_delta()`.

**Swing** — your `swing_next()` reproduces `raibert_touchdown_location()` +
`next_foot_location()`. Your per-hip-local frame (default `(0,0)`) and the
reference's body-frame `default_stance` differ only by a constant offset that
**cancels** in the differential update `(touchdown − foot)/time_left`, so the
stride is identical: it oscillates ±`0.5·Ts·vx` around the hip, exactly as the
reference oscillates ±`0.5·Ts·vx` around `default_stance`.

**Yaw handling** is also correct: you rebuild the body-frame position
(`org + local`) before applying `rotz(−wz·dt)`, matching how the reference
rotates the body-frame `foot_location`.

---

## 3. Differences from "byte-identical"

None of these break forward walking; they are why it is not *identical*.

1. **Body height 70 vs 80 mm.** You walk at `NEUTRAL_Z = 70`; BSP
   `default_z_ref = −0.08` = 80 mm. Deliberate (your neutral calibration).
2. **Lower leg `L2 = 56` vs `60 mm`.** BSP `LEG_L2 = 0.060`. Your legacy IK
   models the shin 4 mm short, so commanded vs actual foot travel is slightly
   scaled. Deliberate calibration.
3. **Abduction offset dropped: 26 mm → 0.** BSP IK models
   `ABDUCTION_OFFSET = 0.026` (foot sits 26 mm lateral of the abduction axis);
   your legacy `zd = z/cos(th0)` model assumes zero offset. **Irrelevant for
   forward walk** (`y = 0`, `th0 = 0`), but feet land differently on
   **strafe/turn**. ← the main functional gap.
4. **Abduction sign** front `−th0` / rear `+th0`: fine at `th0 = 0`, unverified
   for turning.
5. Minor: default stance `y` 49.5 vs 50 mm (0.5 mm); default speed 100 vs
   200 mm/s full stick (a control-range choice, not the gait).

So: **forward = same. Strafe/turn = not yet**, because of #3.

---

## 4. What was added — exact IK module

New, self-contained files (your existing `fRIK/fLIK/rRIK/rLIK` are untouched,
so stand / twerk / jump / stretch behave exactly as before):

- **`main/stanford_kinematics.h` / `.c`** — byte-for-byte port of
  `pupper/Kinematics.py`:
  `leg_explicit_inverse_kinematics()` + `four_legs_inverse_kinematics()`, with
  the true BSP geometry (`L1 = 50`, `L2 = 60`, `ABDUCTION_OFFSET = 26 mm`,
  `LEG_ORIGINS`, `ABDUCTION_OFFSETS`). It outputs the 12 servo angles for the
  four foot targets in one call.
- **`tools/verify_stanford_ik.py`** — the direction check used below.

### Servo mapping — the one calibrated piece

The kinematics are exact. The final joint-angle → servo-degree step is expressed
**relative to the resting stance**, so servo-centre = your calibrated "Ini"
pose (no horn re-flash, no dip at gait start), and it uses your firmware's
`servo_write` scale (0.263°/count, correct for your servos) rather than the
BSP's `MICROS_PER_RAD`.

Direction signs (`SK_SIGN[3][4]`): Stanford's IK uses `theta = atan2(−x, …)`,
so its hip/knee angle deltas are **sign-inverted** vs your firmware's polarity.
Applying the raw BSP `servo_multipliers` drives hip+knee **backwards**. The
verified working signs are the BSP multipliers negated on every row:

```
abduction : [-1, -1,  1,  1]
hip       : [ 1, -1,  1, -1]   <- verified vs legacy forward walk (all 4 legs)
knee      : [ 1, -1,  1, -1]   <- verified vs legacy forward walk (all 4 legs)
```

`tools/verify_stanford_ik.py` sweeps the foot forward and confirms every hip and
knee servo moves the **same direction** as your proven legacy walk (MATCH on all
4 legs), while abduction stays at neutral during forward walk. Abduction
direction is only exercised by strafe/turn — confirm it on the robot the first
time you turn, and flip the abduction row if a hip yaws the wrong way. That is
the only hardware knob.

---

## 5. How to enable it

In `main.c` (top of file):

```c
#define SG_USE_EXACT_IK 0   // <- change to 1 to use the exact BSP IK
```

- `0` (default): build behaves exactly as today (legacy planar IK, 70 mm walk).
- `1`: the Stanford-walk loop calls `stanford_kinematics_servo_deg()` and walks
  at 80 mm with the true geometry + abduction offset. Forward walk keeps the
  same servo directions; strafe/turn become geometrically correct.

Flash, stand the robot, tap the Stanford button. It should trot forward as
before. Then test strafe and a slow turn; if a hip abducts the wrong way, flip
that leg's entry in the `abduction` row of `SK_SIGN` in
`main/stanford_kinematics.c`.

Everything is opt-in and reversible — set the macro back to `0` to return to
your current firmware exactly.

---

## 6. Web controller

Your firmware's built-in control page already mirrors the mini_pupper /
Stanford command model: the joystick posts `/js?f=&s=&t=` (each −1..1) and the
firmware maps `vx = f·speed`, `vy = s·200 mm/s`, `wz = t·2 rad/s`
(`JOY_VX_MAX = 200`, `JOY_WZ_MAX = 2.0` = BSP `max_x_velocity 0.20`,
`max_yaw_rate 2`). No change needed for exactness.

`stanford_controller.html` (repo root) is a standalone version of that
controller you can open from any phone/desktop browser: enter the robot's IP,
two pads (move / turn) post to the same `/js` endpoint at 20 Hz. The only
"cap" difference is that your `sgspeed` control allows up to 400 mm/s (2× the
BSP full-stick 200) — headroom, not a gait change.

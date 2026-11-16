# Natural face test and jaw x lip-corner workspace (2026-11-10)

New head prototype, session neutrals from knee run `191452_knee`, half modiolus
ranges (up 350 / diag 290 / down 150 ticks). All runs are in
`tuning_runs/runs/2026-11-10/`. Tools: `scripts/head-natural`,
`scripts/head-jawcorner`, `scripts/head-eyes` (see `host_ros/README.md`).

Every run kept the same stops: line abort 280 mA, eye/lid abort 280 mA, jaw
guard (400 mA for 2 s: back off; 600 mA for 0.5 s: torque off), jaw current cap
650 mA, magnet-drop check, temperature stop 52 C, session envelope from the
panel marks. Ticks are servo encoder ticks (4096 per horn turn); the linkage
from horn to skin was not measured.

## 1. What the head can do now

| Channel | Measured (runs) | Limit found |
|---|---|---|
| Eye saccades (0/2 yaw together, 1 pitch) | reach 0.09-0.63 s, median 0.24-0.29 s at 2500 ticks/s; peaks 103-185 mA (201049-203145) | pitch 1 draws most (up to 185 mA) |
| Blink (lids 5/6) | final (205555): close 0.32-0.38 s, close + open 0.82-0.94 s; peaks lid 5 213, lid 6 247 mA. Close 5000 ticks/s at 80000 ticks/s^2, open 3500, speed per lid scaled by its travel, depth per lid (5: 0.55, 6: 1.0 of the closed mark) | hard start/stop at 5000: lid 5 -294 mA (abort, 200812); opening at 5000: lid 5 275 mA (too near 280). Left lid travel 1537 ticks at the 5000 ticks/s cap sets the close time |
| Fake smile (up + diag, no oculi) | holds 20-60 mA per line for 120 s | none hit |
| Talking jaw (syllables 0.18-0.32 s, phrases with pauses) | opens 50 ticks at goal 3880 (279 mA), 78 at 3800 (460 mA), 110-118 at 3700 (611-660 mA peaks) | static hold: about 50 ticks under 400 mA (goal 3880 -> 3952 at 250-282 mA, 200431) |
| Heat, 3 x 120 s talking at goal 3700 | jaw 38 -> 42, 40 -> 43, 42 -> 46 C (rise slowing: 43 C at 13 s, 45 at 84 s, 46 at 116 s); all other servos <= 43 C | 52 C stop not reached |

Pictures: the eyes and the smile read clearly on camera. At 110 ticks the
lips part a little and the chin drops (difference images of 201601); it reads
as talking, but with a small mouth. At 50 ticks only the chin outline moves.

Lid ladder (`head-natural --lid-ladder`, 204310): the right lid covers the
eye at 0.4-0.6 of its closed mark and uncovers it again past 0.7 (the lid
passes over the eye); the left lid covers it only at 1.0 (a thin strip at 0.9).
So a blink closes lid 5 to 0.55 only, which also shortens it. In the blink
stills the right eye is fully covered; the left lid often leaves a thin strip
(it is shut for only about 0.04 s).

## 2. Jaw x lip-corner coupling

Probe `head-jawcorner` (200431, jaw goal 3880, about 45 ticks of opening) and
the talking runs (201049-203145, up to 118 ticks):

- **Down lines in a smile (followers at 10 mA preload).** The jaw pushes line
  8 (right down) out: 6-29 ticks at 45 jaw ticks; up to 164 ticks from its neutral at 78 jaw
  ticks (40 of them from `--jaw-release down=40`, 201327). Line 9 (left down) does not move (0-4 ticks): its line path has high
  friction (known since 2026-10-18), so the left corner does not get the same
  release. The two sides are not symmetric.
- **Down lines driving (corners held down, `--smile -1`, jaw talking at 3700).**
  Without release the down-line loads rise when the jaw opens: line 8 mean
  27 -> 34 mA (max 43), line 9 23 -> 29 mA (max 39) (201904). With
  `--jaw-release down=60` they fall below the closed-jaw value: 8 23 -> 19 mA,
  9 20 -> 10 mA (202045). Per line, change of load jaw open vs closed:

  | release (ticks at the full 300-tick jaw goal span) | line 8 | line 9 |
  |---|---|---|
  | 0 (201904) | +7.5 mA | +5.7 mA |
  | 8=38, 9=22 (204936) | -3.5 mA | -4.8 mA |
  | 60 (202045) | -4.2 mA | -10.1 mA |

  Zero crossing: line 8 about 25, line 9 about 12 ticks (about 0.08 and 0.04
  down-line ticks per jaw tick). The remaining errors (+-5 mA) are below the
  line stiction (20-50 mA), so friction sets the precision. Default chosen:
  `--jaw-release 8=30 --jaw-release 9=15`. So the jaw does pull
  against the down lines, but the force is small at these openings (+6-8 mA
  mean) and a position release removes it.
- **Up and diag lines.** No load change over +-2 mA when the jaw opens 45 ticks
  in any pose (200431).
- **Follower drift (hysteresis).** A down line that the jaw pushed out does not
  come back: at 10 mA preload the follower sits inside the stiction band
  (20-50 mA, 2026-10-15), so line 8 ratcheted 284 ticks slack over 40 s of
  talking. Drivers are commanded by position, so this does not change a
  driven pose, but the slack state of a follower depends on history.
- **Jaw opening limit: the lines are not what stops it (2026-11-11).**
  Operator hypothesis: with the corner lines slack and the jaw driven hard
  (650 mA for about 1 s), the jaw should open much more than 110 ticks.
  Test: `head-jawprobe` pulse mode (`jaw_probe.py --pulse-ma`): the jaw goal
  leads the jaw by push / 3.5 mA per tick for 0.8 s, trimmed so the push stays
  at the target, then a fast close and 5 s rest. Line 8 (right down) was not
  moved: it was not connected to the corner that day (knee 121358).

  | run | lines | 450 mA | 520 mA | 580 mA |
  |---|---|---|---|---|
  | 122625 (A) | as they were (slack, knee test start) | 103 ticks (3914) | 102 (3910) | 107 (3901) |
  | 122720 (B) | 11, 14, 9, 15, 19 paid out 300 ticks | 107 (3895) | 110 (3888) | 119 (3877) |
  | 163937 (C) | down lines 8 and 9 slackened a lot by the operator (8 at 1997, 9 at 3467); no line took load (8: 0 mA, 9: <= 18 mA, others <= 6 mA) | 104 (3879) | 117 (3864) | 115 (3863) |
  Face-session test 2026-11-11 (knee 172157, `head-face --frame` jaw = 1 for
  0.9 s, three times, goal 3700, 650 mA cap, peak guard 1.0 s): release
  8=30 / 9=15 (run 172513): 131-146 ticks open (4022 -> 3876) at 613-650 mA;
  release at the new geometric cap 8=3333 / 9=3333 (run 172616): up to 164
  ticks (4020 -> 3856) at 545-600 mA, the lines paying out 800+ ticks during
  each hold (all lines <= 39 mA). The video shows a clearly wider lip gap with
  the large release. So a large down-line release gives about 20 ticks more
  and needs about 70 mA less jaw current; the absolute stop stays near
  3856-3877 in every test today.

  Release sweep, same day (knee 173247, same frames, `--jaw-release 8=R
  9=R`, head-face line speed 300 ticks/s; payout = line position minus its
  knee at the widest jaw; runs 173550-174002):

  | R (at j = 1) | payout 8 / 9 at the widest | opening | widest tick | jaw mA |
  |---|---|---|---|---|
  | 0 | 29-178 / 116-244 (follower drift) | 105-115 | 3888-3898 | 552-650 |
  | 300 | 216-300 / 272-297 | 117-123 | 3876-3882 | 514-628 |
  | 500 | 310-345 / 301-329 | 121-123 | 3875-3877 | 526-616 |
  | 700 | 351-365 / 334-339 | 124-128 | 3871-3875 | 483-531 |
  | 900 | 396-408 / 393-407 | 131-134 | 3867-3870 | 472-498 |
  | 1200 | 536-562 / 539-567 | 134-141 | 3859-3866 | 481-560 |
  | 1600 | 605-768 / 600-772 | 140-143 | 3856-3859 | 448-549 |
  | 2200 | 604-629 / 601-626 | 141-144 | 3856-3859 | 438-545 |

  The opening stops growing at a payout of about 600 ticks per down line
  (about 7.4 mm of line on the 8.1 mm horn), about 4.3 horn ticks per real jaw
  tick (39 % of the 11.1 geometric bound). That is about 30 ticks (+28 %) more
  opening than with no release, at about 100 mA less jaw current. The line
  currents read only 20-30 mA in every run; the jaw current is the sensor
  that shows the down lines holding the jaw.

  Mouth opening in millimetres (2026-11-11, `scripts/head-mouthdepth`,
  run 175813, knee 175339, release 8=1250 / 9=1250, jaw j = 1 for 0.9 s three
  times; RealSense depth aligned to color, face about 305 mm from the
  camera): lip gap at the mouth centre 5.9-6.2 mm open, no gap closed;
  lower lip moves 7.6-9.2 mm (6.8-8.1 mm down), chin 7.1-8.6 mm, upper lip
  1.5-2.2 mm, nose tip (should not move) 0.7-2.1 mm (noise and head sway).
  So the lower lip and chin drop about 7-8 mm. Human reference (operator, a
  clinical jaw-tracking study, n = 30, not read here): mean maximum vertical
  speech opening 11.5 mm, SD 3.2 mm (about 8-15 mm). The head's mouth
  motion is at the low end of that band; its jaw turns about 12 degrees, but
  the chin moves only about 40 % of what a rigid jaw point 90 mm from the
  axis would (about 19 mm), so most of the jaw motion goes into the skin.

  Lower lips pulled 50 ticks first (operator; servos 3/4, lip action 0.125
  of their 400-tick session span; run 180418, else as 175813): no change
  the camera can see. Closed, the lower lip sat 0.7-1.0 mm from where it was
  without the pull (nose reference 1.1-3.0 mm = noise); open, the lower lip
  dropped 7.7-7.9 mm and the lip gap was 5.3-6.3 mm (5.9-6.2 mm without).
  Lower-lip ladder (runs 180604-180721, same settings, lip 0.25 / 0.5 /
  0.75 / 1.0 = 100-400 ticks, all inside the marks, <= 68 mA): servos 3/4
  pull the lower lip DOWN. At rest it sits 1.0 / 2.3 / 3.7 / 5.3 mm lower,
  the lips part 1-3 mm at 0.75-1.0 even with the jaw closed, and with the jaw
  open the lip gap is 6.3-7.1 / 7.7-8.0 / 8.2-8.7 / 10.1-11.7 mm (about 6 mm
  without). Lower lip 1.0 plus the open jaw reaches the human speech mean
  (11.5 mm, operator's source). head-natural --lips now moves the lower lip
  with the jaw (and lets it rest on p/b/m); it used to pull it on p/b/m,
  which opened the mouth there.

  (opened ticks from the jaw's closed start; widest absolute tick in brackets;
  late mean current 443/482/555 mA in A, 419/479/558 mA in B; 90 % of the
  opening in 0.08-0.11 s every time.) The jaw stops against a stiff wall:
  in run 122451 the jaw held 3913 at 521 mA and 3906 at 600 mA (7 ticks for
  80 mA). Slack in five lines adds 4-12 ticks; 100 mA more adds about 5-12
  ticks. At the widest, the camera shows only a thin gap between the lips
  (stills of 122720). The old head showed the same wall (2026-10-20/23: jaw
  stopped near 3840 with all lines loose; 610 -> 700 mA gave about 15 ticks).
  Not measured: pushes above 592 mA peak (the jaw abort must stay under 95 %
  of its 650 mA cap, 617 mA), and line 8 paid out. The limit is in the jaw
  mechanism or the chin/lip skin, not in the corner lines; a wider mouth needs
  a mechanical change (for example a longer jaw horn lever, less skin
  stiffness at the chin, or a lower-lip line), not more current.

## 3. A safe workspace for the ML model

The model should not output servo ticks. It outputs actions, and a rule layer
(already in `workspace_guard.py` + `face_stream.py`) turns them into ticks.
These rules are hard, not learned:

1. **Action space.** Per side (d in 0..1, v in -1..1); jaw j in 0..1; upper
   face (brow, oculi, lip, upper lip) 0..1 per side; for natural motion also
   gaze (x, y in -1..1) and lid closure 0..1.
2. **Down never pulls against up/diag.** v >= 0: up and diag drive, down only
   follows; v < 0: down drives, up and diag follow (operator rule 2026-10-20).
3. **Down release with the jaw, per line.** Each down line's neutral moves
   toward slack in proportion to the jaw's *measured* position:
   `--jaw-release 8=30 --jaw-release 9=15` (ticks at the full 300-tick jaw
   goal span; about 0.1 and 0.05 line ticks per jaw tick). Re-measure per
   session with the `--smile -1` talking test (section 2).
4. **Jaw is current-limited, not position-limited.** The jaw is P-only and the
   skin is stiff: it stops 70-190 ticks short of its goal, by its load. So:
   - static openings (held > 2 s): at most about 50 ticks (< 400 mA);
   - dynamic openings (syllables <= 0.3 s): up to about 110-120 ticks at
     600-650 mA peaks;
   - heat: +3-4 C per 2 min of continuous talking near 45 C; plan pauses
     (`head-babble --jaw-duty`) for long data runs and log temperatures.
   The model should get the *measured* jaw position as its state, and a
   calibrated map goal -> reached opening, not the commanded j alone.
5. **Speeds and accelerations.** Lines <= 2500 ticks/s whenever the oculi
   move (12 slipped at ~4600, 2026-10-23); eyes 2500 ticks/s at 40000
   ticks/s^2; lids close 5000 and open 3500 ticks/s at 80000 ticks/s^2, each
   lid's speed scaled by its travel (hard starts and stops make current spikes).
6. **Currents.** Lines, eyes, lids <= 280 mA abort (300 mA rule); jaw <= 650
   mA cap. The magnet-drop check only uses samples where the servo has caught
   up with its goal (fixed 2026-11-10: fast moves gave false stops).
7. **Per-session calibration.** Neutrals drift between sessions and after a
   power cycle (line 8 moved about 1000 ticks within one hour today; turn
   counts can jump one turn). Each session: knee test -> (optional) jaw probe
   -> run. Never re-use neutrals across a controller restart (head-face
   refuses them).

**Commandable workspace in code.** `host_ros/src/head_ros/head_ros/face_workspace.py`
holds these rules for an ML policy or a babbling generator: `WorkspaceSpec`
(all the numbers above; written to `tuning_runs/config/face_workspace.json`),
`project(action, budget, t)` (clips every action dimension, refuses unknown
keys) and `JawBudget` (rule 4: the jaw above j = 0.4, the goal that held
under the guard, only in pulses <= 0.2 s and <= 40 % of any 10 s, the limits
the tested talking stayed inside). `head-natural` sends its jaw through the
same budget; `spec.release_args()` gives the session's `--jaw-release` flags.
Tests: `tests/face_workspace_test.py`.

**Checked over the whole action space on the head** (`head-natural --explore`,
run 210401, 120 s): uniformly random targets every 0.4-1.5 s for both sides
(d 0..1, v -1..1, so the down lines also drove while the jaw moved), the jaw
(through the budget), brows, oculi, lower and upper lips, gaze and lid
closure, plus blinks, all through `project()`. It finished with no guard
stop and nothing clamped. Every face servo moved across its range (oculi 742
and 731 ticks, lids 1129/1539, pitch 2428, upper lips 792/738); peaks: lines
<= 110 mA, lids 223/252 mA, jaw 643 mA; highest temperature 42 C. The jaw
budget cut 1814 updates of random long jaw holds, as designed.

**Validated as a commandable map** (`head-natural --validate 40`, run 211350;
report `face_workspace.py --report RUN` -> `workspace_validation.json`): 40
held poses, every action dimension stratified (each dimension's range cut in
40 bins, each bin used once), 0.5 s move + 1.4 s hold, measured over the last
0.6 s of each hold. All 40 poses finished, nothing clamped, no guard stop,
42 C at most. Commanded goal vs reached position per servo:

| Servo | Commanded span (ticks) | Error median / p90 / max (ticks) | Peak mA |
|---|---|---|---|
| eyes 0 / 2, pitch 1 | 398 / 398 / 2409 | 4-9 / 6-20 / 6-26 | 22-94 |
| lids 5 / 6 | 1442 / 1192 | 3 / 5 / 5-6 | 18-21 |
| lower lips 3 / 4, upper lips 13 / 18 | 391 / 391 / 791 / 721 | 4-6 / 5-8 / 6-10 | 22-38 |
| brows 10 / 16, oculi 12 / 17 | 507 / 507 / 723 / 741 | 3-8 / 5-16 / 5-20 | 20-71 |
| modiolus lines 8 9 11 14 15 19 | 181-353 | 2-4 / 7-14 / 8-15 | 28-54 |
| jaw 7 (held, budget caps it at j 0.4) | 118 | 52 / 56 / 62 | 215 |

So every servo except the jaw reaches its commanded tick to within a few
ticks; the jaw (P-only, stiff skin) reaches about 55-60 % of its commanded
opening when held (goal 120 ticks open -> 64-68 reached, 174-215 mA). The ML
map should therefore use the jaw's measured position, as rule 4 says.

How to find the workspace again on a new session or head:

```sh
scripts/head-knee --limit 650 --camera realsense --down-clearance 800 --down-max-travel 2000 --jaw-tick 4000
scripts/head-jawcorner run --neutrals <knee>/neutrals.json --range up=350 --range diag=290 --range down=150 --line-speed 1000 --jaw-speed 600 --jaw-open 3880 --camera realsense
scripts/head-jawcorner report <run folder>
```

## 4. Is this hardware fit for expression ML?

Fit for data collection with the rule layer above:
- control is repeatable: every run that started finished or stopped on a
  guard, no target was clamped by the firmware, no servo error;
- 20 servos stream at 50 Hz with per-update command logs and camera frame
  times (`commands.csv`, `video.frames.csv`) for paired data;
- temperatures stay well under the stop for minutes of combined motion.

Limits that a model cannot learn around (hardware changes):
- **Mouth amplitude is small.** 110-118 jaw ticks (about 10 degrees of horn)
  at the jaw's current cap; wide-open visemes ("ah") are out of reach. A softer
  skin at the jaw, a jaw linkage with more leverage, or a stronger servo is
  needed for natural speech. Checked 2026-11-11 (section 2, jaw pulse runs
  122625/122720): with five corner lines paid out 300 ticks and 0.8 s pulses
  up to 580 mA the jaw still stops at 107-119 ticks; the lines are not the
  limit, so no rule layer or model can buy more opening.
- **Blinks are at the hardware limit, about 3-7 times slower than a
  human's closing.** Best inside the limits: close 0.36 s, blink 0.82-0.94 s.
  Human blinks (review: Willett et al. 2023, Front. Syst. Neurosci.,
  doi 10.3389/fnsys.2023.1242654): closing 50-130 ms, opening about twice as
  slow, whole blink about 100-400 ms. The left lid needs its full 1537 ticks
  to cover the eye. The XC330-T181 no-load speed is 104 rev/min at 11.1 V
  (`hardware/datasheets/xc330-t181.pdf`) = about 7100 ticks/s, so even with no
  load and no ramps that lid needs at least 0.22 s to close; at the 5000
  ticks/s software cap (70 % of no-load, operator 2026-10-23) it is 0.31 s,
  and faster reopening read 275 mA (280 mA abort). Human-speed blinks need
  about 2-3 times less lid travel per close (linkage ratio) or a faster servo;
  no software setting gets there.
  Full-revolution blink, 2026-11-11 (operator: the lid linkage is a crank;
  `scripts/head-lidrev`, run 181754, lids 5/6 together, 80000 ticks/s^2,
  one turn in the closing direction and one back at each speed; operator
  allowed past the closed marks, full turns and up to 7000 ticks/s for this
  test). Both lids turned fully at 1000-6000 ticks/s. At 7000 lid 6 read
  349 mA while reopening and the 280 mA abort turned torque off. Left eye
  covered/uncovered from the video (15 fps, +-0.07 s; the right eye's red
  target is not measurable with the colour mask used):

  | ticks/s | back-and-forth: close / reopen | full turn: eye covered from - to |
  |---|---|---|
  | 3500 | 0.37-0.48 s / 0.43 s | 0.35 - 1.08 s |
  | 5000 | 0.30-0.36 s / 0.35 s | 0.32 - 0.85 s |
  | 6000 | 0.30-0.32 s / 0.33 s | 0.33 - 0.74 s |
  | 7000 | 0.29-0.30 s / (aborted) | - |

  The full turn closes the eye exactly as fast (same path) but keeps it
  covered for most of the turn and opens it only near the end, so a full-turn
  blink takes longer than closing and reopening (0.74-0.85 s against about
  0.65-0.71 s at 5000-6000). Above 5000 ticks/s closing gains almost nothing
  (0.30 -> 0.29 s); the lids lag their goal by 220-280 ticks there, so the
  servo, not the software limit, sets the speed. Human-speed blinks still need
  less lid travel per blink (a different lid linkage) or a faster servo.
  With the lid current cap raised to 600 mA for one session (operator: full
  speed while under 600 mA; `head-lidrev --current 600 --accel 200000
  --no-rev`, run 182353_face_lidrev), back-and-forth closing took lid 5 0.24 s
  and lid 6 0.32 s at 5000 ticks/s (0.26 / 0.36 s at 300 mA) and lid 5
  0.20 s at 7000; at 7000 lid 6 read -570 mA (braking at the closed end) and
  the 570 mA abort stopped the run. The board profile was set back to 300 mA
  for lids 5/6 afterwards (generation 135). More current buys about 10 %.
- **Left/right asymmetry and friction.** Line 9 barely moves with the jaw;
  followers ratchet; knees move between runs.
- **Line anchors are not robust yet.** On the new head, a corner line stopped
  moving its corner on 11-10 (line 14, knee 213852) and again on 11-11 (line 8:
  2000 ticks taken in at ~20 mA, no corner motion, knee 121358); on 11-09
  lines 8/9 were wound onto their horns the other way. Each time the knee neutrals became invalid and every face run was
  blocked until the operator fixed it. For long ML data runs the anchors
  (magnets, horn winding) need a fix, or a live check that stops the run when
  a driver's load and the camera both show no motion.

Verdict (2026-11-11). Good enough to start ML data collection for small,
mid-range expressions (smile, corner moves, gaze, brows, lids) through the
rule layer: the control, logging, guards and workspace projection work and
repeat. Not yet good enough for natural talking or human-like blinks: the
mouth opens about 110 ticks at most (a stiff limit, not the lines) and the
lids close 3-7 times slower than a human's, and both need hardware changes.
Robustness for unattended runs is limited by the line anchors (above) and the
rare BUS fault at Enable (below). The spoken-text demo (`head-natural --say`,
section 1) ran to the end on the head on 2026-11-11 (run 142831, demo video
18): 8 s of speech with audio, 5 blinks, 20 saccades, lines <= 91 mA, jaw
opening 114 ticks at up to 655 mA, 41 C. The mouth motion is small because of
the jaw limit. Three earlier takes stopped about 1.5 s into the speech on the
magnet-drop check, until the check got its 0.3 s settle window after a
release or jaw move (`workspace_guard.DROP_SETTLE_S`).
- **Firmware: BUS fault at Enable.** 4 of about 30 Enables today were refused
  with a BUS fault (195943, 203019, 203033, 204803). Source (`src/main.c`
  `service_preparation_locked`, `src/dxl.c` `head_dxl_prepare_step`): one
  preparation step failed 1 + 3 retries in a row, before torque on; the 200 ms
  "stale" feedback is the preparation pausing telemetry, not the cause. Host
  mitigation built: `--retry-enable-bus-fault` (default on in `head-natural`)
  clears the fault and enables once more, only for BUS, only during Enable,
  only with torque verified off; tested on the simulated board. A later
  test of 30 back-to-back sessions (Enable, 0.5 s hold, Disable) started 30
  of 30 on the first try, so the retry was not yet needed on the head; over
  the day 4 of about 65 Enables faulted (about 6 %), each before torque on and
  each cleared. The root
  cause (which step fails, and why four times in a row on a clean bus) needs
  firmware logging and a flash. The logging is written and built, not
  flashed: `builds/prep-diag/` records the last preparation step that failed
  for good in DIAGNOSTICS byte 173 (`(phase << 5) | servo`; `headctl
  diagnostics` prints it as `last_preparation_failure phase=... servo=...`).
  Tests: `tests/run_dxl_regression.py` ("DXL preparation failure note"), and
  the protocol, logic and main-loop suites pass. After a flash, the next BUS
  fault at Enable names its phase and servo. Each time the
  feedback of all 20 servos went 200 ms stale at once while the bus itself was
  100 % complete (`headctl bus-stats`). It fails safe (torque off, cleared
  with `headctl clear-fault`), but it is the open "Watchdog feed sits inside
  head_lock" item and needs a firmware fix for long unattended data runs.

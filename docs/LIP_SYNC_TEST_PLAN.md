# Lip-sync hardware test plan

Status: plan, 2026-11-06. No test in this file has been run. The software
in section 6 was built on 2026-11-08 (simulated board only).

## 1. Goal

Make the face move its lips in sync with an audio file. The path is hybrid:

1. Tune a small viseme (mouth-shape) library from camera images.
2. Play audio files with viseme timing. This is the first demo and the baseline.
3. Record motor babbling around the visemes.
4. Train the Hu et al. method (VAE + Facial Action Transformer) on that data.
   Compare it with the visemes and with an amplitude-driven jaw.

The mouth stays tendon-driven. A redesign is a decision for later, and only if
the tests below fail.

This plan does the hardware tests first. They tell if the face can do speech.

## 2. Reference: Hu et al. 2026

"Learning realistic lip motions for humanoid face robots", Sci. Robot. 11,
eadx3017 (2026), DOI 10.1126/scirobotics.adx3017.

Facts from the paper:

- 10 lip DOF on push-pull linkages with magnetic skin connectors (not cables).
- 20,000 babbling frames, RGB camera, "speech-relevant random commands".
- The target is a Wav2Lip video of the robot's own face, from TTS audio
  (5,173 frames). There is no human-to-robot retargeting.
- VAE to a 16-D latent. The transformer takes the 2 last commands and the
  latents L(t), L(t+1), and gives commands A(t), A(t+1).
- A one-sided closure loss on the upper lip, lower lip and jaw (p/b/m).
- Human study, 1,300 raters: method 62.5 %, amplitude jaw 23.2 %,
  MediaPipe-landmark nearest neighbour 14.4 %.
- Stated limit: people shape the lips 80-300 ms before the sound.
- The paper did not test a hand-tuned viseme baseline.

Facts from the released code (Zenodo 17804235, CC-BY-4.0, read 2026-11-06, not
run):

- Included: the VAE code and one checkpoint, the transformer model class, the
  nearest-neighbour latent match, the metric, and two baselines.
- Not included: transformer training (`om/Latent2Cmds/train_model.py` holds
  only "# Will upload when paper get accept."), babbling, data collection,
  motor interface, camera, and Wav2Lip/TTS steps.
- The command vector is 5 values in [0, 1]. Frames pair with commands by row
  index, with no timestamps. Images are 128 x 128 from a fixed pixel crop. The
  amplitude baseline assumes 30 fps.

Thus HEAD must write the data collection, the Wav2Lip step and the
transformer training. The VAE and the model class can be adapted.

## 3. Binding limits

These rules apply to every test in this file:

- All lines 300 mA or less. Jaw 7 may peak to 650 mA. Jaw guard: 400 mA for
  2 s steps it back, 600 mA for 0.5 s turns torque off.
- Never drive past an operator end mark. Lower lips 3/4: range 735 ticks
  (`workspace_guard.UPPER_RANGES`). Modiolus: up 700, diag 580, down 300.
- Jaw: closed 4000, open 3880. It opens toward lower ticks.
- Skip eyelids 5/6, but keep them in the profile.
- Find the neutrals again each session (`head-knee --camera realsense`).
  Neutrals moved by about +-200 ticks between sessions.
- No homing or return move over 4096 ticks. Home never moves jaw 7.
- Before each hardware run: the operator is at the 12 V switch, the panel is
  closed, and the operator gives the go-ahead.
- To stop: Ctrl-C, or create `firmware/tuning_runs/STOP`.

## 4. Gaps found in the present tools

| # | Gap | Source | Status (2026-11-08, simulated board only) |
|---|---|---|---|
| G1 | `head-face` does not drive the upper lips 13/18. | `workspace_guard.py`: `SIDES`, `UPPER_SIDES` | Built: driven from the panel marks (`ulip=R,L`). |
| G2 | The upper lips 13/18 have no end marks. | `tuning_runs/panel/2026-10-23_*_tuning.json`: `"marks": {}` | Open: test T1 (operator). |
| G3 | The camera runs at 1280 x 720, 15 fps (67 ms per frame). That is too coarse for lip timing. | `face_camera.py:25` | Built: `face-preview --list-modes`, `head-face --camera-mode`. Mode choice open (T0). |
| G4 | `head-face` speeds are 300 ticks/s (lines) and 60 ticks/s (jaw), "chosen, not measured". At 60 ticks/s the jaw needs 2 s for its 120-tick range. | `face_stream.py:67-68`, `host_ros/README.md` | Open: test T3. `--line-speed`/`--jaw-speed` already exist. |
| G5 | `head-face` takes keyframes. Speech needs a dense trajectory (one target per 20 ms frame), timed to the audio. | `face_stream.py` | Built: `--trajectory`, `--audio`, `--audio-advance-ms`, `commands.csv`. |
| G6 | Markers exist only for the modiolus (R, L) and the lower lip. There is no upper-lip marker, and no measure of the lip gap. | `tuning_runs/config/face_markers.json` | Code built (`mouth_shape.py`). Dots and marker entries open (T0). |

The firmware is not the speed limit. With the profiles on disk, each mouth
servo may move 20 ticks per control cycle (about 10,000 ticks/s at the
configured 500 Hz; `control.c:271-276`). The profile on the board was not
read. Check it with `headctl` before T3.

## 5. Tests

Run them in this order. T1 and T2 block the rest.

### T0. Camera setup

1. Find the color modes that the D435i supports:
   `firmware/scripts/face-preview --list-modes`. Choose the highest frame
   rate that still shows the mouth clearly. Record the mode. Use it with
   `--camera-mode` in every run.
2. Aim the camera at the mouth with `face-preview`.
3. Add ink dots: upper lip centre, upper lip left and right, lower lip left and
   right. Add them to `face_markers.json`.
4. Record 10 s of a still face. Record the dot jitter (px, SD) at the new mode.

Pass: a mode at 30 fps or more that shows the inner lip edge.
(30 fps: this is the rate the Columbia baseline assumed. It is a chosen
target, not a measured need.)

### T1. Upper-lip ends (operator)

1. In the tuning panel, jog 13 and 18 from neutral. Find the pull direction.
2. Mark the end for each. Write a short note in `servo_notes.json`.

Pass: both upper lips have a direction, a neutral and an end mark.

### T2. Lip seal for p/b/m

1. Jaw closed (4000). Find the session neutrals.
2. Pull the upper lips 13/18 down and the lower lips 3/4 up, in small steps,
   within the marks and 300 mA.
3. At each step, save a still of the mouth.

Pass: one pose where the lips touch along the full width, with every line
under 300 mA. Record the pose relative to neutral, and the currents.
If no pose seals: stop. This is the main reason for a mechanism change.

### T3. Speed and step response, one line at a time

For each mouth line (3, 4, 13, 18, 8, 9, 11, 14, 16, 19) and the jaw:

1. Step from neutral to 50 % of its range and back, 5 times.
2. Repeat at higher host speed limits: 300, 600, 1200, 2400 ticks/s
   (lines). Jaw: 60, 120, 240, 480 ticks/s. These values are chosen test
   steps, not limits.
3. Do not go above 2500 ticks/s on any line. That is the last speed with no
   slip on oculi 12 (2026-10-23). It slipped at about 4600.
4. Log telemetry (goal and present tick, current) and the camera.

Stop a line at the first sign of slip: load falls under half its peak (the
present live check), or the dot does not return to its start.

Measure, per line and speed:
- the time from goal change to 90 % of the present-tick move;
- the time from goal change to 90 % of the dot move;
- the dot error after return (hysteresis).

Pass: the full-range move takes 100 ms or less with no slip. (100 ms: the low
end of the 80-300 ms pre-shaping time in the paper. Chosen target.)

Note: try this first with `head-face --preview` and `--frame` keyframes with
short `hold=` and `--line-speed` / `--jaw-speed`. Check that the keyframes
make the step shape you expect. If not, add a trajectory input (G5) first.

### T4. Jaw duty cycle at speech rate

1. Cycle the jaw between 4000 and a small opening (start at 3940, then 3910,
   then 3880). Use a pattern of short opens and closes, for 60 s, then 3 min,
   then 5 min. Use a cool-down between runs.
2. Log jaw current and temperature.

Stop: the jaw guard, any Overload or Overheat flag, or the operator.

Pass: 5 min at a speech-like pattern with no guard trip and no Overload.
Record the temperature curve. (Known: continuous hold at 450-517 mA tripped
Overload in 6-12 s on 2026-10-13. At 3880 the hold took 81-264 mA.)

### T5. Latency

1. Play a short click from the speaker and send a jaw step at the same host
   time. Do this 20 times.
2. Record with a separate camera that hears the speaker and sees the face
   (for example a phone at a known frame rate).
3. Measure the delay from click to the first visible jaw motion.

Also measure the delay from goal change to the first present-tick change in
telemetry (100 Hz), from the T3 logs.

Pass: a constant delay with a small spread. Its value is the audio offset for
playback.

### T6. Repeatability and drift

1. Pick 3 poses (seal from T2, open jaw, wide corners).
2. Approach each pose 5 times from above and 5 times from below.
3. Do it again in a second session, after a new neutral search.
4. Save a still at each arrival.

Measure the dot position spread inside a session, between directions, and
between sessions.

Pass: the spreads are a few times the still-face jitter from T0 or less.
If between-session spread is large, the learned model needs a per-session
calibration step.

### T7. Babbling rate

From T3 and T4, choose safe speeds and a jaw duty. Calculate the time to
collect 20,000 frames at the T0 frame rate, with cool-down pauses.
(At 30 fps: 20,000 / 30 = about 11 min of motion, before pauses.)

## 6. Software

Built on 2026-11-08 and tested on the simulated board and synthetic data
only (`firmware/tests/README.md`, `lipsync/README.md`):

- Upper lips from the panel marks, trajectory input with audio, command log,
  camera mode (G1, G3, G5).
- Mouth measures from the dots and from photos (`mouth_shape.py`, G6).
- Viseme tool (`head-viseme`): targets from photos, calibration, try,
  search, library.
- Babbling generator (`head-babble`).
- Dataset builder, baselines (amplitude jaw, viseme playback) and the
  VAE / transformer training, synthesis and evaluation (`lipsync/`). The
  torch scripts have not run: torch is not installed.

Not checked yet, and checked first on the head:

- The gap measure threshold (0.4 x median) and the dot exclusion radius.
  Both are chosen values, from one pair of stills.
- The settle rule of the viseme tool (3 ticks over 10 updates, then 0.5 s).
- Camera frame times are host times at poll, not camera times. The T5 delay
  includes that.

## 7. Open decisions

- The viseme target images: what does a good "M" or "O" look like? For example,
  photos of a person's mouth, a drawn chart, or (later) Wav2Lip frames of
  the robot.
- The number of visemes. The paper used 12 categories to design its
  mechanism.
- The TTS and the Wav2Lip version, and their licences.

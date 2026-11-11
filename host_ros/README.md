# ROS 2 desktop tools

The ROS 2 Jazzy workspace contains three cooperating desktop processes:

- `head_bridge` owns the Teensy serial port and exposes `/head/*` topics,
  services, and actions. Its `port` parameter defaults to `auto`: it opens
  CDC1 through `/dev/serial/by-id/usb-ZEPHYR_Head*-if02`, exclusively, and
  finds it again after a controller reset;
- `head_proprioception_collector` owns the experiment session, control lease,
  rosbag process, streamed JSONL, and safe finalization;
- `head_proprioception_gui` is a thin Qt operator interface to the collector.

After building and sourcing the workspace, run each process in its own terminal
from the project root:

```bash
ros2 run head_ros head_bridge
ros2 run head_ros head_proprioception_collector --ros-args \
  --params-file proprioception/configs/collection.yaml
ros2 run head_ros head_proprioception_gui
```

Or launch all three together:

```bash
ros2 launch head_ros head_collection.launch.py \
  data_root:=$PWD/proprioception/data/raw
```

To inspect and exercise the real GUI without ROS, the collector, or hardware,
install PyQt5 and run its built-in preview backend from the project root:

```bash
python3 -m venv /tmp/project-head-gui-preview
/tmp/project-head-gui-preview/bin/python -m pip install PyQt5
/tmp/project-head-gui-preview/bin/python \
  firmware/host_ros/src/head_ros/head_ros/collector_gui.py --preview
```

Preview mode supplies representative controller, session, and servo data in
process. Its recording, contact, and abort buttons only update that temporary
demo state and never issue ROS requests.

The GUI exposes **Enter routing hold (0°)** only while collection is idle and
the firmware is torque-off in `HOMING_REQUIRED` or `READY`. After confirmation,
it acquires the control lease and requests firmware `ROUTING`, which ramps all
active servos to raw tick 0 and holds them there for tendon installation. The
button changes to **Disable routing hold** while active. The GUI renews the
lease every 400 ms; closing the GUI requests Disable, and a lost GUI/heartbeat
falls back to the firmware lease watchdog. Keep physical actuator-power
removal accessible throughout routing.

The GUI's Space shortcut starts recording only when the collector reports
`ARMED`; the next Space press ends recording, requests the firmware's common
`DISABLE` path, captures the shutdown tail, and finalizes the session under
`proprioception/data/raw/`. The red Abort/Disable control follows the same safe
shutdown path and preserves an aborted session for diagnosis.

For automatic fixture labels, publish `head_msgs/ProprioceptionEvent` messages
on `/proprioception/events` with paired `CONTACT_START`/`CONTACT_END` records,
the same `event_id`, and the latest observed `mcu_uptime_ms`. The GUI also has
manual contact start/end buttons for early bench trials.

Detailed live plots should use PlotJuggler or `rqt_plot` against `/head/state`;
the operator GUI intentionally limits itself to status, metadata, commands, and
a compact servo table. The existing `headctl` remains the serial-level fallback
when the ROS bridge is not running.

## Joint jog panel

`head_jog_gui` is a manual panel for moving individual joints through the
bridge, and for saving poses and going back to them in one click. Run it with
the bridge:

```bash
ros2 run head_ros head_jog_gui
```

There are two ways to get to ENABLED:

- **Enable here (no homing)**, from torque-off `HOMING_REQUIRED`. Nothing
  moves. `q` 0..1 is the profile's raw `min_tick..max_tick`, so the profile
  must come from tuning (`scripts/propose_profile.py`). That tool keeps the
  base profile's values for any servo without a start and an end marked, and those placeholders (for example `0..4095`, a full turn) are
  then the range the panel can drive. Tune every active servo, or remove
  untuned ones from the active mask, first. The firmware refuses
  if any servo is outside its range, and the panel names that servo and its
  `q` first. This relies on raw multi-turn ticks being the same as when the
  profile was made; compare the ticks in the panel with the tuning record if
  in doubt.
- **Home**, then **Enable** from READY. Homing moves every servo.

On ENABLED the panel streams a complete `POSITION` target for every active
joint every 20 ms. The firmware faults if no target arrives within about
150 ms of entering ENABLED (`HEAD_COMMAND_TIMEOUT_MS` plus
`HEAD_TIMEOUT_HOLD_MS` in `include/head.h`), so the stream starts on the first
ENABLED state sample. Each joint has a slider and an exact target box in
normalized `q`, jog steps of ±0.001, ±0.01 and ±0.05, and live telemetry
including raw ticks. Esc requests Disable.

**Poses.** *Capture present…* stores every active joint's present `q` by
name in `~/.ros/head_jog_poses.json` (`--poses <path>` selects another file).
*Go to pose* sets all of that pose's targets at once; the joints it does not
name keep their targets. Each servo then moves at the profile's
`max_position_fraction_per_control_cycle` and acceleration limits, so joints
do not arrive together. A pose stores `q`, not ticks, so it means the same
place only while the profile range is the same.

The panel is fail-closed on measured position. It seeds its target from
`ServoState.present_normalized` and refuses Enable, capture and streaming
while any active joint's value is not a finite number in 0..1. If it sees
ENABLED without usable values it requests Disable. It stops streaming as soon
as `/head/state` is older than 0.25 s, leaving the firmware command watchdog
to remove torque.

The bridge computes `present_normalized` from the raw tick and the firmware's
`POSITION_RANGE` reply, and publishes NaN while it has no range that matches
the generation in the latest state sample (firmware without capability bit
14, a range not read yet, or right after homing or a profile change).

To inspect the panel without ROS or hardware, the preview backend simulates a
controller; `--preview-without-normalized` reproduces a bridge connected to
firmware without `POSITION_RANGE`:

```bash
ros2 run head_ros head_jog_gui --preview
```

## Tuning panel

The tuning panel finds each mechanism's range before any limits are known. It
does not use ROS: it talks to the Teensy over USB with `headctl`'s serial code,
and every firmware protection stays active (lease, verified shutdown,
per-branch current budget, telemetry and servo-fault supervision).

```bash
firmware/scripts/head-tuning
firmware/scripts/head-tuning --preview
```

`--preview` runs against a simulated board and writes its records to
`firmware/tuning_runs/preview/`. The launcher needs a Python with PyQt5 and
pyserial; `bench/.venv` has both (`bench/requirements.txt`).

1. Connect and click Start tuning. The load limit box is filled with the
   lowest `operating_current_ma` in the board's profile (600 if it cannot be
   read); type a lower one to start gently. Each servo's Goal Current is the
   limit or its own `operating_current_ma`, whichever is lower (firmware from
   2026-10-20, `builds/per-servo-tuning`), so a higher limit, up to the highest
   `operating_current_ma`, raises only the servos whose profile allows it (the
   jaw at 650 mA, everything else 300 mA in `profile_20261020_jaw650_rest300.json`). Start low and raise it slowly: Stop,
   type a higher limit, and Start again. No homing happens. Every servo holds
   where it is, and that pose is recorded as the start (neutral).
2. Pick one servo. Each click on −50 … +50 moves it that many ticks
   (1 tick = 0.088°); holding a button does not repeat. The servo makes
   current only by trailing its goal (about 3–4 mA per tick), so the load
   limit you typed decides where steps stop: a step away from the start is
   refused once the servo trails by limit ÷ 3 ticks (at least 20), or reads
   the limit. At 400 mA that is 133 ticks; a blocked servo's goal can lead
   by that much plus one step, which is how far it can jump if the
   mechanism suddenly gives way. Use a low limit to find where an end
   starts. A refused step does not stop the push: the goal stays ahead and
   the servo keeps pulling at the limit. A servo below the limit is left
   holding: with Position I = 0 it holds a load of X mA by trailing its goal
   by about X ÷ 3 ticks, so a goal far ahead of a still servo is normal. When
   the goal is 20 or more ticks ahead and the load stays at 95 % of the limit
   for 0.3 s (the servo is at its ceiling), or a step is refused that far
   ahead, the panel **backs off** by itself: it steps the goal back
   to within 20 ticks of the servo, says so, and suggests marking the end
   there. Type a limit below the profile's `operating_current_ma`, which is
   also the servo's Current Limit: at that ceiling the servo's own Overload
   protection can trip (seen 2026-10-12 on servo 7 at 600 mA), latching
   `FAULT SERVO`; only a servo power cycle clears it. The firmware also refuses a step away at one turn of travel. A
   step back toward the start is always allowed.
3. Mark the start (home) and the end as you find them: most servos have one
   end, the farthest they go from home; use **Mark 2nd end** only for one
   that moves both ways. With one end, the proposed range stops exactly at
   home and the end-margin is taken at the far end only. Every step and mark is saved
   at once to `firmware/tuning_runs/panel/<date>_<time>_tuning.json`; each Start begins a
   new file. Marks are never lost on Stop: at Connect and at each Start the
   panel reads the earlier files, and the *Marked* column shows their marks
   as "(earlier)"; ✓ means the servo has a start and an end, which is all the
   proposal needs. The *Notes* column is for anything a later user should
   know: double-click a cell and type. Notes are saved at once in
   `tuning_runs/config/servo_notes.json` and show in every later session; each
   record also keeps the notes typed during it. The start mark is the servo's home (`home_tick` in the
   proposed profile), so click **Mark start** for every servo at least once;
   without one, the proposal uses wherever the latest session started.
4. **Home selected** and **Home all** drive servos back to their home: the
   latest start mark from any session, or where this session started if none
   was clicked. The *From home* column shows the distance, and a dialog lists
   it before anything moves. Home works on where the servo **is**, not on its
   goal: a servo that needs load to sit at home (the jaw) stays short of its
   goal by about load ÷ 3 ticks, so after each fresh sample the goal moves on
   by the servo's remaining error, at most 50 ticks, until the servo is within
   5 ticks of home. It then holds there, and the message says with how many
   mA. The goal never leads the servo by more than the tuning lag limit
   (limit ÷ 3 ticks); if that is not enough, Home stops and reports the load
   it needed. Servos get one step each in turn, so co-actuated tendons return
   together. Every firmware check still applies. Esc stops it. Home does
   not start at all if any servo is more than one turn (4096 ticks) from
   its target: one horn turn is about 51 mm of line, and so far a distance
   that large has meant the servo's turn count changed after a servo power
   cycle (2026-10-15), not that the horn turned. Re-mark that servo.
   Home never moves the jaw (servo 7): it cannot move while the modiolus
   tendons are tight, and Home pushed it into them until it tripped
   Overload (2026-10-15). Move it with the jaw sweep or single steps.
5. STOP or Esc requests the verified torque-off path; Esc works even while a
   note is being typed. If the board faults, the message names the fault
   and, for `SERVO`, which servo raised which Hardware Error bit (Overload,
   Electrical Shock, Overheating, ...) with its load, temperature and
   voltage; the record's `faults` list keeps every servo's reading then. Closing the window does
   the same, and a lost window or USB link lets the lease expire.
6. That fault snapshot is taken after the servo has already switched itself
   off, so it cannot show what led up to the fault. For that, each Start also
   writes `<time>_tuning.telemetry.csv` next to the record, until Stop: one
   row per tuned servo every 100 ms of controller time, plus a row set at once
   whenever the board state, the fault or a Hardware Error changes. Columns:
   goal and present tick, current (mA), PWM (%), temperature, voltage,
   Hardware Error, online and feedback age, with host UTC and controller time
   (`mcu_time_ms`). Each fault in the record has the `mcu_time_ms` of its
   sample, so it can be found in the file. PWM needs firmware with
   `PRESENT_PWM` (capability bit 15); with older firmware the column is empty.
   The table also shows each servo's live *PWM (%)* (same firmware need) and
   *Temp (°C)*, the servo's own Present Temperature.
   20 servos make about 65 MB per hour.
7. **Jaw sweep** (box 3) measures how the modiolus tendons must follow the
   jaw. The jaw (7) cannot open unless the tendons that lift the modiolus
   (right 11/14, left 16/19) pay out to give it room, and they must reel in
   when it closes. Their servos do not feel the jaw push: on 2026-10-15 the
   jaw load rose from 32 to 600 mA while theirs changed by 10 mA at most. So
   the jaw's own load is the sensor. Type the jaw servo, the tick where the
   jaw is open, the number of stages, every tendon with the tick direction
   that **tightens** it (`8+ 11+ 14+ 9+ 16- 19-`), and which of them pay out
   on the way open (*Release on open*, e.g. `11 14 16 19`). The boxes open
   with the values of the last sweep started (`tuning_runs/config/jaw_sweep_settings.json`),
   or, before the first one, the values confirmed on the head on 2026-10-15
   (`8+ 11+ 14+ 9+ 16- 19-`, release `8 9 11 14 16 19`, preload 60, band 10,
   push 60, abort 280, travel 600, 1 stage). The open tick is filled with
   where the jaw is at each Start; **Jaw now** fills it again. A wrong
   direction unwinds the line, so the dialog lists each one: check them.
   For scale: the datasheet's 0.950 N·m/A on the 8.1 mm horn is about
   0.12 N of line per mA (an upper bound; gear friction takes some), so
   60 mA is about 7 N. Stage 0 is a hold at the start (pretension
   only); then the jaw opens in equal stages and closes the same way. Each
   stage has two phases:
   - **move**: the jaw steps at most 5 ticks at a time toward the target and
     pushes at most *jaw push limit* mA more (by load, and by goal lead at
     about 3 mA per tick) than it held toward the target at the last stage.
     While it is held there on the way open, each release tendon pays out
     2 ticks per sample. If every release tendon has paid out 150 ticks
     (about 1.9 mm of line; 60 until 2026-10-20) since the jaw last gained a tick, the sweep
     stops and holds: the release set is not what blocks the jaw (on
     2026-10-18, 11/14/16/19 paid out about 200 ticks each with the jaw
     still). Release tendons are never reeled in during a move;
     the other tendons are held at the preload.
   - **hold**: the jaw holds the target, carrying whatever the preload needs
     up to the abort load. Every tendon reaches the preload **from below**:
     it pays out until it reads at most preload − band, then only pulls in
     until it reads at least preload − band/2 (by half the load error ÷ 3 mA
     per tick, at most 10 ticks). Friction holds a resting horn's reading
     anywhere in a band of about 20–50 mA, so a reading reached from one side
     is repeatable; a line that went slack reads low and is pulled in. The
     A line can only pull: a reading against the pull direction is the
     servo pushing its horn out against friction, so it counts as no line
     load (0), never as an error; the file keeps the signed readings. The
     stage is saved once the jaw is within 5 ticks of its target and every
     tendon has finished its approach and stayed in band for 5 samples.
     Each row keeps how far each horn took in line during the hold
     (`taken_in_ticks`). At stage 0 (pretension) a tendon that took in less
     than 20 ticks is listed in `unverified` and the panel says so: on
     2026-10-15 servo 9 read the preload after 5 ticks while the others took
     in 120–430, so its load was probably friction.

   **Slack while opening** (default `8 9`): on the way open these tendons are
   not pulled back to the preload in each hold (they only pay out if loaded
   over preload + half the band), so they do not pull the jaw shut; at the
   pretension and on the way back they are re-tensioned like the others.
   On 2026-10-18, with every line re-tensioned at each stage, the jaw needed
   −134 mA at −15 ticks and passed the 280 mA abort at −35. Each row lists
   them under `slack`.

   Each row has the goal, present tick and load of the jaw and every
   tendon, in `tuning_runs/panel/<date>_<time>_jawsweep.json` (format `head-jaw-sweep-4`;
   `propose_profile.py` does not read it). The sweep stops and **goes limp**
   when the jaw or a tendon reaches the abort load, in either direction. It
   stops and **holds** when a tendon would move more than the travel limit
   from where the sweep started (a wrong pull direction ends here: the horn
   unwinds and never takes load), when a stage does not finish in 30 s (the message says if the jaw
   was held at its push limit), on a refused step, or when back-off starts.
   Esc stops at any time. The abort load must be above the preload + band
   and the jaw push limit, and below 95 % of the load limit. The gains, step
   sizes, probe and settle count are first guesses, tried only on the
   simulated board. In `--preview` the jaw and the right modiolus run on a
   simple tendon model: the jaw opens toward higher ticks, 11 and 14 hide
   behind friction and hold the jaw closed until they pay out, 8 feels the
   jaw (`8+ 11- 14+`, release `11 14`). It is not a model of the head.

### Where the tools keep their files

Everything goes into `firmware/tuning_runs/` (`--records` changes it;
`--preview` uses `tuning_runs/preview/`). The layout is in
`head_ros/records_layout.py`:

```text
tuning_runs/
  STOP          create it to stop a running tool
  config/       servo_notes.json, jaw_sweep_settings.json, face_markers.json,
                face_reference.jpg (the knee test finds the face against it)
  maps/         the workspace map and the runs it came from
  panel/        tuning-panel records, <date>_<time>_tuning.json (+ telemetry);
                Home reads its marks from these
  videos/       videos and pictures kept by hand
  runs/<date>/<time>_<kind>[_<label>]/
                one folder per tool run: record.json, telemetry.csv (or
                tuning.json + tuning.telemetry.csv), video.webm,
                video.frames.csv, stills/, neutrals.json, ...
```

`<date>` and `<time>` are local time; times inside the files are UTC. Kinds:
`knee`, `neutrals`, `ranges`, `face_<tour>` (or `face_frames`), `sweep`,
`coact`, `jawprobe`. A recording is saved as MP4 and turned into
`video.webm` when the run ends (this desktop's player cannot open the MP4).
`head-face` without `--neutrals` uses the newest `runs/*/*/neutrals.json`.

### Jaw sweeps from the command line

`firmware/scripts/head-sweep` runs the same jaw sweep as box 3 without the
window, so a sequence can run unattended while someone stays at the 12 V
switch. The panel must be closed: only one program can hold the port.

```bash
firmware/scripts/head-sweep --limit 400 --sweep 0:1 --sweep -80:4
firmware/scripts/head-sweep --limit 400 --sweep -200:8 --camera realsense
firmware/scripts/head-sweep --preview --limit 600 --sweep 0:1 --sweep 40:2
```

- Each `--sweep OPEN:STAGES` opens the jaw by OPEN ticks from where it is
  when that sweep starts (on the head the jaw opens toward **lower** ticks,
  so opening is negative; `0:1` is pretension only). Sweeps run one after
  the other in one tuning session; the next starts only if the one before
  finished.
- The other values come from `tuning_runs/config/jaw_sweep_settings.json` (the
  panel's last sweep); `--pull`, `--release`, `--preload-ma`, `--band-ma`,
  `--jaw-push-ma`, `--abort-ma`, `--max-travel-ticks` and `--defer-on-open`
  override them.
- It writes the same files as the panel: a tuning record, its telemetry
  CSV, and one `_jawsweep.json` per sweep. Each saved stage prints one line.
- **Stops:** Ctrl-C, SIGTERM, or creating `tuning_runs/STOP` turn torque
  off (exit 3); it refuses to start while that file exists. Back-off at the
  load limit, a board fault, or the sweep's abort load also turn torque off.
  A sweep that stops and holds ends the sequence (exit 2). A killed process
  or lost USB link lets the 1 s lease run out. When the sweeps finish,
  torque goes off (exit 0), or with `--after hold` the servos hold until a
  stop.
- `--camera realsense` saves a picture before each sweep and at each saved
  stage, in `<sweep>_frames/`. It needs pyrealsense2, numpy and Pillow
  (`bench/requirements.txt`); nothing else imports them.
- With `--camera realsense` and a `tuning_runs/config/face_markers.json`, each
  picture is also searched for the face markers (ink dots, skin-visible
  magnets, eye holes; see `face_markers.py`), and each sweep writes
  `<sweep>_markers.json` with every marker's pixel position and its shift
  since the sweep's first picture, minus the mean shift of the reference
  markers (the eye holes). Each picture prints one line of shifts. On a still
  face the dots jitter by up to about 0.3 px (sd), the eye holes by 0.01 px
  (2026-10-18). Positions are in pixels: the depth stream gave a value on
  only 12 % of the face at this distance.
- Each tool prints the per-servo caps it read from the board at start. The
  lines' abort (`--abort-ma`) is checked against their cap, the jaw's own
  `--jaw-abort-ma` against the jaw's. The jaw guard (`--jaw-hold-ma/-s`,
  `--jaw-peak-ma/-s`, default 400 mA for 2 s and 600 mA for 0.5 s) steps the
  jaw back and ends the run when it stays over the hold load, and turns
  torque off over the peak load: the jaw may peak high but not stay there
  (it tripped Overload in 6-12 s at 450-517 mA on 2026-10-13).
  `firmware/host_ros/src/head_ros/head_ros/jaw_probe.py` (run with the bench
  venv, or `scripts/head-jawprobe`) pays the modiolus lines out and then opens
  the jaw as far as its push limit allows, recording jaw load against
  position. Pulse mode (`--pulse-ma`, repeat for a ladder; 2026-11-11) gives
  short strong pulses instead: the goal leads the jaw by push / 3.5 mA per
  tick for `--pulse-s` (shorter than `--jaw-peak-s`), trimmed to hold the push,
  then a fast close and `--rest-s`; `--payout 0` is the no-slack baseline;
  stops at 52 C; the jaw goal never goes under 3600. With `--camera
  realsense` it records a video and a still at the end of each pulse.

  ```sh
  scripts/head-jawprobe --limit 650 --abort 280 --jaw-abort-ma 615 --jaw-push 400 \
      --payout 300 --open -300 --lines "11+ 14+ 9- 15- 19-" \
      --pulse-ma 450 --pulse-ma 520 --pulse-ma 580 --pulse-s 0.8 --jaw-peak-s 1.0 \
      --camera realsense
  ```
- `coact_test.py` (bench venv) pulls one modiolus's lines together: up and
  diag in five mixes while down only follows (keeps the preload), then down
  alone while up and diag follow. Each combination stops at the end load, its
  full single-line range, or a driver's load falling under half its peak
  (magnet losing grip). `workspace_guard.py` turns the face actions (a 2-D
  point per modiolus and the jaw from 0 to 1) into targets inside the measured
  workspace (`tuning_runs/maps/20261020_workspace_map.json`), with speed limits, a
  follower regulator and the live stop checks. Each line's neutral follows the
  jaw: it is interpolated between the neutral found with the jaw closed and
  the one found with it open (in one session they differed by -50 to +166
  ticks; two points per line, so the straight line between them is an
  assumption).
- `firmware/scripts/head-ranges --neutral-only` finds this session's neutrals
  (TUNING mode, no end search) and writes `neutrals.json` in its run folder: the
  neutral goal ticks at the first `--jaw-tick` (closed) and the last (open),
  and the controller boot session. The mapping and the combined test used
  preload 35 mA, band 8 mA:

  ```sh
  scripts/head-ranges --limit 650 --neutral-only --slack-ticks 300 --jaw-tick 4000 --jaw-tick 3900 --preload-ma 35 --band-ma 8 --jaw-push-ma 580 --abort-ma 280 --jaw-abort-ma 600 --max-travel-ticks 2000
  ```

- `firmware/scripts/head-face` (`face_stream.py`, no ROS) drives the face in
  ENABLED mode: it enables at the present pose, then every 20 ms (50 Hz)
  turns the actions into targets with `workspace_guard.py` and sends one
  target frame for every active servo; servos off the face hold their pose
  from Enable. It refuses neutrals from another controller boot. Every stop
  turns torque off (the face relaxes): a line at the abort load (280 mA), a
  driving line whose load falls under half its peak, the jaw guard (over
  400 mA for 2 s: back to closed first; over 600 mA for 0.5 s), a board
  fault, stale state, Ctrl-C/SIGTERM or `tuning_runs/STOP`. A stalled host
  makes the firmware fault on its own (WATCHDOG after 100 + 50 ms). Speeds
  default to 300 ticks/s for lines and 60 ticks/s for the jaw (chosen, not
  measured). Moves come from keyframes, for example
  `--frame 'hold=4 right=0.5,1 left=0.5,1 jaw=0.3'`; without `--frame` it
  runs a short demo tour. It writes `record.json` and `telemetry.csv` in its run folder.
  `--preview` runs it on a simulated board. Enable needs every active servo
  inside the profile's min..max, and the targets are clamped to it.
- `head-face --camera realsense` records the run (`video.webm`, converted from the
  recorded MP4 at the end, with `video.frames.csv` giving each frame's UTC time to match the
  telemetry CSV) and saves stills before Enable, at the end of each demo
  move, at the final rest and 1 s after torque off (`stills/`).
- `firmware/scripts/head-knee` (`knee_test.py`) finds where each modiolus
  line just becomes tight at the skin's own rest pose: it slackens all six
  lines (the silicone then puts both action points at rest), takes one line
  in at a time while the other two of that point stay slack, and marks the
  knee where the load first stays above the servo's own no-load current
  (baseline over the first 60 ticks; threshold baseline + max(8 mA, 4 x
  noise)). Each line is paid back out before the next. Then the three lines
  of a point go to their knees together and hold 2 s: if the knees describe
  one rest pose, each reads only a little over its baseline. Writes
  `record.json` in its run folder; `--camera realsense` records it as well.

  ```sh
  scripts/head-knee --limit 650 --camera realsense
  ```

  With `--camera realsense` the knee comes from the camera (since
  2026-10-21): the slack picture of each mouth corner is the reference, and
  a line's camera knee is where the corner first moves more than 0.5 px for
  3 frames in a row (a still head reads about 0.15 px); the take-in stops at
  1.5 px. The corner boxes are fixed for the present camera aim; check
  `stills/00_corner_regions.jpg` (the face is found again each run). When all six lines have a
  camera knee it writes `neutrals.json` (kind `camera_knee`),
  which `head-face` uses without the up/diag slack offset. The knees are
  found with the jaw closed; the open-jaw neutrals are a copy.

- Since 2026-11-10 every `head-face` session (and the tools built on it, and
  `head-eyes`) also
  stops at **52 C** on any servo (`TEMP_STOP_C`, operator rule; `--temp-stop-c`
  may only lower it) and logs each servo's highest temperature in
  `record.json`. The magnet-drop check uses a driver's load only while it is
  within 8 ticks of its goal: the servos are P-only, so a fast move reads its
  lag as load (run 200042: false "magnet" stop at 1000 ticks/s). Since
  2026-11-11 it also waits 0.3 s (`DROP_SETTLE_S`) before it takes a new peak
  after a line is let go, another line on its side moves or the jaw target
  moves: the skin follows a release late, and talking runs 124404, 125342 and
  130153 stopped on that (lines 14, 14, 19). Not yet run on the head.
  `--jaw-release down=N` pays the down lines out N ticks at the fully open
  jaw goal, in proportion to the jaw's measured position (the open-jaw
  neutral moves toward slack); see `firmware/docs/NATURAL_FACE_AND_JAW_WORKSPACE.md`.
- `firmware/scripts/head-natural` (`natural_face.py`) runs the natural face:
  eye saccades (0/2 together, 1 pitch), blinks (5/6), a fake smile (up + diag,
  no oculi) and talking jaw syllables in phrases, all at once, inside the panel
  marks and the head-face guards; eyes and lids stop at 280 mA and move with a
  speed and acceleration limit. Defaults: lines 1000, jaw 4000, eyes 2500
  ticks/s, jaw current cap 650 mA. Tested on the head 2026-11-10:

  ```sh
  scripts/head-natural --neutrals tuning_runs/runs/<date>/<time>_knee/neutrals.json --range up=350 --range diag=290 --range down=150 --seconds 40 --camera realsense
  ```

  `--smile -1..0` holds the corners down instead (tests the jaw against
  driving down lines); `--no-talk`, `--seed`, `--seconds` (10-120);
  `--lid-ladder` steps the lids to 0.3..1.0 of their closed mark with stills
  (where the eye is covered); `--blink-depth [5=|6=]F`, `--lid-open-speed`.
  Measured defaults (2026-11-10): jaw goal 3700, `--jaw-release 8=30
  --jaw-release 9=15`, lids close 5000 / open 3500 ticks/s at 80000 ticks/s^2
  with the speed scaled per lid by its travel, blink depth 5: 0.55, 6: 1.0;
  the jaw goes through `face_workspace.JawBudget`.

  `--say TEXT` (2026-11-11, `speech_face.py`; simulator only so far) speaks a
  text instead of the random syllables. eSpeak NG (`libespeak-ng.so.1`)
  makes the audio and the time of each phoneme (moved 45 ms earlier: the
  events lag the sound, measured on 103 stops); each phoneme maps to a viseme
  (jaw, corner spread, rounding, p/b/m seal; table chosen, not tuned on the
  head), which the mouth starts 70 ms before the sound, reaches in up to
  80 ms and holds. The jaw also follows the audio loudness (35 %). Spread adds
  to the fake smile, rounding takes part of it away. While speaking, the eyes
  look away at some phrase starts and come back to the camera, and the lids
  blink at most phrase ends. The audio plays with `aplay` at the speech start
  (`--audio-advance-ms` shifts it; the audio-vs-motion delay is not measured);
  with `--camera realsense` the run also writes `video_with_audio.webm`
  (the audio placed by video frame count and sped up by the camera's real
  frame rate, about 14.9 for 15 fps). `--lips` also presses the lower and upper
  lips for p/b/m (their effect on this head is not measured). `--voice`,
  `--wpm`, `--pitch`, `--speech-at`, `--no-audio`; `--seconds` defaults to
  just long enough for the text.

  ```sh
  scripts/head-natural --neutrals tuning_runs/runs/<date>/<time>_knee/neutrals.json --camera realsense --say "Hello! My name is Head. I can blink, look around, and smile while I talk."
  ```
- `face_workspace.py` is the commandable workspace for ML policies:
  `WorkspaceSpec` (measured limits; `tuning_runs/config/face_workspace.json`),
  `project()` clips any action, `JawBudget` allows the jaw past its static
  limit only in short pulses with a duty limit.
- `firmware/scripts/head-jawcorner run [head-face args]` steps the jaw
  0 -> 0.5 -> 1 -> 0 in each of seven corner poses (keyframes, head-face
  safety); `head-jawcorner report RUN` gives per hold each line's position from
  its neutral and load, and per pose what moved with the jaw
  (`jaw_corner_report.json`).
- `firmware/scripts/face-preview` shows the camera live, to aim it; S saves
  the picture as `tuning_runs/runs/<date>/<time>_camera.jpg`. Close it before a run with
  `--camera realsense`: only one program can open the camera.
  `face-preview --list-modes` prints the color modes the camera offers;
  `--mode WxH@FPS` shows one of them. `head-face --camera-mode WxH@FPS` records
  in that mode (default 1280x720@15, the mode tested on 2026-10-18).

### Lip sync (2026-11-08, simulated board only)

The plan and the hardware tests are in `firmware/docs/LIP_SYNC_TEST_PLAN.md`;
the training side is in `lipsync/README.md`. None of this has run on the head.

- **Upper lips 13/18.** `head-face` drives them (`ulip=R,L` in a keyframe,
  `upper_lip_right/left` in a trajectory) only when the tuning panel records
  hold a neutral and an end mark for both. The pull direction is the sign of
  end - neutral, and each lip stops at its own end mark
  (`workspace_guard.upper_lips_from_marks`). Without both marks they hold
  their pose, as before, and the run says so.
- **Trajectories.** `head-face --trajectory FILE` streams a mouth trajectory
  CSV (`mouth_actions.py`: `t_s` plus the 9 mouth actions, relative to the
  session neutral, linear between rows). `--audio SPEECH.wav` plays the file
  with ALSA `aplay` from the trajectory's `audio_start_s`;
  `--audio-advance-ms` starts it that much earlier (the measured delay, test T5).
  The session limits, speeds and stops are the same as for keyframes.
- **Command log.** Every `head-face` run writes `commands.csv`: host UTC,
  script time, the mouth actions and the targets sent to the mouth servos,
  one row per update. With `video.frames.csv` it pairs camera frames with
  commands (`lipsync/scripts/build_dataset.py`).
- **Viseme tool.** `firmware/scripts/head-viseme targets --photos DIR` measures
  photos of a person's mouth (MediaPipe; `rest.jpg` plus one photo per viseme).
  `head-viseme search --viseme NAME --save` and `head-viseme try --viseme NAME
  --pose '...'` run one ENABLED session (the head-face session, same safety):
  the robot's rest and six extreme poses first (its own span), then each pose
  is held until the targets settle, measured with the dots
  (`mouth_shape.py`: width, opening, lip gap) and scored against the photo.
  Each pose gets a side-by-side picture in `composites/`; the trials are in
  `viseme_trials.json`. Poses go to `tuning_runs/config/viseme_library.json`.
  It needs `face_markers.json` entries `modiolus_right/left`, `lower_lip`,
  `upper_lip` and the eye holes.
- **Babbling.** `firmware/scripts/head-babble --minutes M --out FILE` writes a
  trajectory of random poses around the library visemes (noise, dwell,
  blocks with rest pauses, a jaw open-time limit), paced for the speeds it is
  given. Play it with `head-face --trajectory FILE --camera realsense` and the
  same speeds.

To turn records into a proposed profile, choose the two safety margins and
run `firmware/scripts/propose_profile.py`. It writes a new file for review
and never uploads.

## Maintenance calibration

Maintenance calibration is not exposed in the GUI yet. Protocol v2 does not
report the active calibration servo/phase, and the current ROS configuration
service does not expose the full profile. Keep using the reviewed action and
`headctl` workflow until those safety-relevant interfaces are versioned.

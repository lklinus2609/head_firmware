# Demo videos

Robot face (tendon head), recorded with the RealSense D435i (1280 x 720,
15 fps) during the tuning sessions of 2026-10-21 to 2026-10-23. All are H.264
MP4. Each row names the run it came from in `../tuning_runs/`.

| File | Shows | Source |
|---|---|---|
| `01_range-of-motion.mp4` | Both mouth corners through their workspace (up, up + diag, diag, down, each side, both, alternating), then jaw open with both corners up. 78 s. | `videos/2026-10-21_1613_range-of-motion.webm` |
| `02_smile.mp4` | Smile. | `videos/smile.mp4` (2026-10-21) |
| `03_duchenne-smile.mp4` | Duchenne smile: mouth corners with the orbicularis oculi. | `videos/duchenne.mp4` (2026-10-21) |
| `04_upper-face.mp4` | Upper face: brows and orbicularis oculi, each side and both. | `videos/upper_face.mp4` (2026-10-21) |
| `05_brows.mp4` | Both brows pulled to the operator's panel end marks (right 496, left 520 ticks): inward and slightly down. | `runs/2026-10-23/182129_face_frames` |
| `06_orbicularis-oculi.mp4` | Both orbicularis oculi to their end marks (right 991, left 796 ticks). | `runs/2026-10-23/183038_face_frames` |
| `07_lower-lip.mp4` | Lower lip pulled down 735 ticks per side (right rest shifted +350, left -50 to look even). | `runs/2026-10-23/181157_face_frames` |
| `08_jaw-open-close.mp4` | Jaw opened and closed twice, fast (goal 3680 at 300 ticks/s; the jaw reaches about 3850 at about 600 mA). | `runs/2026-10-23/184607_face_frames` |
| `09_duchenne-smile-fast.mp4` | Duchenne smile tour at about 70 % servo speed (5000 ticks/s limit; corners about 2350, oculi about 4600 ticks/s real). Mouth-only smile, then the closed-jaw Duchenne smile. The take ends early: the right orbicularis oculi magnet slipped during the second smile and the load check turned torque off. 7.5 s. | `runs/2026-10-23/193939_face_duchenne` |

## New head prototype (2026-11-10)

Recorded the same way, but these are **VP8 WebM** copies of the run videos:
this machine has no H.264 encoder (no ffmpeg; GStreamer and OpenCV have none).
The originals stay in their run folders, where `record.json` and
`video.frames.csv` refer to them. Results: `../docs/NATURAL_FACE_AND_JAW_WORKSPACE.md`.

| File | Shows | Source |
|---|---|---|
| `10_natural-face.webm` | The natural face (`head-natural`, measured defaults): eyes looking around, blinks (close 0.36 s), fake smile (no oculi), talking jaw at goal 3700 with the down lines released 8=30 / 9=15. 45 s. | `runs/2026-11-10/205555_face_natural` |
| `11_full-action-space-explore.webm` | Random actions over the whole ML action space through `face_workspace.project()`: both corners (up and down), jaw (budget), brows, oculi, lips, gaze, lids, blinks. Finished, nothing clamped. 125 s. | `runs/2026-11-10/210401_face_natural` |
| `12_workspace-validation-40-poses.webm` | 40 held, stratified poses over every action dimension (the commandable-map check). 84 s. | `runs/2026-11-10/211350_face_natural` |
| `13_corners-down-while-talking.webm` | Corners held down while the jaw talks, with the per-line release (8=38, 9=22). 35 s. | `runs/2026-11-10/204936_face_natural` |
| `14_fast-blinks.webm` | Blinks at the hardware limit: close 5000 ticks/s, open 3500, per-lid depth and speed; eyes looking around, no mouth. 25 s. | `runs/2026-11-10/204824_face_natural` |
| `15_lid-ladder.webm` | Both lids stepped to 0.3 ... 1.0 of their closed mark: the right lid covers the eye at 0.4-0.6, the left only at 1.0. 19 s. | `runs/2026-11-10/204310_face_natural` |
| `16_eyes-and-lids-tour.webm` | Eye and lid range-of-motion tour at 2500 ticks/s: each eye, pitch, each lid, both together, two blinks. 26 s. | `runs/2026-11-10/185607_eyes` |
| `17_duchenne-smile-new-head.webm` | Duchenne smile tour on the new head, half modiolus range, 2500 ticks/s. 30 s. | `runs/2026-11-10/193559_face_duchenne` |

Not included: a sad face. None of the sad takes was accepted (the corners did
not go down convincingly).

Each take is played by `scripts/head-face` with `--camera realsense`; see
`../host_ros/README.md`.

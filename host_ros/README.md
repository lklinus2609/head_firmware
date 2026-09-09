# ROS 2 desktop tools

The ROS 2 Jazzy workspace contains three cooperating desktop processes:

- `head_bridge` owns the Teensy serial port and exposes `/head/*` topics,
  services, and actions;
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

Maintenance calibration is not exposed in the GUI yet. Protocol v2 does not
report the active calibration servo/phase, and the current ROS configuration
service does not expose the full profile. Keep using the reviewed action and
`headctl` workflow until those safety-relevant interfaces are versioned.

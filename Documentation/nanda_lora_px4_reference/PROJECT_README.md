# Reference: Nanda's PX4/LoRa autoland project — read-only, later-stage material

Received 2026-09-17. This is the independent PX4 + Gazebo + ROS 2 + MAVROS work Nanda
mentioned building (long-range LoRa-ranging EKF homing, then AprilTag vision for terminal
landing) — the `vision_node.py` here is what "pad tracker" referred to earlier, and this
project's own patrol-waypoint logic is the "controlled landing profile" Nanda described.
Its own `README.md` (kept in place, one directory down at `src/`'s sibling) has the full
setup/run instructions for its own PX4/ROS 2/MAVROS stack.

**Not ported or run yet — deliberately.** Per `docs/06-open-questions.md` Q11's
2026-09-15 update: this project stays ArduPilot (matches the Pixhawk 2.4.8 already owned;
PX4 ≥1.14 doesn't support that flight-controller target at all, independent of ROS 2), so
none of this runs as-is. Kept here as reference for **Stage 3** (beacon fusion, per
`docs/04-roadmap-checklist.md`) — not required for Stage 2 (current work), and not
something to start porting until Stage 2 passes.

## What's actually in here

- `src/gz_lora_range_plugin/` — a Gazebo system plugin simulating noisy LoRa ranging to
  4 ground anchors. Gazebo-plugin C++, not PX4/ROS2-specific — the most directly portable
  piece once we're on Gazebo (autopilot-agnostic; this project's own Gazebo stays regardless
  of which autopilot's Gazebo bridge is above it).
- `src/roland_lora/src/ekf_node.cpp` — the EKF that fuses the 4 noisy LoRa ranges into a
  position estimate. Filter math itself is autopilot-agnostic (same category as ROLAND's
  own EKF, `docs/02-roland-reference.md`) — the *math* ports, the ROS 2 node wrapper around
  it doesn't (this project defaults to ROS2-free, `docs/06-open-questions.md` Q11).
- `src/roland_lora/src/autoland_node.cpp` — the state machine: patrol → LoRa homing →
  vision terminal approach → trigger native `AUTO.LAND` at 30cm. This is PX4-specific
  (talks to PX4 via MAVROS/uXRCE-DDS, triggers PX4's own `AUTO.LAND`) — the *shape* of the
  state machine (long-range fusion → handoff to vision at close range → native autopilot
  land trigger) is the reusable idea; the implementation talks to the wrong autopilot.
- `src/roland_lora/scripts/vision_node.py` — AprilTag detection (`apriltag` python package,
  not this project's chosen `cv2.aruco` — same library discrepancy already flagged for
  Dinesh's reference code, same reasoning applies here).
- `src/roland_lora_gazebo/` — Gazebo world/models: `x500_lora` (PX4's own airframe, not
  ours), `lora_anchor`, `apriltag_pad` (a 2x2m tag, much larger than our 60cm board spec —
  don't copy the marker size, we already have a validated board design, `CLAUDE.md` §3).

## Before touching this for real (when Stage 3 actually starts)

1. Read `ekf_node.cpp`/`ekf_node.hpp` for the filter math, port the *math*, not the ROS 2
   node scaffolding around it.
2. Read `autoland_node.cpp` for the state-machine shape (when to trust LoRa vs. vision),
   re-implement against ArduPilot's own mode/param interface (`PLND_*`, same as Stage 1/2),
   not MAVROS.
3. The Gazebo plugin and world files are the closest to directly reusable, once this
   project is actually on Gazebo (not yet, per `CLAUDE.md` §0b — still sim-first, Stage 2
   is synthetic-camera, pre-Gazebo).

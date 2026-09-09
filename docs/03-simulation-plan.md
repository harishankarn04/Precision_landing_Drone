# 03 — Simulation Plan

**Current answer: Gazebo + ArduPilot, ROS 2 status open.** Simulator changed to Gazebo
2026-08-26 (from Webots); autopilot changed to PX4 the same day, then reverted back to
ArduPilot 2026-09-09 — see `CLAUDE.md` §0 and §0b for why. **ROS 2 is no longer assumed** —
default plan is to build ROS2-free, using `gz-transport` directly for the Gazebo camera
feed; see `06-open-questions.md` Q11 before committing either way. **Nothing on this stack
is set up yet.** This doc doesn't have verified setup steps for it — write those when the
work actually starts, don't invent them speculatively ahead of time.

---

## What's still true from the original (ArduPilot+Webots) attempt

The original plan got a real closed-loop landing working before pivoting away from Webots,
and a few lessons from that are directly relevant again now that ArduPilot is back
(Webots itself is still dropped in favour of Gazebo):

- **Prove the simplest possible loop before adding perception.** Get the autopilot flying
  and taking basic commands (arm, takeoff, land) in the simulator *first*, with no camera
  and no custom code, before wiring in anything else. This caught real bugs early both times.
- **Design the frame source as a swappable interface from the start** — whatever module
  turns "a rendered image" into "a decoded pose" should not care whether the image came
  from a synthetic renderer, Gazebo, or a real camera. Write it once.
- **A GUI simulator (Gazebo, in this case) needs a genuinely stable base case before layering
  a camera/sensor-heavy world on top.** The Webots attempt hit its worst bug (a hard crash)
  specifically on the camera-equipped world, not the plain one — isolate simple-vs-complex
  worlds when debugging stability.
- **MATLAB is for EKF design (later stage), not the primary simulator.** It can't run
  autopilot firmware or render markers. Still true regardless of autopilot choice.
- **Team compute split still holds:** Hari's M1 for day-to-day development, the teammate's
  Linux Mint + 40-series GPU machine for real-time/results-grade Gazebo runs, synced via
  GitHub. Whether Gazebo dev happens natively on the M1, in a VM, or waits for the GPU
  machine is a decision to make when actually starting — not pre-decided here.

## What needs deciding when this stage actually starts

- Gazebo version/distribution, and which machine hosts primary development
- ArduPilot SITL setup and how it's built/run (native, Docker, or otherwise), and how it
  connects to the Gazebo world (the `ardupilot_gazebo` plugin is the likely route)
- Whether ROS 2 is used at all (`06-open-questions.md` Q11) — default plan is no: read the
  Gazebo camera topic via `gz-transport` directly, talk to ArduPilot via `pymavlink`/MAVLink
  as already planned. Only reconsider if the guide confirms a specific ROS 2 requirement
- QGroundControl setup — which machine hosts it (Gazebo + ROS 2 Humble are already
  installed in Hari's Parallels Ubuntu VM, though ROS 2 may end up unused; ArduPilot SITL
  and QGC still need adding there or wherever else this ends up running)
- How the marker board is represented in Gazebo (a world model, at minimum)
- Camera sensor plugin choice and its published topic/format, feeding into the same
  swappable frame-source interface mentioned above

None of this is decided yet — resolve it when the work begins, and update this file with
what was actually verified to work (not what's assumed to work).

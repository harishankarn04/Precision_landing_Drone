# 03 — Simulation Plan

**The question:** ROS + Gazebo? MATLAB? macOS or a Linux VM?

**Short answer:** build it in **three tiers**, and start at Tier 0 today on macOS.
Do **not** start by installing ROS and Gazebo — that is the most common way this kind of
project burns its first month on toolchain setup and produces nothing.

---

## Tier 0 — ArduPilot SITL + synthetic camera, natively on macOS ⭐ START HERE

**What it is.** ArduPilot's Software-In-The-Loop simulator runs the *real ArduCopter
firmware* as a desktop process. It builds and runs natively on macOS ARM. There is no
camera in SITL — so we render one ourselves.

**The trick:** the AprilTag board's geometry is known exactly (§01, Table 3.2), and SITL
publishes exact vehicle pose. So a ~200-line Python script can, every frame:

1. Read vehicle pose from SITL over MAVLink
2. Compute the camera pose (vehicle pose + the known camera mount offset/orientation)
3. Project the flat board into that virtual camera (`cv2.warpPerspective` of a board image,
   using the thesis's calibrated intrinsics `fx=497.88, fy=496.68, k1=0.140, k2=−0.266`
   as a stand-in until we calibrate our own camera)
4. Hand the resulting 640×480 frame to **our reimplemented detection pipeline**
5. Pipeline emits `LANDING_TARGET` → back into SITL → ArduCopter's LAND controller acts on it

```
  ArduCopter SITL ──pose──► virtual camera renderer ──image──► our pipeline
        ▲                                                          │
        └───────────────── MAVLink LANDING_TARGET ─────────────────┘
```

> **Since we are building the pipeline from scratch (no inherited code), this harness is
> also our development and test rig.** Write the frame source as a swappable interface —
> `SyntheticRenderer` (Tier 0) → `GazeboCamera` (Tier 1) → `PiCamera2` (hardware) — and the
> same detection/fusion code carries through all three unchanged.

**What this validates — genuinely a lot:**
- ✅ Closed-loop autonomous touchdown (the senior's #1 unfinished item) end to end
- ✅ All `PLND_*` tuning, especially `PLND_ACC_P_NSE`, `PLND_LAG`, `LAND_SPEED`
- ✅ The multi-tag fusion estimator across the full altitude range, with **exact ground truth**
- ✅ Detection-continuity handoff between the 24 cm and 8 cm tags
- ✅ Target-lost / reacquire / abort logic
- ✅ Moving-platform behaviour — just animate the board's world pose
- ✅ Statistical landing-accuracy runs: script 200 landings overnight, get mean/σ for free

**What it does NOT validate:** real lighting, rolling-shutter skew, motion blur, vibration
coupling, actual Pi compute latency, real UWB multipath. Those need Tier 1 and real flight.

**Cost:** one to two days. **No Linux, no ROS, no Gazebo, no VM, ~2 GB disk.**

### Setup sketch
```bash
brew install gcc-arm-none-eabi   # (already present)
pip install --user mavproxy pymavlink empy pexpect future
git clone --recursive https://github.com/ArduPilot/ardupilot ~/dev/ardupilot
cd ~/dev/ardupilot && ./waf configure --board sitl && ./waf copter
# then
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```
Also install **QGroundControl** (macOS .dmg) or Mission Planner for parameter editing.

> Bonus sanity check: ArduPilot has a *built-in* precision-landing simulator
> (`PLND_TYPE = 4` with the `SIM_PLD_*` parameters). Use it on day 1 to confirm the
> ArduPilot side works **before** wiring in our own perception. If SITL won't precision-land
> with its own synthetic target, the problem is parameters, not our code.

---

## Tier 1 — ArduPilot SITL + Gazebo Harmonic + ROS 2 Jazzy, on Linux

**When:** once Tier 0 closed-loop landing works and you need photorealistic imagery,
physical platform dynamics, UWB simulation, and terrain.

**Stack (all current LTS — verify versions at setup time):**

| Layer | Choice | Why not the alternative |
|---|---|---|
| OS | **Ubuntu 24.04 LTS (Noble)** | 20.04/22.04 are EOL or nearly |
| Middleware | **ROS 2 Jazzy** | ROS 1 Noetic is EOL (May 2025). ROLAND is ROS 1 — accept that you're porting, not reusing |
| Simulator | **Gazebo Harmonic** (`gz-sim`, LTS) | Gazebo Classic EOL Jan 2025. Harmonic is what `ardupilot_gazebo` targets now |
| Autopilot | **ArduPilot SITL** + `ardupilot_gazebo` plugin | **Not PX4.** ROLAND uses PX4; our hardware is ArduPilot. Matching the hardware matters more than matching the paper |
| Bridge | `ros_gz_bridge`, `mavros` (ROS 2) or plain `pymavlink` | Use plain `pymavlink` if you don't otherwise need ROS — it's much less to maintain |

**Do you actually need ROS 2 at all?** Honestly: *maybe not for Phase 1.* Gazebo Harmonic
can talk to ArduPilot SITL directly via the `ardupilot_gazebo` plugin, and publish camera
frames on `gz` topics that a plain Python script can subscribe to. ROS 2 earns its place
when you add the UWB plugin, the ground vehicle, RViz visualisation, and `rosbag2` logging —
i.e. for the moving-platform work. **Add ROS 2 when you feel the pain, not before.**

### Where to run Linux — settled ✅ (revised 2026-08-14)

| Machine | Role |
|---|---|
| **M1 MacBook**, 16 GB, ~60 GB free — Hari's, and the only one he has | Tier 0 natively; **and a UTM VM for Tier 1 development** |
| **Linux Mint + 40-series NVIDIA GPU** — teammate's | Tier 1 at full frame rate: real experiments, the numbers that go in the report |

**Yes, run Gazebo in a VM on the M1 — for development.** I initially advised against this;
that was wrong for this workflow. The reasoning that overrides it: **Hari has only the Mac,
and needing a teammate's machine for every iteration is a crippling development loop.**
5–15 fps under software rendering is entirely sufficient to answer *"does my code work?"*,
which is the question being asked 95 % of the time. Push to GitHub, `git pull` on the GPU
machine, and re-run there when you need speed or real numbers.

- **UTM** (free, Apple Virtualization) + Ubuntu 24.04 **arm64** desktop
- Allocate 8 GB RAM, 6 CPUs, 40 GB disk (~30 GB actual). ⚠️ Watch the Mac's 60 GB free
- 💡 If Gazebo is unstable on software rendering, force the older render engine:
  `gz sim --render-engine ogre`
- Architecture difference (arm64 VM vs x86 Mint) doesn't matter — everything we write is
  Python and config; only the `ardupilot_gazebo` plugin is compiled, and it builds on both

**Still avoid Gazebo natively on macOS.** Harmonic has a Homebrew formula, but
`ardupilot_gazebo` isn't supported there and plugin/rendering issues will eat weeks.

⚠️ **Linux Mint note:** Mint 22.x is built on Ubuntu 24.04, so Gazebo Harmonic installs
normally. ROS 2 Jazzy's apt repo keys off the Ubuntu codename — on Mint, set it to `noble`
manually instead of letting it auto-detect.

### What makes the two-machine split work

**`frame_source.py`** — a swappable interface with three implementations:
`SyntheticSource` (Tier 0, Mac) · `GazeboSource` (Tier 1) · `PiCameraSource` (hardware).
Everything downstream — detection, fusion, MAVLink — is written once and never touched
again. Build this on the first commit; retrofitting it later is painful.

---

## Tier 2 — Hardware-in-the-loop and real flight

Real Pi 4B + real camera + real Matek H743, but with SITL or a bench rig standing in for
flight. Then tethered → LOITER-observe → LAND, per the senior's methodology (§01, ch. 4).
This is where rolling shutter, vibration, and real latency finally show up.

---

## What about MATLAB / Simulink?

**Use it for one specific thing, not as your simulator.**

| Good for | Bad for |
|---|---|
| Designing and tuning the **EKF** (Sensor Fusion & Tracking Toolbox): covariance tuning, observability analysis, NEES/NIS consistency checks | Rendering AprilTags / camera imagery |
| Clean, report-quality plots of estimator error, Monte-Carlo runs | Running the actual ArduCopter firmware |
| Control-loop analysis of the descent controller (step response, damping — directly relevant to the `PLND_ACC_P_NSE` underdamping issue) | Producing anything that transfers to `PLND_*` parameters or the Pi |
| Cheap to justify in a report; examiners like it | Being reproducible by a teammate without a licence |

**Recommendation:** if Amrita has campus MATLAB licences, use Simulink as a **side channel
for the EKF chapter** of the report — design the filter there, prove it converges, then
implement the same equations in Python/C++ for the real system. Do **not** make MATLAB the
primary simulation, and do not spend time on MATLAB↔ArduPilot co-simulation.

---

## Recommended sequence — **strictly serial, sim before hardware**

Decision (2026-08-14): **no hardware work, no procurement, and no beacon work until a
drone lands itself in simulation.** The official Jul–Nov timeline is treated as nominal.

```
Stage 1   ArduPilot SITL on macOS. Drone flies. Learn modes, params, logs.
          Gate: PLND_TYPE=4 + SIM_PLD_* → SITL precision-lands with its own target
Stage 2   Synthetic camera + our AprilTag pipeline + closed loop.        ← still Mac-only
          Gate ⭐ drone lands on the board, repeatably, 20+ scripted runs with mean/σ
Stage 3   Gazebo Harmonic. Real imagery and physics.        Mac VM for dev · GPU box for real runs
          Gate: same pipeline, Gazebo camera, same landing performance
Stage 4   Beacon (UWB) in sim — port ROLAND's UwbPlugin; target found with marker hidden
Stage 5   Moving platform in sim
Stage 6   Hardware — salvaging parts where possible
```

**Why this order:** stages 1–2 need no Gazebo, no ROS, no VM, and no money, and they teach
the domain knowledge (MAVLink, flight modes, `PLND_*`) that makes Gazebo worth having.
Starting at Gazebo means weeks of toolchain pain before learning anything about precision
landing. And a working simulation is the evidence that justifies spending on parts.

Step-by-step instructions live in **`07-sim-setup.md`**.

---

## Open decisions

- ✅ Linux for Tier 1: teammate's Linux Mint + 40-series GPU; UTM VM on the Mac for dev
- ✅ MATLAB: available to all three — reserved for the EKF work (stage 4)
- ✅ ROS 2: **not needed** for stages 1–2; add at stage 3 for `ros_gz_bridge` only
- ⬜ Exact Gazebo world content (surface texture, lighting, obstacles) — stage 3

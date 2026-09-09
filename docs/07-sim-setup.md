# 07 — Simulation Setup Guide (hands-on)

**For someone who has never used a robotics simulator.** Read the stage you're on; ignore
the rest until you get there.

**Governing rule: simulation first, hardware last.** No parts get bought until a drone
lands itself in simulation. See `CLAUDE.md` §4.

---

## The five stages

| Stage | What works at the end | Where it runs | Gazebo? | ROS? |
|---|---|---|---|---|
| **1** | A simulated drone flies, you can command it, you understand ArduPilot's modes and parameters | 🖥️ Mac, native | ❌ | ❌ |
| **2** | ⭐ **The drone lands itself on the AprilTag board** — full perception pipeline, closed loop | 🖥️ Mac, native | ❌ | ❌ |
| **3** | Same thing, but with real rendered imagery and real physics | 🐧 Linux (Parallels VM on Hari's Mac for dev · friend's GPU for speed) | ✅ | ✅ |
| **4** | Beacon (UWB) fused in — target still found when the marker is hidden | 🐧 Linux | ✅ | ✅ |
| **5** | Landing on a *moving* platform | 🐧 Linux | ✅ | ✅ |
| — | *then* hardware, salvaging parts | | | |

**Why stages 1–2 have no Gazebo:** Gazebo is the part that eats weeks of setup. Stages 1–2
give you a working, closed-loop precision landing on your Mac with none of it — and they
teach you MAVLink, ArduPilot modes, and the `PLND_*` parameters, which you need before
Gazebo is any use to you. Going to Gazebo first means three weeks of toolchain pain and
zero understanding of the actual problem.

---

# Stage 1 — Make a simulated drone fly (Mac, native)

## What the pieces are

| Piece | What it actually is |
|---|---|
| **ArduPilot SITL** | *Software In The Loop.* The **real ArduCopter firmware**, compiled for your Mac instead of a flight controller chip. Same code that would fly the real drone. It simulates the physics itself |
| **MAVLink** | The protocol autopilots speak. Every message (`LANDING_TARGET`, `HEARTBEAT`, …) goes over it |
| **MAVProxy** | A command-line ground station. Type `mode GUIDED`, `arm throttle`, `takeoff 10` |
| **QGroundControl** | A graphical ground station — map, parameter editor, flight modes. Nicer for browsing the ~1200 ArduPilot parameters. **GCS of record for this project** (decided 2026-09-09 — cross-platform, unlike Mission Planner which is Windows-only) |
| **pymavlink** | The Python library for speaking MAVLink. Our pipeline uses this to send `LANDING_TARGET` |

Mental model:

```
   ArduCopter firmware (SITL)  ←── MAVLink over UDP ──→  MAVProxy / QGroundControl
        + built-in physics                              (you, giving commands)
                    ▲
                    └──────────── MAVLink ────────────  our Python scripts
```

In the real drone the same picture holds: SITL is replaced by the Matek H743, and UDP is
replaced by a UART cable to the Raspberry Pi. **That is why this transfers.**

## Prerequisites — status on this machine

- ✅ Xcode Command Line Tools / clang 16
- ✅ Python 3.13 + `.venv/` in the project with `pymavlink`, `MAVProxy`, `opencv-contrib-python`
- ✅ ArduPilot source cloned to `~/Documents/gitClone/ardupilot`
- ⬜ QGroundControl — download the macOS `.dmg` from qgroundcontrol.com

> **Note on Python:** the venv deliberately uses **Python 3.13**, not the system 3.14.
> MAVProxy/pymavlink wheels lag the newest Python release. Always work inside `.venv`.

## Build and run

```bash
cd ~/Documents/gitClone/ardupilot && ./waf configure --board sitl && ./waf copter
```

First build takes 10–20 minutes. Then:

```bash
cd ~/Documents/gitClone/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550
```

> ⚠️ **Do not use `--map` or `--console`.** They need wxPython, which is painful to install
> on Apple Silicon — **confirmed missing on this machine** (`MAVProxy` imports fine, `wx`
> does not). Use QGroundControl for the map instead — connect it to `UDP 127.0.0.1:14550`
> and it finds the vehicle automatically.

**Alternative if `sim_vehicle.py` misbehaves — run the binary directly (verified working):**

```bash
cd ~/Documents/gitClone/ardupilot && build/sitl/bin/arducopter --model quad --speedup 5 --defaults Tools/autotest/default_params/copter.parm
```

It prints `Waiting for connection ....` and listens on **TCP 127.0.0.1:5760**. Point
QGroundControl or a `pymavlink` script at that. This is the leanest possible setup and is
what our own scripts will use.

You'll land in a `STABILIZE>` prompt. First flight:

```
mode GUIDED
arm throttle
takeoff 10
```

The altitude should climb to 10 m. Then `mode LAND` and watch it come down.
**When that works, Stage 1 is done.** Nothing was harmed and you now have a flying drone.

## What to actually learn in Stage 1

Don't rush past this — it's the domain knowledge everything else rests on.

- [ ] Flight modes: `STABILIZE`, `ALT_HOLD`, `LOITER`, `GUIDED`, `LAND`, `RTL`.
      **Why `LAND` matters: ArduPilot only applies precision-landing corrections in LAND mode.**
- [ ] Reading and setting parameters (`param show PLND*`, `param set PLND_ENABLED 1`)
- [ ] Saving and loading a `.param` file — this is how the whole config gets version-controlled
- [ ] Where the DataFlash `.BIN` logs land, and how to open one in MAVExplorer
      (`.venv/bin/MAVExplorer.py`)
- [ ] Arming checks: what they refuse and why. **Never learn to disable these reflexively**
- [ ] Failsafes and geofence (`FENCE_*`) — set them in sim so the habit is already there

## Stage 1 acceptance gate ⭐

**ArduPilot's own built-in precision-landing simulator lands the drone.**

```
param set PLND_ENABLED 1
param set PLND_TYPE 4          # 4 = SITL's internal simulated target
param set SIM_PLD_ENABLE 1
```

Take off, then `mode LAND`, and watch it correct toward the simulated target.

**If this does not work, stop and fix it before writing a single line of perception code.**
It proves ArduPilot's side of the chain is configured correctly, so that when our own
pipeline later fails, you know the fault is in *our* code.

### ⚠️ Gotcha found on this machine (2026-08-14)

`PLND_ENABLED` is an **enable flag**: while it is `0`, *every other* `PLND_*` parameter is
hidden and won't appear in a parameter list or a `param show`. Set `PLND_ENABLED 1` **first**,
then the family becomes visible. (Same pattern applies to many ArduPilot subsystems.)

### Verified SITL defaults — measured live, not copied from docs

| Parameter | SITL default | Thesis value | Meaning |
|---|---|---|---|
| `PLND_TYPE` | 0 (disabled) | **1** = companion computer | 1 = MAVLink `LANDING_TARGET` · 2 = IR-Lock · 4 = SITL internal target |
| `PLND_EST_TYPE` | 1 | 1 | estimator type |
| `PLND_ACC_P_NSE` | **2.5** | *(left at default)* | 🎯 **The knob the thesis says to reduce.** Higher = trusts vision more aggressively = underdamped descent. This is our baseline to sweep down from |
| `PLND_LAG` | 0.02 s | **0.25 s** | pipeline latency compensation. The thesis's 0.25 s is 12× the default — because a Pi doing AprilTag detection is *slow* |
| `PLND_STRICT` | 1 | 1 | strict measurement acceptance |
| `PLND_ALT_MIN` | 0.75 m | — | below this, stop applying corrections |
| `PLND_ALT_MAX` | 8.0 m | — | above this, ignore the target |
| `PLND_TIMEOUT` | 4.0 s | — | ⭐ target-lost timeout → **fallback ladder rung L3** |
| `PLND_RET_MAX` | 4 | — | ⭐ max retry attempts → **rung L4** |
| `PLND_RET_BEHAVE` | 0 | — | ⭐ what to do on retry (climb / go-around) |
| `PLND_CAM_POS_X/Y/Z` | 0, 0, 0 | Z = −0.05 m | camera offset from CG |
| `SIM_PLD_ALT_LMT` | 15 m | — | simulated target's max detection altitude |
| `SIM_PLD_DIST_LMT` | 10 m | — | simulated target's max detection distance |

> ✅ **Good news for the failure-mode work:** `PLND_TIMEOUT`, `PLND_RET_MAX` and
> `PLND_RET_BEHAVE` confirm that rungs **L3 (abort and climb to reacquire)** and
> **L4 (give up precision, plain GPS LAND)** of our fallback ladder already exist in
> ArduPilot. We configure and demonstrate them rather than writing them.
> See `05-risks-and-failsafes.md` §B1.

---

# Stage 2 — Precision landing with a synthetic camera (Mac, native)

*Detail to be written when Stage 1 passes.* The shape of it:

SITL has no camera. So we render one. The tag board's geometry is known exactly, and SITL
tells us the vehicle's exact pose — so a Python script projects the board into a virtual
camera each frame and hands the image to our detection pipeline, which sends
`LANDING_TARGET` back to SITL.

```
 ArduCopter SITL ──pose──► synthetic camera ──image──► our pipeline ──┐
        ▲                                                             │
        └──────────────── MAVLink LANDING_TARGET ─────────────────────┘
```

Components to build, in order:
1. `sim/board.py` — generate the tag board (24 cm ID 0 centre + four 8 cm IDs 1–4 at ±0.22 m)
   as both a texture and a print-ready PDF
2. `sim/synthetic_camera.py` — pose → 640×480 image via `cv2.warpPerspective`
3. `src/frame_source.py` — **the swappable interface**: `SyntheticSource` / `GazeboSource` /
   `PiCameraSource`. Everything downstream is written once and never changed again
4. `src/detector.py` — tag36h11 detection + per-tag `solvePnP` (`SOLVEPNP_ITERATIVE`)
5. `src/fusion.py` — quality-weighted multi-tag fusion + disagreement metric
6. `src/mavlink_out.py` — `LANDING_TARGET` sender (**boot-relative µs**, non-zero angular size)
7. `sim/run_landing.py` — the loop that ties it together
8. `sim/campaign.py` — run N landings from randomised offsets, log touchdown error

> 💡 **Library decision (verified on this machine):** use **`cv2.aruco` with
> `DICT_APRILTAG_36h11`**, not `pupil-apriltags`. OpenCV 5.0 supports the tag36h11 family
> natively with `CORNER_REFINE_APRILTAG`. This removes a dependency *and* sidesteps the
> y-up/y-down sign trap the thesis documents — everything stays in OpenCV conventions.

**Stage 2 acceptance gate ⭐⭐:** the drone takes off, flies to an offset position, enters
LAND, and touches down within a few centimetres of the board centre — repeatably, over 20+
scripted runs, with a mean/σ you can put in the report.

> 💡 **Future idea, not yet started (2026-09-09):** Stages 1–2 are exactly the part of this
> project that's headless-friendly — ArduPilot SITL + Python, no Gazebo, no GPU, no GUI.
> Once this code exists, it's a good candidate for a GitHub Actions workflow that runs the
> SITL + synthetic-camera landing campaign on every push, so a `git pull` on the Linux side
> starts from already-validated code rather than something that might be broken. Deliberately
> **not** in scope: running actual Gazebo (Stage 3+) in CI — GitHub-hosted runners have no
> GPU, and a self-hosted runner is more setup/maintenance than this project needs right now.
> Worth running that Stage 1–2 workflow on **both x86_64 and arm64 runners** when it exists
> — Hari's dev VM is arm64, teammates are on x86_64, and cross-arch build differences are
> already a known risk (see the architecture note above). See `04-roadmap-checklist.md`
> Stage 0.

---

# Stage 3 — Gazebo (Linux)

*Detail to be written when Stage 2 passes.* Notes captured now so they aren't lost:

**Stack:** **Ubuntu (Parallels VM on Hari's M1 Mac) + Gazebo + ArduPilot SITL +
`ardupilot_gazebo` plugin + QGroundControl.** Gazebo and ROS 2 Humble are already installed
in that VM; ArduPilot SITL, the `ardupilot_gazebo` plugin, and QGroundControl still need
adding.

> 🟡 **ROS 2 is an open question, not a settled part of this stage** — see
> `06-open-questions.md` Q11. The professor asked for ROS 2, but nothing in the pipeline
> technically needs it: ArduPilot↔Gazebo goes through `ardupilot_gazebo` directly, and our
> pipeline↔ArduPilot goes through `pymavlink`/MAVLink either way. The one place ROS 2 would
> normally help is getting camera frames out of Gazebo (`ros_gz_bridge`) — **default plan
> is to skip that and use Gazebo's own `gz-transport` Python API to read the camera topic
> directly, no ROS 2 involved.** Confirm the actual scope of the requirement with the guide
> before starting this stage; if ROS 2 turns out to be mandatory, swap in `ros_gz_bridge`
> for the camera feed — nothing else changes.

**Two machines, one repo:**
- **Hari's Parallels Ubuntu VM (22.04 arm64, 6 vCPU / 12 GB RAM):** for *writing and
  checking* code. **Gazebo Harmonic** (`gz-harmonic`, gz sim 8.15.0) + **ROS 2 Humble
  Desktop** are already installed — Harmonic specifically because no arm64 build of Gazebo
  Classic exists, not a free choice. Whatever fps software rendering gives is fine for
  "does this work?" — check available RAM/CPU/disk allocation before assuming headroom,
  since it shares the M1's ~60 GB free.
  - ⚠️ **Verified rendering bug + fix:** Ogre2 + the VM's GPU passthrough (virgl, shows as
    "Apple M1 (Compat)") crashes the Gazebo GUI during mipmap generation. Fix:
    `LIBGL_ALWAYS_SOFTWARE=1` (forces CPU/llvmpipe rendering), set permanently in
    `~/.bashrc`. Staying on Ogre2 (not switching to the older Ogre1) despite the CPU cost —
    accept ~5–15 fps and no visual polish, priority is functional sim over graphics quality.
  - ⚠️ **Missing package:** `ros_gz_sim` (the launch wrapper) isn't published for
    arm64/Humble via apt, even though `ros-humble-ros-gz-bridge`/`-interfaces`/`-image` are.
    Options when this is actually needed: build `ros_gz_sim` from source
    (github.com/gazebosim/ros_gz, humble branch), or launch `gz sim` directly and bridge
    topics manually.
  - 🟡 **Unmeasured risk, flagged for later:** the software-rendered Gazebo GUI and our
    vision-model inference will compete for the same CPU cores on a 6-vCPU VM. Benchmark
    rendering fps vs. inference latency *separately* before assuming the combined pipeline
    is fast enough — don't discover this contention mid-integration.
- **Friend's Linux Mint + 40-series GPU:** full frame rate, real experiments, the numbers
  that go in the report. `git pull` and run.
  - 🟡 **Architecture note:** this VM is arm64; the teammate's Linux Mint/GPU machines are
    x86_64. Expect native-build differences (compiled dependencies, prebuilt binaries) —
    don't assume something that works in the VM builds identically there, or vice versa.

**This split is exactly why `frame_source.py` matters** — the same pipeline code runs
against the synthetic camera on your Mac and the Gazebo camera on either Linux machine,
with one line changed.

⚠️ **Linux Mint note:** ROS 2's apt repo keys off the Ubuntu codename. On Mint you may need
to set it manually rather than letting it auto-detect (confirm against whichever ROS 2
distro Mint's base Ubuntu version actually supports — don't assume it matches Humble).
Gazebo's own repo installs normally.

---

# Stage 4 — Beacon in simulation

*Deferred by decision (2026-08-14): no beacon work until a drone lands itself in sim.*
When we get here: port ROLAND's `gazebosensorplugins/src/UwbPlugin.cpp` from Gazebo Classic
to `gz-sim`, then fuse range measurements alongside vision. See `02-roland-reference.md` §2.

# Stage 5 — Moving platform in simulation

Animate the board, add a ground vehicle model, enable the velocity estimator and lag
compensation, port ROLAND's tracking/landing state machine.

# Stage 6 — Hardware

Only once stages 1–5 work. Parts to be salvaged where possible. BOM discussion parked —
see `04-roadmap-checklist.md`.

---

## Common beginner questions

**Do I need ROS?** Not for stages 1–2, and per the current default plan, not for stage 3
either — `gz-transport` can pull camera images out of Gazebo directly, without ROS 2. ROS 2
only enters the picture if the guide confirms her requirement means something specific we'd
otherwise miss (`06-open-questions.md` Q11). Don't add it speculatively either way.

**Is SITL "real"?** The autopilot code is 100% real — same C++ that runs on the flight
controller. What's simulated is the physics, sensors, and (in stage 3) the camera. This is
the standard way autopilot software is developed, including by ArduPilot's own developers.

**Will sim results transfer to hardware?** The control logic, parameters, and MAVLink
integration transfer very well. What doesn't: vibration, rolling-shutter artifacts, real
lighting, actual Raspberry Pi frame rate. That's what hardware testing is for — but you'll
arrive there with the software already correct.

**Why not just use MATLAB?** It can't run ArduCopter firmware and can't render AprilTags.
It *is* the right tool for designing the EKF later (stage 4) — see `03-simulation-plan.md`.

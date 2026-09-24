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
- ✅ QGroundControl installed, connects over UDP, joystick calibrated and flying (2026-09-09)

> **Note on Python:** the venv deliberately uses **Python 3.13**, not the system 3.14.
> MAVProxy/pymavlink wheels lag the newest Python release. Always work inside `.venv`.

## Build and run

```bash
cd ~/Documents/gitClone/ardupilot && ./waf configure --board sitl && ./waf copter
```

> ⚠️ **Verified gotcha (2026-09-09):** on a fresh Homebrew Python (system `python3`, not
> the project `.venv`), `./waf copter` fails silently partway through with a one-line
> message like `you need to install empy with '...'` (then `pexpect`, one at a time) and
> exits 0 — easy to mistake for success since there's no "BUILD FAILED". Fix once,
> up front, with ArduPilot's own official list
> (`Tools/environment_install/install-prereqs-mac.sh`):
> ```bash
> python3 -m pip install --break-system-packages setuptools lxml matplotlib pymavlink MAVProxy pexpect geocoder flake8 junitparser empy==3.3.4 dronecan
> ```
> `--break-system-packages` is needed because Homebrew's Python is externally-managed
> (PEP 668). After this, `./waf copter` compiles cleanly.

First build takes 10–20 minutes. Then:

```bash
cd ~/Documents/gitClone/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550
```

> ⚠️ **Do not use `--map` or `--console`.** They need wxPython, which is painful to install
> on Apple Silicon — **confirmed missing on this machine** (`MAVProxy` imports fine, `wx`
> does not). Use QGroundControl for the map instead.

> ✅ **Verified working end-to-end (2026-09-09):** `sim_vehicle.py` with `--out=udp:...`
> is the route to use — QGroundControl **autoconnects over UDP** with zero manual link
> setup, unlike running `build/sitl/bin/arducopter` directly, which only opens a TCP port
> QGC won't auto-detect (that route was tried and dropped — don't reach for it). A joystick
> plugged in and calibrated under QGC's **Joysticks** settings tab drives the vehicle
> directly over MAVLink — also verified working.

You'll land in a `STABILIZE>` prompt. First flight:

```
mode GUIDED
arm throttle
takeoff 10
```

The altitude should climb to 10 m. Then `mode LAND` and watch it come down.
**When that works, Stage 1 is done.** Nothing was harmed and you now have a flying drone.

> ✅ **Actually verified end-to-end (2026-09-09), by Hari, live:** armed, took off, landed.
> Two real gotchas hit and fixed along the way — both worth knowing before they cost
> anyone else the same afternoon:
>
> 1. **`AP: Arm: Throttle (RC3) is not neutral`** — arming refused. With no real RC
>    transmitter or hardware joystick connected, ArduPilot has no confirmed "throttle is
>    at idle" signal and correctly refuses to arm. Fix: in the MAVProxy console,
>    `rc 3 1000` before arming (channel 3 = throttle in ArduCopter's RC convention;
>    forces it to the low/idle value ArduPilot wants to see).
> 2. **QGC's Virtual Joystick doesn't fix this on its own** — it defaults to throttle
>    *centered*, not at the bottom, so it reads as "not neutral" too, and it actively
>    fights typed MAVProxy commands if left on. **Turn it off** (General Settings →
>    Virtual Joystick) for typed-command flying; revisit it later, on its own, for actual
>    manual-override testing.
> 3. **Which terminal window to type in, if `sim_vehicle.py` opened two:** the one
>    titled/running `sim_vehicle.py` with a live command prompt (`STABILIZE>`, `GUIDED>`,
>    …) is MAVProxy's console — type there. The other window is just the raw ArduCopter
>    process's log stream (`Waiting for connection...`, `bind port ...`) — no prompt,
>    nothing to type there.

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

> ✅ **Actually passed (2026-09-11), by Hari, live — after two real bugs traced through
> ArduPilot's own source** (not guessed — `libraries/AC_PrecLand/AC_PrecLand.cpp`,
> `AC_PrecLand_SITL.cpp`, `SIM_Precland.cpp`), both baked into
> `sim/precision_landing.parm` so they can't recur:
>
> 1. **`SIM_PLD_LAT`/`LON`/`HEIGHT` default to `(0,0,0)`.** ArduPilot correctly treats
>    that as "no beacon location defined" and the target can never be found at any
>    altitude — set them to the SITL home position (at the time this was diagnosed,
>    ArduPilot's default `-35.363262, 149.165237`, height 0; changed 2026-09-15 to
>    Amrita Vishwa Vidyapeetham, Bengaluru — see `sim/run_sitl.sh` and
>    `sim/precision_landing.parm` for the current values, always keep them matched).
> 2. **`AC_PrecLand::construct_pos_meas_using_rangefinder()` requires either a real
>    rangefinder (none configured — `RNGFND1_TYPE` unset) or the SITL backend's simulated
>    distance-to-target to be nonzero**, which is only true when `SIM_PLD_OPTIONS` bit 0
>    ("Enable target distance") is set. Without it, `PrecLand: Target Found` can never
>    print, full stop, regardless of beacon position or flight altitude.
>
> Confirmed working via the actual log sequence: `PrecLand: Target Found` →
> (~2 s EKF settle, `PLND_EST_TYPE`'s Kalman filter needs `EKF_INIT_TIME_MS`=2000 ms of
> continuous good detections) → `PrecLand: Init Complete` → clean landing with **no**
> `PrecLand: Failsafe Measures` (which had appeared in every failed attempt before this).
>
> **Load `sim/precision_landing.parm` automatically** so this never has to be retyped by
> hand again (retyping it was exactly how the first few attempts silently failed —
> forgetting one param with no obvious error pointing at which one). Use
> `./sim/run_sitl.sh` (see `sim/README.md`) — it resolves this file's path and ArduPilot's
> clone location automatically, no per-machine path editing, confirmed working across
> different usernames/OSes on the team already.
>
> ✅ **Full AUTO-mode mission validated (2026-09-11)** — not just LAND from GUIDED, a
> complete `sim/test_mission.waypoints` run (takeoff → 3-waypoint square, ~30 m out →
> land), exercising synopsis sub-objective 7 ("autonomous waypoint navigation") together
> with precision landing in one flight: `Target Found`/`Init Complete` near home on
> climb-out → **`Target Lost`** correctly once out of the beacon's range mid-mission →
> **`Target Found`**/`Init Complete` again on the return leg → clean touchdown, auto-disarm.
> That lost→reacquired cycle is real evidence the state machine degrades and recovers
> correctly, not just a single lucky lock. One more fix was needed to get here:
> `AUTO_OPTIONS = 2` (`AllowTakeOffWithoutRaisingThrottle`) — AUTO mode's mission-takeoff
> (unlike GUIDED's direct `takeoff` command) requires a physical throttle raise as a
> safety confirmation and disarms a few seconds in if it never sees one; with no real
> stick in SITL, this bit has to be set explicitly. Now baked into
> `sim/precision_landing.parm` alongside the rest.
>
> **Stage 1 is genuinely complete.** Both gates pass: basic flight, and ArduPilot's own
> precision-landing simulator, now also proven across a real waypoint mission rather than
> just a straight-up takeoff/land. Stage 2 — our own AprilTag perception replacing
> `SIM_PLD` — is next.

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

**Started 2026-09-22.** ArduPilot SITL + Gazebo Harmonic + the `ardupilot_gazebo` plugin
are now working end to end (arm, takeoff, fly in the Gazebo GUI) — on the Mac Mini (see
below), not the Parallels VM. QGroundControl itself is **not** installed on either Linux
machine — see the GCS architecture decision below for why.

**Stack:** Gazebo Harmonic + ArduPilot SITL + `ardupilot_gazebo` plugin, on Linux (Parallels
VM or the Mac Mini, see below) + **QGroundControl on Hari's own Mac**, connected over the
LAN — not installed on the Linux side at all.

> 💡 **GCS architecture decision (2026-09-22):** QGroundControl runs on Hari's Mac only.
> Whichever machine is doing the actual Gazebo/SITL work (Mac Mini, or later the
> teammate's Linux box) sends MAVLink to that Mac's LAN IP instead of running its own GCS.
> This is what `sim/run_gazebo.sh`'s `GCS_IP` variable is for — point it at whichever
> machine is running QGC, and the same script works unchanged regardless of which Linux
> box is doing the simulation. Goal: "work on the Mac with QGC normally; when Gazebo work
> is needed, use whichever Linux machine is available, and it just works."

## A third machine: the Mac Mini (native Ubuntu, real GPU)

2018 Intel Mac Mini i7 (i7-8700B, 6-core/12-thread, 62GB RAM, 327GB free disk), running
**native Ubuntu 22.04** (not virtualized) — SSH-accessible, added 2026-09-22. Its Intel
UHD 630 gives **real GPU rendering via Mesa's `iris` driver** (confirmed:
`glxinfo | grep "OpenGL renderer"` → `Mesa Intel(R) UHD Graphics 630 (CFL GT2)`, OpenGL
4.6) — a genuine upgrade over the Parallels VM's forced `llvmpipe` software rendering
(structural limitation: no arm64 GPU passthrough exists in Parallels on Apple Silicon at
all). Docker was found pre-installed but masked/disabled on this machine — consistent with
the decision below not to use Docker anyway.

Checking `glxinfo`/GPU capability over a bare SSH session fails with "unable to open
display" — there's no X display in a non-interactive SSH shell. Workaround: this machine
has an actual logged-in GNOME session (`gdm`), so `export DISPLAY=:1` and
`export XAUTHORITY=/run/user/1000/gdm/Xauthority` (adjust the UID) targets that real
session's display instead. The same trick is needed to launch Gazebo's GUI over SSH; if
actually sitting at the machine, DISPLAY is already set correctly and none of this matters.

**Docker was considered and rejected** (2026-09-22) for running Gazebo+SITL+QGC — it
doesn't solve the actual constraint (GPU rendering; a container still needs the same host
GPU passthrough or the same software-rendering fallback already in use), no current
official ArduPilot/Gazebo Docker image exists for Gazebo Harmonic specifically, and
QGroundControl has no established Docker-run pattern either. See
`~/.claude/plans/all-done-setup-assuming-tidy-pelican.md` (if still present) for the full
research behind this call.

## Setup steps that actually worked (Mac Mini, Ubuntu 22.04)

```bash
# System deps (needs sudo -- run these yourself, not scriptable non-interactively)
sudo apt update
sudo apt install -y python3.10-venv libfuse2
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav \
    gstreamer1.0-gl
# (if apt complains about a package name, you likely hit a copy-paste line-wrap --
#  install one package per line instead)

# ArduPilot itself
git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

python3 -m venv ~/.venvs/ardupilot
source ~/.venvs/ardupilot/bin/activate
pip install empy==3.3.4 pexpect future pyyaml pymavlink MAVProxy

./waf configure --board sitl
./waf copter

# The ardupilot_gazebo plugin (official ArduPilot Gazebo bridge, not the old khancyr one)
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

Gazebo Harmonic itself was installed via the official apt repo (`packages.osrfoundation.org`,
`gz-harmonic` package) — already covered in the environment section below.

### Two real gotchas found getting this to actually fly

1. ⚠️ **`sim_vehicle.py -f gazebo-iris --model JSON` resolves which default params it
   *should* load (prints the correct `default_params_filename` dict:
   `["default_params/copter.parm", "default_params/gazebo-iris.parm"]`) but does NOT
   actually forward them to the `arducopter` binary for "external" (JSON/Gazebo) frames** —
   confirmed by checking the actual `RiTW: Starting ArduCopter : ...` command line, which
   had no `--add-param-file` at all. Without those files, `FRAME_CLASS`/`FRAME_TYPE` stay
   unset → `AP: Frame: UNSUPPORTED` → arming fails with `PreArm: Motors: Check frame class
   and type`, even right after a `-w` (wipe-eeprom) fresh boot — this is a real
   `sim_vehicle.py` gap for this ArduPilot version, not a stale-eeprom issue (ruled out by
   checking the eeprom file's own mtime matched the fresh run). **Fix: pass both files via
   `--add-param-file` explicitly yourself** — `sim/run_gazebo.sh` does this automatically.
2. ⚠️ **`mavproxy.py` isn't on `PATH` unless the venv is activated first** — obvious in
   hindsight, but easy to miss when launching non-interactively (e.g. via `nohup` or a
   script), since an interactive shell that's already `source`d the venv once won't notice
   it's missing. Fails with `[Errno 2] No such file or directory: 'mavproxy.py'`.
   `sim/run_gazebo.sh` activates `~/.venvs/ardupilot` itself for exactly this reason —
   `set +u` / `set -u` bracket the `source`, since venv activate scripts reference `$PS1`,
   which is unset under `set -u` in a non-interactive script.

## `sim/run_gazebo.sh`

New portable launch script (same design as `sim/run_sitl.sh`: env-var overrides, sensible
default clone locations, clear errors instead of silent failure) that launches Gazebo +
the plugin + SITL together, with MAVLink sent to a **required** `GCS_IP` (no silent
default — UDP `--out` is "send to this address," and a wrong guess drops every packet with
no error anywhere) *and* to localhost at the same time, so a GCS running on the Linux
machine itself also works without needing a separate flag:

```bash
GCS_IP=<IP of the machine running your GCS> ./sim/run_gazebo.sh
```

Must be run from a real graphical terminal session on the Linux machine (not a bare
non-interactive SSH command) — Gazebo's GUI needs a real `DISPLAY` to render into; the
script checks for this and fails with a clear message rather than hanging.

### GCS choice: QGC on Mac, Mission Planner (via Mono) on Linux

**QGroundControl's AppImage does not run on Ubuntu 22.04** (confirmed 2026-09-22, Mac
Mini) — it requires `GLIBC >= 2.38` / `GLIBCXX >= 3.4.32`; 22.04 ships older versions
(glibc 2.35). Not a quick fix, and this would likely bite any Ubuntu-22.04-class machine
(the friend's Linux box included), not just this one.

**Decision: QGroundControl stays on Hari's Mac** (works fine there, no change) — **Mission
Planner via Mono is the GCS for whichever Linux machine is doing the actual Gazebo work**
(mission/waypoint editing during testing), not QGC. This isn't a downgrade — Mission
Planner via Mono is already independently confirmed working end-to-end on Linux Mint
(`sim/README.md` §3b, 2026-09-11: connects, flies, precision-lands) — Mono's runtime is
far less version-sensitive across different Linux machines than a specific AppImage's
glibc target, making it the actually-portable choice for "clone this repo on any Linux
box and it works," not a fallback. Setup is the same three lines already in
`sim/README.md` §3b (`sudo apt install mono-complete`, download+extract, `mono
MissionPlanner.exe`, connect UDP 14550 on `127.0.0.1`) — no changes needed there.

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

## The real AprilTag board + downward camera (2026-09-23/24)

Everything above got a *stock* Gazebo world flying (`iris_runway.sdf`, no custom board or
camera). The next real gap was representing our own multi-scale board and a downward
camera in Gazebo itself, instead of `sim/synthetic_camera.py`'s stand-in — see
`sim/README.md` §5 for the actual run commands; this section is the story of what was
built and the real bugs found getting it working, not a repeat of the commands.

**What was added:** `sim/gazebo_models/precision_landing_board/` (a static box textured
with a composite image generated by `sim/board.py`'s `render_board_texture()` — the same
tag-placement code `synthetic_camera.py` already used, so there's exactly one place tag
geometry math lives, not two copies that could drift apart), `sim/gazebo_models/
iris_downward_camera/` (the plugin's own `iris_with_ardupilot` plus one fixed, non-gimbaled
camera link), `sim/gazebo_worlds/precision_landing_world.sdf` (the stock world with those
two swapped in), and `src/gazebo_source.py` (a `BaseFrameSource` implementation reading the
camera's `gz-transport` topic directly — no ROS 2, per the standing default in
`06-open-questions.md` Q11).

### Real bugs found getting this to actually correct the landing, not just "run"

Getting Gazebo to *load* the board/camera was the easy part. Getting `PLND_TYPE=1` to
actually use the resulting `LANDING_TARGET` messages to correct a real 1m GPS-position
offset during landing took four separate real bugs, found in this order:

1. **`run_gazebo.sh` never opened the companion-port `--out`** (`udp:127.0.0.1:14540`) that
   `run_sitl.sh` already had — `run_landing.py` timed out on `wait_heartbeat` because
   literally nothing was ever sent to that port. `sim/run_sitl.sh` had this from the start;
   `run_gazebo.sh` was written separately and missed it. Fixed by adding the matching
   `--out`.
2. **Camera FOV was left at a placeholder 90°** (`1.5708` rad) instead of matching the
   senior's actual calibrated hardware (`fx=497.88` at 640×480 →  ~65.9°/`1.1490` rad, per
   `01-inherited-system.md:99`). At 90° FOV, ground footprint width = 2×altitude, so a 0.6m
   board was only ~19px wide in a 640px frame at 10m altitude — right at or below
   `cv2.aruco`'s default minimum-marker-size threshold, undetectable except very close to
   the ground. At the corrected 65.9° FOV the same board is ~30px at 10m, ~99px at 3m.
   Fixed in both `iris_downward_camera/model.sdf`'s `<horizontal_fov>` and
   `src/gazebo_source.py`'s `CAMERA_HORIZONTAL_FOV_RAD` — **these two must stay in sync
   manually**, nothing enforces it.
3. **`LANDING_TARGET`'s `time_usec` was time-since-Python-script-started, not the flight
   controller's boot-relative clock** (`src/mavlink_out.py`) — this is CLAUDE.md §7 item
   1's exact gotcha, reintroduced when `mavlink_out.py` was first written and never caught
   because the synthetic-camera Stage 2 testing happened to start the script soon enough
   after SITL boot that the mismatch stayed small. Once real Gazebo mission testing
   involved a longer time between SITL boot and script start, ArduPilot's staleness check
   silently discarded every single message as implausibly old — `send()` reported success
   every call, detections were happening, but the vehicle never once corrected laterally.
   Fixed by reading real `time_boot_ms` off the most recently received `ATTITUDE`/
   `LOCAL_POSITION_NED` message (both already stream on this link) via pymavlink's own
   per-type message cache, instead of `time.time() - script_start_time`.
4. **`GazeboSource.get_frame()` returned the cached frame by reference**, and
   `run_landing.py` draws detection overlays (`drawDetectedMarkers`, `drawFrameAxes`)
   directly onto whatever it's handed, in place. The main loop runs faster than the
   camera's 30Hz update rate, so `get_frame()` was often called twice between real Gazebo
   frames — the second call got back the first call's frame with debug graphics already
   drawn over the tag pattern, breaking detection. This produced an exact 1:1 detected/lost
   alternation in the logs at every altitude, not just near the ground. Fixed by returning
   `.copy()`.

Bugs 1 and 3 in particular are the kind that look like "it's just not working" with no
obvious symptom pointing at the cause — worth remembering the diagnostic approach (checking
the actual command line invoked, checking the actual message fields sent) rather than
guessing at ArduPilot param tuning first.

### VM path: built, works, abandoned by choice (2026-09-24)

The Parallels Ubuntu VM (§ above) hit a real Ogre2 rendering crash the Mac Mini never did:
**any textured camera sensor** (not specific to our board — reproduced on the plugin's own
stock `iris_runway.sdf` too) crashes with `Ogre::UnimplementedException` in
`GL3PlusTextureGpu::copyTo` while generating hardware mipmaps, when Ogre2 falls back to its
EGL headless-device path against this VM's virtio-gpu/virgl. **`LIBGL_ALWAYS_SOFTWARE=1`
alone does NOT fix this** — this stage's earlier note that it did was only ever verified
against physics, never an actual camera sensor. The real fix: run under `xvfb-run` (a
virtual X server), which gives Ogre2 a working GLX path instead, avoiding the crashing EGL
code path entirely. `run_gazebo.sh` auto-detects a missing `DISPLAY` and switches to this
mode, plus a lighter world (`sim/gazebo_worlds/precision_landing_world_vm.sdf`,
`sim/gazebo_models/lite_ground/`, `iris_downward_camera_vm/`) that trims only rendering cost
— the plugin's stock `runway` model's *collision* was always just a flat plane (checked by
reading its own `model.sdf`), its expense was purely a `airfield.dae` mesh plus four 2K PBR
texture maps for zero physics benefit, so swapping it costs nothing physics-wise. Confirmed
working end to end (real camera frames captured, tag pattern visible) — but still too slow
to be practically usable even with these optimizations, on this VM's hardware (Parallels on
Apple Silicon has no arm64 GPU passthrough at all, a structural limit, not something more
software tuning fixes). **Decision (2026-09-24): stick to the Mac Mini (or an equivalent
native-Linux, real-GPU machine) for Gazebo work going forward.** The VM-only files remain in
the repo, unused by default, in case this changes.

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

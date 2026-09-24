# Running the simulator

Quick reference. For the full story of how each of these settings was found (the real
bugs, the source-code tracing) see [`../docs/07-sim-setup.md`](../docs/07-sim-setup.md) —
this file is just the commands.

**Each person runs the full stack locally** (SITL + your own ground station), all on
`127.0.0.1`. This isn't one shared simulation — everyone gets their own independent flight.
That's deliberate: simplest to set up, nothing to get wrong across a network, and it's what
we've actually tested.

---

## 1. One-time setup

You need ArduPilot cloned and built for SITL, and this repo cloned, **on the machine you're
actually flying from** — your Mac, your teammate's Linux laptop, wherever.

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter
```

First build takes 10–20 minutes. `pymavlink` and `MAVProxy` need to be importable by
whatever Python runs `sim_vehicle.py` — a project `.venv` (as used on the Mac) or a plain
`pip install pymavlink MAVProxy` both work.

## 2. Start SITL, every time

```bash
./sim/run_sitl.sh
```

Run from anywhere inside this repo (or `cd` into it first). **No paths to edit** — the
script finds its own location and ArduPilot's automatically, checking (in order) an
`ARDUPILOT_DIR` you set, then `~/ardupilot`, then `~/Documents/gitClone/ardupilot`. If your
ArduPilot clone lives somewhere else, just export the variable once:

```bash
export ARDUPILOT_DIR=/wherever/you/put/ardupilot
```

This wraps the same `sim_vehicle.py` command as before, with `precision_landing.parm`
always loaded — that file carries every precision-landing and AUTO-mode fix already found;
without it you'll hit the same bugs (target never found, AUTO takeoff disarms itself) from
scratch. Change the ground-station port with `OUT_PORT=14551 ./sim/run_sitl.sh` if you ever
need to; anything else you pass through goes straight to `sim_vehicle.py`
(`./sim/run_sitl.sh --speedup 2`).

It also opens a second link on `COMPANION_PORT` (default **14540**) for
`src/mavlink_out.py` (Stage 2's `LANDING_TARGET` sender) — deliberately separate from
`OUT_PORT`/14550, since ArduPilot's SITL TCP serial driver only tracks one client per port
and silently drops a second connection with no heartbeat. Nothing to configure for this
unless you're running two companion scripts at once.

> **If you edit `precision_landing.parm` and the change doesn't seem to take effect,
> wipe SITL's persisted EEPROM.** Confirmed 2026-09-15: `--add-param-file` only sets
> DEFAULTS — it does NOT override a param ArduPilot has already saved to
> `<ardupilot clone>/eeprom.bin` from any earlier run. `PLND_TYPE`/`SIM_PLD_LAT`/`LON`
> silently kept loading Sep-11-era values for hours despite the file on disk being
> correctly edited, with no error anywhere pointing at the mismatch. Fix:
> `./sim/run_sitl.sh -w` (ArduPilot's own `--wipe-eeprom` flag) forces a genuinely clean
> load from the param file. Do this any time `.parm` changes matter and the observed
> behavior doesn't match what's in the file.

Wait a few seconds after boot before arming — GPS/EKF needs to settle. Arming too early
gives a harmless `PreArm: Need Position Estimate` that goes away on its own.

## 3a. Connect with QGroundControl

Just open QGC — it autoconnects to `UDP 14550` with nothing to configure.

## 3b. Connect with Mission Planner (via Mono, on Linux)

**Confirmed working end-to-end on Linux Mint, 2026-09-11** (Mono 6.8.0.105) — connects,
flies, precision-lands. ArduPilot's own `MissionPlanner` repo still notes "not all
functions are available on Linux," so don't expect full Windows parity, but core
flying/telemetry works fine.

```bash
sudo apt install mono-complete
```

`mono-complete` alone is enough on Ubuntu 22.04 — it's a meta-package that pulls in
everything needed. The individual `libmono-system-windows-forms4.0-cil` /
`libmono-winforms4.0-cil` / etc. packages from older guides aren't required (and may not
even resolve as separate packages on 22.04) — confirmed 2026-09-22 on the Mac Mini.

Download the latest Mission Planner zip from ArduPilot's firmware site, extract it, then:

```bash
mono MissionPlanner.exe
```

Then connect it the same way as QGC: UDP, port **14550**, on `127.0.0.1` (since SITL is
running on the same machine as Mission Planner in this setup).

> **If it hangs at "Connecting..." or times out:** before suspecting Mission Planner or
> Mono, check that SITL is actually still running — `ps aux | grep -i -E
> "sim_vehicle|arducopter|mavproxy"`. This was the actual cause the one time this looked
> broken (2026-09-11): SITL had exited and nothing was sending MAVLink at all, so *any* GCS
> would have timed out identically. Confirm SITL is up first — it's a much more common
> cause than a genuine Mission Planner/Mono bug.
>
> If SITL is confirmed running and it's still not connecting, get more detail with:
> ```bash
> MONO_LOG_LEVEL=debug mono MissionPlanner.exe
> ```

> **Connecting across two different machines instead** (e.g. Mission Planner on a laptop,
> SITL running elsewhere on the network) is possible but not what we've tested — you'd
> replace `127.0.0.1` in the `sim_vehicle.py --out=udp:...` command with the *listening*
> machine's actual LAN IP, and firewalls become a real variable. Only go this route if
> running SITL locally on the Mission-Planner machine genuinely isn't an option.

## 4. Fly the validated test mission

In the MAVProxy console (the terminal `sim_vehicle.py` is running in — the one with a
`STABILIZE>`/`GUIDED>` prompt, not the other SITL log window):

```
wp load <path to your clone of this repo>/sim/test_mission.waypoints
mode GUIDED
rc 3 1000
arm throttle
mode AUTO
```

> **Must be the full path — a relative one silently fails to resolve.** MAVProxy's working
> directory is wherever `run_sitl.sh` `cd`s into (your ArduPilot clone, so `--add-param-file`
> resolves correctly), not this repo — so `wp load sim/test_mission.waypoints` looks for a
> `sim/` folder inside your ArduPilot clone and doesn't find one. Confirmed 2026-09-15.

Expected: takeoff → a ~30 m square loop → precision-landing engages and locks
(`PrecLand: Target Found` → `Init Complete`) near home, correctly reports
`PrecLand: Target Lost` once out of the beacon's simulated range mid-mission, re-locks on
the way back, and lands cleanly with a normal auto-disarm. This exact sequence has been
confirmed working end to end — if you see something different, something's missing from
your setup (most likely: the `--add-param-file` wasn't picked up, or you're arming too
early after boot).

## Why `rc 3 1000`?

There's no real transmitter or joystick in this setup, so ArduPilot has no throttle signal
at all by default — and it refuses to arm unless it believes the throttle stick is at idle
(channel 3 = throttle, in ArduCopter's standard mapping). `rc 3 1000` manually injects that
"stick at minimum" value so the arm check passes. It's not needed once flying — GUIDED and
AUTO command thrust directly and ignore raw RC3 — it only matters at the moment of arming.

---

## 5. Gazebo — the real AprilTag board + downward camera (current stage)

Everything above (Stages 1–2, `run_sitl.sh`) uses a synthetic-camera stand-in — no real
rendering, no real Gazebo. This section is the actual Gazebo stage: a real board model, a
real downward camera, real rendered images going through the same detection/fusion code.

**Only tested on the Mac Mini (native Ubuntu 22.04, real GPU) as of 2026-09-24.** A
Parallels-VM path was also built and works (`GZ_WORLD`/Xvfb auto-detection in
`run_gazebo.sh`) but is too slow to be useful even after optimizing rendering cost — see
`../docs/07-sim-setup.md` Stage 3 for what was tried. **Decision: Gazebo work happens on
the Mac Mini (or an equivalent native-Linux machine with a real GPU) only, going forward.**

### One-time setup (native Ubuntu, real GPU)

```bash
sudo apt update
sudo apt install -y python3.10-venv libfuse2 \
    libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav \
    gstreamer1.0-gl

# ArduPilot itself (Stage 1's steps, if not already done)
git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
python3 -m venv ~/.venvs/ardupilot
source ~/.venvs/ardupilot/bin/activate
pip install empy==3.3.4 pexpect future pyyaml pymavlink MAVProxy opencv-contrib-python numpy
./waf configure --board sitl && ./waf copter

# The ardupilot_gazebo plugin (official ArduPilot repo, not the older khancyr fork)
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build
GZ_VERSION=harmonic cmake ..
make -j$(nproc)
```

Gazebo Harmonic itself (`gz-harmonic`, and the `python3-gz-transport13`/`python3-gz-msgs10`
apt packages `src/gazebo_source.py` needs) comes from the official
`packages.osrfoundation.org` apt repo — see `../docs/07-sim-setup.md` Stage 3 for the
one-time repo-add step if it's not already configured.

### Every time: run it

```bash
# Terminal 1 -- Gazebo + the plugin + ArduPilot SITL together
GCS_IP=<IP of whichever machine is running QGroundControl/Mission Planner> ./sim/run_gazebo.sh
```

Then, once armed and flying (same `wp load` / `mode AUTO` flow as Stage 1, using
`sim/test_mission_precision.waypoints` — see below):

```bash
# Terminal 2 -- the real vision pipeline, reading Gazebo's actual rendered camera
source ~/.venvs/ardupilot/bin/activate
python3 sim/run_landing.py --source gazebo
```

`--source gazebo` is the only thing that changed from Stage 2's synthetic-camera version —
`--source synthetic` (the default) still works unchanged, same `ArucoDetector`/`TagFusion`/
`LandingTargetSender` code either way, per `src/frame_source.py`'s whole point.

### Flying the AprilTag-only landing test

`sim/test_mission_precision.waypoints` is deliberately different from
`sim/test_mission.waypoints`: its final `NAV_LAND` waypoint is offset **1m** from the
board's real position (anchored to ArduPilot's default CMAC home coords,
`-35.3632620, 149.1652370`, matching this Gazebo world's `<spherical_coordinates>` — NOT
Amrita's coordinates, a mismatch that silently sends the mission to the wrong side of the
planet if you copy Stage 1's Amrita-based waypoint file here instead). GPS/mission nav
alone would touch down 1m off-target; a landing that actually centers on the tag proves the
AprilTag correction (not GPS) is what put it there — that's the actual point of this
mission, not just "fly somewhere and land."

```
wp load <full path to this repo>/sim/test_mission_precision.waypoints
mode GUIDED
rc 3 1000
arm throttle
mode AUTO
```

### Known real bugs already found and fixed here (don't re-debug these from scratch)

- **`run_gazebo.sh` was missing the companion-port `--out`** that `run_sitl.sh` already
  had — without it, nothing is ever sent to port 14540 at all, so `run_landing.py` times out
  waiting for a heartbeat. Fixed 2026-09-23.
- **Camera FOV was 90°, not the real hardware's ~65.9°** (`fx=497.88` at 640×480, per
  `../docs/01-inherited-system.md:99`) — the wider FOV spread the 0.6m board over too few
  pixels to detect except very close to the ground. Fixed in both
  `sim/gazebo_models/iris_downward_camera/model.sdf` and
  `src/gazebo_source.py`'s `CAMERA_HORIZONTAL_FOV_RAD` (must stay in sync between the two).
- **`LANDING_TARGET`'s timestamp was script-uptime, not the flight controller's
  boot-relative clock** (`src/mavlink_out.py`) — CLAUDE.md §7 item 1's exact gotcha.
  ArduPilot's staleness check silently discarded every message because they all looked
  implausibly old; `send()` reported success every time, but the vehicle never actually
  moved in response. Fixed by reading real `time_boot_ms` off the most recent
  `ATTITUDE`/`LOCAL_POSITION_NED` message instead.
- **`GazeboSource.get_frame()` returned its cached frame by reference, not a copy** — since
  `run_landing.py` draws detection overlays directly onto whatever it gets back, and the
  main loop runs faster than the camera's 30Hz update rate, this corrupted the cached frame
  with overlay graphics about half the time, causing an exact 1:1 detected/lost alternation
  regardless of altitude. Fixed by returning `.copy()`.

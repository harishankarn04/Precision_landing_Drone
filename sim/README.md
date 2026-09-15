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
sudo apt install mono-complete mono-runtime libmono-system-windows-forms4.0-cil \
    libmono-system-core4.0-cil libmono-winforms4.0-cil libmono-corlib4.0-cil \
    libmono-system-management4.0-cil libmono-system-xml-linq4.0-cil
```

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

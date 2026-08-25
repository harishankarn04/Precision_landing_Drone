# 05 — Drawbacks, Failure Modes, Fallbacks, and Missing Elements

Three sections:
**A.** what is missing / weak in the inherited system,
**B.** the failure modes and the fallback logic we must design,
**C.** project-level (non-technical) risks.

---

# A. Missing or weak elements

Ordered by *value per unit effort*. The first five are cheap and unblock everything else.

### A1. No ground-truth accuracy measurement 🔴 critical, cheap
The senior reported the **disagreement metric** (max pairwise distance between per-tag
centre estimates). The thesis is explicit that this measures *geometric self-consistency,
not absolute accuracy* — any error that affects all tags identically (focal-length
miscalibration being the obvious one) is completely invisible to it.

**There is currently no number for "how far from the pad's centre does it actually land."**
That is the headline number a precision-landing project must report.

**Fix — a ground-truth protocol, defined before any landing tests:**
- Simulation gives it for free (Tier 0/1). Use sim for the large-N statistics.
- Hardware: print a target board with a concentric-ring scale (5 cm rings) around the
  centre; photograph each touchdown from a fixed overhead camera; measure offset of a
  marked point on the airframe from the board centre. Cheap, repeatable, good enough for
  ±2 cm.
- Better if available: total station, or the department's motion-capture rig if one exists.
- **Report `N`, mean, σ, max, and the failure/abort count.** N ≥ 20 per configuration.

### A2. No rangefinder 🔴 critical, cheap
Below ~0.6 m the 24 cm centre tag leaves the FOV; barometric altitude drifts and is
corrupted by ground effect; the corner tags are the only vision source and they're near the
image edge (down-weighted to 0.3 by the edge-safety factor).

**Fix:** TF-Luna or TFmini-S (senior's own recommendation), or Benewake TFmini Plus, on a
spare UART with `RNGFND1_TYPE`. **Also:** the **MatekSys 3901-L0X already on the airframe**
has both optical flow and a short-range LiDAR and has never been configured. Free win.

### A3. Optical flow unused 🟠 cheap
`3901-L0X` is mounted and wired but never set up. It directly improves position hold during
the terminal descent — which is exactly where the underdamped behaviour was observed.
`FLOW_TYPE`, `EK3_SRC*_VELXY`, and flow calibration.

### A4. Rolling shutter camera 🔴 blocking for moving platforms
IMX219 is rolling shutter. On a *static* pad with slow descent this is tolerable (and the
jump filter papers over the spikes). On a **moving** platform, with relative motion and a
yawing airframe, rolling-shutter skew and motion blur will corrupt corner localisation
systematically — not as random spikes the jump filter can catch, but as a *biased* pose.

**Fix — procure now (August is the procurement month):** Raspberry Pi Global Shutter Camera
(IMX296) or an Arducam OV9281 global-shutter module. Note the GS camera needs a C/CS lens
chosen for FOV, and **a full recalibration** (new intrinsics, new sensor-mode lock).

### A5. No explicit EKF 🟠 the core technical contribution
Currently raw fused pose → `LANDING_TARGET` → ArduPilot's internal precision-landing
estimator. Consequences: no principled uncertainty, no graceful degradation when the marker
is briefly lost, no way to fuse a beacon. This is senior's future-work item **L7** and
ROLAND's §3.1–3.3 is the template. See `02-roland-reference.md`.

### A6. `PLND_ACC_P_NSE` too aggressive 🟠 free
Senior's explicit #1 tuning recommendation: **reduce it** to stop over-aggressive
corrections and underdamped descent. One parameter. Sweep it in Tier-0 sim.

### A7. "Beacon" is undefined 🔴 scope-critical
The word is in our project title and nowhere in the inherited system. Until it's resolved
(UWB / IR-Lock / something else), a whole subsystem is unspecified and unprocured.
→ `06-open-questions.md` Q3.

### A8. No moving platform exists 🔴 blocking for the headline result
Nothing in the inherited hardware moves. We need a platform, a way to command its motion
repeatably, and ideally a source of its velocity (encoders, or GPS telemetry, or purely
vision-estimated). → Q4.

### A9. Compute headroom 🟠
12–18 fps on the Pi 4B, at 640×480, with ~0.25 s end-to-end latency. For a platform moving
at 0.8–1.5 m/s that's 20–40 cm of target travel per control update. Marginal.
Options: optimise (ROI-crop tracking once locked — big easy win), Pi 5, or Jetson Orin Nano
(mandatory only if we go markerless/YOLO). **Measure before buying.**

### A10. Single point of failure: the camera bracket 🟡
The *entire* coordinate chain assumes the camera is rigidly mounted at a known attitude with
the CSI ribbon toward the tail. A bent bracket after a hard landing silently corrupts every
measurement. **Add a startup self-check** (e.g. hover over the board and verify reported
offsets against a known placement) and a pre-flight checklist item.

### A11. No end-to-end mission 🟡
Auto takeoff → nav to target area → acquire → precision land has never been run as one
sequence. Senior's L9.

### A12. Reproducibility / bus factor 🟠
The pipeline lives on one Raspberry Pi as a systemd service. If that SD card dies, or the
code isn't in git, the project restarts from the thesis text. **Get the code into this repo
this week, with the calibration YAML, the systemd unit, and the tag board PDF.** → Q2.

### A13. Lighting and environment envelope untested 🟡
All reported tests appear to be daytime outdoor on concrete. Untested: low sun angle,
shadows across the board, wet/reflective surface, dusk, indoor fluorescent flicker.
Cheap to characterise, and a reviewer will ask.

---

# B. Failure modes and fallback design

This is the part most student projects skip and every examiner asks about. **Design the
fallback ladder explicitly, implement it, and demonstrate each rung deliberately.**

## B1. The fallback ladder

```
 L0  Precision landing on fused vision (+ beacon)          ← nominal
 L1  Vision degraded → beacon/UWB-only relative guidance   ← if marker lost/occluded
 L2  Beacon lost too  → dead-reckon on last estimate + EKF prediction, for T_hold seconds
 L3  Estimate stale > T_hold → ABORT: climb to reacquire altitude, re-search
 L4  N aborts exceeded → give up precision: plain GPS LAND at the last known pad position
 L5  GPS also unhealthy → LAND in place / RTL, depending on altitude and battery
 L6  Any time → pilot takes manual control on ELRS      ← always available, always primary
```

ArduPilot gives you rungs L3/L4 partly for free — learn these before writing custom logic:
`PLND_STRICT`, `PLND_RET_BEHAVE`, `PLND_RET_MAX`, `PLND_ALT_MIN`, `PLND_ALT_MAX`,
`PLND_TIMEOUT`, `PLND_OPTIONS`. **Read the parameter reference for your firmware version
and verify each behaviour in SITL** before relying on it.

## B2. Failure catalogue

| # | Failure | Detection | Fallback |
|---|---|---|---|
| F1 | Marker out of FOV (wind gust, aggressive correction) | no detections for k frames | L1/L2; widen search by climbing |
| F2 | Marker occluded / partially covered | tag count drops, disagreement spikes | down-weight, L1 |
| F3 | **Single-tag tilt artifact** at altitude | only 1 tag visible → no disagreement available | **Never trust tilt from a single tag.** Gate tilt output on tag count ≥ 2 |
| F4 | Vibration spike | jump filter trips | already handled — but **count and log trips**; a rising trip rate is a mechanical warning |
| F5 | Sun glare / washed-out board | detections stop, decision margin collapses | L1; consider matte-laminate board, anti-glare print |
| F6 | Rolling-shutter bias under relative motion | pose disagrees with beacon range | cross-check vision range vs UWB range; reject vision if inconsistent |
| F7 | Companion computer crash / hang | **watchdog: FC sees LANDING_TARGET stream stop** | ArduPilot times out → L4. systemd restarts the service (already configured) |
| F8 | Serial link corruption at 921600 | MAVLink CRC failures | count bad packets; if rate high → L4 |
| F9 | Miscalibrated camera (post-crash, bracket bent) | reported altitude disagrees with rangefinder/baro | **This is why A2 matters** — the rangefinder is an independent altitude witness. Refuse to arm on disagreement |
| F10 | Pose ambiguity sign flip | tilt sign oscillates frame to frame | already handled by Z-axis normalisation — keep a counter |
| F11 | Target moves faster than the UAV can track | tracking error grows monotonically | do not enter landing mode; hold `h_fix`, or abort |
| F12 | **Commit-point problem** on a moving platform | below some altitude an abort is more dangerous than continuing | define an explicit commit altitude; below it, continue and accept the landing |
| F13 | Ground effect / prop wash near the platform | altitude estimate wobbles in the last 0.3 m | rangefinder + a short "cut throttle and drop" terminal phase (`LAND_ALT_LOW`) |
| F14 | Landing gear tips over on the platform edge | — | mechanical: wider gear, a shallow cone/funnel pad, or a net. Also demand a tighter `T_xy` before commit |
| F15 | Battery failsafe mid-tracking | ArduPilot battery FS | tracking flights are long — set failsafe thresholds *before* the long test flights, not after |
| F16 | GPS glitch / EKF variance during descent | ArduPilot EKF failsafe | this is exactly why we want the beacon: relative guidance survives GPS loss |
| F17 | Radio link loss | ELRS failsafe | configure ArduPilot RC failsafe explicitly; do not leave at default |
| F18 | Beacon multipath / NLOS (UWB) | range residual vs vision | scale `R` with range (ROLAND does this: `R ∝ d`), reject outlier ranges |

## B3. Non-negotiable safety gates

1. **Re-enable the full ArduPilot arming-check suite.** The senior relaxed them for
   bench/tethered work; the thesis flags re-enabling as mandatory before unsupervised
   outdoor autonomous flight. Treat this as a hard gate, in writing, before Phase-2 flying.
2. **Geofence** (`FENCE_*`) — enable altitude and radius fence for every autonomous test.
3. **Manual override always live** — ELRS pilot on the sticks, mode switch mapped to
   STABILIZE/ALT_HOLD, for every single autonomous flight. This was the senior's primary
   safety mechanism; keep it.
4. **Props off** for every first run of new code. Bench → tethered → LOITER-observe → LAND.
5. **Never let new perception code command the vehicle on its first flight.** LOITER mode
   receives and logs `LANDING_TARGET` without acting on it. Characterise there first.
6. **Two-person rule** for flight tests: one on the sticks, one on the laptop. Nobody
   debugs and pilots simultaneously.
7. **Battery discipline** — 4S Li-ion, log every cycle, no puffed packs, LiPo bag.
8. **Disagreement metric should gate the output, not just be logged.** If disagreement
   exceeds a threshold, suppress the `LANDING_TARGET` send rather than sending a bad one.
   (Currently it's a diagnostic; make it a guard.)

## B4. Site and regulatory
- Campus flight permission for autonomous outdoor testing — confirm with the guide.
- India DGCA drone rules: a sub-250 g exemption does **not** apply to a 5" quad with a Pi
  and 4S pack. Check nano/micro classification, and whether campus counts as a green zone.
  Get this documented early — it is an easy thing to be blocked by in October.
- Fly over grass, not concrete, wherever possible. Have a fire extinguisher and a first-aid
  kit at the field.

---

# C. Project-level risks

| Risk | Impact | Mitigation |
|---|---|---|
| Senior's code unavailable/undocumented | Weeks lost re-deriving 9 non-obvious gotchas | **Contact Akash Dinesh immediately.** Same guide can facilitate. → Q2 |
| Timeline: procurement in Aug, flights in Oct | Long-lead items (UWB, GS camera) miss the window | Order this month; place orders before finalising the design |
| Toolchain rabbit hole (ROS/Gazebo on M1) | A month with nothing to show | **Tier 0 first** (`03-simulation-plan.md`). Sim results are report-able on their own |
| Scope: moving platform + beacon + EKF + inclined is a lot | Nothing finished properly | Sequence strictly: C1 → C6 → C3 → C2 → C4. Ship each before starting the next |
| Crash destroys the airframe mid-project | Weeks lost | Keep spare props/motors/arms in stock. Sim-first policy. Never test new control code on the only airframe without a tether |
| Only one person understands the pipeline | Bus factor | All three of you should be able to run bench tests. Pair on the first integration |
| Disk space / dev machine limits | Tier-1 sim blocked | Free space, external SSD, or a lab Linux PC → Q6 |

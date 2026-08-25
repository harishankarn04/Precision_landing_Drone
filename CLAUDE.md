# CLAUDE.md — Precision Landing Drone

Context file for AI assistants (Claude Code, Copilot, Antigravity, Cursor).
**Read this first, then `docs/`.** Everything here is derived from the three PDFs in
`Documentation/`. Nothing has been implemented yet in this repo — it is currently
documentation-only.

---

## 1. What this project is

**Course:** B.Tech VII/VIII Semester Project (23ECE498 / 23EAC498), AY 2025–2026
**Institution:** Amrita School of Engineering, Bengaluru — Dept. of ECE
**Batch:** B001 · **Domain/TAG:** Robotics & Autonomous Systems
**Guide:** Dr. Sreeja Kochuvila

**Team**
| Name | Register No. |
|---|---|
| Nandakrishna J | BL.EN.U4ECE23029 |
| A Somesh Ayyappan | BL.EN.U4ECE23102 |
| Hari Shankar N | BL.EN.U4ECE23118 |

**Declared title (Phase 1 synopsis):**
> Design and Development of a Beacon-Based Autonomous Precision Landing System for
> Unmanned Aerial Vehicles on Static and Moving Platforms.

**One-line goal:** land a multirotor accurately on a pad that GNSS cannot resolve —
first static, then *moving* — continuing a completed M.Tech project and borrowing the
sensor-fusion architecture from the ROLAND paper.

---

## 2. The three source documents

| File | What it is | Why it matters |
|---|---|---|
| `Documentation/Report_Full_v2_No_First_Page.pdf` (61 pp) | **Senior's completed M.Tech thesis** — Akash Dinesh (BL.EN.P2RAU24001), May 2026, same guide | **This is the baseline we inherit and extend.** Working hardware + working vision pipeline. See `docs/01-inherited-system.md` |
| `Documentation/ROLAND_....pdf` (6 pp) | ICCAS 2021 paper, KAIST (Kim, Lee, Choi, Jeon, Kim, Myung) | **Target architecture** for the moving-platform half. Code: https://github.com/engcang/ROLAND. See `docs/02-roland-reference.md` |
| `Documentation/Synopsis Details_Format_Project_Phase_1.pdf` (3 pp) | **Our own Phase-1 synopsis**, as submitted | Defines the officially-approved objectives and the Jul–Nov 2026 timeline. See `docs/00-project-brief.md` |

---

## 3. The 30-second technical picture

**Inherited (working, validated on real hardware):**

```
Pi Camera V2 (IMX219, rolling shutter, downward)
  → Raspberry Pi 4B 8GB (Python 3, picamera2 + OpenCV + pupil-apriltags + pymavlink)
      · undistort (calibrated: fx 497.88, fy 496.68, RMS 0.255 px)
      · detect tag36h11, reject decision margin < 25
      · per-tag solvePnP (SOLVEPNP_ITERATIVE — *not* IPPE_SQUARE)
      · quality-weighted multi-tag fusion → single landing-centre estimate
      · camera→body NED transform, jump filter, velocity/lag estimator
  → MAVLink2 LANDING_TARGET over UART @921600 (SERIAL6)
      → Matek H743-SLIM V3 running ArduCopter (PLND_TYPE=1)
          → LAND mode position correction → DShot ESCs → 4× motors
```

Landing target = **60 cm board**, one **24 cm** tag36h11 (ID 0) centre + four **8 cm**
tags (IDs 1–4) at (±0.22, ±0.22) m. Multi-scale layout solves the range-vs-FOV problem.

**Validated:** detection 0.6 m → 8.56 m; inter-tag disagreement 1.1–5.0 cm; genuine
inclined-surface tilt recovery (−13° to −21°) distinguished from single-tag PnP artifact.

**NOT done by the senior — this is our opening:**
- ❌ Closed-loop autonomous **touchdown** never completed or characterised
- ❌ No **ground-truth** accuracy measurement (only internal self-consistency)
- ❌ **Moving platform** never attempted (velocity/lag code exists but is dormant)
- ❌ No explicit **EKF** fusing IMU/vision (relies on ArduPilot's internal estimator)
- ❌ No **rangefinder**; the onboard MatekSys 3901-L0X optical-flow unit is **unused**
- ❌ No **beacon** of any kind (despite "Beacon-Based" being in our title)

---

## 4. Working decisions

⚠️ **We inherit no airframe and no code** (confirmed 2026-08-14). The senior's thesis is a
**specification we build from**, not a system we take over. Everything in §3 above describes
the *target* to reproduce, not something we currently possess.

### 🔴 The governing strategy: **simulation first, hardware last**

No parts are bought and no airframe is built until **a drone lands itself in simulation**.
The official Jul–Nov 2026 timeline is treated as nominal. A working simulation is also the
evidence that justifies spending on parts — and parts will be **salvaged** where possible.

Order: `sim runs → drone lands in sim → beacon in sim → moving platform in sim → hardware`.
Beacon work does **not** start before a flying, landing sim drone exists.

| Decision | Status |
|---|---|
| Autopilot stack | ✅ **ArduPilot / ArduCopter**. Do **not** switch to PX4 — ROLAND uses PX4, we do not |
| Marker system | ✅ AprilTag tag36h11 multi-scale board, per thesis Table 3.2 |
| Simulator | ✅ **Webots** (native macOS, official ArduPilot support) is the active simulator — see §5a. **Gazebo is deferred, unconfirmed** — revisit only if a specific need arises (e.g. large open-world moving-platform scenarios); do not set it up speculatively |
| AprilTag library | ✅ **`cv2.aruco` + `DICT_APRILTAG_36h11`**, *not* `pupil-apriltags`. Verified working on OpenCV 5.0 with `CORNER_REFINE_APRILTAG`. Removes a dependency and sidesteps the y-up/y-down sign trap — everything stays in OpenCV conventions |
| Python | ✅ **3.13** in `.venv/` (not the system 3.14 — MAVProxy/pymavlink wheels lag) |
| Ground control station | ✅ **MAVProxy** (already installed, text-based, no extra app). **Not QGroundControl** — QGC is a bigger GUI app for the same job; MAVProxy is simpler and is what the manual-override keyboard/`rc` work will build on later |
| ROS 2 | ✅ **Needed** (professor's requirement). **Achieved once and frozen**: `ros2_ws/` — a ROS 2 node commanded a precision landing in SITL, 2.03 m → **0.01 m**, 2026-08-20. Do not expand it further until Stage 4 (beacon) actually needs a second ROS node. Its Docker image is not kept built between sessions — `docker build -t precision-landing-ros2 ros2_ws` before reusing it |
| Flight control | ✅ **ArduPilot does the flying.** ROS 2 is the *interface layer*, not a replacement controller. We do **not** write our own PID/RTH — ArduPilot's LAND + `PLND_*` already do it, and that is what transfers to the Matek H743 |
| Manual override in sim | ⏸️ **Real requirement, not a distraction** — mirrors the ELRS radio override on the real aircraft (fallback ladder rung L6, `docs/05-risks-and-failsafes.md`). Build it once autonomous flight itself exists to override, not before |
| Scope target | ✅ **Sequence: simple sim flies → autonomous landing on our custom board → beacon.** One thing working at a time; do not start the next until the current one is proven |
| "Beacon" modality | ⏸️ **Deferred by decision.** UWB recommended. Not procurement-blocking any more — we prototype it in simulation. `docs/06-open-questions.md` Q3 |
| Hardware / BOM | ⏸️ **Parked** until Stage 3+. Notes retained in `docs/04-roadmap-checklist.md` |

---

## 5. Environment (as of 2026-08-14)

| Machine | Owner | Role |
|---|---|---|
| **Apple M1, 16 GB, macOS 15.7.3, arm64**, ~60 GB free | Hari — *the only machine he has* | Stages 1–2 natively; **plus a UTM VM for Stage-3 development** |
| **Linux Mint + 40-series NVIDIA GPU** | teammate | Stage 3+ at full frame rate; results-grade runs. Synced via **GitHub** |
| ~1660-class NVIDIA laptop | teammate | Secondary |
| MATLAB | all three | EKF design (Stage 4) |

**Set up in this repo:** `.venv/` (Python 3.13 · `pymavlink` · `MAVProxy` ·
`opencv-contrib-python` 5.0 · `numpy`). ArduPilot cloned to `~/Documents/gitClone/ardupilot`,
configured for SITL. **Webots** installed at `/Applications/Webots.app`.

**Gazebo is not set up and not currently planned.** If a real need for it shows up later
(the team has GPU machines that could run it), reopen that decision then — don't build it
speculatively now. `docs/03-simulation-plan.md` has the old evaluation if useful as reference.

### 5a. The two simulator paths that exist, and how they relate

| Path | Command | What it's for |
|---|---|---|
| **Plain SITL** | `sim/run_sitl.sh` | Fastest way to fly ArduCopter with no camera at all — parameter sweeps, testing MAVLink messages, nothing visual |
| **Webots** | `sim/run_webots.sh` then `sim/run_sitl_webots.sh` | 🎯 **The active path.** A real 3D world, a real downward camera (`sim/camera_view.py` runs live AprilTag detection on it), native macOS GUI. This *is* Stage 2 — no need to write our own synthetic camera renderer, Webots gives us a real one |
| ROS 2 + Docker | `ros2_ws/` | **Frozen at its Stage-1 result** (§4 table). Not the thing to touch right now |

Sequence: get Webots + a real camera + our own AprilTag board landing the drone
autonomously, end to end, before touching ROS 2, Gazebo, or the beacon again.

---

## 6. Repo layout (intended)

```
Documentation/     # the three source PDFs (read-only reference)
docs/              # our derived working documents — START HERE
  00-project-brief.md        objectives, timeline, deliverables
  01-inherited-system.md     full spec of the senior's system + gotchas
  02-roland-reference.md     ROLAND architecture + repo assessment
  03-simulation-plan.md      simulator decision + staged setup
  04-roadmap-checklist.md    the actual task checklist
  05-risks-and-failsafes.md  drawbacks, failure modes, fallback design
  06-open-questions.md       unresolved decisions needing team/guide input
sim/               # (future) simulation configs, worlds, launch files
src/               # (future) perception + control code
hardware/          # (future) BOM, wiring, 3D-print files, calibration data
logs/              # (future) flight logs, .BIN, video, analysis notebooks
```

---

## 7. Conventions & hard-won gotchas (inherited — do not relearn these)

These cost the senior real debugging time. They are recorded in the thesis Ch. 6.7.

0. ⚠️ **`LANDING_TARGET` frame must be `MAV_FRAME_BODY_FRD`** (or `LOCAL_FRD`).
   **This CORRECTS the thesis**, which specifies `MAV_FRAME_BODY_NED` — current
   ArduPilot rejects that outright in `AC_PrecLand_MAVLink::handle_msg`. Verified in
   source and on live SITL, 2026-08-20. Also prefer `position_valid=1` and send the
   offset vector `(forward, right, down)` directly; with `position_valid=0` ArduPilot
   reconstructs direction as `(-tan(angle_y), tan(angle_x), 1)`, so `angle_x` means
   *right* and `angle_y` means *negated forward* — a classic sign trap.
1. **MAVLink timestamp** — `LANDING_TARGET` needs **boot-relative microseconds**, not
   Unix epoch. Epoch timestamps are *silently* rejected by ArduPilot with no error.
2. **Angular-size fields must be non-zero** or ArduPilot's rejection logic drops the message.
2b. **A new MAVLink link is silent until you request streams.** ArduPilot streams
   telemetry *per link* — a GCS requesting data on SERIAL0 does nothing for a companion
   computer on SERIAL1/UART. Send `MAV_CMD_SET_MESSAGE_INTERVAL` on connect.
2c. **`PLND_ENABLED` needs a reboot.** It is an enable-flag; the backend is constructed
   at boot. Setting it on a running autopilot silently does nothing.
3. **PnP solver** — use `SOLVEPNP_ITERATIVE`. `SOLVEPNP_IPPE_SQUARE` (the "correct"
   choice for square planar markers) gave garbage on the IMX219. Huge reprojection error
   = wrong solver, not bad calibration.
4. **Sign convention** — `pupil-apriltags` uses y-**up**; OpenCV uses y-**down**. Mixing
   them yields plausible magnitudes with *inverted direction* — the worst kind of control bug.
   Body-frame mapping used: `x_body = −y_cam − x_offset`, `y_body = +x_cam − y_offset`,
   `altitude = z_cam`.
5. **Do not pass distortion coefficients to solvePnP** after already undistorting the
   frame — that double-corrects. Pass a null distortion vector.
6. **Lock the camera sensor mode.** `picamera2` may silently pick a cropped mode whose
   effective focal length invalidates your calibration. Lock binned full-FOV
   (1640×1232 → 640×480).
7. **Square-marker pose ambiguity** — normalise the rotation so the marker Z-axis points
   *toward* the camera, else tilt sign flips frame to frame.
8. **Vibration jump filter is mandatory under power.** Reject any frame whose horizontal
   position moves >30 cm or altitude >40 cm from the last accepted frame, unless that
   frame is >0.5 s stale. Without it, ArduPilot chases vibration spikes into divergent
   oscillation.
9. **Calibrate against a pixel-accurate printed target.** A chessboard with tiny gaps
   between squares gives internally-consistent but metrically-wrong intrinsics.

---

## 8. Key ArduPilot parameters (inherited baseline)

| Param | Value | Note |
|---|---|---|
| `PLND_ENABLED` | 1 | |
| `PLND_TYPE` | 1 | companion computer / MAVLink |
| `PLND_EST_TYPE` | 1 | |
| `PLND_STRICT` | 1 | |
| `PLND_LAG` | 0.25 s | measured end-to-end pipeline latency |
| `PLND_CAM_POS_X/Y/Z` | Z = −0.05 m | camera offset from CG |
| `PLND_ACC_P_NSE` | (left at default **2.5**) | **thesis's #1 recommendation: reduce this** — too aggressive → underdamped descent. 2.5 is our sweep baseline |
| `LAND_SPEED` | 30 cm/s | |
| `WPNAV_SPEED_DN` | 30 cm/s | |

⚠️ `PLND_ENABLED` is an **enable flag** — while it is 0, every other `PLND_*` parameter is
hidden from parameter lists. Set it to 1 first. (Verified on live SITL, 2026-08-14.)

✅ Rungs L3/L4 of our fallback ladder already exist in ArduPilot: `PLND_TIMEOUT` (4 s),
`PLND_RET_MAX` (4), `PLND_RET_BEHAVE`, `PLND_ALT_MIN` (0.75 m), `PLND_ALT_MAX` (8 m).
Configure and demonstrate these rather than writing custom abort logic. Full verified
default table in `docs/07-sim-setup.md`.

⚠️ Arming checks were relaxed during the senior's bench/tethered work. **Re-enabling the
full ArduPilot arming-check suite is a non-negotiable gate before any unsupervised
outdoor autonomous flight.**

---

## 9. Rules for AI assistants working in this repo

- **Safety first.** This is a flying vehicle with exposed 5-inch props on a 4S pack.
  Never suggest disabling arming checks, failsafes, or geofence for convenience.
  Every new control behaviour gets tested in SITL → tethered → LOITER-observe → LAND,
  in that order. The senior's LOITER-then-LAND methodology is the house standard.
- **Simulate before flying.** Prefer changes that can be validated in SITL first.
- **Don't switch autopilot stacks.** ROLAND is PX4; our hardware is ArduPilot.
  Port ROLAND's *ideas* (EKF structure, UWB fusion, landing controller), not its code.
- **Keep `docs/` current.** When a decision in §4 or `docs/06-open-questions.md` is
  resolved, update it in the same commit as the code.
- Units: metres, seconds, radians internally; degrees only in UI/report text.
- Frames: state the frame in every variable name (`_cam`, `_body`, `_ned`, `_marker`).

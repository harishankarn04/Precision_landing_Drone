# CLAUDE.md — Precision Landing Drone

Context file for AI assistants (Claude Code, Copilot, Antigravity, Cursor).
**Read this first, then `docs/`.**

---

## 0. Read this before anything else: the stack pivoted (2026-08-26)

**Current plan: Gazebo + ROS 2 + PX4.** ArduPilot and Webots are **dropped**.

This reverses an earlier "never switch to PX4" decision (§4 used to say the opposite —
ignore any trace of that if you see it in old commits or a stale summary). The change was
a deliberate call by Hari, made after a session that got a Webots+ArduPilot flight working
but hit repeated friction doing it (two real bugs in ArduPilot's own example parameter
file, one Webots crash, plus general two-window/GUI-focus friction with keyboard control).

**What that means for everything below:**
- Nothing has been built yet on the new stack. **No Gazebo, no PX4, no ROS 2 setup exists
  in this repo right now** — do not assume otherwise, and do not start setting it up
  speculatively unless asked.
- All ArduPilot/Webots-specific code from the previous stack has been **deleted** from the
  working tree (it's still recoverable from git history, commit `392ef7c`, if ever needed —
  ROS 2 bridge proving MAVLink↔ROS worked, Webots launch scripts, a keyboard teleop). None
  of it runs against the new stack, so it isn't worth resurrecting as a starting point.
- The **senior's thesis system** (§3 below) genuinely runs on ArduPilot hardware — that's a
  historical fact about *their* build, not a statement about ours, and doesn't change. Its
  perception-pipeline spec (marker layout, PnP solver, fusion math) is autopilot-agnostic
  and still the reference to rebuild from. Its ArduPilot-specific integration details
  (§7 items 0–2c, §8's parameter table) won't transfer to PX4 as-is — treat them as *prior
  art on what an autopilot-integration gotcha looks like*, not as directions to follow.
- `docs/03-simulation-plan.md` and `docs/04-roadmap-checklist.md` still describe the old
  Webots-first staging in places — being rewritten. Treat this file (`CLAUDE.md`) as the
  source of truth on the current plan if the two disagree.

**Sequence, same shape as before, new tooling:** Gazebo runs → PX4 flies in it → ROS 2 talks
to PX4 → our own perception pipeline closes the loop → beacon → moving platform → hardware.
Simulation-first, hardware-last still holds (§4).

---

## 0b. Second pivot (2026-09-09): back to ArduPilot, Gazebo kept, GCS = QGroundControl

**Current plan: Gazebo + ArduPilot + QGroundControl (ROS 2 status: open question — see
below and §4).** Hari reversed the PX4 half
of the §0 pivot above, two weeks later. **Gazebo stays** — it's still the simulator,
Webots is still dropped. What changes: the autopilot goes back to **ArduPilot** (PX4
dropped again). The ground control station is **QGroundControl** — Mission Planner was
considered same-day and rejected: it's Windows-only and this project runs on macOS/Linux,
where Mission Planner has no real support; QGC is cross-platform and already the tool named
in the pre-pivot setup notes (`docs/07-sim-setup.md`).

This is a deliberate, explicit call by Hari — don't read it as drift or "forgetting" §0's
"don't switch autopilot stacks again without being asked" rule; the ask happened. Do not
revert to PX4 again without a similarly explicit instruction.

**What this means concretely, layered on top of §0 above:**
- Everywhere §0–§9 below says "PX4" as the *current* choice, read **ArduPilot** instead.
  Where §0/§7/§8 marked ArduPilot-specific content (items 0–2c, the §8 parameter table) as
  "historical" or "won't transfer to PX4 as-is" — that framing is now stale. ArduPilot is
  the live target again, so that content (including the senior's `PLND_*` parameters and
  the ArduPilot MAVLink gotchas) is directly applicable, not just prior art.
- ROLAND's own stack is PX4-based (`docs/02-roland-reference.md`), so the "our autopilot
  now matches ROLAND's" convenience from the first pivot is gone again — ROLAND's PX4
  integration details go back to being *ideas to port*, not configuration to copy directly.
  Its estimator/controller math (§2–§3 of that doc) is autopilot-agnostic and unaffected.
- **Gazebo + ROS 2 aren't purely hypothetical anymore.** Hari has a Linux VM (Ubuntu, via
  Parallels on the M1 Mac) with **Gazebo and ROS 2 Humble already installed** — this repo
  still has no ArduPilot SITL, no `ardupilot_gazebo` bridge config, and no QGroundControl
  set up *in it*, but the underlying toolchain exists and is ready when that stage starts.
  See §5.
- **ROS 2's status was downgraded the same day (still 2026-09-09), after this section was
  first written:** it's no longer a settled requirement, it's an open question. Nothing in
  the pipeline technically needs it — see `docs/06-open-questions.md` Q11 and §4's ROS 2
  row. Default is to build **ROS2-free** (`gz-transport` directly for the Gazebo camera
  bridge) and confirm the professor's actual intent before Stage 3. The Parallels VM having
  ROS 2 Humble already installed (previous bullet) doesn't mean it's committed to being used.
- `docs/00`–`06` and `README.md` still carry PX4-pivot language in places — being corrected
  to match. Treat this file as the source of truth if any of them disagree.

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
| `Documentation/Report_Full_v2_No_First_Page.pdf` (61 pp) | **Senior's completed M.Tech thesis** — Akash Dinesh (BL.EN.P2RAU24001), May 2026, same guide | The perception-pipeline specification we rebuild from. See `docs/01-inherited-system.md` |
| `Documentation/ROLAND_....pdf` (6 pp) | ICCAS 2021 paper, KAIST (Kim, Lee, Choi, Jeon, Kim, Myung) | **Target architecture** for the moving-platform half — ROLAND is PX4-based, we're ArduPilot again (§0b), so port its ideas, not its autopilot integration. Code: https://github.com/engcang/ROLAND. See `docs/02-roland-reference.md` |
| `Documentation/Synopsis Details_Format_Project_Phase_1.pdf` (3 pp) | **Our own Phase-1 synopsis**, as submitted | Defines the officially-approved objectives and the Jul–Nov 2026 timeline. See `docs/00-project-brief.md` |
| `Documentation/dinesh_reference_code/` (received 2026-09-15) | Dinesh's actual production Python — not just the thesis PDF | Reference for Stage 1's fusion/velocity/jump-filter logic once that's being built. **Not run yet** (sim-first) — has a known frame-bug discrepancy vs `CLAUDE.md` §7 item 0, see its own README before trusting either version |

---

## 3. The senior's system — the perception spec we rebuild from (their hardware, same autopilot as us again)

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
**This board design, the detection/fusion math, and the MAVLink `LANDING_TARGET`/`PLND_*`
integration above are all what we're rebuilding — on the same ArduPilot autopilot as the
senior (§0b), this time on Gazebo instead of hardware first.** Still verify each gotcha
(§7) and parameter (§8) on our own Gazebo SITL build rather than assuming they transfer
byte-for-byte — the senior's board, wiring, and firmware version aren't identical to ours.

**Validated (on their ArduPilot hardware):** detection 0.6 m → 8.56 m; inter-tag
disagreement 1.1–5.0 cm; genuine inclined-surface tilt recovery (−13° to −21°) distinguished
from single-tag PnP artifact.

**NOT done by the senior — this is our opening:**
- ❌ Closed-loop autonomous **touchdown** never completed or characterised
- ❌ No **ground-truth** accuracy measurement (only internal self-consistency)
- ❌ **Moving platform** never attempted (velocity/lag code exists but is dormant)
- ❌ No explicit **EKF** fusing IMU/vision (relies on ArduPilot's internal estimator)
- ❌ No **rangefinder**; the onboard MatekSys 3901-L0X optical-flow unit is **unused**
- ❌ No **beacon** of any kind (despite "Beacon-Based" being in our title)

---

## 4. Working decisions (updated 2026-08-26)

⚠️ **We inherit no airframe and no code** (confirmed 2026-08-14). The senior's thesis is a
**specification we build from**, not a system we take over.

### 🔴 The governing strategy: **simulation first, hardware last**

No parts are bought and no airframe is built until **a drone lands itself in simulation**.
The official Jul–Nov 2026 timeline is treated as nominal. A working simulation is also the
evidence that justifies spending on parts — and parts will be **salvaged** where possible.

Order: `sim runs → drone lands in sim → beacon in sim → moving platform in sim → hardware`.
Beacon work does **not** start before a flying, landing sim drone exists.

| Decision | Status |
|---|---|
| Autopilot stack | ✅ **ArduPilot** (reverted 2026-09-09, was briefly PX4 from 2026-08-26). Matches the senior's system and our own MAVLink/`PLND_*` gotchas directly again; no longer matches ROLAND's own stack (PX4) |
| Simulator | ✅ **Gazebo** (changed 2026-08-26, was Webots — this part of the pivot was kept). Not set up yet in this repo — nothing to assume is working |
| Ground control station | ✅ **QGroundControl** (decided 2026-09-09; Mission Planner considered and rejected same-day — Windows-only, we need macOS/Linux) |
| ROS 2 | 🟡 **Open question** (`docs/06-open-questions.md` Q11) — professor asked for it, but nothing in the pipeline technically needs it (ArduPilot↔Gazebo and ArduPilot↔pipeline both go via MAVLink/`ardupilot_gazebo`, not ROS 2). **Default: build ROS2-free**, using `gz-transport` directly for the Gazebo camera bridge; confirm the actual requirement with the guide before Stage 3 |
| Marker system | ✅ AprilTag tag36h11 multi-scale board, per thesis Table 3.2 — unchanged, autopilot-agnostic |
| AprilTag library | ✅ **`cv2.aruco` + `DICT_APRILTAG_36h11`**, *not* `pupil-apriltags`. Removes a dependency and sidesteps the y-up/y-down sign trap — everything stays in OpenCV conventions |
| Scope target | ✅ **Sequence: simple sim flies → autonomous landing on our custom board → beacon.** One thing working at a time; do not start the next until the current one is proven |
| "Beacon" modality | ⏸️ **Deferred by decision.** UWB recommended. Prototype in simulation once a landing loop exists. `docs/06-open-questions.md` Q3 |
| Hardware / BOM | ⏸️ **Parked** until simulation stages pass. Notes retained in `docs/04-roadmap-checklist.md` |

---

## 5. Environment (as of 2026-09-09)

| Machine | Owner | Role |
|---|---|---|
| **Apple M1, 16 GB, macOS 15.7.3, arm64**, ~60 GB free | Hari — *the only physical machine he has* | Dev machine (native Mac work + host for the Parallels VM below) |
| **Ubuntu 22.04 arm64 VM under Parallels, on the M1** (6 vCPU / 12 GB RAM) | Hari | **Gazebo Harmonic (`gz-harmonic`, gz sim 8.15.0) and ROS 2 Humble Desktop already installed** — the only piece of the new stack that's actually set up anywhere so far. Gazebo Harmonic wasn't a free choice: no arm64 build of Gazebo Classic exists. `ros-humble-ros-gz-bridge`/`-interfaces`/`-image` are present, but `ros_gz_sim` (the launch wrapper) isn't published for arm64/Humble via apt — build from source or launch `gz sim` + bridge topics manually if needed. ArduPilot SITL, the ArduPilot↔Gazebo bridge, and QGroundControl are not yet installed. See `docs/07-sim-setup.md` Stage 3 for the rendering gotcha (Ogre2+virgl crash, fixed with `LIBGL_ALWAYS_SOFTWARE=1`) |
| **Linux Mint + 40-series NVIDIA GPU** | teammate | Results-grade Gazebo runs. Synced via **GitHub** |
| ~1660-class NVIDIA laptop | teammate | Secondary |
| MATLAB | all three | EKF design (later stage) |

**Set up in this repo:** `.venv/` (Python 3.13 · `pymavlink` · `MAVProxy` ·
`opencv-contrib-python` 5.0 · `numpy`). **Set up on Hari's machine, outside the repo:**
Gazebo + ROS 2 Humble in the Parallels Ubuntu VM. **Not yet set up anywhere:** ArduPilot
SITL, the ArduPilot↔Gazebo bridge, QGroundControl. No decision has been made yet on
whether Gazebo/ArduPilot work happens in that VM, natively on Linux Mint, or both — figure
that out when actually starting, don't pre-plan it speculatively.

`~/Documents/gitClone/ardupilot` still exists on disk from the previous stack (SITL build
artifacts etc.) but is no longer part of the plan — leave it alone, don't clean it up
proactively, it costs nothing sitting there.

---

## 6. Repo layout (current)

```
Documentation/     # the three source PDFs (read-only reference)
docs/              # our derived working documents — START HERE after this file
  00-project-brief.md        objectives, timeline, deliverables
  01-inherited-system.md     senior's system spec (perception logic and ArduPilot
                              integration details both directive again, per §0b)
  02-roland-reference.md     ROLAND architecture + repo assessment (PX4-based, no longer
                              stack-matched to us)
  03-simulation-plan.md      simulator/autopilot plan — being rewritten for Gazebo+ArduPilot
  04-roadmap-checklist.md    the task checklist — being rewritten for Gazebo+ArduPilot
  05-risks-and-failsafes.md  drawbacks, failure modes, fallback design — ArduPilot parameter
                              names throughout are directive again, per §0b
  06-open-questions.md       unresolved decisions needing team/guide input
  07-sim-setup.md            hands-on setup guide, stage by stage — start at Stage 1
sim/               # empty — previous ArduPilot/Webots scripts deleted, ready for Gazebo work
src/               # (future) perception + control code
hardware/          # (future) BOM, wiring, 3D-print files, calibration data
logs/              # (future) flight logs, .BIN, video, analysis notebooks
```

---

## 7. Conventions & hard-won gotchas (inherited — do not relearn these)

These cost the senior real debugging time on their ArduPilot system (thesis Ch. 6.7).
Items 0–2c are ArduPilot-MAVLink-specific and apply directly again now that we're back on
ArduPilot (2026-09-09, §0b); items 3–9 are about the vision pipeline itself and apply
regardless of autopilot.

0. ⚠️ `LANDING_TARGET` frame must be `MAV_FRAME_BODY_FRD` (or `LOCAL_FRD`), not
   `MAV_FRAME_BODY_NED` as the thesis states — ArduPilot rejects that outright.
1. MAVLink timestamp needed **boot-relative microseconds**, not Unix epoch.
2. Angular-size fields had to be non-zero for ArduPilot's `LANDING_TARGET` rejection logic.
2b. A new MAVLink link was silent until streams were requested per-link.
2c. `PLND_ENABLED` needed a reboot to take effect — an ArduPilot enable-flag quirk.
3. **PnP solver** — use `SOLVEPNP_ITERATIVE`. `SOLVEPNP_IPPE_SQUARE` (the "correct"
   choice for square planar markers) gave garbage on the IMX219. Huge reprojection error
   = wrong solver, not bad calibration. **Still applies** — this is an OpenCV/camera fact,
   not an autopilot fact.
4. **Sign convention** — `pupil-apriltags` uses y-**up**; OpenCV uses y-**down**. Mixing
   them yields plausible magnitudes with *inverted direction* — the worst kind of control bug.
   Body-frame mapping used: `x_body = −y_cam − x_offset`, `y_body = +x_cam − y_offset`,
   `altitude = z_cam`. **Still applies** — and we're using `cv2.aruco` throughout anyway
   (§4), which sidesteps this specific trap by staying in one convention.
5. **Do not pass distortion coefficients to solvePnP** after already undistorting the
   frame — that double-corrects. Pass a null distortion vector. **Still applies.**
6. **Lock the camera sensor mode.** `picamera2` may silently pick a cropped mode whose
   effective focal length invalidates your calibration. Lock binned full-FOV
   (1640×1232 → 640×480). **Still applies**, once we're on real Pi hardware.
7. **Square-marker pose ambiguity** — normalise the rotation so the marker Z-axis points
   *toward* the camera, else tilt sign flips frame to frame. **Still applies.**
8. **Vibration jump filter is mandatory under power.** Reject any frame whose horizontal
   position moves >30 cm or altitude >40 cm from the last accepted frame, unless that
   frame is >0.5 s stale. **Still applies** — this is a control-loop fact independent of
   which autopilot is consuming the corrected position.
9. **Calibrate against a pixel-accurate printed target.** A chessboard with tiny gaps
   between squares gives internally-consistent but metrically-wrong intrinsics.
   **Still applies.**

---

## 8. ArduPilot parameters (directive again, per §0b — starting point, not gospel)

What the senior tuned and why. Directly applicable now that we're back on ArduPilot
(2026-09-09) — use as the starting point for our own tuning, not an assumption that these
exact values suit our airframe/build unchanged.

| Param | Value | Note |
|---|---|---|
| `PLND_ENABLED` | 1 | |
| `PLND_TYPE` | 1 | companion computer / MAVLink |
| `PLND_EST_TYPE` | 1 | |
| `PLND_STRICT` | 1 | |
| `PLND_LAG` | 0.25 s | measured end-to-end pipeline latency |
| `PLND_CAM_POS_X/Y/Z` | Z = −0.05 m | camera offset from CG |
| `PLND_ACC_P_NSE` | (left at default **2.5**) | thesis's #1 recommendation: reduce this — too aggressive → underdamped descent |
| `LAND_SPEED` | 30 cm/s | |
| `WPNAV_SPEED_DN` | 30 cm/s | |

⚠️ Arming checks were relaxed during the senior's bench/tethered work. **Re-enabling the
full ArduPilot arming-check suite is a non-negotiable gate before any unsupervised outdoor
autonomous flight.**

---

## 9. Rules for AI assistants working in this repo

- **Safety first.** This is a flying vehicle with exposed 5-inch props on a 4S pack.
  Never suggest disabling arming checks, failsafes, or geofence for convenience.
  Every new control behaviour gets tested in sim → tethered → hover-observe → LAND,
  in that order. The senior's LOITER-then-LAND methodology applies directly again — we're
  back on ArduPilot (§0b), so LOITER/LAND are the actual modes to use, not a principle to
  reinterpret for another autopilot.
- **Simulate before flying.** Prefer changes that can be validated in Gazebo first.
- **Don't switch autopilot stacks again without being asked.** ArduPilot is now the
  decision (reverted 2026-09-09, §0b) — don't drift back to PX4 suggestions out of habit
  from the intervening two weeks of PX4-flavored context.
- **QGroundControl** is the GCS of record (decided 2026-09-09; Mission Planner rejected —
  Windows-only, doesn't fit our macOS/Linux machines).
- **Port ROLAND's *ideas*, not blindly its code** (its repo is stale — ROS1/Gazebo
  Classic/2023, see `docs/02-roland-reference.md`). Its *stack choice* (PX4) no longer
  matches ours — treat its EKF/UWB/controller math and its ArduPilot-vs-PX4 integration
  details the same way: useful architecture to port, not configuration to copy directly.
- **Keep `docs/` current.** When a decision in §4 or `docs/06-open-questions.md` is
  resolved, update it in the same commit as the code.
- **Don't set up Gazebo/ArduPilot/ROS 2 speculatively.** Do it when actually asked to start that
  stage of work, not preemptively.
- Units: metres, seconds, radians internally; degrees only in UI/report text.
- Frames: state the frame in every variable name (`_cam`, `_body`, `_ned`, `_marker`).

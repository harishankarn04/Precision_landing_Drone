# 04 — Roadmap & Checklist

Status legend: ⬜ not started · 🔄 in progress · ✅ done · ⛔ blocked · ➖ dropped

## Governing decisions (2026-08-14)

1. **Simulation first, hardware last.** No parts are bought and no airframe is built until
   a drone lands itself in simulation. A working sim is also the evidence that justifies
   spending on parts (and parts will be **salvaged** where possible).
2. **The official Jul–Nov 2026 timeline is nominal.** Correctness over calendar.
3. **Beacon comes after a flying sim drone.** It makes no sense to fuse a beacon into a
   system that cannot yet land. Order: *sim works → landing works in sim → beacon in sim*.
4. We inherit **no airframe and no code**. The thesis is a specification we build from.
5. Development happens on Hari's M1; results-grade runs happen on the teammate's Linux Mint
   + 40-series GPU box, via GitHub.

**Step-by-step setup instructions live in [`07-sim-setup.md`](07-sim-setup.md).**

---

## Stage 1 — A simulated drone flies ✅ **DONE 2026-08-20**

Passed with more than planned: not just a flying drone, but a **closed-loop precision
landing commanded from a ROS 2 node** — 2.03 m start offset → **0.01 m** touchdown,
`PrecLand: Target Found` confirmed. See `../ros2_ws/README.md`.

The target still comes from a stand-in node, not a camera. That is Stage 2.

### Original checklist

Mac, native. No Gazebo, no ROS, no VM, no money.

- [x] Clone ArduPilot → `~/Documents/gitClone/ardupilot`
- [x] Project venv on **Python 3.13** (`.venv/`) with `pymavlink`, `MAVProxy`,
      `opencv-contrib-python` 5.0, `numpy`
- [x] Verify `cv2.aruco.DICT_APRILTAG_36h11` + `CORNER_REFINE_APRILTAG` available
- [x] `./waf configure --board sitl`
- [ ] `./waf copter` build completes 🔄
- [ ] Install QGroundControl (macOS `.dmg`); connect to `UDP 127.0.0.1:14550`
- [ ] `sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550` runs
      *(no `--map` / `--console` — they need wxPython, painful on Apple Silicon)*
- [ ] First flight: `mode GUIDED` → `arm throttle` → `takeoff 10` → `mode LAND`
- [ ] Learn: flight modes · parameters · `.param` save/load · `.BIN` logs in MAVExplorer ·
      arming checks · geofence + failsafes
- [ ] **Gate ⭐ — `PLND_TYPE=4` + `SIM_PLD_ENABLE=1`: SITL precision-lands using its own
      simulated target.** *Do not write perception code until this passes* — it proves
      ArduPilot's half of the chain is right, so later failures are unambiguously ours.

## Stage 2 — Precision landing with a synthetic camera ⭐ *the main event*

Still Mac-only. This is where the senior's system gets rebuilt, and where we pass the point
his thesis stopped at.

- [ ] `sim/board.py` — generate the multi-scale board: 24 cm tag36h11 ID 0 at centre, four
      8 cm tags IDs 1–4 at (±0.22, ±0.22) m on a 60 cm board. Emit both a texture and a
      print-ready PDF (the PDF is needed much later, for hardware)
- [ ] `src/frame_source.py` — **the swappable interface.** `SyntheticSource` /
      `GazeboSource` / `PiCameraSource`. **Build this first** — it is what lets the same
      pipeline run on the Mac, in Gazebo, and on real hardware unchanged
- [ ] `sim/synthetic_camera.py` — vehicle pose → 640×480 image via `cv2.warpPerspective`,
      using the thesis intrinsics (`fx=497.88, fy=496.68, k1=0.140, k2=−0.266`) as a stand-in
- [ ] `src/detector.py` — tag36h11 detection; decision-margin filter; per-tag `solvePnP`
      with **`SOLVEPNP_ITERATIVE`**; **null distortion vector** (frame is pre-undistorted)
- [ ] `src/fusion.py` — quality-weighted fusion `sqrt(area) × margin × edge_safety`;
      **disagreement metric used as an output gate, not just a log line**
- [ ] `src/transforms.py` — camera → body NED, with camera offset compensation
- [ ] `src/jump_filter.py` — >30 cm horizontal / >40 cm altitude / 0.5 s staleness rule
- [ ] `src/mavlink_out.py` — `LANDING_TARGET` sender. **Boot-relative µs timestamps** and
      **non-zero angular-size fields**, or ArduPilot silently drops every message
- [ ] `sim/run_landing.py` — the closed loop
- [ ] **Gate ⭐⭐ — autonomous touchdown on the board in simulation**
- [ ] `sim/campaign.py` — N landings from randomised offsets; log touchdown error,
      time-to-land, abort count
- [ ] Parameter sweep: `PLND_ACC_P_NSE` (thesis says reduce it), `PLND_LAG`, `LAND_SPEED`
- [ ] **Gate ⭐⭐⭐ — 20+ runs with reported mean / σ / max touchdown offset.**
      *This already exceeds the senior's thesis, which never closed the loop.*
- [ ] Fallback ladder L1–L5 implemented and each rung deliberately demonstrated
      (`05-risks-and-failsafes.md` §B1)

## Stage 3 — Gazebo: real imagery and physics

Dev in a UTM VM on the Mac (5–15 fps is fine for "does it work?"); real runs on the
teammate's GPU box via GitHub.

- [ ] Ubuntu 24.04 arm64 in UTM (8 GB RAM, 6 CPU, 40 GB disk — ⚠️ Mac has ~60 GB free)
- [ ] Gazebo Harmonic + ArduPilot SITL + `ardupilot_gazebo` plugin; a copter flies
- [ ] ROS 2 Jazzy — **only** for `ros_gz_bridge` to get camera frames into OpenCV
      *(⚠️ on Linux Mint, set the apt codename to `noble` manually)*
- [ ] World: ground plane, textured tag board at exact scale, downward camera sensor
- [ ] Implement `GazeboSource` — **no other file should need to change**
- [ ] **Gate ⭐ — same pipeline, Gazebo camera, comparable landing performance**
- [ ] Re-run the Stage-2 campaign with real rendering; compare synthetic vs Gazebo results
- [ ] Environment sweeps: lighting, sun angle, surface texture, motion blur

## Stage 4 — Beacon in simulation *(deferred until Stage 3 passes)*

- [ ] Decide the modality (`06-open-questions.md` Q3 — UWB recommended). **Deferrable now
      precisely because we're in sim**: no procurement needed to prototype it
- [ ] Port ROLAND's `gazebosensorplugins/src/UwbPlugin.cpp`: Gazebo Classic → `gz-sim`
- [ ] MATLAB: design and validate the EKF (`02-roland-reference.md` §2) — 6-state world
      frame, range update, position update. Prove convergence *before* writing flight code
- [ ] Implement the EKF in the pipeline; fuse beacon range with vision
- [ ] **Gate ⭐ — target still localised with the marker fully occluded**

## Stage 5 — Moving platform in simulation

- [ ] Ground vehicle model + commanded trajectory (ROLAND's `mobile_move_traj.py` pattern)
- [ ] Velocity estimator + lag compensation active
- [ ] ROLAND's tracking/landing state machine; `V_z ∝ (T_xy − ‖e_xy‖)` descent rule
- [ ] Commit-altitude and abort policy (`05-risks-and-failsafes.md` F12)
- [ ] 0.2 → 0.5 → 0.8 m/s
- [ ] **Gate ⭐ — autonomous landing on a moving platform in simulation**

## Stage 6 — Hardware *(not before Stage 3 passes; ideally not before Stage 5)*

Parked deliberately. When we get here: salvage parts where possible, and revisit the BOM
notes below. The three upgrades worth insisting on are marked ⚡.

- [ ] Revisit BOM · assemble · ArduCopter flash · sensor + ESC calibration · PID tune
- [ ] Pipeline onto the companion computer **from git**, as a systemd service
- [ ] **Calibrate our own camera** (RMS < 0.3 px, pixel-accurate target) and **lock the
      sensor mode** — the calibration is meaningless without it
- [ ] Bench: verify every sign convention by physically translating the board
- [ ] Ground-truth measurement rig (`05-risks-and-failsafes.md` A1)
- [ ] **Re-enable the full ArduPilot arming-check suite** — non-negotiable
- [ ] Bench (props off) → tethered → LOITER-observe → LAND
- [ ] **Gate ⭐ — N ≥ 20 autonomous touchdowns on hardware with ground-truth statistics**

---

## BOM notes — **parked, revisit at Stage 6**

Recorded now so the reasoning isn't lost. Deltas from the thesis marked ⚡.

| Item | Thesis baseline | Note for later |
|---|---|---|
| Airframe | iFlight XL5, 5", Quad-X | 7" worth considering: more payload, longer hover, steadier. Salvage decides this |
| Flight controller | Matek H743-SLIM V3 | Any H743 with ≥5 UARTs + dual IMU |
| ESC / motors | Skystars KM50 50 A · Xing 2207 2450 KV | Match the frame |
| Battery | 4S Li-ion 4200 mAh | Li-ion for endurance on long test flights |
| Companion | Pi 4B 8 GB (12–18 fps) | ⚡ Pi 5 if fps proves binding — **measure in sim first** |
| **Camera** | Pi Cam V2 IMX219, **rolling shutter** | ⚡ **Global shutter (IMX296 / OV9281)** — rolling shutter is a hard blocker for moving platforms |
| **Rangefinder** | *absent* | ⚡ **TF-Luna / TFmini-S** — cheap, unblocks the sub-0.6 m descent |
| **Optical flow** | 3901-L0X fitted, never configured | ⚡ Fit **and configure** it |
| GNSS / radio | GEPRC M10-DI · 915 MHz ELRS | ELRS manual override is the primary safety mechanism |
| Beacon | *absent* | Pending Q3 — prototype it in sim before buying anything |
| Tag board | 60 cm printed | Matte/anti-glare laminate, rigid backing, scale verified with calipers |

---

## Report-ready artefacts to collect as you go

Do **not** leave these to the end. Stages 2–3 alone can fill a Phase-1 report.

- [ ] System architecture + per-frame pipeline diagrams
- [ ] Detection-range vs altitude curve, per tag size (synthetic and Gazebo, overlaid)
- [ ] Disagreement vs altitude vs tag-count plot
- [ ] Landing-error scatter plot — the headline figure
- [ ] `PLND_ACC_P_NSE` sweep: descent damping vs parameter value
- [ ] Marker-loss recovery timeline; failure-mode demonstration clips
- [ ] EKF error + covariance-consistency plots (MATLAB, Stage 4)
- [ ] Screen recordings of every headline sim result

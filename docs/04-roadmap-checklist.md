# 04 — Roadmap & Checklist

Status legend: ⬜ not started · 🔄 in progress · ✅ done · ⛔ blocked · ➖ dropped

## Governing decisions

1. **Simulation first, hardware last.** No parts are bought and no airframe is built until
   a drone lands itself in simulation. A working sim is also the evidence that justifies
   spending on parts (and parts will be **salvaged** where possible).
2. **The official Jul–Nov 2026 timeline is nominal.** Correctness over calendar.
3. **Beacon comes after a flying sim drone.** Order: *sim works → landing works in sim →
   beacon in sim*. Doesn't make sense to fuse a beacon into a system that can't yet land.
4. We inherit **no airframe and no code**. The senior's thesis is a perception-pipeline
   specification we build from, not a system we take over.
5. **Stack (as of 2026-09-09): Gazebo + ArduPilot + QGroundControl, ROS 2 status open.**
   Simulator changed from Webots to Gazebo 2026-08-26 (kept); autopilot changed from
   ArduPilot to PX4 the same day, then reverted back to ArduPilot 2026-09-09 — see
   `CLAUDE.md` §0/§0b for why. GCS is QGroundControl (Mission Planner was considered and
   rejected — Windows-only). **ROS 2 is no longer assumed** — nothing in the pipeline
   technically needs it; default plan is `gz-transport` directly for the Gazebo camera feed
   instead of `ros_gz_bridge` (`06-open-questions.md` Q11). Gazebo + ROS 2 Humble are
   already installed in Hari's Parallels Ubuntu VM (ROS 2 may end up unused), but nothing
   ArduPilot/Gazebo-specific exists in this repo yet; everything below is reset to unstarted.

---

## Stage 0 — Get the new stack running ⬜ *(current — nothing built yet)*

> ⚠️ **Numbering note:** this file's stages assume Gazebo from Stage 0 onward. The more
> detailed hands-on guide, `07-sim-setup.md`, breaks the same work into its own Stages
> 1–2 (Mac-native, ArduPilot SITL only, no Gazebo) before its Stage 3 (Gazebo, Linux) — the
> two docs use the same word "Stage" for different granularity. When actually doing the
> work, follow `07-sim-setup.md`'s finer staging; treat this file's "Stage 0" as covering
> its Stages 1–3 combined.

No detailed steps here yet, deliberately — write them as they're actually verified, not
ahead of time. The shape of it, matching what worked before conceptually:

- [ ] Gazebo installed and a stock world/vehicle runs
- [ ] ArduPilot SITL builds and flies in that world with no custom code — arm, takeoff,
      land, via the `ardupilot_gazebo` plugin or whatever route proves out
- [ ] QGroundControl connects to SITL and can command/monitor the vehicle
- [ ] Vehicle commanded and read via MAVLink/`pymavlink` (default — no ROS 2 needed for
      this). Only add a ROS 2 bridge (`mavros` or similar) if `06-open-questions.md` Q11
      resolves to "ROS 2 is actually required"
- [ ] **Gate ⭐ — a drone flies a basic mission in Gazebo, commanded via MAVLink, with no
      perception code involved.** This is the equivalent of last time's Stage 1, on the
      new stack. Don't move on until this is genuinely solid — the previous attempt's
      biggest time sink was chasing bugs it hadn't fully isolated at this simplest layer.

> 💡 **Future idea, not yet started (2026-09-09):** the non-Gazebo portion of this work
> (`07-sim-setup.md` Stages 1–2 — ArduPilot SITL + synthetic-camera perception, pure Mac
> native Python) is headless-friendly and a good future GitHub Actions candidate: run it on
> every push so a `git pull` on Linux starts from already-validated code. Deliberately out
> of scope for CI: actual Gazebo runs — GitHub-hosted runners have no GPU, and a self-hosted
> runner is more setup than this project needs right now. Run it on **both x86_64 and
> arm64 runners** when it exists — Hari's dev VM is arm64, teammates are on x86_64, and
> that split is a real cross-arch build risk (`07-sim-setup.md` Stage 3). Not built yet;
> recorded here so the idea isn't lost.

## Stage 1 — Precision landing with our own perception ⭐ *the main event*

Same goal as before, same perception spec (`docs/01-inherited-system.md` §3), new autopilot
underneath.

- [ ] Represent the multi-scale AprilTag board (24 cm tag36h11 ID 0 centre + four 8 cm
      IDs 1–4 at ±0.22 m on a 60 cm board) in the Gazebo world
- [ ] A downward camera in Gazebo, read via `gz-transport` directly (default — no ROS 2,
      per Q11), feeding **a swappable frame-source interface** — build this abstraction
      first so the same detection code later runs against Gazebo, a different sim, or real
      hardware unchanged
- [ ] Detection: tag36h11, decision-margin filter, per-tag `solvePnP`
      (`SOLVEPNP_ITERATIVE` — verified necessary on the IMX219; re-verify on whatever
      camera model Gazebo uses, don't assume)
- [ ] Fusion: quality-weighted multi-tag combine, with a disagreement metric used as an
      **output gate**, not just a log line
- [ ] Get the fused position into ArduPilot's control loop via `LANDING_TARGET` and
      `PLND_*` params (`CLAUDE.md` §7 items 0–2c, §8) — verify these on the Gazebo SITL
      build the same way the senior verified them on hardware, don't assume they carry over
- [ ] **Gate ⭐⭐ — autonomous touchdown on the board in simulation**
- [ ] Scripted N-landing campaign from randomised offsets → touchdown error, time-to-land,
      abort count
- [ ] Descent-response tuning sweep — start from `PLND_ACC_P_NSE` (senior's #1 tuning
      recommendation: reduce it, `CLAUDE.md` §8)
- [ ] **Gate ⭐⭐⭐ — 20+ runs with reported mean / σ / max touchdown offset.**
      *This already exceeds the senior's thesis, which never closed the loop.*
- [ ] Fallback ladder L1–L5 implemented and each rung deliberately demonstrated
      (`05-risks-and-failsafes.md` §B1) — `PLND_STRICT`, `PLND_RET_BEHAVE`,
      `PLND_RET_MAX`, `PLND_ALT_MIN/MAX`, `PLND_TIMEOUT`, `PLND_OPTIONS`

## Stage 2 — Beacon in simulation *(deferred until Stage 1 passes)*

- [ ] Decide the modality (`06-open-questions.md` Q3 — UWB recommended)
- [ ] Port ROLAND's `gazebosensorplugins/src/UwbPlugin.cpp` — a Gazebo-Classic→gz-sim
      port either way; ROLAND's own stack is PX4, ours is ArduPilot, so the sensor plugin
      itself ports the same but any autopilot-side glue does not
- [ ] MATLAB: design and validate the EKF (`02-roland-reference.md` §2) before writing
      flight code
- [ ] Implement the EKF in the pipeline; fuse beacon range with vision
- [ ] **Gate ⭐ — target still localised with the marker fully occluded**

## Stage 3 — Moving platform in simulation

- [ ] Ground vehicle model + commanded trajectory (ROLAND's `mobile_move_traj.py` pattern —
      the trajectory driver itself is autopilot-independent, trivially portable)
- [ ] Velocity estimator + lag compensation active
- [ ] ROLAND's tracking/landing state machine; `V_z ∝ (T_xy − ‖e_xy‖)` descent rule
- [ ] Commit-altitude and abort policy (`05-risks-and-failsafes.md` F12)
- [ ] 0.2 → 0.5 → 0.8 m/s
- [ ] **Gate ⭐ — autonomous landing on a moving platform in simulation**

## Stage 4 — Hardware *(not before Stage 1 passes; ideally not before Stage 3)*

Parked deliberately. When we get here: salvage parts where possible, revisit the BOM notes
below, and confirm the chosen flight-controller board actually supports ArduPilot (the
senior's Matek H743-SLIM V3 already does — see thesis hardware inventory,
`01-inherited-system.md` §1).

- [ ] Revisit BOM · assemble · flash ArduPilot · sensor + ESC calibration · tuning
- [ ] Pipeline onto the companion computer **from git**, as a service
- [ ] **Calibrate our own camera** (RMS < 0.3 px, pixel-accurate target) and lock the
      sensor mode — the calibration is meaningless without it
- [ ] Bench: verify every sign convention by physically translating the board
- [ ] Ground-truth measurement rig (`05-risks-and-failsafes.md` A1)
- [ ] **Re-enable the full ArduPilot arming-check suite** — non-negotiable
- [ ] Bench (props off) → tethered → hover-observe → land
- [ ] **Gate ⭐ — N ≥ 20 autonomous touchdowns on hardware with ground-truth statistics**

---

## BOM notes — **parked, revisit at Stage 4**

Recorded now so the reasoning isn't lost. Deltas from the thesis marked ⚡. The flight
controller entry is back to straightforward now that ArduPilot is the target again.

| Item | Thesis baseline | Note for later |
|---|---|---|
| Airframe | iFlight XL5, 5", Quad-X | 7" worth considering: more payload, longer hover, steadier. Salvage decides this |
| Flight controller | Matek H743-SLIM V3 (ran ArduPilot) | Already a known-good ArduPilot board — the senior flew it. Any capable FC with ≥5 UARTs + dual IMU works if we don't salvage this exact one |
| ESC / motors | Skystars KM50 50 A · Xing 2207 2450 KV | Match the frame |
| Battery | 4S Li-ion 4200 mAh | Li-ion for endurance on long test flights |
| Companion | Pi 4B 8 GB (12–18 fps) | ⚡ Pi 5 if fps proves binding — **measure in sim first** |
| **Camera** | Pi Cam V2 IMX219, **rolling shutter** | ⚡ **Global shutter (IMX296 / OV9281)** — rolling shutter is a hard blocker for moving platforms |
| **Rangefinder** | *absent* | ⚡ **TF-Luna / TFmini-S** — cheap, unblocks the sub-0.6 m descent |
| **Optical flow** | 3901-L0X fitted, never configured | ⚡ Fit **and configure** it |
| GNSS / radio | GEPRC M10-DI · 915 MHz ELRS | ELRS manual override is the primary safety mechanism, autopilot-agnostic |
| Beacon | *absent* | Pending Q3 — prototype it in sim before buying anything |
| Tag board | 60 cm printed | Matte/anti-glare laminate, rigid backing, scale verified with calipers |

---

## Report-ready artefacts to collect as you go

Do **not** leave these to the end. Stage 1 alone can fill a Phase-1 report.

- [ ] System architecture + per-frame pipeline diagrams
- [ ] Detection-range vs altitude curve, per tag size
- [ ] Disagreement vs altitude vs tag-count plot
- [ ] Landing-error scatter plot — the headline figure
- [ ] Descent-tuning parameter sweep: damping vs `PLND_ACC_P_NSE` value
- [ ] Marker-loss recovery timeline; failure-mode demonstration clips
- [ ] EKF error + covariance-consistency plots (MATLAB, Stage 2)
- [ ] Screen recordings of every headline sim result

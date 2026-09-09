# Precision Landing Drone

**Design and Development of a Beacon-Based Autonomous Precision Landing System for
Unmanned Aerial Vehicles on Static and Moving Platforms**

B.Tech Project Phase 1 (23ECE498 / 23EAC498) · AY 2025–2026 · Batch B001
Amrita School of Engineering, Bengaluru — Dept. of ECE
Guide: Dr. Sreeja Kochuvila

Team: Nandakrishna J · A Somesh Ayyappan · Hari Shankar N

---

## What this is

GNSS gives 1–3 m of horizontal accuracy near the ground — useless when the landing pad is
40 cm wide. This project closes that gap: a downward-facing camera plus a beacon, fused
onboard, feeding relative-position corrections to a flight controller so a multirotor can
land accurately on a pad — first static, then **moving**.

We rebuild a completed M.Tech thesis's system (AprilTag-based vision pipeline, validated on
real hardware but never closed-loop) and extend it toward the architecture of the ROLAND
paper (beacon + EKF + moving platform), which was itself only ever validated in simulation.

## ⚠️ Stack pivot (2026-09-09): back to Gazebo + ArduPilot (ROS 2 status: open question)

An earlier attempt used ArduPilot SITL + Webots and got a closed-loop landing working, but
hit enough friction (bugs in ArduPilot's own example files, a Webots crash) that the plan
changed on 2026-08-26 to Gazebo + ROS 2 + PX4. Two weeks later, the autopilot half of that
pivot reversed: **current stack is Gazebo + ArduPilot** — Webots is still dropped, PX4 is
not. Ground control station is **QGroundControl** (Mission Planner was considered and
rejected — Windows-only). **ROS 2 is no longer a settled requirement** — nothing in the
pipeline technically needs it (ArduPilot↔Gazebo and ArduPilot↔pipeline both go via MAVLink,
not ROS 2); default plan is to build ROS2-free using `gz-transport` for the Gazebo camera
feed, and confirm what the professor's ROS 2 ask actually means before Stage 3
(`docs/06-open-questions.md` Q11). Gazebo + ROS 2 Humble happen to already be installed in
a Parallels Ubuntu VM on Hari's Mac, but that's just available tooling, not a commitment to
use ROS 2 — nothing ArduPilot/Gazebo-specific is wired up in this repo yet. Full reasoning:
**[`CLAUDE.md`](CLAUDE.md) §0 and §0b.**

## 🔴 Governing rule: simulation first, hardware last

No parts are bought and no airframe is built until **a drone lands itself in simulation**.
We inherit no airframe and no code — the thesis is a *specification* we build from. A
working simulation is also the evidence that justifies spending on parts.

```
Stage 1  drone flies in Gazebo/ArduPilot  🐧 Linux (Gazebo)     ⬜ not started
Stage 2  ⭐ drone lands on the tag board  our own perception    ← the main event
Stage 3  beacon (UWB) fused in
Stage 4  moving platform
Stage 5  hardware, salvaging parts
```

**"Base level" goal:** closed-loop autonomous touchdown with real statistics.
The senior's thesis never got there, and ROLAND was never flown at all.

---

## Start here

| Read | For |
|---|---|
| **[`CLAUDE.md`](CLAUDE.md)** | Full project context, and **the current plan** — read this first, always |
| [`docs/00-project-brief.md`](docs/00-project-brief.md) | Objectives, official timeline, where the contribution lies |
| [`docs/01-inherited-system.md`](docs/01-inherited-system.md) | The senior's system: perception spec, results, gaps. Firmware details are historical (ArduPilot); the perception logic is still the reference |
| [`docs/02-roland-reference.md`](docs/02-roland-reference.md) | ROLAND's EKF + controller — PX4-based, no longer stack-matched to us, and an honest assessment of its repo |
| [`docs/03-simulation-plan.md`](docs/03-simulation-plan.md) | Simulator/autopilot plan — being rewritten for Gazebo+ArduPilot |
| [`docs/04-roadmap-checklist.md`](docs/04-roadmap-checklist.md) | Task checklist — being rewritten for Gazebo+ArduPilot |
| [`docs/05-risks-and-failsafes.md`](docs/05-risks-and-failsafes.md) | Missing pieces, failure modes, the fallback ladder, safety gates |
| [`docs/06-open-questions.md`](docs/06-open-questions.md) | Open decisions — deferred to the stage where they matter |
| [`docs/07-sim-setup.md`](docs/07-sim-setup.md) | Hands-on setup guide, stage by stage — start at Stage 1 |

Source material lives in `Documentation/` (three PDFs, read-only).

---

## System at a glance (perception spec — autopilot-agnostic)

```
Downward camera ─► companion computer ─► AprilTag detect ─► multi-tag fusion
                                                                    │
                                                     relative-position correction
                                                                    ▼
                                                          flight controller (ArduPilot)
                                                     └─► autonomous descent + landing
```

Landing target: 60 cm board — one 24 cm tag36h11 at centre, four 8 cm tags at the corners.
The multi-scale layout keeps the target detectable from 8.5 m all the way to touchdown.
This design came from the senior's ArduPilot-based system and carries over unchanged; the
MAVLink integration on the right side of the diagram is being rebuilt fresh on Gazebo, same
autopilot as the senior's system this time.

---

## Ground rules

- **Simulate first.** Nothing new commands the aircraft until it has flown in Gazebo.
- **Bench → hover-observe → land.** Never skip a validation rung.
- **ArduPilot, not PX4** (reverted 2026-09-09). No longer matches ROLAND's stack (PX4) —
  port ROLAND's ideas, don't expect direct config compatibility.
- **QGroundControl** is the GCS of record (cross-platform; Mission Planner is Windows-only).
- **Manual override always live**, every autonomous flight — ELRS + a mode switch, same
  mechanism the senior used.
- Full arming checks stay **enabled** for any unsupervised outdoor flight.
- **Don't set up Gazebo/ArduPilot/ROS 2 speculatively** — build it when the work actually starts.

---

## Repo layout

```
Documentation/   source PDFs (read-only)
docs/            working documents — start here
sim/             empty — previous ArduPilot/Webots scripts removed; ready for Gazebo work
src/             (future) perception + control code
hardware/        (future) BOM, wiring, 3D prints, calibration data
logs/            (future) flight logs, video, analysis
```

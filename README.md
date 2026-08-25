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
onboard, feeding relative-position corrections to an ArduPilot flight controller so a
multirotor can land accurately on a pad — first static, then **moving**.

We rebuild a completed M.Tech thesis's system (AprilTag-based vision pipeline, validated on
real hardware but never closed-loop) and extend it toward the architecture of the ROLAND
paper (beacon + EKF + moving platform), which was itself only ever validated in simulation.

## 🔴 Governing rule: simulation first, hardware last

No parts are bought and no airframe is built until **a drone lands itself in simulation**.
We inherit no airframe and no code — the thesis is a *specification* we build from. A
working simulation is also the evidence that justifies spending on parts.

```
Stage 1  drone flies in SITL              🖥️ Mac, no Gazebo   🔄 in progress
Stage 2  ⭐ drone lands on the tag board  🖥️ Mac, no Gazebo   ← the main event
Stage 3  Gazebo: real imagery + physics   🐧 Linux
Stage 4  beacon (UWB) fused in            🐧 Linux
Stage 5  moving platform                  🐧 Linux
Stage 6  hardware, salvaging parts
```

**"Base level" goal:** closed-loop autonomous touchdown with real statistics.
The senior's thesis never got there, and ROLAND was never flown at all.

👉 **New to simulation? Start at [`docs/07-sim-setup.md`](docs/07-sim-setup.md).**

---

## Start here

| Read | For |
|---|---|
| **[`CLAUDE.md`](CLAUDE.md)** | Full project context — the single file to load in any editor/AI tool |
| [`docs/00-project-brief.md`](docs/00-project-brief.md) | Objectives, official timeline, where the contribution lies |
| [`docs/01-inherited-system.md`](docs/01-inherited-system.md) | The senior's system: hardware, pipeline, results, and its gaps |
| [`docs/02-roland-reference.md`](docs/02-roland-reference.md) | ROLAND's EKF + controller, and an honest assessment of its repo |
| [`docs/03-simulation-plan.md`](docs/03-simulation-plan.md) | **ROS/Gazebo vs MATLAB vs macOS vs Linux — answered** |
| [`docs/04-roadmap-checklist.md`](docs/04-roadmap-checklist.md) | Sprint checklist + procurement shortlist |
| [`docs/05-risks-and-failsafes.md`](docs/05-risks-and-failsafes.md) | Missing pieces, failure modes, the fallback ladder, safety gates |
| [`docs/06-open-questions.md`](docs/06-open-questions.md) | Open decisions — all currently deferred to the stage where they matter |
| **[`docs/07-sim-setup.md`](docs/07-sim-setup.md)** | 🎯 **Hands-on setup guide — start here if you've never used a simulator** |

Source material lives in `Documentation/` (three PDFs, read-only).

---

## System at a glance

```
Pi Camera (downward) ─► Raspberry Pi 4B ─► AprilTag detect ─► multi-tag fusion
                                                                    │
                              MAVLink LANDING_TARGET (UART 921600)  │
                                                                    ▼
                                              Matek H743 / ArduCopter, PLND_TYPE=1
                                                     └─► LAND-mode position correction
```

Landing target: 60 cm board — one 24 cm tag36h11 at centre, four 8 cm tags at the corners.
The multi-scale layout keeps the target detectable from 8.5 m all the way to touchdown.

---

## Ground rules

- **Simulate first.** Nothing new commands the aircraft until it has flown in SITL.
- **Bench → LOITER-observe → LAND.** Never skip a rung.
- **ArduPilot, not PX4.** ROLAND is PX4; we port its ideas, not its code.
- **Manual override always live** on the ELRS link, every autonomous flight.
- Full arming checks stay **enabled** for any unsupervised outdoor flight.

---

## Repo layout

```
Documentation/   source PDFs (read-only)
docs/            working documents — start here
sim/             (future) simulation configs and launch files
src/             (future) perception + control code
hardware/        (future) BOM, wiring, 3D prints, calibration data
logs/            (future) flight logs, video, analysis
```

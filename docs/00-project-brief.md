# 00 — Project Brief

## Official Phase-1 objective (as submitted in the synopsis)

> To design, evaluate, and develop a reliable UAV platform for autonomous precision
> landing by selecting appropriate hardware, integrating the flight control system,
> establishing stable autonomous flight, and preparing the platform for beacon-based
> landing in subsequent phases.

### Sub-objectives (verbatim)
1. Study and analyze the existing UAV platform and identify hardware limitations.
2. Finalize the UAV architecture (frame, motors, ESCs, flight controller, telemetry, power system).
3. Select and procure upgraded components where required.
4. Assemble and integrate the UAV hardware.
5. Configure and calibrate the flight controller using ArduPilot.
6. Perform sensor calibration and PID tuning for stable flight.
7. Validate manual flight and autonomous waypoint navigation.
8. Prepare interfaces for beacon sensors, camera, and companion computer for Phase 2.

### Official time plan
| Month | Planned work |
|---|---|
| Jul 2026 | Literature survey; study existing platform; finalise requirements & architecture; component comparison |
| Aug 2026 | Finalise hardware; **procure** components; design power distribution & comms architecture |
| Sep 2026 | Assemble UAV; integrate Pixhawk/ArduPilot, power, GPS, telemetry, RX, propulsion; firmware config; sensor & ESC calibration; comms testing |
| Oct 2026 | Ground + flight testing; PID tuning; vibration analysis; EKF verification; autonomous waypoint validation |
| Nov 2026 | Validate stable platform; document specs & performance; integrate preliminary interfaces for beacon receiver, global-shutter camera, companion computer → ready for Phase 2 |

**Today is 14 Aug 2026 → we are in the procurement month.** Anything with a lead time
(global-shutter camera, UWB modules, rangefinder, Jetson/Pi 5) should be ordered *this month*
or it will not arrive in time for the Oct flight-test window.

---

## Resolved: we are building from scratch ✅

Confirmed 2026-08-14 — **we inherit no airframe and no source code.** The senior may still
have the code, but we are not planning around it.

**What this means:**

- The 61-page thesis is a **specification**, and an unusually complete one: exact tag
  layout, exact intrinsics, exact PnP solver, exact ArduPilot parameters, and **nine
  documented integration gotchas** that each cost the senior real debugging time
  (`CLAUDE.md` §7). Rebuilding *from* it is a very different proposition from rebuilding blind.
- The synopsis timeline is literal: Aug procure → Sep assemble → Oct tune and fly.
- **Software must be built in simulation, in parallel.** If the perception work waits for
  the airframe, two months evaporate. Simulation moves from optional to critical path.
- **Advantage:** we design the platform correctly the first time instead of inheriting
  compromises. Global-shutter camera, rangefinder, and configured optical flow go in at
  BOM stage rather than being retrofitted. See `04-roadmap-checklist.md`.

**Agreed scope:** rebuild the pipeline → **closed-loop static touchdown with ground-truth
statistics** (the "base level") → add the beacon. Moving platform is a stretch goal decided
later. See `06-open-questions.md` Q5 for the full rung ladder.

---

## Where the real contribution lies

The senior stopped at: *perception validated in hover; closed-loop touchdown not done;
static targets only.* The paper you chose as reference (ROLAND) does: *moving platform,
EKF fusing multiple sensors, target tracked even when out of camera FOV.*

The gap between those two is exactly your project:

| # | Contribution | Difficulty | Novelty | Notes |
|---|---|---|---|---|
| C1 | **Closed-loop autonomous touchdown**, characterised against ground truth over N trials, static pad | Medium | Low (expected) | Non-negotiable — without it there is no landing system. Senior's explicit #1 future-work item. |
| C2 | **Beacon fusion** (UWB or IR) so the target is localised when the marker is out of FOV / occluded | High | Medium-High | This is what makes the title honest and is ROLAND's core idea |
| C3 | **Explicit EKF** fusing IMU + vision + beacon, replacing bare `LANDING_TARGET` hand-off | High | Medium | Senior's future-work item; ROLAND §3.1–3.3 is the template |
| C4 | **Moving-platform landing** | High | Medium | The headline result. Needs C1+C2+C3 first. |
| C5 | **Inclined + moving** (land on a sloped, moving surface) | Very High | High | Senior proved *tilt estimation* works; nobody has closed the loop on it. Genuine research-level stretch goal. |
| C6 | Rangefinder-aided terminal descent (sub-0.6 m, where vision degrades) | Low | Low | Cheap, high-value, unblocks C1 |
| C7 | Full simulation environment enabling all of the above without crashing hardware | Medium | None | **Infrastructure — do this first.** Everything else is faster afterwards. |

Suggested framing for the report: *"ROLAND validated its approach only in Gazebo
simulation and used a PX4/VIO stack. We port the architecture to a low-cost ArduPilot
platform, validate it in both simulation and on real hardware, and add multi-scale
fiducial fusion (which ROLAND lacks) to its UWB+detection EKF."*

That is a defensible, honest, and genuinely publishable framing: **ROLAND is sim-only;
you go to hardware.** (ROLAND's own conclusion says "The real-world experiment is left
as the future works.")

---

## Deliverables to keep in view

- Working simulation (reproducible by anyone on the team, one command)
- Closed-loop landing statistics with ground truth: mean/σ of touchdown offset, N ≥ 20
- Flight logs (.BIN) + companion logs + overlay video for every reported run
- Phase-1 report (Nov 2026) and Phase-2 report
- A conference/journal submission is realistic if C4 lands

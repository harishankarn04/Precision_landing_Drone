# Reference code from Dinesh — read-only, hardware-stage material

Received 2026-09-15. This is Akash Dinesh's actual production script for the senior's
thesis system (`Documentation/Report_Full_v2_No_First_Page.pdf`), not something in the
thesis PDF itself — real code that ran on his Pi 4B + Pi Camera V2 + Matek H743-SLIM V3
hardware.

**Not used yet.** We're still sim-first (`CLAUDE.md` §4 — no parts bought, no airframe
until a drone lands itself in Gazebo). This is kept here as reference for **Stage 1's
perception pipeline** (fusion, velocity estimation, jump filtering) and, later, for the
hardware stage — not code to run as-is now.

## Files

- `drone_test_landing.py` — full precision-landing pipeline: AprilTag detection
  (`pupil_apriltags`/`apriltag`), per-tag `solvePnP`, quality-weighted multi-tag fusion,
  a weighted-least-squares velocity/acceleration estimator with outlier rejection, a
  frame-to-frame jump filter, video recording + CSV logging, and MAVLink `LANDING_TARGET`
  output to ArduPilot.
- `plot_flight_data.py` — reads the CSV `drone_test_landing.py` produces and plots
  trajectory, position/velocity over time, predicted-vs-measured position, and altitude.

Comments in `drone_test_landing.py` (e.g. "extended for demo so prediction leads
visibly") suggest this ran live in front of people — possibly further along than the
thesis's own claim that closed-loop touchdown was never completed. Worth asking Dinesh
directly if that's the case.

## Known discrepancies vs. this project's decisions — don't copy blindly

1. **MAVLink frame bug.** This script sends `LANDING_TARGET` with
   `MAV_FRAME_BODY_NED` (`drone_test_landing.py` ~line 965). `CLAUDE.md` §7 item 0
   documents this as a **known-wrong** value from the thesis — ArduPilot rejects it
   outright and the correct frame is `MAV_FRAME_BODY_FRD` (or `LOCAL_FRD`). Either this
   is an earlier/untested revision of the script, or that gotcha note needs revisiting —
   confirm with Dinesh before trusting either version.
2. **AprilTag library.** Uses `pupil_apriltags`, with a documented empirical fix for the
   y-up/y-down sign flip (comment: "Y FLIPPED -> 42cm disagreement... Y NOT FLIPPED ->
   0.3cm correct"). This project already decided to use `cv2.aruco` +
   `DICT_APRILTAG_36h11` instead specifically to sidestep that class of bug entirely
   (`CLAUDE.md` §4) — port the *fusion logic*, not the detection library.

## What's actually reusable here

The fusion math (multi-tag weighted averaging, `MARKER_OFFSETS` geometry), the velocity
estimator (weighted quadratic fit + outlier rejection), and the jump filter are all
solver/library-agnostic and are the most directly portable parts once Stage 1's
perception code is actually being written (`docs/04-roadmap-checklist.md` Stage 1).

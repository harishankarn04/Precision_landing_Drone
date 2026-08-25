# 01 — Inherited System (Senior's M.Tech Thesis)

Source: `Documentation/Report_Full_v2_No_First_Page.pdf`
Author: **Akash Dinesh**, BL.EN.P2RAU24001, M.Tech Robotics & Automation, May 2026
Title: *Vision-Based Precision Landing System for Autonomous UAVs Using AprilTag Fusion*
Guide: Dr. Sreeja Kochuvila (**same guide as us**)

> ⚠️ **The code is not in this repository.** Obtaining it is task #1.
> See `06-open-questions.md` Q2. Everything below is reconstructible from the thesis
> if the code is lost, but that would cost weeks.

---

## 1. Hardware inventory (thesis Table 3.1)

| Subsystem | Part |
|---|---|
| Airframe | iFlight XL5, 5-inch carbon-fibre, Quad-X |
| Flight controller | **Mateksys H743-SLIM V3** (STM32H743, dual IMU), ArduCopter |
| ESC | Skystars KM50 50 A 4-in-1, DShot |
| Motors | iFlight Xing 2207 2450 KV, 5" tri-blade props |
| Battery | **4S Li-ion 4200 mAh** (chosen over LiPo for endurance during long test flights) |
| Companion computer | **Raspberry Pi 4B 8 GB** (4× Cortex-A72 @1.5 GHz) |
| Camera | **Pi Camera V2 (Sony IMX219)**, 3.04 mm, downward-facing, CSI |
| GNSS | GEPRC M10-DI on a 3D-printed mast |
| Radio | 915 MHz ExpressLRS (RadioMaster TX ↔ onboard RX) — **manual override = primary safety** |
| Power regulation | Hobbywing UBEC → 5 V rail for the Pi (isolated from motor rail) |
| Companion link | MAVLink2 over UART, **SERIAL6 @ 921600 baud** |
| 3D-printed | GPS mast, Pi holder, camera bracket, 4× landing gear |
| **Present but UNUSED** | **MatekSys 3901-L0X optical flow + short-range LiDAR** ← free upgrade sitting on the airframe |

**Physical layout:** Pi on top (thermal + connector reach), battery at centre of mass,
FC at bottom near geometric centre (minimises IMU–CG lever arm), camera protrudes through
the underside on a rigid printed bracket. **The camera bracket rigidity is load-bearing
for the whole coordinate chain** — the CSI ribbon points toward the tail, and the entire
camera→body transform assumes that exact mounting.

---

## 2. Landing target design (thesis §3.7, Table 3.2)

60 cm × 60 cm board, AprilTag family **tag36h11**:

| Tag ID | Position | Size | Offset (x, y) m |
|---|---|---|---|
| 0 | centre | **24 cm** | (0.00, 0.00) |
| 1 | top-left | 8 cm | (−0.22, +0.22) |
| 2 | top-right | 8 cm | (+0.22, +0.22) |
| 3 | bottom-left | 8 cm | (−0.22, −0.22) |
| 4 | bottom-right | 8 cm | (+0.22, −0.22) |

**Why multi-scale:** solves the range-vs-FOV conflict. Big tag decodes at altitude but
leaves the frame near touchdown; small tags are unreadable at altitude but perfect at
terminal descent. Measured ranges:

- Centre 24 cm tag: **8.56 m down to ~0.6 m**
- Corner 8 cm tags: **~3.5 m down to terminal descent**
- Overlap band ~0.6–3.5 m → never a detection gap during nominal descent

**Why tag36h11 over ArUco:** larger code space, higher inter-code Hamming distance,
stronger decoder margins, better resistance to motion blur and partial occlusion.

---

## 3. Software pipeline (thesis Ch. 4)

Language: **Python 3** on Raspberry Pi OS.
Libraries: `picamera2`, `pupil-apriltags`, `OpenCV`, `pymavlink`.
Runs as a **systemd service** — auto-start on boot, restart on unhandled error
(10 s delay, max 5 restarts / 200 s).

Per-frame pipeline:

```
1. Capture   YUV420 640×480 @30 fps, sensor mode LOCKED to binned full-FOV (1640×1232→640×480)
2. Y-plane   slice luminance channel only (free grayscale)
3. Undistort precomputed initUndistortRectifyMap + remap  (~0.5 ms on Pi 4)
4. Detect    tag36h11, edge refinement; reject decision margin < 25 or unexpected ID
5. Pose      per-tag solvePnP, SOLVEPNP_ITERATIVE, per-tag true size (0.24 / 0.08 m),
             NULL distortion vector (frame already undistorted)
6. Fuse      quality-weighted mean of per-tag landing-centre estimates
7. Transform camera frame → body NED, with camera-offset compensation
8. Filter    frame-to-frame jump filter
9. Send      MAVLink LANDING_TARGET, MAV_FRAME_BODY_NED
```

Achieved: **12–18 fps** end-to-end on the Pi 4B. Measured system latency **≈ 0.25 s**
(camera + MAVLink + control response) — this is what `PLND_LAG` is set to.

### 3.1 Camera calibration (§4.4)
Chessboard, OpenCV `calibrateCamera`:
- `fx = 497.88 px`, `fy = 496.68 px`, principal point near image centre
- `k1 = 0.140`, `k2 = −0.266`
- **RMS reprojection error 0.255 px**

The Phase-1 baseline had used placeholder `fx = fy = 530`. Consequence: below 1.5 m the
reported distance was right within a few cm, but **at ~2.8 m the reported altitude was
≈2.7× smaller than truth** → autopilot applied wildly over-aggressive horizontal
corrections → airframe destabilised. Calibration is not optional.

### 3.2 Multi-tag fusion estimator (§4.7)
Each visible tag independently estimates the *same* board-centre point (subtract that
tag's known intra-board offset from its camera-frame translation). Combine by weighted mean:

```
w_i = sqrt(pixel_area_i) × decision_margin_i(normalised 0–1) × edge_safety_i
edge_safety = 1.0 if all four corners ≥15 px from any image edge, else 0.3
```

`sqrt(area)` rather than `area` because pose accuracy scales *sub*linearly with marker
pixel size — a 4× larger tag is not 4× more trustworthy.

**The disagreement metric** = max pairwise distance between per-tag centre estimates.
This is the pipeline's real-time self-check. Crucially it measures **geometric
self-consistency, not absolute accuracy** — an error affecting all tags identically
(e.g. focal-length miscalibration) is invisible to it.

### 3.3 Velocity estimator + lag compensation (§4.8) — **dormant, built for us**
Maintains a short history of landing-centre positions, fits a quadratic
(position/velocity/acceleration) by exponentially-weighted least squares; rejects
residuals beyond 3σ and refits; suppresses physically implausible speeds; reports a
fit-quality (weighted R²). When speed > threshold **and** fit quality is good, it
predicts the target position 0.25 s ahead and sends the *prediction*.

> For static targets this is dormant. **This is the hook for our moving-platform work —
> the machinery already exists and was deliberately built as "the foundation for the
> dynamic-platform work identified as future development."**

### 3.4 Vibration jump filter (§4.10)
Reject a new measurement if horizontal position differs from last accepted by >30 cm,
or altitude by >40 cm — **unless** the last accepted measurement is >0.5 s stale (allows
recovery from genuine target loss). Rationale: airframe vibration + rolling shutter
produces occasional tens-of-cm position spikes; uncorrected, ArduPilot chases each spike,
the correction excites more vibration, and it diverges.

### 3.5 Logging (§4.11, §5.5)
Two parallel streams per flight, correlated offline:
1. **Companion:** detection state, fused position, altitude, velocity, tilt, tag count,
   disagreement, warning counts; periodic annotated snapshots; **overlay-burned video**
   (starts at target acquisition, ends a few seconds after loss)
2. **Flight controller:** DataFlash `.BIN` — platform state, received LANDING_TARGET
   stream, autopilot response

This dual-stream correlation is what made the hard bugs diagnosable. **Keep this practice.**

---

## 4. Test methodology (thesis Ch. 5) — adopt this verbatim

**Bench → LOITER → LAND.** In that order, every time.

1. **Bench:** camera + Pi held over the printed board by hand; translate/rotate/raise/lower
   through *known* displacements. This is what caught the y-up/y-down sign bug — physically
   move the marker forward/right/down and confirm reported body offsets change in the
   expected direction and sign. Supported by two custom diagnostic scripts kept in sync
   with the flight code: a live visual detection tester, and a multi-solver pose-diagnostic
   that cross-checks pose across solvers and tags.
2. **LOITER:** airborne over the board. Autopilot holds position on *its own* sensors and
   **does not act on vision**. LANDING_TARGET messages are received and logged only. This
   is the safe configuration in which the full perception chain — jump filter, disagreement
   metric — was characterised under genuine vibration, real altitude variation, and outdoor light.
3. **LAND:** only now does ArduPilot act on the corrections (PLND is active only in LAND mode).

ELRS manual override available at every instant throughout. Altitude envelope tested:
**1.7 m – 8.6 m**, over both level and inclined boards.

---

## 5. Results (thesis Ch. 6)

### Multi-tag fusion, level board
| Altitude (m) | Tags visible | Disagreement (cm) | Offset X,Y (m) |
|---|---|---|---|
| 1.74 | 5 | 5.0 | (−0.30, +0.01) |
| 2.46 | 5 | 1.6 | (−0.54, −0.18) |
| 3.63 | 2 | 1.1 | (−0.17, −0.01) |
| 8.56 | 1 | — | (−0.80, −0.62) |

### Inclined board — tilt persists under fusion
| Altitude (m) | Tags | Disagreement (cm) | Pitch (°) |
|---|---|---|---|
| 3.36 | 1 | — | −16.9 |
| 2.46 | 5 | 1.6 | −13.7 |
| 1.68 | 5 | 0.9 | −15.8 |
| 0.84 | 5 | 0.8 | −16.0 |
| 2.29 | 5 | 2.0 | −21.8 |

**The key scientific result:** two superficially identical phenomena are separated by the
disagreement metric.
- *Single-tag tilt artifact:* only one small tag visible at altitude → solvePnP is
  ill-conditioned → reported pitch −21.4° on a physically **level** board at 6.28 m. No
  disagreement metric available (needs ≥2 tags) so nothing contradicts it.
- *Genuine surface tilt:* five tags fused, disagreement 0.8–2.0 cm, tilt persists at
  −13…−21°. If it were an artifact, averaging across five spatially separated tags would
  drive it toward zero. It doesn't. **Low disagreement + persistent tilt = real incline.**

Tilt extraction is derived from the marker Z-axis, made invariant to in-plane yaw, with
Z-axis sign disambiguated so pose-ambiguity flips don't corrupt the reading.

---

## 6. Explicit limitations (what we inherit as *work to do*)

Straight from the thesis conclusion + future work:

| # | Gap | Their recommendation |
|---|---|---|
| L1 | **Closed-loop autonomous touchdown never completed/analysed** | "first and most pressing task" |
| L2 | `PLND_ACC_P_NSE` too aggressive → underdamped position response | **decrease it** |
| L3 | No ground-truth accuracy evaluation | "systematic evaluation of landing accuracy relative to ground truth, flat and inclined, multiple trials" |
| L4 | Vision degrades in the final sub-metre | add **TF-Luna / TFmini-S** rangefinder |
| L5 | 3901-L0X optical flow present but never configured | enable it for position-control performance |
| L6 | Static platforms only | moving-platform landing (velocity + lag machinery already in place) |
| L7 | No IMU–vision fusion | implement a **Kalman filter** → better positioning, survives brief marker loss |
| L8 | **Rolling-shutter** IMX219 → motion blur / skew | **global shutter or HDR camera** |
| L9 | No end-to-end mission | auto takeoff → auto nav → target localisation → precision land |
| L10 | Closed-loop touchdown on a *sloped* surface | identified as future work |
| L11 | Arming checks were relaxed for development | **must be fully re-enabled** before unsupervised autonomous flight |

L1–L5 are cheap and should be closed in Phase 1. L6–L8 are Phase 2 and require procurement
decisions **this month**.

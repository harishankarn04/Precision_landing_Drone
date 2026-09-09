# 02 — ROLAND Reference Architecture

Source: `Documentation/ROLAND_Robust_Landing_of_UAV_on_Moving_Platform_using_Object_Detection_and_UWB_based_Extended_Kalman_Filter.pdf`
**ROLAND**, ICCAS 2021 (21st Int. Conf. on Control, Automation and Systems), Jeju, Korea.
ChanYoung Kim, EungChang Mason Lee, JunHo Choi, JinWoo Jeon, SeokTae Kim, Hyun Myung —
KAIST KI-AI / KI-R, funded by KEPCO.
Code: https://github.com/engcang/ROLAND · Video: https://youtube.com/playlist?list=PLvgPHeVm_WqLP50wNDHvI9gPrta9TcGVT

---

## 1. What problem ROLAND solves

UAV endurance is short. Fix: land on a moving ground/surface vehicle and recharge there.
Existing moving-platform landing methods have three limitations ROLAND attacks:

| Limitation of prior work | ROLAND's answer |
|---|---|
| (a) A special tag/marker must be attached to the landing site | **YOLOv4-tiny-3l object detection** recognises the platform directly — no marker |
| (b) The landing site must stay visible | **UWB ranging** compensates for the camera's limited FoV; EKF keeps propagating when the target is out of frame |
| (c) The platform must be static (or move in a straight line at constant velocity) | **EKF** fusing VIO + wheel encoder + UWB + detection handles accelerating/turning platforms |

> **Note the divergence from us on (a):** ROLAND *removes* the marker. The senior's work
> and our title *depend* on a marker/beacon. We are not obliged to follow ROLAND here —
> and arguably shouldn't. See §5.

---

## 2. The EKF (paper §3.1–3.3)

**Output of interest** `Y = p_target^uav` (3×1) — target position in UAV frame.
**Actual state** is world-referenced (6×1) to keep the model linear:

```
X = [ (p_uav^w)ᵀ  (p_target^w)ᵀ ]ᵀ            (6×1)
```

**Prediction** — driven by odometry increments, `A = I₆ₓ₆`:
```
X⁻ₖ = X̂ₖ₋₁ + [ Δp_uav ᵀ   Δp_target ᵀ ]ᵀ
P⁻ₖ = Pₖ₋₁ + Q
Q   = diag( σ²_V Δt_V I₃ ,  σ²_E Δt_E I₃ )
```
- `Δp_uav` from **VIO** (VINS-Stereo)
- `Δp_target` from the platform's **wheel encoders**
- Both accumulate drift and scale error; `Δt` in `Q` also absorbs communication-delay jitter

**Update 1 — UWB** (nonlinear, Euclidean distance):
```
Zᵘ = hᵘ(X) = ‖p_uav^w − p_target^w‖ = d
X̂ₖ = X⁻ₖ + Kᵘₖ (Zᵘₖ − Hᵘ X⁻ₖ)
Kᵘₖ = P⁻ₖ (Hᵘ)ᵀ [ Hᵘ P⁻ₖ (Hᵘ)ᵀ + Rᵘₖ ]⁻¹        Hᵘ = ∂hᵘ/∂X  (1×6 Jacobian)
Rᵘₖ ∝ d                                          (UWB noise grows with range)
```

**Update 2 — object detection** (linear):
```
Zᶜ = Hᶜ X = p_target^w − p_uav^w
Hᶜ = [ −I₃ | I₃ ]
Rᶜₖ ∝ (RGB-D depth noise) × 1/(YOLO confidence)
```

**Ground removal before using the detection** (§3.3, the neat trick): the YOLO bounding
box's point cloud contains ground points that bias the centroid. Sample N points *outside*
the box (assumed ground), fit the plane `ax+by+cz=1` by **SVD**, then discard any point in
the box within `T_ground` of that plane. The centroid of what remains is the true platform
centre. `T_ground = 0.1 m`.

---

## 3. The landing controller (paper §3.4, Fig. 2b)

State machine: `Start → Tracking Mode ⇄ Landing Mode → Finish`, gated on horizontal error.

**Yaw** — keep the (forward-facing) camera pointed at the target:
```
ω_z,des = K_p,ωz · ‖ [cosψ, sinψ, 0] × (p_target^uav)_xy ‖
```

**Desired position** — bias the setpoint so the target sits better inside the FoV
(`θ_c` = camera downward tilt angle):
```
p_des^w = p_target^w + cot(θ_c)·(p_target^uav)_z · [cosψ, sinψ, 0]ᵀ
e = p_des^w − p_uav^w
```

**Horizontal PID** — note the **feed-forward of the platform's encoder velocity**:
```
V_xy,des = K_p e_xy + K_d ė_xy + K_i ∫e_xy dt + V_target^E
```

**Vertical:**
- *Tracking mode* — hold fixed height `h_fix = 2.0 m` above the target:
  `V_z,des = K_p(e_z + h_fix) + K_d ė_z + K_i ∫(e_z + h_fix)dt`
- *Landing mode* — descend only when horizontally close, and **slow the descent as
  horizontal error grows**:
  ```
  V_z,des = −V_z,max · (T_xy − ‖e_xy‖) / T_xy
  ```

**Experimental parameters (Table 1)** — good starting values for us:

| Parameter | Value |
|---|---|
| Camera FoV | 90° × 67.5° |
| Camera depth range | 6.0 m |
| UWB range | 10.0 m |
| `T_YOLO` (confidence threshold) | 0.8 |
| `(T_xy, T_z)` landing-enable thresholds | (0.25, 0.4) m |
| `T_Vz` | 0.03 m/s |
| `T_ground` | 0.1 m |
| `h_fix` | 2.0 m |

**Sensor rates in sim:** encoder ≤20 Hz, UWB ≤10 Hz, VIO ≤10 Hz, object detection ≤50 Hz.

---

## 4. Results claimed

Gazebo, custom bumpy-ground map. UAV takes off; platform drives away at **0.8 m/s** on a
rectangular trajectory. First 50 s = tracking only (deliberately, to characterise the EKF's
relative-pose estimate while *not* landing). At t = 50 s landing mode enables; **UAV lands
within ~15 s**. EKF stays converged even when the target leaves the camera FoV, because
UWB + encoder keep the update/propagation alive.

> **Critical caveat, stated in their own conclusion:**
> *"The real-world experiment is left as the future works."*
> **ROLAND was never flown.** This is simultaneously the strongest argument for our project
> (we take it to hardware) and a warning (their numbers are simulation-optimistic).

---

## 5. Honest assessment of the GitHub repo for our use

Inspected 2026-08-14. Repo `engcang/ROLAND`, 24 stars, **last pushed 2023-07-17**, C++.

### Stack (confirmed from repo contents)
`package.xml` + `catkin` `CMakeLists.txt` + `.launch` files + `mavros_*` scripts →
**ROS 1** (Noetic era), **Gazebo Classic**, **PX4-SITL**, `iris` drone + `jackal` UGV.

⚠️ **Every layer of that stack is now end-of-life:** ROS 1 Noetic (EOL May 2025), Gazebo
Classic (EOL Jan 2025), Ubuntu 20.04 (EOL Apr 2025). Our own autopilot choice briefly
matched PX4 (2026-08-26–2026-09-09) but has reverted to ArduPilot (`CLAUDE.md` §0b) — so
PX4 in this repo is back to being someone else's autopilot, on top of everything *around*
it (ROS 1, Gazebo Classic, the ancient PX4-SITL version it pins) already being obsolete.

**Do not build your project on top of this repo.** Mine it for parts.

### What is genuinely worth taking

| Asset | Path | Value to us |
|---|---|---|
| 🟢 **UWB Gazebo plugin** | `gazebosensorplugins/src/UwbPlugin.cpp` | **The single most valuable file.** Simulates UWB ranging incl. NLOS. Needs porting from Gazebo-Classic API → `gz-sim` (Harmonic). Based on Barral et al., *Sensors* 19(16):3464, 2019 |
| 🟢 **EKF + landing controller** | `ekf_landing/src/main.cpp`, `src/autoland.cpp`, `include/{autoland,pid_controller,utility}.h` | Reference implementation of §2–§3 above. **Read it, reimplement it — do not link against it.** |
| 🟢 **Moving-platform trajectory** | `ekf_landing/scripts/mobile_move_traj.py` | The rectangular 0.8 m/s trajectory driver — trivially portable |
| 🟡 Tag position publisher plugin | `gazebosensorplugins/src/TagPosPublisherPlugin.cpp` | Ground-truth publisher; useful pattern for our sim |
| 🟡 Jackal UGV Gazebo model | `jackal_package_for_gazebo/` | A ready moving platform; URDF/xacro ports reasonably |
| 🟡 Skid-steer drive plugin | `gazebosensorplugins/src/CustomSkidSteerDrive.cpp` | Encoder-odometry source for the platform |
| 🔴 VINS-Fusion (vendored submodule) | `VINS-Fusion/` | Heavy stereo VIO. **We almost certainly don't need this** — ArduPilot's own EKF3 + GPS/optical-flow should cover `Δp_uav`. Skip. |
| 🔴 YOLOv4-tiny-3l + weights | `ekf_landing/scripts/*.cfg`, `*.weights`, `ros_opencv_dnn.py` | Markerless detection. Contradicts our marker-based approach and needs GPU-class compute. **Skip unless we deliberately go markerless.** |
| 🔴 `rosmsgs` submodule | `valentinbarral/rosmsgs` | UWB ranging message definitions (ROS 1). Reimplement as a small ROS 2 msg package. |

**License:** GitHub reports `NOASSERTION` (a LICENSE file exists but isn't a recognised
SPDX identifier). **Read `LICENSE` before copying any code**, and cite the ICCAS paper
regardless. Vendored VINS-Fusion carries its own (GPL-family) licence — another reason to skip it.

---

## 6. What we should port vs. replace

| ROLAND component | Our decision | Why |
|---|---|---|
| YOLOv4-tiny markerless detection | **Replace** with the senior's multi-scale AprilTag fusion | We already have it, it's more accurate, it runs on a Pi 4, and it gives *orientation* (which YOLO+centroid does not). Also gives us a contribution ROLAND lacks. |
| VIO (VINS-Stereo) for `Δp_uav` | **Replace** with ArduPilot's own EKF3 (GPS + baro + optical flow via the 3901-L0X we already own) | Far less compute; already flight-proven; no extra camera |
| Wheel encoder for `Δp_target` | **Depends on platform** — if we build an RC-car platform, add encoders or a GPS+telemetry downlink | Needed for the velocity feed-forward term. Alternative: estimate platform velocity purely from the vision velocity estimator the senior already wrote |
| **UWB for out-of-FoV ranging** | **Port** — this is ROLAND's core contribution and the honest reading of our "Beacon-Based" title | See `06-open-questions.md` Q3 for module choice |
| EKF structure (§2) | **Port and reimplement** | Directly addresses the senior's future-work item L7 |
| Landing controller (§3) | **Adapt.** ArduPilot has its own precision-landing handling (`PLND_*`, LAND mode) rather than PX4's offboard-setpoint style; ROLAND's `V_z ∝ (T_xy − ‖e_xy‖)` slow-down and the tracking/landing state machine are portable *ideas* but the integration point is different on ArduPilot — work out the mapping when we reach this stage, per `CLAUDE.md` §7 items 0–2c | Don't rewrite the autopilot's controller — ride on it, same principle ROLAND used, different firmware |
| PX4 / MAVROS | **Don't keep as-is.** ROLAND's stack is PX4; ours is ArduPilot again (`CLAUDE.md` §0b) — its `mavros`/PX4-ROS2-interface plumbing is a reference for *how* to wire ROS 2 to an autopilot, not something to reuse directly. ArduPilot's own MAVLink/`LANDING_TARGET` path (§7 of `CLAUDE.md`) is the actual integration point | Our hardware is ArduPilot, not PX4 |

**Net:** we keep ROLAND's *estimator and mission architecture*, and substitute our own,
stronger perception front-end. That's a clean, defensible contribution statement.

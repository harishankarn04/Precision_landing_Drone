# 06 — Open Questions

Unresolved decisions. **Update this file the moment each is answered**, with the answer and
who decided. Several block real work.

---

### Q1 ✅ ANSWERED — we inherit *nothing physical*. Building from scratch.
**Answer (Hari, 2026-08-14):** we have neither the airframe nor the code. The senior may
still have source, but we are not planning around it.

**Consequences — this reframes the whole project:**
- The 61-page thesis becomes a **specification**, not an inheritance. It is an unusually
  good one: exact tag layout, exact intrinsics, exact solver choices, exact ArduPilot
  params, and nine documented gotchas. We rebuild *from* it rather than *on* it.
- Synopsis Reading B applies: Jul–Nov genuinely is procure → assemble → tune → validate.
- **Silver lining:** we get to fix the inherited weaknesses at design time instead of
  living with them — global-shutter camera from day one, rangefinder from day one,
  optical flow configured from day one. See `04-roadmap-checklist.md` procurement.
- **Biggest consequence:** software must be developed in **simulation**, in parallel with
  the hardware build, or we lose two months waiting for parts. Simulation moves from
  "nice to have" to **the critical path**.

---

### Q2 🟠 Try anyway: can we get the senior's source code?
Downgraded from blocking to opportunistic — we plan as if the answer is no.
Still worth one email: even the calibration YAML, the `.param` dump, and the tag-board
print file would save days, and sample logs would let us validate our reimplementation
against known-good data.

**Ask:** Akash Dinesh (BL.EN.P2RAU24001) via Dr. Sreeja Kochuvila — same guide.
**Wanted, in priority order:** `.param` dump · tag board print file (exact scale) ·
sample `.BIN` + companion logs + overlay video · calibration YAML · pipeline source ·
the two diagnostic scripts.
**Answer:** _____

---

### Q3 ⏸️ DEFERRED — beacon comes *after* a landing drone exists in simulation
**Decision (Hari, 2026-08-14):** *"it doesn't make sense to start with beacon if we don't
have a flying drone."* Correct. Beacon work begins at **Stage 4**, once the sim drone
lands itself.

**This also de-fangs the question:** because we prototype in simulation, choosing the
beacon no longer blocks procurement — nothing needs buying to test it. The recommendation
below stands for when we get there.

Our title says **Beacon-Based**. The system has no beacon. Options:

| Option | Pros | Cons |
|---|---|---|
| **UWB** (DWM1001 / DW3000-class) | Matches ROLAND exactly; works when the marker is out of FOV or occluded; gives true range; genuinely novel on an ArduPilot platform | Most work: hardware, firmware, ranging protocol, EKF integration, calibration. ROLAND's Gazebo plugin needs porting to Harmonic |
| **IR-Lock / IR beacon** | Plug-and-play in ArduPilot (`PLND_TYPE=2`); literally designed for this | Short range; sunlight-sensitive; almost no research novelty; not really a *fusion* contribution |
| **Active LED / vision beacon** | Cheap; helps at night and in low light; still uses the existing camera | Still FOV-limited — doesn't solve ROLAND's problem (b) |
| **Reinterpret "beacon" = the AprilTag board** | Zero new hardware; the title is arguably already satisfied | Weak; loses the whole out-of-FOV contribution; examiners may well push back |
| **BLE AoA / RSSI** | Cheap modules | Too imprecise for landing |

**Recommendation: UWB.** It is the only option that makes the title honest, matches the
chosen reference paper, addresses a real failure mode (marker occluded / out of frame), and
produces a defensible contribution. Revisit at Stage 4.

**Answer:** _____

---

### Q4 ⏸️ DEFERRED — moving platform is a Gazebo model first
Physical platform hardware isn't needed until Stage 6. In Stage 5 it's a simulated ground
vehicle (ROLAND ships a Jackal model and a trajectory driver we can adapt).

For later: RC car / rover chassis · a departmental robot · a person pulling a trolley
(simplest, fine for early tests) · a motorised rail (highly repeatable, straight lines only).

**How do we get the platform's velocity?** Wheel encoders (ROLAND's approach) · GPS +
telemetry downlink · or estimate it purely from vision using the velocity estimator the
thesis already specifies. The vision-only route needs *no instrumentation on the pad*,
which is both simpler and arguably a nicer result — worth trying first in sim.

**Answer:** _____

---

### Q5 ✅ ANSWERED — target is "rebuild the senior's system, then add the beacon"
**Answer (Hari, 2026-08-14):** *"implement the senior's project and add the beacon and
landing to it… for now getting a base level is the requirement."*

**Agreed scope ladder — ship each rung before starting the next:**

| Rung | Deliverable | Status |
|---|---|---|
| **R1** | Reimplement the AprilTag multi-scale perception pipeline (in simulation first) | ← **primary Phase-1 goal** |
| **R2** | Build the airframe; get the pipeline running onboard; reproduce the thesis's detection results | Phase 1 |
| **R3** | **Closed-loop autonomous touchdown, static pad, with ground-truth statistics** | the "base level" — *the senior never got here* |
| **R4** | Beacon integration → target localised when the marker is occluded / out of FOV | Phase 2 primary |
| **R5** | Explicit EKF fusing vision + beacon + odometry | Phase 2 |
| **R6** | Moving platform | stretch — decide after R4 |
| **R7** | Inclined + moving | research stretch, not planned |

Note R3 alone already exceeds the senior's thesis. Do not skip ahead to R6.

---

### Q6 ✅ ANSWERED — good compute available, use the split
**Answer (Hari, 2026-08-14):**
- Hari: **M1 MacBook**, 16 GB, ~60 GB free
- Teammate: laptop with **RTX 4090**  ← the simulation workhorse
- Teammate: laptop with **NVIDIA ~GTX/RTX 1660-class** GPU (confirm exact model)
- **MATLAB access for all three**

**Implications:** Tier 1 (Ubuntu 24.04 + Gazebo Harmonic + ArduPilot SITL) is fully
viable on the 4090 machine with real GPU rendering — no software-rendering penalty, no
disk pressure on the Mac. Run it there, natively dual-boot or WSL2, not in a VM on the M1.
MATLAB is available for the EKF-design side channel. See `03-simulation-plan.md`.

**Still open:** exact model of the "1660" GPU · any motion-capture / total station on
campus for hardware ground truth.

---

### Q7 🟡 Do we keep the marker, or go markerless like ROLAND?
ROLAND's headline is *no special marker* (YOLOv4-tiny detects the platform directly).
Our inherited strength is *a very good marker pipeline*.

**Recommendation: keep the marker.** It's more accurate, gives orientation (YOLO+centroid
does not), runs on a Pi 4B without a GPU, and is already validated. Frame it as a
*deliberate substitution*: we replace ROLAND's detection front-end with a more accurate
fiducial front-end and keep its estimator. Going markerless would need a Jetson and a
labelled dataset — a whole extra project.

**Answer:** _____

---

### Q8 🟡 Report/publication intent?
Is a conference or journal submission expected? If yes, "ROLAND on real hardware" is a
strong, honest angle (their conclusion explicitly leaves hardware to future work), and it
changes what we must measure — ground truth and baseline comparisons become mandatory,
not optional.

**Answer:** _____

---

### Q9 ⏸️ DEFERRED (Stage 6) — Flight permissions and site
- Campus permission for autonomous outdoor flight — who signs off?
- DGCA classification for a 5" quad with Pi + 4S pack; is campus a green zone?
- Where do we fly? Grass preferred over concrete.
- Is there an indoor space (net/cage) for early low-altitude tests?

**Answer:** _____

---

### Q10 ⏸️ DEFERRED (Stage 6) — Budget
Not needed until hardware. Parts will be **salvaged** where possible, and a working
simulation is the argument for spending on the rest. Revisit with the BOM notes in
`04-roadmap-checklist.md`.

**Answer:** _____

---

## Currently blocking: **nothing.** ✅

Every open question has been deferred to the stage where it actually matters. The work to
do right now is Stage 1 in `07-sim-setup.md`. This is deliberate — it means simulation
progress cannot be stalled waiting on a decision, a purchase, or a person.

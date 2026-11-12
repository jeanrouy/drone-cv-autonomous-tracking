# Test log

## 2026-11-09 — Offboard v1, loss of offboard stream (informal)

**Configuration:** PX4 main @ c4e4ef98e9, SITL gz_x500, no RC, QGroundControl connected.
COM_OF_LOSS_T = 1.0 s, COM_OBL_RC_ACT = 0 (Position), NAV_DLL_ACT = 0 (disabled).

**Procedure:** offboard_demo v1 publishes heartbeat + position setpoint (0, 0, -2) at 20 Hz.
Offboard mode and arming from QGC. Once stable at 2 m, offboard_demo stopped with Ctrl+C.

**Result:** failsafe triggered, PX4 switched to Return (RTL), landed and disarmed.
QGC: "Failsafe activated: switching to RTL — No offboard signal, No manual control input".

**Analysis:** the configured action (Position) requires manual control; with no RC, PX4 fell
back to RTL. Behaviour is safe. Detection delay not measured precisely (monitor at 2 Hz):
to be measured from the ULog in the formal test E2.

## 2026-11-10 — Offboard v2, automatic entry and operator takeover

**Configuration:** same as previous entry. offboard_demo v2 (explicit state machine,
mode/arm requests with retries, abort when PX4 leaves Offboard).

**Test A — automatic Offboard entry and arming**
- Procedure: vehicle landed, disarmed, Hold. Node started, no operator action.
- Result: Warmup → RequestOffboard → RequestArm → Flying, each command accepted at first attempt.
  - DO_SET_MODE (176): ack after 10 ms, Offboard observed after 150 ms.
  - ARM (400): ack after 15 ms, armed observed after 150 ms.
  - First command to Flying: 0.35 s. Node start to Flying: 3.5 s (DDS discovery: first
    vehicle_status received ~3 s after start).
- Vehicle climbed to 2 m and held position.

**Test B — operator takeover**
- Procedure: in flight (Offboard, 2 m), mode switched to Hold from QGC.
- Result: node detected Offboard exit 60 ms after the QGC command ack, logged ABORT and stopped
  publishing. No attempt to regain control. Landing commanded from QGC.

**Analysis:** both behaviours as designed. Acks from QGC commands are visible to the node
(acks are broadcast for all command sources).
## 2026-11-11 — Offboard v3, scripted mission (takeoff, 4 m square, land)

**Configuration:** same as previous entries. offboard_demo v3: mission as a list of waypoints
relative to home, reached when 3D distance < 0.3 m, 20 s timeout per waypoint, NAV_LAND at the end.
Recorded: bags/mission_01 (local position, vehicle status, trajectory setpoints).

**Result:** mission completed without operator action. Arming to disarm: 40 s.
- Takeoff: motors spool-up ~3.5 s, asymptotic climb to 2 m. Heading step 96° → 0° with ~13° overshoot.
- Square legs: ~1.5 s each, peak speed ~3 m/s.
  Corner overshoot 0.15–0.23 m, cross-track error up to 0.25 m, altitude ±0.08 m.
- Landing: ~0.6 m/s, auto-disarm ~3 s after touchdown. Touchdown 0.17 m from home.

**Analysis:** overshoots come from step position setpoints (4 m jump at each waypoint): maximum
initial error, strong acceleration, inertia carries the vehicle past the target. Takeoff commanded
from QGC (smoothed trajectory) showed centimetre-level tracking instead.
**Possible improvement (phase 5):** rate-limited setpoint (moving target) to trade speed for precision.

## 2026-11-11 — Phase 2 validation campaign (test plan)

**Objective:** demonstrate that the scripted mission (offboard_demo v3) is repeatable.

**Configuration:** PX4 main @ c4e4ef98e9, SITL gz_x500 (headless), no RC, QGroundControl connected.
COM_OF_LOSS_T = 1.0 s, COM_OBL_RC_ACT = 0. Single simulation session for the 5 runs.

**Initial conditions (each run):** vehicle landed, disarmed, mode Hold (set from QGC).

**Procedure (each run):** start rosbag recording, start offboard_demo, no operator action,
stop recording after "Mission terminee", analyse with tools/analyze_mission.py --csv.

**Success criteria:**
- C1: 5/5 missions completed (no abort, no operator action).
- C2: landing offset < 0.5 m on every run.
- C3: corner overshoot < 0.5 m on every run.

**Characterisation (no pass/fail):** mission time, takeoff time, peak speed, cross-track and
altitude errors, run-to-run spread.

**Results:** see docs/campaign_phase2.csv (filled after the runs).
### Results (2026-11-11)

| Criterion | Threshold | Worst case | Verdict |
|---|---|---|---|
| C1 missions completed | 5/5 | 5/5 | PASS |
| C2 landing offset | < 0.5 m | 0.15 m | PASS |
| C3 corner overshoot | < 0.5 m | 0.24 m | PASS |

Characterisation (5 runs): mission 39.8–40.2 s, takeoff 7.1–7.2 s, peak speed 3.01–3.06 m/s,
overshoot 0.21–0.24 m, altitude error 0.11–0.13 m. Details in docs/campaign_phase2.csv.

**Anomaly 1 — analysis artefact and node bug.** First analysis reported ~2 m altitude errors.
Cause: on disarm, PX4 restores the user intention (Offboard); offboard_demo, still in Landing,
published one last setpoint to the disarmed vehicle. The script used the last setpoint as the end
of the last leg, so the window included the landing.
Fixes: node no longer streams to a disarmed vehicle; script ends the last leg at the start of Land
mode. Bags re-analysed without new flights. Non-regression flight: PASS (mission completed, all metrics within campaign range).

**Anomaly 2 — cross-track error decreasing over time.** Run 1: 0.12–0.22 m on all legs, decreasing
along the flight; run 5: 0.02–0.06 m. Hypothesis 1 (yaw rotation at takeoff) rejected: the error is
not concentrated on leg 1. HHypothesis 2 (EKF2 heading convergence after start-up): supported for run 1. Mean heading
uncertainty (sqrt of heading_var): run 1 7.95°, runs 2–5 1.1–1.6°. Orders of magnitude consistent
(8° over a 4 m leg allows up to ~0.56 m; 1.5° allows ~0.1 m). eph constant (0.151 m) on all runs:
horizontal position estimate not involved. Residual decrease between runs 2 and 5 (0.16 → 0.06 m)
not explained by heading_var and not significant with 5 runs.
Limitation: means computed over the whole recording, including ground phase.
**Action:** heading convergence check (heading_var < 0.0012 rad², ~2°) added to the startup
procedure before any measurement run.

## 2026-11-11 — E2: loss of offboard stream during mission (test plan)

**Objective:** verify PX4's reaction when the offboard node stops while the vehicle is moving,
and measure the detection delay.

**Configuration:** COM_OF_LOSS_T = 1.0 s, COM_OBL_RC_ACT = 0 (Position), no RC.
offboard_demo v3 (with disarm fix).

**Initial conditions:** estimator converged (heading_var < 0.0012 rad²), vehicle landed,
disarmed, mode Hold.

**Procedure:**
1. Record vehicle position, status, trajectory setpoints and offboard heartbeat.
2. Start offboard_demo. When it logs "Cap sur le point 3/5" (vehicle accelerating on leg 2),
   stop the node with Ctrl+C.
3. No operator action until the vehicle is disarmed.
4. Stop recording, then shut PX4 down cleanly to close the ULog.

**Prediction:** Position mode impossible without RC, so fallback to Return (RTL), as observed
in the informal test of 2026-11-09.

**Success criteria:**
- E2-a: Offboard exit within COM_OF_LOSS_T + 0.5 s = 1.5 s after the last heartbeat.
- E2-b: no altitude loss greater than 0.5 m in the 5 s following the cut.
- E2-c: vehicle lands and disarms without operator action.


### Results (2026-11-11)

**Initial conditions:** heading_var = 0.0004 rad² (~1.2°) after a warm-up flight; landed, disarmed, Hold.
Cut during leg 2 (vehicle accelerating). Data: bags/e2_01 and PX4 ULog 2026-11-11/15_09_53.ulg.

| Measurement | Rosbag (ROS 2 side) | ULog (PX4 internal) |
|---|---|---|
| Last heartbeat → signal loss detected | — | 1.04 s |
| Signal loss → mode change | — | 0.01 s |
| Last heartbeat → Offboard exit | 1.07 s | 1.05 s |

| Criterion | Threshold | Measured | Verdict |
|---|---|---|---|
| E2-a Offboard exit | ≤ 1.5 s | 1.07 s | PASS |
| E2-b altitude loss (5 s) | < 0.5 m | 0.01 m | PASS |
| E2-c lands and disarms without operator | — | yes, 24.9 s after cut | PASS |

**Observations:**
- Prediction confirmed: Return mode (no RC, Position mode unavailable). QGC: "Failsafe activated:
  switching to RTL — No offboard signal, No manual control input".
- Not predicted: climb to 10.8 m (≈ 9 m above cut altitude) before returning: Return mode altitude
  logic, RTL_* parameters to be reviewed.
- Touchdown 0.03 m from home (more precise than the scripted mission landing).
- Both sources agree within 0.02 s. Detection delay = COM_OF_LOSS_T + ~40 ms (heartbeat period
  and check rate). No added delay between detection and reaction in this configuration.

  ## 2026-11-11 — P3: YOLOv8n CPU benchmark (test plan)

**Objective:** choose the detector configuration for the project and check the phase 3 criterion.

**Setup:** laptop i7-6500U (2 cores / 4 threads), 8 GB RAM, on AC power, browser closed.
Sequence: data/sequences/ref_01.mkv (1800 frames, 640x480 @ 30 FPS, see docs/sequences.md).
Model: YOLOv8n (COCO), class "person" only, conf = 0.25. First 10 frames excluded (warm-up).

**Configurations:** PyTorch 640, PyTorch 320, OpenVINO 640, OpenVINO 320.
**Conditions:** idle; then PX4 SITL + Gazebo running (headless, vehicle landed).

**Measurements (per frame):** detector call latency (median, p95, max) with preprocess /
inference / postprocess breakdown; overall throughput (frames per second including video
decoding); detection rate (frames with at least one person); mean confidence.

**Success criteria:**
- P3-a: chosen configuration reaches >= 10 FPS throughput with Gazebo running.
- P3-b: its detection rate is within 5 points of the PyTorch 640 reference (same condition).

### Results (2026-11-12)

| Config | Idle FPS | Gazebo FPS | Gazebo median / p95 | Detection |
|---|---|---|---|---|
| PyTorch 640 | 10.4 | 8.8 | 111.4 / 117.4 ms | 80.3 % |
| OpenVINO 640 | 10.9 | 8.7 | 111.1 / 118.7 ms | 80.3 % |
| PyTorch 320 | 28.7 | 22.8 | 41.4 / 46.0 ms | 79.8 % |
| **OpenVINO 320** | 37.5 | **28.7** | **32.6 / 36.2 ms** | 79.8 % |

Full data: docs/bench_phase3.csv.

| Criterion | Threshold | OpenVINO 320 (Gazebo) | Verdict |
|---|---|---|---|
| P3-a throughput | >= 10 FPS | 28.7 FPS | PASS |
| P3-b detection vs PyTorch 640 | within 5 points | -0.5 point | PASS |

**Decision:** OpenVINO 320 selected for the detector node.

**Observations:**
- OpenVINO speed-up: +6 % at 640, +25 % at 320 (about +40 % per pixel, since OpenVINO runs
  320x320 vs 256x320 for PyTorch rectangular inference). Smaller than the 2-3x often quoted:
  PyTorch CPU kernels are already optimised on Intel; INT8 quantisation not tested.
- Under Gazebo load: throughput -16 to -24 % (predicted -30 to -50 %), p95 increases more than
  the median. OpenVINO advantage vanishes at 640 but holds at 320 (-21 % latency).
- Detection rate unchanged by backend or load; 640 -> 320 costs 0.5 point on this sequence.
- Under load, median latency (32.6 ms) is close to the camera frame period (33.3 ms): the node
  must always process the latest frame and drop the others.

**Limitations:** simulation headless, vehicle landed, no QGC and no ROS nodes running (lighter
than real test conditions); one run per configuration; one indoor sequence; detector only
(capture and ROS publishing overhead measured in the node).
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
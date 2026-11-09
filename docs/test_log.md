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
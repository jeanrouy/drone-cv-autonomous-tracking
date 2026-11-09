# Test procedures (SITL)

Checklists for every simulation session. Follow them in order.

## Startup

| # | Terminal | Action | Check |
|---|---|---|---|
| 1 | T1 | `fm`, then `pgrep -a px4; pgrep -af "gz sim"; pgrep -a MicroXRCEAgent` | no output (no leftover process) |
| 2 | T1 | `HEADLESS=1 ./scripts/start_sim.sh` | `Ready for takeoff!`, then detach (Ctrl+B, d) |
| 3 | T1 | `ros2 topic hz /fmu/out/vehicle_local_position_v1` | ~50 Hz |
| 4 | T2 | `flatpak run org.mavlink.qgroundcontrol` | vehicle on map, "Ready To Fly" |
| 5 | T3 | `fm`, then `ros2 run fm_control px4_monitor` | `DESARME`, mode `Hold` |
| 6 | T4 | `fm`, then recorder if needed: `ros2 bag record -o bags/<test_name> <topics>` | recording started |
| 7 | T5 | `fm`, then node under test | node startup message |

## Shutdown

Reverse order: command first, infrastructure last.

| # | Action | Why |
|---|---|---|
| 1 | Vehicle **landed and disarmed** (monitor + QGC) | never cut the chain in flight |
| 2 | Ctrl+C on nodes that send commands | no more commands possible |
| 3 | Ctrl+C on recorders, then monitor | complete test data |
| 4 | Close QGroundControl | ground station |
| 5 | `tmux attach -t sim`, then `shutdown` at the `pxh>` prompt | ULog closed cleanly |
| 6 | Detach, `tmux kill-session -t sim`, then the `pgrep` check | no leftover process |

## Emergency (unexpected behaviour in flight)

1. In QGC: select **Land** (or **Return**).
2. Stop the node under test (Ctrl+C): PX4 triggers the offboard-loss failsafe after `COM_OF_LOSS_T`.
3. If QGC is unresponsive: in the `pxh>` console, `commander land`.
4. Note the time and what was observed in `docs/test_log.md` before anything else.

## Before each test

- Note PX4 commit, repository commit and modified parameters in the test log.
- Define the success criterion **before** running the test.
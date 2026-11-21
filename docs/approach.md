# Development and test approach

## Context: no physical drone (yet)

All flight work runs in simulation (PX4 SITL + Gazebo Harmonic, x500 quadcopter). The only real
sensor is a Logitech C922 webcam on a table. The long-term goal is to move the system to a real
drone: a PX4 flight controller, an onboard companion computer for perception, and a camera
mounted on the vehicle.

## Staged validation

Each stage adds one new difficulty, so that a failure points to one cause.

| Stage | Vehicle | Camera | Loop | Validates | Status |
|---|---|---|---|---|---|
| 1. SITL + fixed webcam | simulated x500 | real C922, fixed | open | perception on real images; controller logic, signs, saturations, state machine, safety; end-to-end latency | phases 1–5 |
| 2. SITL + simulated camera | simulated x500 | simulated, onboard | closed | visual servoing: tuning, centring error, loop stability | planned |
| 3. Hardware in the loop | PX4 on a real flight controller, simulated world | simulated | closed | real autopilot hardware and timing | future |
| 4. Real drone | real | real, onboard | closed | flight tests with a safety pilot | future |

## Why an open loop first (stage 1)

The webcam does not move with the simulated drone: the drone reacts to the target, but its
motion has no effect on the image. Visual servoing therefore cannot be tuned in this setup.
What can be validated is everything that does not depend on feedback: command signs, bounds,
behaviour when the target is lost or stale, safety, and latency, on real camera images with
their real defects (lighting, occlusions, motion blur), which simulation does not reproduce.
The controller reads only `/fm/target`: it is identical in stages 1 and 2; only the image
source of the perception node changes.

## What transfers to a real drone

- **As is:** ROS 2 nodes and interfaces, target selector, controller logic, PX4 Offboard
  interface (uXRCE-DDS), procedures and test methodology.
- **To revalidate:** camera optics and mounting (intrinsics, field of view), companion computer
  performance (all latency measurements to be redone), real dynamics (wind, inertia), real-flight
  safety (RC link, safety pilot, geofence, regulations).

## Safety principles kept for the real drone

- The operator designates the target; the system never locks a stranger on its own (ADR-001).
- PX4 stays in charge of safety: loss of the offboard stream triggers a verified failsafe (test E2).
- The controller never fights the operator or the autopilot: if PX4 leaves Offboard, it stops.
- Every capability is validated against criteria defined before the test (docs/test_log.md).
# Design decisions

## ADR-001 — The operator designates the target (2026-11-17)

**Context:** test E8 (public pedestrian video) showed that after a definitive target loss,
automatic re-acquisition (person nearest the image centre) locked a passer-by while the real
target was still in the scene but undetected.

**Decision:** the selector never acquires or re-acquires a target on its own. Locking happens
only on an operator request (`request_lock()`: nearest to the centre, or a given track id).
After a loss longer than `max_lost_time`, the selector returns to SEARCHING and waits.
Relocking rules during a short loss (same id, or nearby consistent track) are unchanged.

**Alternatives considered:**
- Image-centre re-acquisition: rejected, follows strangers (E8 v3).
- Cautious re-acquisition (consistent size, growing radius, centre rule as last resort):
  keeps autonomy, but a passer-by of similar size can still be locked.

**Rationale:** following the wrong person is a safety and mission failure; hovering and waiting
is only a loss of availability. The policy is simple to explain and to verify.

**Consequences:** every definitive loss requires operator action; the controller must handle
SEARCHING by holding position; the ROS node must expose a way for the operator to request a lock.
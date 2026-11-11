# Reference video sequences

Recorded once, reused for every perception benchmark so that results are comparable.
Video files are not versioned (data/ is git-ignored); this file describes them.

## ref_01 — single person, indoor

- **Camera:** Logitech C922, MJPG 640x480 @ 30 FPS, fixed on a table at ~1 m height.
- **Lighting:** indoor, room lights on.
- **Duration:** 60 s.
- **Scenario:**
  1. 0–10 s: empty scene (false positive check).
  2. 10–25 s: person enters from the left, walks slowly across the frame at ~3 m.
  3. 25–40 s: person walks towards the camera (~1.5 m), then back (~5 m).
  4. 40–50 s: person turns around, partially hidden behind a chair or door frame.
  5. 50–55 s: person leaves the frame.
  6. 55–60 s: person comes back into the frame.
#!/usr/bin/env bash
# Lance l'agent uXRCE-DDS et PX4 SITL (x500 + Gazebo) dans une session tmux "sim".
# Usage : ./scripts/start_sim.sh               avec l'interface Gazebo
#         HEADLESS=1 ./scripts/start_sim.sh    sans interface (économise le CPU)
set -e

SESSION=sim
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "La session '$SESSION' tourne déjà. Pour la rejoindre : tmux attach -t $SESSION"
  exit 1
fi

PX4_CMD="cd $PX4_DIR && make px4_sitl gz_x500"
if [ -n "$HEADLESS" ]; then
  PX4_CMD="cd $PX4_DIR && HEADLESS=1 make px4_sitl gz_x500"
fi

tmux new-session -d -s "$SESSION" -n agent "MicroXRCEAgent udp4 -p 8888; bash"
tmux new-window  -t "$SESSION" -n px4 "$PX4_CMD; bash"
tmux attach -t "$SESSION"
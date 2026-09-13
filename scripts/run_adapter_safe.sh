#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
exec ros2 launch estimator_adapter phase1.launch.py \
  output_enabled:=false replay_mode:=true

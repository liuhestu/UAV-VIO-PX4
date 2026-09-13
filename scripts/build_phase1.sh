#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_ROOT}"
colcon build --symlink-install \
  --base-paths "${WORKSPACE_ROOT}/src/open_vins" "${WORKSPACE_ROOT}/src/estimator_adapter"
colcon test --packages-select estimator_adapter
colcon test-result --verbose

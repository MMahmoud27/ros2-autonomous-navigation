#!/bin/bash
# Builds the four navigation packages in a throwaway container and runs their unit tests.
#
# Usage:   ./scripts/run_unit_tests.sh
# Needs:   the robot image, built with `./watod build robot`. It is tagged with your git branch;
#          on a branch other than main, set ROBOT_IMAGE, e.g.
#          ROBOT_IMAGE=ghcr.io/watonomous/wato_asd_training/robot:my-branch ./scripts/run_unit_tests.sh
set -euo pipefail

MONO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${ROBOT_IMAGE:-ghcr.io/watonomous/wato_asd_training/robot:main}"
PACKAGES="costmap map_memory planner control"

# The source is mounted read-only; build output stays inside the container and is discarded
docker run --rm --entrypoint bash -v "$MONO_DIR/src/robot:/tmp/ws/src:ro" "$IMAGE" -c "
  set -e
  source /opt/ros/humble/setup.bash
  cd /tmp/ws
  echo 'Building: $PACKAGES'
  colcon build --packages-select $PACKAGES > build.log 2>&1 || { cat build.log; exit 1; }
  echo 'Running unit tests...'
  colcon test --packages-select $PACKAGES --ctest-args -R _core_test > /dev/null 2>&1 || true
  colcon test-result --verbose
"

#!/usr/bin/env bash
# From the course checkout, use the same Ubuntu 24.04 / Jazzy dev image as CI.
set -eo pipefail
here="$(cd -- "$(dirname -- "$0")" && pwd)"
cd -- "$here/../.."
mode="${1:-mock}"
case "$mode" in
  mock|real|replay) ;;
  *) echo "Usage: $0 [mock|real|replay]" >&2; exit 2 ;;
esac
if ! docker compose version >/dev/null 2>&1; then
  echo "Course smoke needs Docker Compose. Nothing was installed or changed." >&2
  exit 2
fi
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
docker compose --profile dev build dev
docker compose --profile dev run --rm --no-deps -T -w /workspace \
  -e G1_ROS_DISTRO=jazzy -e G1_INSTALL_PREFIX= -e PYTHONNOUSERSITE=1 \
  dev bash --noprofile --norc -c '
    set -eo pipefail
    source /etc/os-release
    test "$ID" = ubuntu && test "$VERSION_ID" = 24.04 || exit 2
    source /opt/ros/jazzy/setup.bash
    dir=/workspace/integration/g1_v1
    case "$1" in
      mock) bash "$dir/smoke.sh" mock ;;
      real)
        stage="$(mktemp -d /tmp/g1-course-build.XXXXXXXX)"
        MAKEFLAGS=-j2 colcon --log-base "$stage/log" build \
          --base-paths ros2_ws/src/capstone_localization \
          --build-base "$stage/build" --install-base "$stage/install" \
          --packages-select capstone_localization --executor sequential
        G1_INSTALL_PREFIX="$stage/install" bash "$dir/smoke.sh" real
        ;;
      replay) bash "$dir/replay_smoke.sh" ;;
    esac
  ' _ "$mode"

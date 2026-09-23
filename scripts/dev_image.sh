#!/usr/bin/env bash
# Rebuilds the development image only when its inputs changed: the Dockerfile, the dev
# environment script or any package.xml (dependencies are installed from them by rosdep).
#   scripts/dev_image.sh         rebuild if needed
#   scripts/dev_image.sh --mark  record the current inputs as built (after `make images`)
set -euo pipefail
cd "$(dirname "$0")/.."

image=capstone-dev:local
stamp=.cache/dev-image.sha256
current=$( { cat docker/Dockerfile docker/dev_env.sh
             find ros2_ws/src -name package.xml -not -path '*/build/*' | LC_ALL=C sort | xargs cat; } | sha256sum | cut -d' ' -f1)
mkdir -p .cache

if [ "${1:-}" = "--mark" ]; then
  echo "$current" > "$stamp"
  exit 0
fi

if docker image inspect "$image" >/dev/null 2>&1 && [ "$(cat "$stamp" 2>/dev/null)" = "$current" ]; then
  exit 0
fi
if docker image inspect "$image" >/dev/null 2>&1; then
  echo ">>> Изменились Dockerfile или зависимости в package.xml: пересобираю образ разработки"
else
  echo ">>> Образа разработки ещё нет: собираю его (первый раз несколько минут)"
fi
docker compose --profile dev build dev
echo "$current" > "$stamp"

#!/usr/bin/env bash
# Opens a window from a project container on the host display.
#   scripts/gui.sh simulator            Webots with the warehouse world (`make sim-gui`)
#   scripts/gui.sh gui COMMAND [ARG...] a ROS tool from the gui image: rqt_graph, rqt, rviz2 (`make rqt-graph`, ...)
# Works with Xorg and Wayland (through XWayland) on Ubuntu 22.04/24.04. Uses the host GPU
# through /dev/dri when available, otherwise renders in software (slower, same picture).
set -euo pipefail
cd "$(dirname "$0")/.."

service="${1:-}"
shift || true
if [ "$service" != simulator ] && { [ "$service" != gui ] || [ "$#" -eq 0 ]; }; then
  sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
fi
if [ -z "${DISPLAY:-}" ]; then
  echo "Нет графического дисплея (DISPLAY не задан). Запустите команду в терминале рабочего стола, а не по SSH." >&2
  exit 1
fi

export HOST_UID="${HOST_UID:-$(id -u)}" HOST_GID="${HOST_GID:-$(id -g)}"
mkdir -p .cache/home .cache/ccache

# X11 cookie usable from inside the container: same cookie, hostname-independent ("ffff" family).
CAPSTONE_XAUTH="${XDG_RUNTIME_DIR:-/tmp}/capstone-$(id -u)-$$.xauth"
export CAPSTONE_XAUTH
: > "$CAPSTONE_XAUTH"
chmod 600 "$CAPSTONE_XAUTH"
trap 'rm -f "$CAPSTONE_XAUTH"' EXIT
if command -v xauth >/dev/null 2>&1; then
  xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$CAPSTONE_XAUTH" nmerge - 2>/dev/null || true
else
  echo "Внимание: нет программы xauth (sudo apt install xauth); окно откроется, только если X-сервер пускает вашего пользователя без cookie." >&2
fi

files=(-f compose.yml -f compose.gui.yml)
if [ -d /dev/dri ]; then
  render_node="$(find /dev/dri -maxdepth 1 -name 'renderD*' 2>/dev/null | head -n1)"
  DRI_GID="$(stat -c %g "${render_node:-/dev/dri}")"
  export DRI_GID
  files+=(-f compose.dri.yml)
else
  echo "Нет /dev/dri: окно будет рисоваться программно (медленнее)." >&2
fi

docker compose "${files[@]}" --profile gui build "$service"
echo ">>> Открываю окно. Закройте его или нажмите Ctrl+C, чтобы остановить."
status=0
interrupted=0
trap 'interrupted=1' INT
if [ "$service" = simulator ]; then
  docker compose "${files[@]}" run --rm simulator || status=$?
else
  docker compose "${files[@]}" --profile gui run --rm gui \
    bash -c 'source /etc/capstone_dev.sh && exec "$@"' _ "$@" || status=$?
fi
# Ctrl+C is a normal way to finish.
if [ "$interrupted" -eq 1 ] || [ "$status" -eq 130 ]; then
  exit 0
fi
# RViz 2 (Jazzy) often aborts while shutting down; the session itself worked.
if [ "$status" -eq 134 ] && [ "${1:-}" = rviz2 ]; then
  echo "RViz завершился с ошибкой при выходе (код 134) — известная ошибка RViz 2, на работу не влияет."
  exit 0
fi
exit "$status"

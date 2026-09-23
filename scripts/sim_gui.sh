#!/usr/bin/env bash
# Opens the Webots window from the simulator container on the host display (`make sim-gui`).
# Works with Xorg and Wayland (through XWayland) on Ubuntu 22.04/24.04. Uses the host GPU
# through /dev/dri when available, otherwise Webots renders in software (slower, same scene).
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -z "${DISPLAY:-}" ]; then
  echo "Нет графического дисплея (DISPLAY не задан). Запустите make sim-gui в терминале рабочего стола, а не по SSH." >&2
  exit 1
fi

export HOST_UID="${HOST_UID:-$(id -u)}" HOST_GID="${HOST_GID:-$(id -g)}"

# X11 cookie usable from inside the container: same cookie, hostname-independent ("ffff" family).
CAPSTONE_XAUTH="${XDG_RUNTIME_DIR:-/tmp}/capstone-$(id -u).xauth"
export CAPSTONE_XAUTH
: > "$CAPSTONE_XAUTH"
chmod 600 "$CAPSTONE_XAUTH"
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
  echo "Нет /dev/dri: Webots будет рисовать программно (медленнее)." >&2
fi

echo ">>> Открываю Webots. Закройте окно или нажмите Ctrl+C, чтобы остановить симулятор."
status=0
docker compose "${files[@]}" run --rm simulator || status=$?
rm -f "$CAPSTONE_XAUTH"
# 130: stopped with Ctrl+C, which is a normal way to finish.
if [ "$status" -eq 130 ]; then
  exit 0
fi
exit "$status"

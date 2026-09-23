#!/usr/bin/env bash
# Prepares an Ubuntu 22.04/24.04 computer for the project, then runs `make setup`.
#   scripts/setup_host.sh             install what is missing (git, make, Docker Engine), then make setup
#   scripts/setup_host.sh --no-setup  install what is missing, do not run make setup
#   scripts/setup_host.sh --check     only check and report, change nothing (`make doctor`)
# Docker Engine is installed with the official convenience script from https://get.docker.com.
set -euo pipefail
cd "$(dirname "$0")/.."

mode=install
run_setup=1
for arg in "$@"; do
  case "$arg" in
    --check) mode=check ;;
    --no-setup) run_setup=0 ;;
    -h|--help) sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Неизвестный аргумент: $arg (см. --help)" >&2; exit 2 ;;
  esac
done

failures=0
need_relogin=0
user_name="${USER:-$(id -un)}"
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  SUDO="sudo"
fi

ok()   { echo "  [OK]      $*"; }
info() { echo "  [INFO]    $*"; }
warn() { echo "  [ВНИМАНИЕ] $*"; }
fail() { echo "  [ОШИБКА]  $*"; failures=$((failures + 1)); }
installing() { [ "$mode" = install ]; }

apt_install() {
  echo "  ... устанавливаю: $*"
  $SUDO apt-get update -qq
  DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y -qq "$@" >/dev/null
}

echo "== Система"
# shellcheck disable=SC1091
. /etc/os-release
case "${ID:-}-${VERSION_ID:-}" in
  ubuntu-22.04|ubuntu-24.04) ok "$PRETTY_NAME" ;;
  ubuntu-*) warn "$PRETTY_NAME: проверено только на Ubuntu 22.04 и 24.04" ;;
  *) warn "${PRETTY_NAME:-неизвестная ОС}: скрипт рассчитан на Ubuntu 22.04/24.04" ;;
esac
if [ "$(uname -m)" = x86_64 ]; then
  ok "архитектура x86_64"
else
  fail "архитектура $(uname -m): Webots R2025a для Linux выпускается только для x86_64"
fi

echo "== Программы"
missing=()
for tool in git make curl; do
  if command -v "$tool" >/dev/null 2>&1; then ok "$tool"; else missing+=("$tool"); fi
done
if [ "${#missing[@]}" -gt 0 ]; then
  if installing; then
    apt_install "${missing[@]}" ca-certificates
    ok "установлено: ${missing[*]}"
  else
    fail "не установлено: ${missing[*]} (sudo apt install ${missing[*]})"
  fi
fi

echo "== Docker"
if command -v snap >/dev/null 2>&1 && snap list docker >/dev/null 2>&1; then
  fail "Docker установлен из snap, он не поддерживается. Удалите его: sudo snap remove docker, затем запустите скрипт снова"
elif ! command -v docker >/dev/null 2>&1; then
  if installing; then
    echo "  ... устанавливаю Docker Engine через https://get.docker.com"
    installer="$(mktemp)"
    curl -fsSL https://get.docker.com -o "$installer"
    $SUDO sh "$installer"
    rm -f "$installer"
    ok "Docker установлен"
  else
    fail "Docker не установлен (запустите scripts/setup_host.sh без --check)"
  fi
fi

if command -v docker >/dev/null 2>&1; then
  if docker compose version >/dev/null 2>&1; then
    ok "$(docker compose version | head -n1)"
  elif dpkg -s docker.io >/dev/null 2>&1; then
    fail "установлен пакет docker.io из Ubuntu без Compose v2. Удалите его: sudo apt remove docker.io docker-compose, затем запустите скрипт снова"
  else
    fail "нет плагина Docker Compose v2 (команда 'docker compose'). Переустановите Docker: curl -fsSL https://get.docker.com | sudo sh"
  fi

  if command -v systemctl >/dev/null 2>&1 && [ -d /run/systemd/system ]; then
    if systemctl is-active --quiet docker; then
      ok "служба docker запущена"
    elif installing; then
      $SUDO systemctl enable --now docker
      ok "служба docker запущена и включена при старте системы"
    else
      fail "служба docker не запущена (sudo systemctl enable --now docker)"
    fi
  fi

  if [ "$(id -u)" -ne 0 ]; then
    if id -nG "$user_name" | tr ' ' '\n' | grep -qx docker; then
      ok "пользователь $user_name в группе docker"
    elif installing; then
      $SUDO usermod -aG docker "$user_name"
      need_relogin=1
      ok "пользователь $user_name добавлен в группу docker"
    else
      fail "пользователь $user_name не в группе docker (sudo usermod -aG docker $user_name и перезайти в систему)"
    fi
  fi

  if docker info >/dev/null 2>&1; then
    ok "Docker доступен без sudo"
    context="$(docker context show 2>/dev/null || true)"
    if [ "$context" = desktop-linux ]; then
      warn "используется Docker Desktop (контекст desktop-linux). Проект рассчитан на Docker Engine; переключиться: docker context use default"
    fi
  elif [ "$need_relogin" -eq 1 ]; then
    info "доступ к Docker без sudo появится после перезахода в систему"
  elif installing; then
    fail "Docker не отвечает: проверьте 'sudo systemctl status docker'"
  else
    fail "Docker недоступен текущему пользователю. Если вас только что добавили в группу docker, перезайдите в систему"
  fi
fi

echo "== Диск"
docker_root=/var/lib/docker
[ -d "$docker_root" ] || docker_root=/
free_gb="$(df -BG --output=avail "$docker_root" 2>/dev/null | tail -n1 | tr -dc '0-9' || true)"
if [ -z "$free_gb" ]; then
  info "не удалось определить свободное место"
elif [ "$free_gb" -ge 20 ]; then
  ok "свободно ${free_gb} ГБ"
else
  warn "свободно ${free_gb} ГБ; образам и сборке нужно около 15–20 ГБ"
fi

echo "== Окно Webots (нужно только для make sim-gui)"
if [ -n "${DISPLAY:-}" ]; then ok "дисплей ${DISPLAY} (${XDG_SESSION_TYPE:-x11})"; else info "нет DISPLAY: доступен только режим без окна (make sim)"; fi
if command -v xauth >/dev/null 2>&1; then
  ok "xauth"
elif installing && [ -n "${DISPLAY:-}" ]; then
  apt_install xauth
  ok "установлено: xauth"
else
  info "нет xauth (sudo apt install xauth)"
fi
if [ -d /dev/dri ]; then ok "видеокарта через /dev/dri"; else info "нет /dev/dri: окно Webots будет рисоваться программно"; fi

echo
if [ "$failures" -gt 0 ]; then
  echo "Найдено проблем: $failures. Исправьте их по подсказкам выше и запустите скрипт снова."
  exit 1
fi
if [ "$mode" = check ]; then
  echo "Компьютер готов к работе с проектом."
  exit 0
fi

if [ "$run_setup" -eq 0 ]; then
  echo "Компьютер подготовлен. Дальше: make setup"
elif [ "$need_relogin" -eq 1 ]; then
  echo ">>> Запускаю make setup с правами группы docker (в новых терминалах они появятся после перезахода в систему)"
  sg docker -c "make setup"
else
  make setup
fi
if [ "$need_relogin" -eq 1 ]; then
  echo
  echo "ВАЖНО: перезайдите в систему (или перезагрузите компьютер), чтобы docker и make работали без sudo в любом терминале."
fi

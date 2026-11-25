#!/usr/bin/env bash
# Локальная установка Webots без sudo и Docker (Ubuntu 24.04, x86_64).
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cache="$root/.cache"
archive="$cache/webots-download/webots-R2025a-x86-64.tar.bz2"
digest=c5127fb4206c57a5ae5523f1b7f3da8b670bc8926d9ae08595e139f226f38c38
[[ $(uname -m) == x86_64 ]] || { echo 'Требуется Linux x86_64.' >&2; exit 1; }
mkdir -p "$(dirname "$archive")" "$cache/native-deps" "$cache/integration-debs"
if ! [[ -f "$archive" ]] || ! echo "$digest  $archive" | sha256sum --check --status; then
  echo 'Скачиваю Webots R2025a (143 MiB)…'
  curl --fail --location --silent --show-error --retry 2 \
    https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a-x86-64.tar.bz2 \
    --output "$archive.part"
  echo "$digest  $archive.part" | sha256sum --check --status
  mv "$archive.part" "$archive"
fi
if ! [[ -x "$cache/webots/bin/webots-bin" ]]; then
  tar -xjf "$archive" -C "$cache"
fi
if ! command -v Xvfb >/dev/null && ! [[ -x "$cache/native-deps/usr/bin/Xvfb" ]]; then
  echo 'Распаковываю Xvfb локально; системные пакеты не изменяются.'
  (cd "$cache/integration-debs" && apt-get download xvfb xserver-common)
  for package in "$cache/integration-debs/"*.deb; do
    dpkg-deb --extract "$package" "$cache/native-deps"
  done
fi
export PATH="$cache/native-deps/usr/bin:$PATH"
export WEBOTS_HOME="$cache/webots"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/webots:${LD_LIBRARY_PATH:-}"
export QT_QPA_PLATFORM=xcb
command -v xauth >/dev/null || { echo 'Установите пакет xauth.' >&2; exit 1; }
xvfb-run -a "$WEBOTS_HOME/bin/webots-bin" --version
echo 'Webots готов. Следующая команда: make g5-system-build'

#!/usr/bin/env bash
# Copies the standard Webots R2025a assets (PROTO, meshes, textures) that simulation/worlds use
# into simulation/webots_assets, so the simulator image is built and run without any download.
# Run it after adding standard Webots objects to a world; commit simulation/webots_assets afterwards.
#
# How: the official asset archive (693 MB, kept in .cache/) fills a Webots cache; every world is
# loaded once without network; the cache files Webots read are copied under their GitHub paths.
set -euo pipefail
cd "$(dirname "$0")/.."

version=R2025a
zip=.cache/assets-$version.zip
zip_sha256=852e776380f333b593ecb68f33c189e7efb2f35a0014027a1cdc60f8339a3f8f
zip_url=https://github.com/cyberbotics/webots/releases/download/$version/assets-$version.zip

mkdir -p .cache
if [ ! -f "$zip" ]; then
  echo ">>> Скачиваю архив ресурсов Webots $version (693 МБ, один раз)"
  curl -fL --retry 5 -C - -o "$zip.part" "$zip_url"
  mv "$zip.part" "$zip"
fi
echo "$zip_sha256  $zip" | sha256sum -c --quiet -

echo ">>> Собираю образ Webots (этап webots)"
docker build -q --target webots -t capstone-webots:local -f docker/Dockerfile . >/dev/null

echo ">>> Загружаю миры и собираю использованные файлы"
docker run --rm -i --network none -u "$(id -u):$(id -g)" -e USER=webots -e HOME=/tmp -e XDG_CACHE_HOME=/tmp/cache \
  -v "$PWD":/repo -w /repo capstone-webots:local bash -s "$version" "$zip" <<'EOF'
set -euo pipefail
version="$1" zip="$2"
cache="$XDG_CACHE_HOME/Cyberbotics/Webots/assets"
prefix="https://raw.githubusercontent.com/cyberbotics/webots/$version/"
mkdir -p "$cache"
python3 - "$zip" "$cache" <<'PY'
import sys, zipfile
zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])
PY
# Files Webots reads get a fresh access time: reset all of them first, then compare with a marker.
find "$cache" -type f -exec touch -a -d @0 {} +
marker="$(mktemp)"
sleep 1
touch "$marker"
bash docker/check_webots_worlds.sh --settle 5 simulation/worlds/*.wbt

out=simulation/webots_assets
rm -rf "$out/projects"
used="$(mktemp)"
find "$cache" -type f -anewer "$marker" -printf '%f\n' | sort > "$used"
count=0
while read -r hash url; do
  grep -qx "$hash" "$used" || continue
  case "$url" in "$prefix"*) ;; *) continue ;; esac
  target="$out/${url#"$prefix"}"
  mkdir -p "$(dirname "$target")"
  cp "$cache/$hash" "$target"
  count=$((count + 1))
done < <(grep -E '^[0-9a-f]{40} ' "$cache/log.txt")
echo ">>> $count файлов, $(du -sh "$out/projects" | cut -f1) в $out"
EOF
echo "Готово. Проверьте мир (make smoke) и закоммитьте simulation/webots_assets."

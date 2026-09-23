#!/usr/bin/env bash
# One-time GitHub setup of the team repository, run by its owner (needs the gh CLI, see docs/GITHUB_SETUP.md):
#   - teams infra, g1..g5 in the organization with write access to the repository;
#   - @ORG in .github/CODEOWNERS replaced with the organization name (commit the change afterwards);
#   - protection of main: pull requests only, CI green, one approval from the code owners.
# Usage: scripts/github/bootstrap_repo.sh <org>/<repo> [--dry-run]
set -euo pipefail
cd "$(dirname "$0")/../.."

repo_full="${1:-}"
dry_run=0
[ "${2:-}" = "--dry-run" ] && dry_run=1
if [[ ! "$repo_full" =~ ^[A-Za-z0-9-]+/[A-Za-z0-9._-]+$ ]]; then
  sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
fi
org="${repo_full%%/*}"
repo="${repo_full#*/}"

# Required status checks: job names from .github/workflows/ci.yml.
checks=("shellcheck" "make setup (ubuntu-22.04)" "make setup (ubuntu-24.04)")
teams=(infra g1 g2 g3 g4 g5)

run() {
  if [ "$dry_run" -eq 1 ]; then
    echo "  [dry-run] $(printf '%q ' "$@")" >&2
  else
    "$@"
  fi
}

if [ "$dry_run" -eq 0 ]; then
  command -v gh >/dev/null || { echo "Нужен GitHub CLI: https://cli.github.com (sudo apt install gh)" >&2; exit 1; }
  gh auth status >/dev/null || { echo "Выполните gh auth login и повторите" >&2; exit 1; }
  owner_type="$(gh api "users/$org" --jq .type)"
  if [ "$owner_type" != Organization ]; then
    echo "$org — не организация GitHub. Команды и CODEOWNERS по командам работают только в организации." >&2
    exit 1
  fi
fi

echo "== Команды организации $org"
for team in "${teams[@]}"; do
  if [ "$dry_run" -eq 0 ] && gh api "orgs/$org/teams/$team" >/dev/null 2>&1; then
    echo "  $team: уже есть"
  else
    description="Group $team of the capstone project"
    [ "$team" = infra ] && description="Build environment, CI and repository settings"
    run gh api -X POST "orgs/$org/teams" -f name="$team" -f privacy=closed -f description="$description" >/dev/null
    echo "  $team: создана"
  fi
  permission=push
  [ "$team" = infra ] && permission=maintain
  run gh api -X PUT "orgs/$org/teams/$team/repos/$org/$repo" -f permission="$permission" >/dev/null
  echo "  $team: доступ к $repo — $permission"
done

echo "== CODEOWNERS"
if grep -q '@ORG/' .github/CODEOWNERS; then
  run sed -i "s|@ORG/|@$org/|g" .github/CODEOWNERS
  echo "  @ORG заменено на @$org — закоммитьте .github/CODEOWNERS и отправьте в main"
else
  echo "  уже настроен"
fi

echo "== Настройки репозитория"
run gh api -X PATCH "repos/$org/$repo" -F delete_branch_on_merge=true >/dev/null
echo "  ветки удаляются после слияния PR"

echo "== Защита main"
contexts="$(printf '"%s",' "${checks[@]}")"
protection=$(cat <<JSON
{
  "required_status_checks": {"strict": false, "contexts": [${contexts%,}]},
  "enforce_admins": false,
  "required_pull_request_reviews": {
    "dismiss_stale_reviews": true,
    "require_code_owner_reviews": true,
    "required_approving_review_count": 1
  },
  "restrictions": null,
  "required_conversation_resolution": true,
  "allow_force_pushes": false,
  "allow_deletions": false
}
JSON
)
if [ "$dry_run" -eq 1 ]; then
  echo "[dry-run] gh api -X PUT repos/$org/$repo/branches/main/protection --input - <<< '$protection'"
elif ! gh api -X PUT "repos/$org/$repo/branches/main/protection" --input - <<< "$protection" >/dev/null; then
  echo "  Не удалось включить защиту. Для приватного репозитория на бесплатном тарифе она недоступна — сделайте репозиторий публичным." >&2
  exit 1
fi
echo "  main: только через PR, CI (${checks[*]}), одно одобрение владельцев кода"
echo
echo "Готово. Добавьте участников в команды: https://github.com/orgs/$org/teams"

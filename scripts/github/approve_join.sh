#!/usr/bin/env bash
# Processes join requests filed with .github/ISSUE_TEMPLATE/join.yml. For every open request it shows
# the author, name and group and asks what to do:
#   y — invite the author to the organization and the chosen team, comment with next steps, close
#   r — reject: comment and close;  n — skip for now;  q — stop
# Run by the repository owner (gh with the admin:org scope, see docs/GITHUB_SETUP.md).
# Usage: scripts/github/approve_join.sh [--yes | --dry-run]
set -euo pipefail
cd "$(dirname "$0")/../.."

mode=ask
case "${1:-}" in
  --yes) mode=yes ;;
  --dry-run) mode=dry ;;
  "") ;;
  *) sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac

repo="$(git remote get-url origin | sed -E 's#^(git@github\.com:|https://github\.com/)##; s#\.git$##')"
org="${repo%%/*}"

# One tab-separated line per request: issue number, author login, team (g1..g5 or ?), name.
requests="$(gh issue list --repo "$repo" --state open --limit 200 --json number,title,author,body | python3 -c '
import json, re, sys
for issue in json.load(sys.stdin):
    if not issue["title"].startswith("[join]"):
        continue
    sections = dict(re.findall(r"^### (.+?)\n+(.*?)(?=\n### |\Z)", issue["body"] or "", re.S | re.M))
    group = re.match(r"\s*G([1-5])\b", sections.get("Группа", ""))
    name = " ".join(sections.get("Имя и фамилия", "").split()) or "?"
    team = "g" + group.group(1) if group else "?"
    print("\t".join([str(issue["number"]), issue["author"]["login"], team, name]))
')"

if [ -z "$requests" ]; then
  echo "Открытых заявок нет."
  exit 0
fi

comment_and_close() {
  gh issue comment "$1" --repo "$repo" --body "$2" >/dev/null
  gh issue close "$1" --repo "$repo" >/dev/null
}

while IFS=$'\t' read -r number login team name <&3; do
  echo "#$number  @$login  $name  → $team"
  if [ "$team" = "?" ]; then
    echo "  группа не распознана, пропускаю: https://github.com/$repo/issues/$number"
    continue
  fi
  answer=y
  if [ "$mode" = dry ]; then
    echo "  [dry-run] пригласил бы @$login в $org/$team"
    continue
  elif [ "$mode" = ask ]; then
    read -r -p "  Пригласить? [y]es / [n]o, позже / [r]eject / [q]uit: " answer
  fi
  case "$answer" in
    y|Y)
      state="$(gh api "orgs/$org/teams/$team/memberships/$login" --jq .state 2>/dev/null || true)"
      if [ "$state" = active ]; then
        comment_and_close "$number" "@$login уже в команде \`$team\`. Если доступа нет, напишите владельцу репозитория."
        echo "  уже в команде, заявка закрыта"
        continue
      fi
      gh api -X PUT "orgs/$org/teams/$team/memberships/$login" -f role=member >/dev/null
      comment_and_close "$number" "Приглашение в организацию \`$org\` и команду \`$team\` отправлено. Примите его в течение 7 дней: https://github.com/orgs/$org/invitation

Затем выполните «Быстрый старт»: https://github.com/$repo#быстрый-старт"
      echo "  приглашение отправлено, заявка закрыта"
      ;;
    r|R)
      comment_and_close "$number" "Заявка отклонена владельцем репозитория. Если это ошибка, напишите ему напрямую."
      echo "  отклонено"
      ;;
    q|Q) break ;;
    *) echo "  пропущено" ;;
  esac
done 3<<< "$requests"

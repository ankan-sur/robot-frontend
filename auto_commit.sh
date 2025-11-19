#!/usr/bin/env bash
set -euo pipefail

# Auto-commit all changes in this workspace with a generated message (or custom one).
# Usage:
#   ./auto_commit.sh
#   ./auto_commit.sh -m "fix: adjust UI"
#   ./auto_commit.sh "chore: update deps"

msg=""
if [[ ${1:-} == "-m" || ${1:-} == "--message" ]]; then
  shift || true
  msg=${1:-}
  shift || true
elif [[ $# -gt 0 ]]; then
  # Allow passing a single message without -m
  msg=$1
  shift || true
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not a git repository. Run this from inside a repo root."
  exit 1
fi

# Stage all changes (new, modified, deleted)
git add -A

# Determine staged files
staged=$(git diff --cached --name-only)
if [[ -z "$staged" ]]; then
  echo "No changes to commit."
  exit 0
fi

if [[ -z "$msg" ]]; then
  count=$(echo "$staged" | wc -l | tr -d ' ')
  short_list=$(echo "$staged" | head -n 6 | tr '\n' ', ' | sed 's/, $//')
  more=$(( count - 6 ))
  summary="$short_list"
  if (( count > 6 )); then
    summary="$short_list, +$more more"
  fi

  # Derive a simple scope from top-level directories (e.g. frontend/src)
  scope=$(echo "$staged" | awk -F/ '{print $1"/"$2}' | sed 's#/$##' | sort -u | tr '\n' ',' | sed 's/,$//')
  scope=${scope:-repo}

  msg="chore(${scope}): update ${count} file(s): ${summary}"
fi

git commit -m "$msg"
echo "Committed with message:"
echo "  $msg"

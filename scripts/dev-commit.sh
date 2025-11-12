#!/usr/bin/env bash
set -euo pipefail

# Commit-all helper for this repo.
# - Adds all changes
# - Commits with an appropriate message
# - Does NOT push (you run `git push` when ready)

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: not inside a git repository" >&2
  exit 1
fi

# Allow custom message via args or MSG env var
USER_MSG=${MSG:-}
if [[ $# -gt 0 ]]; then
  USER_MSG="$*"
fi

# Default message for the latest set of changes.
# Note: This gets updated by the assistant whenever we change files.
read -r -d '' DEFAULT_MSG <<'EOF'
feat(ui): restore POI picker; Nav2 "Go"; natural-size camera

docs: rewrite guides for rosbridge + WVS + rosboard + system_topics

chore(ros): set POIs type to interfaces/Points; robust parsing in hooks

Details:
- Controls: re-add POI dropdown populated from /pois; Go sends Nav2 goal
- VideoFeed: size to natural image; keep WVS fallback
- MapView: remove map dropdown; keep map/robot/POIs render
- Hooks/Config: align POIs to interfaces/Points; flexible JSON/array parsing
- Docs: refresh README, ARCHITECTURE, CONNECTION_GUIDE, HOSTNAME_AND_SERVE,
        RASPBERRY_PI_SETUP, ROBOT_SETUP, FRONTEND_DOCUMENTATION, REFACTORING_NOTES
EOF

COMMIT_MSG=${USER_MSG:-$DEFAULT_MSG}

# Stage and commit
git add -A

if git diff --cached --quiet; then
  echo "No staged changes to commit." >&2
  exit 0
fi

echo "Committing changes..."
git commit -m "$COMMIT_MSG"

echo "\nDone. Review with: git show --stat -1"
echo "Push when ready: git push"


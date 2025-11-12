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
feat(ui): map select via service; POIs via service; compact camera with maximize

docs: rewrite guides for rosbridge + WVS + rosboard + system_topics

chore(cleanup): remove unused backend + rclpy glue; add .gitignore; add stack test script

chore(teleop): linear/angular sliders for cmd_vel (no generic scaler)

Details:
- Controls: Map dropdown from /available_maps; calls /system/map/select; POIs via /system/map/pois
- VideoFeed: compact above Controls; modal maximize; WVS fallback retained
- Debug: DebugLog embedded in DebugPanel
- Hooks/Config: add services.systemMapSelect/systemMapPois; new useAvailableMaps; usePoisForMap with service + topic fallback
- Remove: backend/, ros_interface/, robot_control/, scripts/start_robot.sh,
          scripts/requirements.txt, scripts/test_connection.sh (obsolete)
- Add: .gitignore; scripts/test_robot_stack.sh (checks rosbridge/WVS/rosboard)
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

#!/usr/bin/env bash
set -euo pipefail

# Backwards-compatible wrapper; prefer scripts/dev-commit.sh
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
"$SCRIPT_DIR/dev-commit.sh" "$@"

echo "Note: This script no longer pushes by default. Run 'git push' when ready."

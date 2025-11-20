#!/bin/bash
# Git diagnostic script for robot deployment
# Run this on the robot to check git status

echo "=========================================="
echo "GIT DIAGNOSTIC FOR ROBOT FRONTEND"
echo "=========================================="
echo ""

cd ~/robot-ui/frontend || { echo "ERROR: Directory ~/robot-ui/frontend not found!"; exit 1; }

echo "1. Current directory:"
pwd
echo ""

echo "2. Current branch:"
git branch --show-current
echo ""

echo "3. All branches:"
git branch -a
echo ""

echo "4. Current commit:"
git log -1 --oneline
echo ""

echo "5. Remote URL:"
git remote -v
echo ""

echo "6. Git status:"
git status
echo ""

echo "7. Last 5 commits:"
git log -5 --oneline --graph --all
echo ""

echo "8. Uncommitted changes:"
git diff --stat
echo ""

echo "=========================================="
echo "RECOMMENDED FIXES:"
echo "=========================================="
echo ""
echo "If you're on wrong branch, run:"
echo "  git checkout clean"
echo ""
echo "If you have local changes, run:"
echo "  git stash"
echo "  git pull origin clean"
echo "  git stash pop"
echo ""
echo "If detached HEAD, run:"
echo "  git checkout clean"
echo "  git pull origin clean"
echo ""
echo "To force match remote:"
echo "  git fetch origin clean"
echo "  git reset --hard origin/clean"
echo "  npm run build"
echo ""

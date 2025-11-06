# Git Setup Guide for Robot

## Fix: Avoid entering password every time

### Option 1: Use SSH Keys (Recommended)

1. **On your computer**, generate SSH key if you don't have one:
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

2. **Copy your public key**:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   # Or on Windows:
   type %USERPROFILE%\.ssh\id_ed25519.pub
   ```

3. **Add SSH key to GitLab**:
   - Go to GitLab: https://gitlab.msu.edu/-/profile/keys
   - Click "Add new key"
   - Paste your public key
   - Save

4. **On robot**, change remote URL to use SSH:
   ```bash
   cd ~/ece480_capstone_henry_ford_health
   git remote set-url origin git@gitlab.msu.edu:girasege/ece480_capstone_henry_ford_health.git
   ```

5. **Copy SSH key to robot** (if needed):
   ```bash
   # On your computer, copy key to robot
   ssh-copy-id pi@fordward.local
   # Or manually copy ~/.ssh/id_ed25519 to robot's ~/.ssh/
   ```

### Option 2: Use Git Credential Helper (Quick Fix)

**On robot**, cache credentials:
```bash
git config --global credential.helper store
# Then enter password once, it will be saved
```

**Or** cache for 1 hour:
```bash
git config --global credential.helper 'cache --timeout=3600'
```

## Fix: Handle Local Changes

When you have local changes that conflict with pull:

### Option 1: Stash (temporary save)
```bash
git stash
git pull
git stash pop  # Restores your changes
```

### Option 2: Discard local changes (if you don't need them)
```bash
git checkout -- frontend/src/ros/config.ts
git pull
```

### Option 3: Commit your changes first
```bash
git add .
git commit -m "Local changes"
git pull
# Resolve conflicts if any
```

## Quick Commands

```bash
# Check what's changed locally
git status

# See what's different
git diff frontend/src/ros/config.ts

# Discard ALL local changes (be careful!)
git reset --hard HEAD

# Force pull (discards local changes)
git fetch origin
git reset --hard origin/refactor/ros-docker-rosbridge
```


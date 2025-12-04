# Git Setup

Git configuration for working with the robot repository.

## SSH Key Setup (Recommended)

Avoid entering passwords every time.

### Generate SSH Key

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

### Add to GitLab/GitHub

Copy your public key:

```bash
cat ~/.ssh/id_ed25519.pub
```

Add to your Git hosting service:
- GitLab: Settings → SSH Keys
- GitHub: Settings → SSH and GPG Keys

### Update Remote URL

```bash
git remote set-url origin git@github.com:username/repo.git
```

## Credential Caching

Alternative to SSH keys.

### Store Permanently

```bash
git config --global credential.helper store
```

### Cache for 1 Hour

```bash
git config --global credential.helper 'cache --timeout=3600'
```

## Handling Local Changes

### Stash Changes

```bash
git stash
git pull
git stash pop
```

### Discard Changes

```bash
git checkout -- path/to/file
git pull
```

### Reset to Remote

```bash
git fetch origin
git reset --hard origin/main
```

## Useful Commands

```bash
# Check status
git status

# View changes
git diff

# Pull latest
git pull origin main

# Push changes
git push origin main
```

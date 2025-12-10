# GitHub Repository Setup Instructions

This repository is ready to be pushed to GitHub. Follow these steps:

## Prerequisites

You need Git installed. Choose one of the following options:

### Option A: Install Git for Windows

1. **Download Git for Windows:**
   - Visit: https://git-scm.com/download/win
   - Download the installer (64-bit Git for Windows Setup)
   - Run the installer with default settings

2. **Verify installation:**
   ```powershell
   git --version
   ```

3. **Configure Git (first time only):**
   ```powershell
   git config --global user.name "Your Name"
   git config --global user.email "your.email@example.com"
   ```

### Option B: Use GitHub Desktop (GUI Alternative)

1. **Download GitHub Desktop:**
   - Visit: https://desktop.github.com/
   - Install GitHub Desktop

2. **Create repository:**
   - Open GitHub Desktop
   - File → Add Local Repository
   - Browse to `swift-autonomous-drone-snapshot` folder
   - Click "Publish repository" to create on GitHub

### Option C: Manual Upload via GitHub Web Interface

1. **Create repository on GitHub:**
   - Go to https://github.com/new
   - Repository name: `swift-autonomous-drone-snapshot`
   - Set to Public
   - Do NOT initialize with README, .gitignore, or license

2. **Upload files:**
   - After creating the repository, click "uploading an existing file"
   - Drag and drop all files from `swift-autonomous-drone-snapshot` folder
   - Commit with message: "Initial commit: Swift Autonomous Drone snapshot"

## Setup Using Git (Command Line)

Once Git is installed, follow these steps:

### 1. Initialize Git Repository

```powershell
cd swift-autonomous-drone-snapshot
git init
```

### 2. Add All Files

```powershell
git add .
```

### 3. Create Initial Commit

```powershell
git commit -m "Initial commit: Swift Autonomous Drone snapshot repository"
```

### 4. Create Repository on GitHub

**Option 1: Using GitHub CLI (if installed)**
```powershell
gh repo create swift-autonomous-drone-snapshot --public --source=. --remote=origin --push
```

**Option 2: Manual Setup**
1. Go to https://github.com/new
2. Repository name: `swift-autonomous-drone-snapshot`
3. Set to Public
4. Do NOT initialize with README, .gitignore, or license
5. Click "Create repository"

### 5. Connect and Push

```powershell
git remote add origin https://github.com/YOUR_USERNAME/swift-autonomous-drone-snapshot.git
git branch -M main
git push -u origin main
```

**Note:** Replace `YOUR_USERNAME` with your actual GitHub username.

## Setup Using GitHub Desktop (GUI)

1. **Open GitHub Desktop**
2. **File → Add Local Repository**
3. **Browse to:** `swift-autonomous-drone-snapshot` folder
4. **Click "Publish repository"**
5. **Set repository name:** `swift-autonomous-drone-snapshot`
6. **Set to Public**
7. **Click "Publish Repository"**

## Verify

After pushing, verify the repository at:
`https://github.com/YOUR_USERNAME/swift-autonomous-drone-snapshot`

## Next Steps

1. Update `PROJECT.json` with the GitHub URL:
   ```json
   "github": "https://github.com/YOUR_USERNAME/swift-autonomous-drone-snapshot"
   ```

2. Update `README.md` with your portfolio URL (if applicable):
   - Replace `YOUR_PORTFOLIO_URL` with your actual portfolio URL

3. Add media files (optional):
   - Copy `images/gazebo_drone_hovering.jpeg` to `demo/hero.png`
   - Copy `images/rqt_graph.png.png` to `docs/architecture_diagram.png`

4. Commit and push updates:
   ```powershell
   git add .
   git commit -m "Update repository links"
   git push
   ```

## Troubleshooting

### "git is not recognized"
- Install Git for Windows (see Option A above)
- Or use GitHub Desktop (Option B)
- Or upload manually via web interface (Option C)

### "Permission denied" when pushing
- Make sure you're logged into GitHub
- Use GitHub Desktop for easier authentication
- Or set up SSH keys: https://docs.github.com/en/authentication/connecting-to-github-with-ssh

### "Repository already exists"
- If you already created the repo on GitHub, just connect:
  ```powershell
  git remote add origin https://github.com/YOUR_USERNAME/swift-autonomous-drone-snapshot.git
  git push -u origin main
  ```

## Repository Checklist

### ✅ Included Files
- [x] Sanitized ROS 2 node code (illustrative versions)
- [x] README.md with project overview
- [x] LICENSE (MIT)
- [x] .gitignore (excludes build files, CAD, logs, etc.)
- [x] case_study.md
- [x] PROJECT.json
- [x] Demo launch script
- [x] Documentation structure

### ❌ Excluded Files (by design)
- [x] CAD files (*.STEP, *.SLDPRT, *.stl)
- [x] PCB files (*.brd, *.sch)
- [x] Raw logs (*.log, *.csv)
- [x] Large binary files (>10MB)
- [x] Build artifacts (build/, install/, log/)
- [x] Sensitive data (keys, configs with credentials)
- [x] Tuned PID parameters (replaced with illustrative examples)
- [x] Calibration data

## Security Notes

All sensitive information has been removed:
- No hardcoded credentials
- No private keys
- No calibration parameters
- No tuned PID gains (replaced with example values)
- No proprietary algorithms (illustrative pseudocode only)

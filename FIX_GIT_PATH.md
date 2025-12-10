# Fix Git "Not Recognized" Issue

Git is installed but not in your PATH. Here are solutions:

## 🔧 Solution 1: Add Git to PATH (Recommended)

### Step 1: Find Git Installation

1. Open File Explorer
2. Navigate to: `C:\Program Files\Git\bin\`
   - OR: `C:\Program Files (x86)\Git\bin\`
3. Copy the full path (e.g., `C:\Program Files\Git\bin`)

### Step 2: Add to PATH

**Method A: Via Settings (Windows 10/11)**
1. Press `Win + X` → Click "System"
2. Click "Advanced system settings" (on the right)
3. Click "Environment Variables" button
4. Under "User variables", find "Path" → Click "Edit"
5. Click "New" → Paste the Git bin path (e.g., `C:\Program Files\Git\bin`)
6. Click "OK" on all windows
7. **Close and reopen PowerShell/Command Prompt**

**Method B: Via PowerShell (Quick Fix)**
```powershell
# Add Git to PATH for current session (temporary)
$env:PATH += ";C:\Program Files\Git\bin"

# Test
git --version
```

**Method C: Via Command Prompt (Permanent)**
```cmd
setx PATH "%PATH%;C:\Program Files\Git\bin"
```
Then close and reopen your terminal.

## 🚀 Solution 2: Use Git with Full Path

You can use Git directly without adding to PATH:

```powershell
# Find Git first
& "C:\Program Files\Git\bin\git.exe" --version

# Then use it with full path
cd swift-autonomous-drone-snapshot
& "C:\Program Files\Git\bin\git.exe" init
& "C:\Program Files\Git\bin\git.exe" add .
& "C:\Program Files\Git\bin\git.exe" commit -m "Initial commit"
```

## 🎯 Solution 3: Use Git Bash (Easiest)

Git Bash is usually installed with Git and has Git in its PATH:

1. Press `Win` key
2. Type "Git Bash"
3. Open "Git Bash"
4. Navigate to your folder:
   ```bash
   cd /c/Users/User/OneDrive/Documents/sahil/sahil/swift_drone/clean_folder_swift_drone/swift-autonomous-drone-snapshot
   ```
5. Use Git commands normally:
   ```bash
   git init
   git add .
   git commit -m "Initial commit"
   ```

## 🖥️ Solution 4: Use GitHub Desktop (No PATH Needed)

If you want to avoid PATH issues entirely:

1. Download: https://desktop.github.com/
2. Install and sign in
3. File → Add Local Repository
4. Browse to `swift-autonomous-drone-snapshot`
5. Click "Publish repository"

## ✅ Verify Git is Working

After fixing PATH, test with:
```powershell
git --version
```

You should see something like: `git version 2.x.x`

## 📝 Quick Setup Script

Save this as `setup_git.ps1` and run it:

```powershell
# Add Git to PATH for current session
$gitPath = "C:\Program Files\Git\bin"
if (Test-Path $gitPath) {
    $env:PATH += ";$gitPath"
    Write-Host "Git added to PATH for this session"
    git --version
} else {
    Write-Host "Git not found at $gitPath"
    Write-Host "Please check your Git installation location"
}
```

## 🔍 Find Your Git Installation

Run this to find where Git is installed:

```powershell
Get-ChildItem -Path "C:\Program Files" -Filter "git.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object FullName
Get-ChildItem -Path "C:\Program Files (x86)" -Filter "git.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object FullName
```

---

**Recommendation:** Use **Git Bash** (Solution 3) - it's the quickest fix and doesn't require changing system settings.


# Quick Git Setup Script
# This adds Git to PATH for the current PowerShell session

Write-Host "Looking for Git installation..." -ForegroundColor Yellow

# Common Git installation paths
$gitPaths = @(
    "C:\Program Files\Git\bin\git.exe",
    "C:\Program Files (x86)\Git\bin\git.exe",
    "$env:LOCALAPPDATA\Programs\Git\bin\git.exe",
    "$env:ProgramFiles\Git\bin\git.exe"
)

$gitFound = $null
foreach ($path in $gitPaths) {
    if (Test-Path $path) {
        $gitFound = Split-Path $path
        Write-Host "Found Git at: $gitFound" -ForegroundColor Green
        break
    }
}

if ($gitFound) {
    # Add to PATH for this session
    $env:PATH += ";$gitFound"
    Write-Host "Git added to PATH for this session!" -ForegroundColor Green
    Write-Host ""
    
    # Test Git
    Write-Host "Testing Git..." -ForegroundColor Yellow
    git --version
    
    Write-Host ""
    Write-Host "Git is now available! You can use git commands." -ForegroundColor Green
    Write-Host ""
    Write-Host "To initialize the repository, run:" -ForegroundColor Cyan
    Write-Host "  cd swift-autonomous-drone-snapshot" -ForegroundColor White
    Write-Host "  git init" -ForegroundColor White
    Write-Host "  git add ." -ForegroundColor White
    Write-Host "  git commit -m 'Initial commit'" -ForegroundColor White
} else {
    Write-Host "Git not found in common locations." -ForegroundColor Red
    Write-Host ""
    Write-Host "Options:" -ForegroundColor Yellow
    Write-Host "1. Use Git Bash (search 'Git Bash' in Start menu)" -ForegroundColor Cyan
    Write-Host "2. Use GitHub Desktop (https://desktop.github.com/)" -ForegroundColor Cyan
    Write-Host "3. Manually add Git to PATH (see FIX_GIT_PATH.md)" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "To find Git, check:" -ForegroundColor Yellow
    Write-Host "  - C:\Program Files\Git\" -ForegroundColor White
    Write-Host "  - C:\Program Files (x86)\Git\" -ForegroundColor White
}


$ErrorActionPreference = 'Stop'

function Require-Administrator {
    $identity = [Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsBuiltInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    if (-not $identity.IsInRole($principal)) {
        throw 'This script must be run from an elevated PowerShell session (Run as Administrator).'
    }
}

function Ensure-Command {
    param([string]$Name)
    if (-not (Get-Command $Name -ErrorAction SilentlyContinue)) {
        throw "Required command '$Name' is not available."
    }
}

function Install-WithWinget {
    param([string]$Id)
    $package = & "$env:LOCALAPPDATA\Microsoft\WindowsApps\winget.exe" list --id $Id --exact 2>$null
    if ($LASTEXITCODE -ne 0 -or -not ($package -join "`n" -match [regex]::Escape($Id))) {
        Write-Host "Installing $Id ..."
        & "$env:LOCALAPPDATA\Microsoft\WindowsApps\winget.exe" install --id $Id --accept-source-agreements --accept-package-agreements --silent
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to install $Id"
        }
    } else {
        Write-Host "$Id is already installed."
    }
}

Require-Administrator

$wingetPath = Join-Path $env:LOCALAPPDATA 'Microsoft\WindowsApps\winget.exe'
if (-not (Test-Path $wingetPath)) {
    throw 'winget was not found. Please install App Installer from the Microsoft Store.'
}

Write-Host 'Installing CMake and Ninja...'
Install-WithWinget -Id 'Kitware.CMake'
Install-WithWinget -Id 'Ninja-build.Ninja'

Write-Host 'Installing Visual Studio Build Tools with MSVC and CMake support...'
& $wingetPath install --id Microsoft.VisualStudio.2022.BuildTools --override "--wait --passThru --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows11SDK.22000 --add Microsoft.VisualStudio.Component.VC.CoreBuildTools --add Microsoft.VisualStudio.Component.VC.CMake.Project --add Microsoft.VisualStudio.Component.CMake.Tools --add Microsoft.VisualStudio.Component.VC.Redist.14.Latest --add Microsoft.VisualStudio.Component.VC.14.40.17.10.x86.x64 --add Microsoft.VisualStudio.Component.VC.14.40.17.10.ATL --add Microsoft.VisualStudio.Component.VC.14.40.17.10.MFC" --accept-source-agreements --accept-package-agreements --silent
if ($LASTEXITCODE -ne 0) {
    throw 'Visual Studio Build Tools installation failed.'
}

Ensure-Command -Name 'cmake'
Ensure-Command -Name 'ninja'

$qtRoot = 'C:\Users\Dmytr\Qt\6.8.3\msvc2022_64'
if (-not (Test-Path $qtRoot)) {
    throw "Qt installation was not found at $qtRoot. Please install Qt 6.8 MSVC 2022 64-bit and rerun this script."
}

Write-Host 'Configuring the project...'
cmake -S . -B build -G 'Ninja' -DCMAKE_PREFIX_PATH="$qtRoot"
if ($LASTEXITCODE -ne 0) {
    throw 'CMake configure failed.'
}

Write-Host 'Building CAVR Studio...'
cmake --build build --config Debug
if ($LASTEXITCODE -ne 0) {
    throw 'Build failed.'
}

Write-Host 'Build completed. You can launch the app with:'
Write-Host 'build\apps\cavr-studio\Debug\cavr-studio.exe'

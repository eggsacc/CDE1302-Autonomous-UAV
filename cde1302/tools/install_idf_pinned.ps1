<#
Install the ESP-IDF version this firmware is pinned to (tools/idf-pin.env) on
Windows, next to any other ESP-IDF install, without touching it.

    powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1

Then, in every new PowerShell window:

    . .\tools\idf_pinned.ps1        # note the leading dot
    idf.py -p COM5 build flash monitor

Why not the released ESP-IDF 6.0: the camera library this firmware uses
(esp32-camera 2.1.5) does not build with it.  Why these folders: ESP-IDF does
not support paths with spaces, and the default tools folder is under your user
profile, which may contain one.

Safe to re-run: an existing checkout at the right commit is kept; install.ps1
only downloads tools that are missing.  ~1 GB download, ~3 GB on disk.
#>
param(
    [string]$IdfPath   = "C:\esp\idf-pinned",
    [string]$ToolsPath = "C:\Espressif\tools-pinned"
)
# "Continue", not "Stop": in Windows PowerShell 5.1, "Stop" turns anything git
# writes to stderr (even progress) into a fatal error.  Exit codes are checked
# explicitly after every native command instead.
$ErrorActionPreference = "Continue"

function Fail($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

foreach ($p in @($IdfPath, $ToolsPath)) {
    if ($p -match '\s') { Fail "'$p' contains a space; ESP-IDF does not support that" }
}
$pinFile = Join-Path $PSScriptRoot "idf-pin.env"
$ref = (Select-String -Path $pinFile -Pattern '^IDF_CHECKOUT_REF=(\S+)').Matches[0].Groups[1].Value
if (-not $ref) { Fail "IDF_CHECKOUT_REF not found in $pinFile" }
Write-Host "Pinned ESP-IDF commit: $ref"

# ---- 1. Source at exactly that commit ----
# Superproject: just that one commit.  Submodules: partial clones (history, but
# file contents fetched on demand), NOT --depth 1: a shallow submodule fetch
# cannot always reach the commit ESP-IDF records, and git then silently checks
# out the branch tip instead -- a different ESP-IDF than the pin.
$haveRepo = Test-Path (Join-Path $IdfPath ".git")
if ($haveRepo) {
    $head = (git -C $IdfPath rev-parse HEAD).Trim()
    if ($head -ne $ref) { Fail "$IdfPath is at $head, not $ref. Move it away and re-run." }
    Write-Host "Source already at the pinned commit: $IdfPath"
} else {
    New-Item -ItemType Directory -Force $IdfPath -ErrorAction Stop | Out-Null
    git -C $IdfPath init -q
    if ($LASTEXITCODE) { Fail "git init failed" }
    git -C $IdfPath remote add origin https://github.com/espressif/esp-idf.git
    if ($LASTEXITCODE) { Fail "git remote add failed" }
    git -C $IdfPath -c advice.detachedHead=false fetch --depth 1 origin $ref
    if ($LASTEXITCODE) { Fail "git fetch failed" }
    git -C $IdfPath -c advice.detachedHead=false checkout -q FETCH_HEAD
    if ($LASTEXITCODE) { Fail "git checkout failed" }
}
git -C $IdfPath submodule update --init --recursive --force --filter=blob:none --jobs 8
if ($LASTEXITCODE) { Fail "git submodule update failed" }
# Every submodule must sit on the exact commit ESP-IDF records ('+' = different
# commit, '-' = not initialised, 'U' = conflict).
$bad = git -C $IdfPath submodule status --recursive | Where-Object { $_ -match '^[-+U]' }
if ($bad) { Fail ("submodules not at their pinned commits:`n" + ($bad -join "`n")) }
Write-Host "All submodules at their pinned commits."

# ---- 2. Toolchain + Python env for the ESP32-S3, into $ToolsPath ----
# Clear variables another ESP-IDF shell (e.g. the EIM 6.0 profile) may have set.
# MSYSTEM: set when launched from Git Bash; install.ps1 then refuses ("MSys/Mingw").
foreach ($v in "IDF_PATH", "IDF_PYTHON_ENV_PATH", "IDF_COMPONENT_LOCAL_STORAGE_URL",
               "ESP_ROM_ELF_DIR", "OPENOCD_SCRIPTS", "MSYSTEM") {
    Remove-Item "Env:$v" -ErrorAction SilentlyContinue
}
$env:IDF_TOOLS_PATH = $ToolsPath
& (Join-Path $IdfPath "install.ps1") esp32s3
if ($LASTEXITCODE) { Fail "install.ps1 failed (exit $LASTEXITCODE)" }

Write-Host ""
Write-Host "Done. In each new PowerShell window, from the cde1302 folder:" -ForegroundColor Green
Write-Host "    . .\tools\idf_pinned.ps1"

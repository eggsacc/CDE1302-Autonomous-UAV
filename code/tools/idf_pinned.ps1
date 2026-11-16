<#
Open the pinned ESP-IDF (installed by tools\install_idf_pinned.ps1) in THIS
PowerShell window.  Dot-source it, from the cde1302 folder:

    . .\tools\idf_pinned.ps1
    idf.py menuconfig
    idf.py -p COM5 build flash monitor

Refuses if the checkout is not the commit in tools/idf-pin.env, so you never
build the drone firmware with a different ESP-IDF by accident.
#>
param(
    [string]$IdfPath   = "C:\esp\idf-pinned",
    [string]$ToolsPath = "C:\Espressif\tools-pinned"
)

if ($MyInvocation.InvocationName -ne '.') {
    Write-Host "Run it with a leading dot so it changes this window:  . .\tools\idf_pinned.ps1" -ForegroundColor Yellow
    return
}
$pinFile = Join-Path $PSScriptRoot "idf-pin.env"
$ref = (Select-String -Path $pinFile -Pattern '^IDF_CHECKOUT_REF=(\S+)').Matches[0].Groups[1].Value
if (-not (Test-Path (Join-Path $IdfPath "export.ps1"))) {
    Write-Host "No ESP-IDF at $IdfPath. Run: powershell -ExecutionPolicy Bypass -File tools\install_idf_pinned.ps1" -ForegroundColor Red
    return
}
$head = (git -C $IdfPath rev-parse HEAD).Trim()
if ($head -ne $ref) {
    Write-Host "$IdfPath is at $head but tools/idf-pin.env pins $ref. Re-run install_idf_pinned.ps1." -ForegroundColor Red
    return
}

# Forget any other ESP-IDF environment opened earlier in this window.
foreach ($v in "IDF_PATH", "IDF_PYTHON_ENV_PATH", "IDF_COMPONENT_LOCAL_STORAGE_URL",
               "ESP_ROM_ELF_DIR", "OPENOCD_SCRIPTS", "MSYSTEM") {
    Remove-Item "Env:$v" -ErrorAction SilentlyContinue
}
$env:IDF_TOOLS_PATH = $ToolsPath
# ESP-IDF names its Python env after whichever python is first on PATH, so with
# a conda env active it looks for e.g. idf6.2_py3.12_env and fails.  Point it at
# the env install_idf_pinned.ps1 actually created, whatever python is active.
$envs = @(Get-ChildItem (Join-Path $ToolsPath "python_env") -Directory -Filter "idf*_env" -ErrorAction SilentlyContinue)
if ($envs.Count -eq 1) {
    $env:IDF_PYTHON_ENV_PATH = $envs[0].FullName
} elseif ($envs.Count -eq 0) {
    Write-Host "No ESP-IDF Python env in $ToolsPath\python_env. Re-run install_idf_pinned.ps1." -ForegroundColor Red
    return
}   # several: let ESP-IDF pick the one matching the active python
. (Join-Path $IdfPath "export.ps1")
if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
    Write-Host "ESP-IDF activation failed (see the error above); idf.py is not available." -ForegroundColor Red
    return
}
Write-Host "Pinned ESP-IDF ready ($($ref.Substring(0, 10))). idf.py is on PATH." -ForegroundColor Green

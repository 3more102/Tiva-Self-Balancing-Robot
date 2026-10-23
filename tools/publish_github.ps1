# Push this project to the existing user-owned repository.
# Requires Git and GitHub CLI (gh) already authenticated.
param([string]$Owner='3more102', [string]$RepoName='Tiva-Self-Balancing-Robot')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw 'Install Git.' }
if (-not (Get-Command gh -ErrorAction SilentlyContinue)) { throw 'Install GitHub CLI.' }
& gh auth status
if ($LASTEXITCODE -ne 0) { throw 'Run gh auth login.' }
Set-Location $root
if (-not (Test-Path .git)) { & git init -b main }
& git add .
& git diff --cached --quiet
if ($LASTEXITCODE -ne 0) {
  & git commit -m 'Update Tiva self-balancing robot project'
  if ($LASTEXITCODE -ne 0) { throw 'Set Git user.name and user.email.' }
}
$remote="https://github.com/$Owner/$RepoName.git"
if (-not (& git remote)) { & git remote add origin $remote }
& git push -u origin main
if ($LASTEXITCODE -ne 0) { throw 'Git push failed. Fetch existing commits and reconcile before retrying.' }
Write-Output "Published: https://github.com/$Owner/$RepoName"

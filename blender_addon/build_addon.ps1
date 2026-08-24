$pkg = "$PSScriptRoot\jubilee_digital_twin"
$zip = "$PSScriptRoot\jubilee_digital_twin.zip"

Remove-Item $zip -ErrorAction SilentlyContinue
# Collect only .py and .md files — skip __pycache__ and .pyc
$files = Get-ChildItem -Path $pkg -Recurse -File |
    Where-Object { $_.Extension -in '.py', '.md' -and $_.FullName -notmatch '__pycache__' }
$tmp = "$env:TEMP\jubilee_digital_twin"
Remove-Item $tmp -Recurse -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path $tmp | Out-Null
foreach ($f in $files) {
    $rel = $f.FullName.Substring($pkg.Length + 1)
    $dst = Join-Path $tmp $rel
    New-Item -ItemType Directory -Path (Split-Path $dst) -Force | Out-Null
    Copy-Item $f.FullName $dst
}
Compress-Archive -Path $tmp -DestinationPath $zip
Remove-Item $tmp -Recurse
Write-Host "Built $zip"
Write-Host "Install: Blender -> Preferences -> Add-ons -> Install from File -> jubilee_digital_twin.zip"

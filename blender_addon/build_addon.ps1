$pkg = "$PSScriptRoot\jubilee_digital_twin"
$zip = "$PSScriptRoot\jubilee_digital_twin.zip"

Remove-Item $zip -ErrorAction SilentlyContinue
Compress-Archive -Path $pkg -DestinationPath $zip
Write-Host "Built $zip"
Write-Host "Install: Blender -> Preferences -> Add-ons -> Install from File -> jubilee_digital_twin.zip"

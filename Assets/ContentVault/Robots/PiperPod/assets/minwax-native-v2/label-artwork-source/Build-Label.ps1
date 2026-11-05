$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$taskSource = Join-Path $PSScriptRoot 'BuildLabel.cs.txt'
Add-Type -TypeDefinition (Get-Content -LiteralPath $taskSource -Raw) -ReferencedAssemblies System.Drawing
[MinwaxLabelArtwork]::Generate($PSScriptRoot)
Write-Output 'Generated editable SVG, 4096px label albedo, 1024px packed metallic/roughness and front preview.'

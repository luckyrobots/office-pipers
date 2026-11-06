<#
Checks that this package is self-contained, for when you change it. Changes nothing.

Nothing needs running to use the package: while LuckyLive.exe is running, it registers the assets
listed in package.manifest in the open project (under their fixed handles, so the prefabs keep
their meshes, materials and graphs), and runs luckylive/curve_path.py in Edit mode.

This warns about any asset reference that points outside the package, any prefab that claims
another prefab's handle, duplicated nested prefab entities, and any asset file missing from
package.manifest. Run it with  powershell -ExecutionPolicy Bypass -File Check-Package.ps1
(or right-click > Run with PowerShell).

  -NoPause     Don't wait for Enter before closing
#>
param([switch]$NoPause)

$ErrorActionPreference = 'Stop'

# "Run with PowerShell" closes the window on exit, so wait for Enter to keep the message readable
function Finish([int]$code) {
    if (-not $NoPause) { Read-Host 'Press Enter to close' | Out-Null }
    exit $code
}
$package = $PSScriptRoot
$manifest = Get-Content (Join-Path $package 'package.manifest') -Raw | ConvertFrom-Json

# The project's registry, if the package is in one, names what an outside reference points at
$entries = New-Object System.Collections.ArrayList
$assets = $package
while ($assets -and -not (Test-Path (Join-Path $assets 'AssetRegistry.hzr'))) { $assets = Split-Path $assets -Parent }
if ($assets) {
    $text = Get-Content (Join-Path $assets 'AssetRegistry.hzr') -Raw
    foreach ($m in [regex]::Matches($text, '(?m)^  - Handle: (\d+)\r?\n    FilePath: (.*?)\r?\n    Type: (\S+)')) {
        [void]$entries.Add([pscustomobject]@{ Handle = [UInt64]$m.Groups[1].Value; FilePath = $m.Groups[2].Value; Type = $m.Groups[3].Value })
    }
}

# Check the package is self-contained: every asset reference in its files points at a package
# asset, each prefab claims its own handle, and every asset file is in the manifest
$handles = @{}
foreach ($asset in $manifest.assets) { $handles[[string]$asset.handle] = $asset.path }
$problems = 0
foreach ($asset in $manifest.assets) {
    $file = Join-Path $package $asset.path
    if (-not (Test-Path $file) -or $asset.type -notin 'Prefab', 'StaticMesh', 'Material') { continue }
    $content = Get-Content $file -Raw
    $reported = @{}
    # Asset references: mesh, mesh source, graph and prefab handles, and material table slots
    foreach ($m in [regex]::Matches($content, '(?m)^\s*(AssetID|MeshSource|PodGraph|\d+): (\d{6,})\s*$')) {
        $key, $ref = $m.Groups[1].Value, $m.Groups[2].Value
        if ($reported.ContainsKey("$key $ref")) { continue }
        $reported["$key $ref"] = $true
        if (-not $handles.ContainsKey($ref)) {
            $outside = @($entries | Where-Object { [string]$_.Handle -eq $ref } | ForEach-Object { $_.FilePath })
            $where = if ($outside) { "$($outside[0]), outside the package" } else { 'nothing in this project' }
            Write-Host "  $($asset.path): $key $ref refers to $where." -ForegroundColor Yellow
            $problems++
        }
    }
    # Prefab entities: its own entities claim its handle; nested instances of another prefab must
    # name a package prefab and an entity that prefab has, or the editor can't resolve them and
    # the instance loses its parts and positions
    if ($asset.type -ne 'Prefab') { continue }
    foreach ($m in [regex]::Matches($content, '(?m)^\s+Prefab: (\d+)\r?\n\s+SourceEntity: (\d+)\r?\n\s+NestedPrefabEntityID: (\d+)')) {
        $ref, $source, $nested = $m.Groups[1].Value, $m.Groups[2].Value, $m.Groups[3].Value
        if ($nested -eq '0') {
            if ($ref -ne [string]$asset.handle -and -not $reported.ContainsKey("own $ref")) {
                $reported["own $ref"] = $true
                Write-Host "  $($asset.path) claims prefab handle $ref, but the manifest gives it $($asset.handle). Two prefabs with one handle confuse the editor." -ForegroundColor Yellow
                $problems++
            }
            continue
        }
        # Duplicating a nested instance's child in the editor gives two entities one source; the
        # editor then can't map the instance back and crashes when one is selected
        if ($reported.ContainsKey("nested $ref $source")) {
            if (-not $reported.ContainsKey("duplicate $ref $source")) {
                $reported["duplicate $ref $source"] = $true
                Write-Host "  $($asset.path): several entities are nested copies of entity $source in $($handles[$ref]). Unpack the duplicates or make them separate instances." -ForegroundColor Yellow
                $problems++
            }
            continue
        }
        $reported["nested $ref $source"] = $true
        if (-not $handles.ContainsKey($ref)) {
            $outside = @($entries | Where-Object { [string]$_.Handle -eq $ref } | ForEach-Object { $_.FilePath })
            $where = if ($outside) { "$($outside[0]), outside the package" } else { 'nothing in this project' }
            Write-Host "  $($asset.path): nested prefab $ref refers to $where." -ForegroundColor Yellow
            $problems++
        } elseif ($handles[$ref] -like '*.hprefab' -and (Test-Path (Join-Path $package $handles[$ref])) -and
                  (Get-Content (Join-Path $package $handles[$ref]) -Raw) -notmatch "(?m)^  - Entity: $source\s*$") {
            Write-Host "  $($asset.path): nested entity $source isn't in $($handles[$ref])." -ForegroundColor Yellow
            $problems++
        }
    }
}
foreach ($f in Get-ChildItem $package -Recurse -File -Include *.hprefab, *.hsmesh, *.hmaterial, *.hpodgraph, *.glb, *.png, *.jpg) {
    $rel = [Uri]::UnescapeDataString((New-Object Uri ((Resolve-Path $package).Path.TrimEnd('\') + '\')).MakeRelativeUri((New-Object Uri $f.FullName)).ToString())
    if ($rel -notin @($manifest.assets | ForEach-Object { $_.path })) {
        Write-Host "  $rel isn't in package.manifest, so it gets a different handle in each project." -ForegroundColor Yellow
        $problems++
    }
}
if ($problems -eq 0) {
    Write-Host 'Package check: all references stay inside the package.' -ForegroundColor Green
} else {
    Write-Host "Package check: $problems problem(s) above. The package may not work when copied to another project." -ForegroundColor Yellow
}
if ($problems -eq 0) { Finish 0 } else { Finish 1 }

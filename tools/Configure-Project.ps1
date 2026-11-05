param([string]$ExportDirectory)
$ErrorActionPreference='Stop'
$projectRoot=(Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if([string]::IsNullOrWhiteSpace($ExportDirectory)){
    $ExportDirectory=Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'LuckyRobots/OfficePipers/TubeEpisodes'
}
$ExportDirectory=[IO.Path]::GetFullPath($ExportDirectory)
$targets=@{
    'TubePickPlaceNative.hscene'='piper_tube_task_native_v2.xml'
    'TubePickPlace.hscene'='piper_tube_task.xml'
}
$utf8=New-Object Text.UTF8Encoding($false)
foreach($entry in $targets.GetEnumerator()){
    $scene=Join-Path $projectRoot ('Assets/Scenes/'+$entry.Key)
    $model=(Resolve-Path -LiteralPath (Join-Path $projectRoot ('Assets/ContentVault/Robots/PiperPod/'+$entry.Value))).Path.Replace('\','/')
    $content=[IO.File]::ReadAllText($scene)
    foreach($field in @(@('PositionActuatorModelSource',$model),@('ExportDirectory',$ExportDirectory.Replace('\','/')))){
        $pattern='(?m)(Name: '+[regex]::Escape($field[0])+'\r?\n[ \t]+Type: String\r?\n[ \t]+Value: )[^\r\n]*'
        $matches=[regex]::Matches($content,$pattern)
        if($matches.Count -ne 1){throw "Expected one $($field[0]) field in $scene"}
        $quoted=ConvertTo-Json -InputObject ([string]$field[1]) -Compress
        $content=[regex]::Replace($content,$pattern,[Text.RegularExpressions.MatchEvaluator]{param($match) $match.Groups[1].Value+$quoted})
    }
    [IO.File]::WriteAllText($scene,$content,$utf8)
    Write-Output ('Configured '+$entry.Key)
}
Write-Output ('Trajectory exports: '+$ExportDirectory)
Write-Output 'Open office pipers.hproj in the matching Lucky Engine development build and rebuild scripts.'

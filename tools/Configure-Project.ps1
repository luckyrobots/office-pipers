param([string]$ExportDirectory)
$ErrorActionPreference='Stop'
$projectRoot=(Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$defaultExportRoot=Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'LuckyRobots/OfficePipers'
if(-not [string]::IsNullOrWhiteSpace($ExportDirectory)){
    $ExportDirectory=[IO.Path]::GetFullPath($ExportDirectory)
}
$targets=[ordered]@{
    'TubePickPlaceNative.hscene'=@{Model='piper_tube_task_native_v2.xml'; Output='TubeEpisodes'}
    'TubePickPlace.hscene'=@{Model='piper_tube_task.xml'; Output='TubeEpisodes'}
    'TubeRelayLoop.hscene'=@{Model='piper_tube_relay.xml'; Output='TubeRelayLoop'}
    'TubeRelayLoopWithOneArm.hscene'=@{Model='piper_tube_task_native_v2.xml'; Output='TubeRelayLoopWithOneArm'}
}
$utf8=New-Object Text.UTF8Encoding($false)
foreach($entry in $targets.GetEnumerator()){
    $scene=Join-Path $projectRoot ('Assets/Scenes/'+$entry.Key)
    $model=(Resolve-Path -LiteralPath (Join-Path $projectRoot ('Assets/ContentVault/Robots/PiperPod/'+$entry.Value.Model))).Path.Replace('\','/')
    $sceneExport=if($ExportDirectory){$ExportDirectory}else{Join-Path $defaultExportRoot $entry.Value.Output}
    $content=[IO.File]::ReadAllText($scene)
    foreach($field in @(@('PositionActuatorModelSource',$model),@('ExportDirectory',$sceneExport.Replace('\','/')))){
        $pattern='(?m)(Name: '+[regex]::Escape($field[0])+'\r?\n[ \t]+Type: String\r?\n[ \t]+Value: )[^\r\n]*'
        $matches=[regex]::Matches($content,$pattern)
        if($matches.Count -ne 1){throw "Expected one $($field[0]) field in $scene"}
        $quoted=ConvertTo-Json -InputObject ([string]$field[1]) -Compress
        $content=[regex]::Replace($content,$pattern,[Text.RegularExpressions.MatchEvaluator]{param($match) $match.Groups[1].Value+$quoted})
    }
    [IO.File]::WriteAllText($scene,$content,$utf8)
    Write-Output ('Configured '+$entry.Key+' to export to '+$sceneExport)
}
Write-Output 'Open office pipers.hproj in the matching Lucky Engine development build and rebuild scripts.'

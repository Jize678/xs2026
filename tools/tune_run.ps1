param(
    [double]$Kp = 0.600,
    [double]$Ki = 0.560,
    [double]$Kd = 0.630,
    [int]$RunMs = 4000,
    [int]$Target = 500,
    [string]$Port = 'COM12'
)
$ErrorActionPreference = 'Stop'

$sp = New-Object System.IO.Ports.SerialPort $Port,115200,'None',8,'One'
$sp.ReadTimeout = 200
$sp.NewLine = "`r`n"
$sp.Open()
Start-Sleep -Milliseconds 500
$sp.DiscardInBuffer()

function Send([string]$s) { $sp.Write($s + "`r`n") }
function Drain([int]$ms) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object System.Text.StringBuilder
    while ($sw.ElapsedMilliseconds -lt $ms) {
        if ($sp.BytesToRead -gt 0) { [void]$buf.Append($sp.ReadExisting()) }
        Start-Sleep -Milliseconds 5
    }
    return $buf.ToString()
}

Send 'tune';  [void](Drain 250)
Send ('vl' + $Target); [void](Drain 200)
Send ('kpl' + $Kp.ToString('0.000')); [void](Drain 120)
Send ('kil' + $Ki.ToString('0.000')); [void](Drain 120)
Send ('kdl' + $Kd.ToString('0.000')); [void](Drain 200)

$sw = [Diagnostics.Stopwatch]::StartNew()
Send ('! ' + $RunMs + ' l')
$raw = New-Object System.Text.StringBuilder
$stopped = $false
while ($sw.ElapsedMilliseconds -lt ($RunMs + 3500)) {
    if ($sp.BytesToRead -gt 0) { [void]$raw.Append($sp.ReadExisting()) }
    if ($raw.ToString() -match '\[tune\] stop') { $stopped = $true; break }
    Start-Sleep -Milliseconds 10
}
[void](Drain 200)
$sp.Close()

$lines = ($raw.ToString() -split "`r?`n")
$rows = @()
foreach ($ln in $lines) {
    if ($ln -match '^\s*(\d{1,6}),(\d{1,5}),(-?\d{1,5}),(-?\d{1,5}),(\d{1,5}),(-?\d{1,5}),(-?\d{1,5})\s*$') {
        $rows += [pscustomobject]@{
            t = [int]$Matches[1]; tl = [int]$Matches[2]; fl = [int]$Matches[3]; ul = [int]$Matches[4]
            tr = [int]$Matches[5]; fr = [int]$Matches[6]; ur = [int]$Matches[7]
        }
    }
}

Write-Output ("runs stopped={0} samples={1} kp={2} ki={3} kd={4} target={5}" -f $stopped, $rows.Count, $Kp, $Ki, $Kd, $Target)
if ($rows.Count -ge 8) {
    $n = $rows.Count
    $steady = $rows[[int]($n * 0.6)..($n - 1)]
    $f = $steady | ForEach-Object { $_.fl }
    $u = $steady | ForEach-Object { $_.ul }
    $mean = ($f | Measure-Object -Average).Average
    $sd = [math]::Sqrt((($f | ForEach-Object { ($_ - $mean) * ($_ - $mean) } | Measure-Object -Sum).Sum) / $f.Count)
    $sorted = $f | Sort-Object
    $p10 = $sorted[[int]($sorted.Count * 0.1)]
    $p90 = $sorted[[int]($sorted.Count * 0.9)]
    $rise = ($rows | Where-Object { $_.fl -ge ($Target * 0.9) } | Select-Object -First 1)
    $peak = ($rows | ForEach-Object { $_.fl } | Measure-Object -Maximum).Maximum
    $t80 = ($rows | Where-Object { $_.t -ge ($RunMs * 0.8) } | Select-Object -First 1)
    $tailMean = if ($t80) { ($rows | Where-Object { $_.t -ge ($RunMs * 0.8) } | ForEach-Object { $_.fl } | Measure-Object -Average).Average } else { 0 }
    $outMin = ($u | Measure-Object -Minimum).Minimum
    $outMax = ($u | Measure-Object -Maximum).Maximum
    Write-Output ("steady mean={0:N1} sd={1:N1} p10={2} p90={3} min={4} max={5}" -f $mean, $sd, $p10, $p90, ($f | Measure-Object -Minimum).Minimum, ($f | Measure-Object -Maximum).Maximum)
    Write-Output ("tail(>=80%%) mean={0:N1}  u_range={1}..{2}  u_mean={3:N0}" -f $tailMean, $outMin, $outMax, ($u | Measure-Object -Average).Average)
    Write-Output ("peak={0}  t_reach90={1} ms  right=" -f $peak, $(if ($rise) { $rise.t } else { -1 }))
    Write-Output (($rows | Select-Object -First 6 | ForEach-Object { "{0}:{1}/{2}/{3}" -f $_.t, $_.fl, $_.ul, $_.tl }) -join '  ')
    Write-Output (($rows | Select-Object -Last 4 | ForEach-Object { "{0}:{1}/{2}/{3}" -f $_.t, $_.fl, $_.ul, $_.tl }) -join '  ')
    $nonZeroR = ($rows | Where-Object { $_.ur -ne 0 -or $_.fr -ne 0 } | Measure-Object).Count
    Write-Output ("right_activity_samples={0} (should be 0)" -f $nonZeroR)
} else {
    Write-Output '--- raw tail ---'
    Write-Output (($lines | Where-Object { $_.Trim() -ne '' } | Select-Object -Last 12) -join "`n")
}

#requires -Version 5.1
<#
  XS2026 速度环自动调参 runner（配合 速度环调参说明.md §9 使用）

  用法（都在工程根目录下执行）：
    .\tools\tune_auto.ps1 check                       # 自检：? / tune / kq，读回当前增益与模式
    .\tools\tune_auto.ps1 gains -Kp 1.23 -Ki 0.56 -Kd 0.00 -Side l
    .\tools\tune_auto.ps1 group -Tag base -Target 500 -RunMs 1500 -Reps 3 -Side l
    .\tools\tune_auto.ps1 raw -Cmd "?"                # 发一条任意指令，收 1.5 s 回显
    .\tools\tune_auto.ps1 halt                        # hlt + 0，收工
    .\tools\tune_auto.ps1 selftest                    # 离线自检：用合成信号验证解析与指标算法（不碰串口）

  说明：
    - 每组的原始 CSV 存到 tune_data\，汇总追加到 tune_data\summary.csv；
    - 指标口径严格照 §9.3/§10：去头 300 ms、去尾 100 ms、≤20 点作废、100 ms 块平均判定；
    - 本脚本只发 tune 指令，不改固件、不烧录。
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('help', 'check', 'gains', 'group', 'raw', 'halt', 'selftest')]
    [string]$Mode = 'help',

    [string]$Port = 'COM12',
    [ValidateSet('l', 'r', 'all')][string]$Side = 'l',

    [int]$Target = 500,
    [int]$RunMs = 1500,
    [int]$Reps = 3,

    [double]$Kp = -1,
    [double]$Ki = -1,
    [double]$Kd = -1,

    [string]$Tag = 'run',
    [string]$Cmd = '',
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'
$Inv = [Globalization.CultureInfo]::InvariantCulture

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path (Split-Path -Parent $PSScriptRoot) 'tune_data'
}

function Write-Info([string]$Text) { Write-Host $Text -ForegroundColor Cyan }

#---------------------------------------------------------------- 串口基础
$script:SP = $null
$script:Buf = New-Object System.Text.StringBuilder

function Open-Port([string]$Name) {
    $sp = New-Object System.IO.Ports.SerialPort $Name, 115200, `
        ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
    $sp.ReadTimeout = 200
    $sp.WriteTimeout = 1000
    $sp.DtrEnable = $false
    $sp.RtsEnable = $false
    $sp.Open()
    Start-Sleep -Milliseconds 200
    $sp.DiscardInBuffer()
    [void]$script:Buf.Clear()
    Write-Info ("[port] {0} open @115200 8N1" -f $Name)
    return $sp
}

function Close-Port($sp) {
    if ($null -ne $sp -and $sp.IsOpen) { $sp.Close(); $sp.Dispose(); Write-Info '[port] closed' }
}

function Pump-Lines($sp, [switch]$Quiet) {
    $out = @()
    if ($sp.BytesToRead -gt 0) {
        [void]$script:Buf.Append($sp.ReadExisting())
        $s = $script:Buf.ToString()
        $idx = $s.LastIndexOf("`n")
        if ($idx -ge 0) {
            $complete = $s.Substring(0, $idx)
            [void]$script:Buf.Remove(0, $idx + 1)
            foreach ($l in ($complete -split "`n")) {
                $line = $l.TrimEnd("`r")
                if ($line.Length -gt 0) { $out += $line }
            }
        }
    }
    if (-not $Quiet) { foreach ($l in $out) { Write-Host ("  << " + $l) -ForegroundColor DarkGray } }
    return $out
}

function Collect($sp, [int]$Ms, [string]$Until = '', [switch]$Quiet) {
    $lines = @()
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($true) {
        $new = Pump-Lines $sp -Quiet:$Quiet
        if ($new.Count -gt 0) {
            $lines += $new
            if ($Until -ne '') {
                foreach ($l in $new) { if ($l -match $Until) { return $lines } }
            }
        }
        if ($sw.ElapsedMilliseconds -ge $Ms) { break }
        Start-Sleep -Milliseconds 5
    }
    return $lines
}

function Send-Line($sp, [string]$Text, [switch]$Quiet) {
    if (-not $Quiet) { Write-Host ("  >> " + $Text) -ForegroundColor Yellow }
    $sp.Write($Text + "`r`n")
    Start-Sleep -Milliseconds 60
}

function Send-Cmd($sp, [string]$Text, [int]$WaitMs = 600) {
    Send-Line $sp $Text
    return (Collect $sp $WaitMs -Quiet)
}

#---------------------------------------------------------------- 增益与参数
function Format-Gain([double]$v) { return $v.ToString('0.000', $Inv) }

function Set-Gains($sp, [string]$Side1, [double]$kp, [double]$ki, [double]$kd) {
    if ($kp -ge 0) { [void](Send-Cmd $sp ('kp' + $Side1 + (Format-Gain $kp)) 250) }
    if ($ki -ge 0) { [void](Send-Cmd $sp ('ki' + $Side1 + (Format-Gain $ki)) 250) }
    if ($kd -ge 0) { [void](Send-Cmd $sp ('kd' + $Side1 + (Format-Gain $kd)) 250) }
}

function Parse-TuneParams([string[]]$Lines) {
    $p = [ordered]@{ L_kp = $null; L_ki = $null; L_kd = $null; R_kp = $null; R_ki = $null; R_kd = $null; tgtL = $null; tgtR = $null; run = $null; limit = $null; raw = $Lines }
    foreach ($l in $Lines) {
        if ($l -match '^\[tune\] ([LR]) kp=(-?\d+)\.(\d{3}) ki=(-?\d+)\.(\d{3}) kd=(-?\d+)\.(\d{3})') {
            $w = $Matches[1]
            $p["${w}_kp"] = [double]::Parse($Matches[2] + '.' + $Matches[3], $Inv)
            $p["${w}_ki"] = [double]::Parse($Matches[4] + '.' + $Matches[5], $Inv)
            $p["${w}_kd"] = [double]::Parse($Matches[6] + '.' + $Matches[7], $Inv)
        }
        elseif ($l -match '^\[tune\] tgt L=(-?\d+) R=(-?\d+) mm/s run=(\d+) limit=(-?\d+)') {
            $p.tgtL = [int]$Matches[1]; $p.tgtR = [int]$Matches[2]
            $p.run = [int]$Matches[3]; $p.limit = [int]$Matches[4]
        }
    }
    return [pscustomobject]$p
}

function Get-TuneParams($sp) {
    $lines = Send-Cmd $sp 'kq' 700
    return Parse-TuneParams $lines
}

#---------------------------------------------------------------- CSV 与指标
$CsvRegex = '^\s*(\d{1,7}),(\d{1,5}),(-?\d{1,5}),(-?\d{1,5}),(\d{1,5}),(-?\d{1,5}),(-?\d{1,5})\s*$'

function ConvertTo-Rows([string[]]$Lines, [string]$Side1) {
    $rows = New-Object System.Collections.Generic.List[object]
    foreach ($l in $Lines) {
        if ($l -match $CsvRegex) {
            # 列序 ms,tL,fL,uL,tR,fR,uR → $Matches[1] 是 ms，所以左轮 t/f/u = 2/3/4
            $ti = 2; $fi = 3; $ui = 4
            if ($Side1 -eq 'r') { $ti = 5; $fi = 6; $ui = 7 }
            $rows.Add([pscustomobject]@{
                    ms = [int]$Matches[1]
                    t  = [int]$Matches[$ti]
                    f  = [int]$Matches[$fi]
                    u  = [int]$Matches[$ui]
                })
        }
    }
    return , $rows.ToArray()
}

function Get-Metrics([object[]]$Rows, [int]$Target, [string]$Side1) {
    $res = [ordered]@{
        valid = $false; reason = ''; n = 0; dur_ms = 0; gaps = 0; gap_txt = ''
        t90 = $null; t50 = $null; peak_raw = $null; peak_block = $null
        overshoot_pct = $null; rise_dev_pct = $null; steady_mean = $null
        steady_err_pct = $null; steady_dev_pct = $null; steady_sd = $null
        p10 = $null; p90 = $null; out_band = $null; u_mean = $null; u_max = $null
    }
    if ($Rows.Count -lt 5) { $res.reason = 'too few rows'; return [pscustomobject]$res }

    $rows = $Rows | Sort-Object ms
    $res.n = $rows.Count
    $first = $rows[0].ms
    $last = $rows[$rows.Count - 1].ms
    $res.dur_ms = $last - $first

    # 断层：dt <= 0 或 > 60 ms（§9.3：连续丢帧 > 3 帧的片段无效）
    $gapTxt = @()
    for ($i = 1; $i -lt $rows.Count; $i++) {
        $dt = $rows[$i].ms - $rows[$i - 1].ms
        if ($dt -le 0 -or $dt -gt 60) { $gapTxt += ("i={0} dt={1}" -f $i, $dt) }
    }
    $res.gaps = $gapTxt.Count
    $res.gap_txt = ($gapTxt | Select-Object -First 4) -join '; '

    # 有效窗口：去头 300 ms、去尾 100 ms
    $s0 = $first + 300
    $s1 = $last - 100
    $win = @($rows | Where-Object { $_.ms -ge $s0 -and $_.ms -le $s1 })
    if ($win.Count -lt 20) { $res.reason = ('window rows={0} (<20)' -f $win.Count); return [pscustomobject]$res }

    # ---- 100 ms 块平均（全段，从第一帧起算）：t90 / 上升偏差 / 峰值都用这一份
    # 注意用普通哈希表：OrderedDictionary 的 int 键会被当成下标
    $ALL = @{}
    foreach ($r in $rows) {
        $b = [int][math]::Floor(($r.ms - $first) / 100)
        if (-not $ALL.ContainsKey($b)) { $ALL[$b] = New-Object System.Collections.Generic.List[double] }
        $ALL[$b].Add([double]$r.f)
    }
    $aStart = @(); $aMean = @()
    foreach ($k in @($ALL.Keys | Sort-Object)) {
        $arr = $ALL[$k]
        $sum = 0.0; foreach ($v in $arr) { $sum += $v }
        $aStart += ($first + 100 * $k)
        $aMean += ($sum / $arr.Count)
    }

    # ---- 有效窗口（去头 300 ms、去尾 100 ms）的分布统计
    $res.p10 = ([double[]]$win.f | Sort-Object)[[int][math]::Floor(0.10 * ($win.Count - 1))]
    $res.p90 = ([double[]]$win.f | Sort-Object)[[int][math]::Floor(0.90 * ($win.Count - 1))]
    $res.out_band = @($win | Where-Object { [math]::Abs($_.f - $Target) -gt 50 }).Count
    $res.u_mean = [math]::Round((($win | Measure-Object -Property u -Average).Average), 0)
    $res.u_max = ($win | Measure-Object -Property u -Maximum).Maximum

    # ---- 上升：第一块 >= 90% 目标的时刻（相对第一帧）
    $t90idx = -1
    for ($i = 0; $i -lt $aMean.Count; $i++) {
        if ($aMean[$i] -ge 0.9 * $Target) { $t90idx = $i; break }
    }
    $t90Abs = $null
    if ($t90idx -ge 0) {
        $t90Abs = $aStart[$t90idx] + 100
        $res.t90 = $t90Abs - $first
        # ② 口径：到达 90% 之后允许的最大偏离（起跳段本身就在低处，不能算进来）
        $dev = 0.0
        for ($i = $t90idx; $i -lt $aMean.Count; $i++) {
            $d = [math]::Abs($aMean[$i] - $Target) / $Target * 100.0
            if ($d -gt $dev) { $dev = $d }
        }
        $res.rise_dev_pct = [math]::Round($dev, 1)
    }

    # ---- 峰值与超调（全段块均）
    $res.peak_raw = ($rows | Measure-Object -Property f -Maximum).Maximum
    $res.peak_block = ($aMean | Measure-Object -Maximum).Maximum
    if ($null -ne $res.peak_block) {
        $res.overshoot_pct = [math]::Round((($res.peak_block / $Target) - 1.0) * 100.0, 1)
    }

    # ---- 稳态：从 max(去头点, t90) 起的块；不足 2 块则取后 40%
    $stFrom = $s0
    if ($null -ne $t90Abs -and $t90Abs -gt $stFrom) { $stFrom = $t90Abs }
    $st = @()
    for ($i = 0; $i -lt $aMean.Count; $i++) {
        if ($aStart[$i] -ge $stFrom) { $st += $aMean[$i] }
    }
    if ($st.Count -lt 2) {
        $st = @()
        $startIdx = [int][math]::Floor($aMean.Count * 0.6)
        for ($i = $startIdx; $i -lt $aMean.Count; $i++) { $st += $aMean[$i] }
    }
    if ($st.Count -gt 0) {
        $sm = ($st | Measure-Object -Average).Average
        $res.steady_mean = [math]::Round($sm, 1)
        $res.steady_err_pct = [math]::Round((($sm - $Target) / $Target) * 100.0, 2)
        $dev = 0.0
        foreach ($v in $st) {
            $d = [math]::Abs($v - $Target) / $Target * 100.0
            if ($d -gt $dev) { $dev = $d }
        }
        $res.steady_dev_pct = [math]::Round($dev, 2)
        if ($st.Count -gt 1) {
            $sd = 0.0
            foreach ($v in $st) { $sd += ($v - $sm) * ($v - $sm) }
            $res.steady_sd = [math]::Round([math]::Sqrt($sd / ($st.Count - 1)), 1)
        }
        else { $res.steady_sd = 0 }
    }

    $res.valid = $true
    return [pscustomobject]$res
}

function Get-VerDict($m, [int]$Target) {
    # ① t90 ∈ [100,300] ms
    $rise = ($null -ne $m.t90) -and ($m.t90 -ge 100) -and ($m.t90 -le 300)
    # ② 到达 90% 之后的最大偏离 <= 10%
    $band = ($null -ne $m.rise_dev_pct) -and ($m.rise_dev_pct -le 10.0)
    # ③ 稳态块均偏差 <= ±2%（用均值判，最坏块 dev 只作参考）
    $steady = ($null -ne $m.steady_err_pct) -and ([math]::Abs($m.steady_err_pct) -le 2.0)
    return [pscustomobject]@{ A_rise = $rise; B_band = $band; C_steady = $steady; all = ($rise -and $band -and $steady) }
}

function Fmt($v) { if ($null -eq $v) { return 'n/a' } return $v }

function Show-Metrics($m, $v) {
    Write-Host ''
    Write-Host ('  n={0}  dur={1}ms  gaps={2} {3}' -f $m.n, $m.dur_ms, $m.gaps, $m.gap_txt) -ForegroundColor Gray
    Write-Host ('  t90={0} ms   peak_raw={1}   peak_block={2}   overshoot={3}%' -f (Fmt $m.t90), $m.peak_raw, $m.peak_block, (Fmt $m.overshoot_pct))
    Write-Host ('  steady mean={0}  err={1}%  worstdev={2}%  sd={3}   p10={4} p90={5}  outband={6}' -f `
            (Fmt $m.steady_mean), (Fmt $m.steady_err_pct), (Fmt $m.steady_dev_pct), (Fmt $m.steady_sd), $m.p10, $m.p90, $m.out_band)
    Write-Host ('  u_mean={0}  u_max={1}' -f $m.u_mean, $m.u_max)
    $c = 'Red'; if ($v.all) { $c = 'Green' }
    Write-Host ('  [1]t90in100-300={0}  [2]post-t90dev<=10%={1}  [3]steadyerr<=2%={2}' -f $v.A_rise, $v.B_band, $v.C_steady) -ForegroundColor $c
}

function Get-Median($vals) {
    $a = @($vals | Where-Object { $null -ne $_ } | Sort-Object)
    if ($a.Count -eq 0) { return $null }
    $mid = [int][math]::Floor($a.Count / 2)
    if ($a.Count % 2 -eq 1) { return $a[$mid] }
    return (($a[$mid - 1] + $a[$mid]) / 2.0)
}

#---------------------------------------------------------------- 主流程
function Do-Check($sp) {
    Write-Info '== step 1: status (?) =='
    [void](Send-Cmd $sp '?' 600)
    Write-Info '== step 2: enter tune =='
    [void](Send-Cmd $sp 'tune' 700)
    Write-Info '== step 3: read back gains (kq) =='
    $p = Get-TuneParams $sp
    Write-Host ('  L: kp={0} ki={1} kd={2}' -f $p.L_kp, $p.L_ki, $p.L_kd) -ForegroundColor Green
    Write-Host ('  R: kp={0} ki={1} kd={2}' -f $p.R_kp, $p.R_ki, $p.R_kd) -ForegroundColor Green
    Write-Host ('  tgt L={0} R={1} run={2} limit={3}' -f $p.tgtL, $p.tgtR, $p.run, $p.limit) -ForegroundColor Green
    return $p
}

function Do-Group($sp) {
    if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }
    Write-Info ('== group tag={0} side={1} target={2} run={3}ms reps={4} ==' -f $Tag, $Side, $Target, $RunMs, $Reps)

    if ($Kp -ge 0 -or $Ki -ge 0 -or $Kd -ge 0) {
        Write-Info '-- set gains'
        Set-Gains $sp $Side $Kp $Ki $Kd
    }
    $p = Get-TuneParams $sp
    $side2 = $Side.ToUpper()
    Write-Host ('  now {0}: kp={1} ki={2} kd={3}' -f $side2, $p."${side2}_kp", $p."${side2}_ki", $p."${side2}_kd") -ForegroundColor Green

    $all = @()
    for ($rep = 1; $rep -le $Reps; $rep++) {
        Write-Info ("-- rep {0}/{1}" -f $rep, $Reps)
        # 每次停车目标清零，必须重发目标
        if ($Side -eq 'r') { [void](Send-Cmd $sp ('vr' + $Target) 250) } else { [void](Send-Cmd $sp ('vl' + $Target) 250) }

        $pre = Send-Cmd $sp ('! ' + $RunMs + ' ' + $Side) 300
        $busy = @($pre | Where-Object { $_ -match 'busy' })
        if ($busy.Count -gt 0) {
            Write-Warning '  busy: still running, sending hlt and waiting 1s'
            [void](Send-Cmd $sp 'hlt' 300)
            Start-Sleep -Milliseconds 800
            if ($Side -eq 'r') { [void](Send-Cmd $sp ('vr' + $Target) 250) } else { [void](Send-Cmd $sp ('vl' + $Target) 250) }
            $pre = Send-Cmd $sp ('! ' + $RunMs + ' ' + $Side) 300
        }

        $lines = Collect $sp ($RunMs + 3000) -Until '\[tune\] stop' -Quiet
        # 把发指令后 300 ms 里已经收到的那几行也并进来，避免丢掉启动段
        $lines = @($pre) + @($lines)
        $csvLines = @($lines | Where-Object { $_ -match $CsvRegex })
        $rows = ConvertTo-Rows $csvLines $Side

        $file = Join-Path $OutDir ('{0:yyyyMMdd_HHmmss}_{1}_{2}_r{3}.csv' -f (Get-Date), $Tag, $Side, $rep)
        Set-Content -Path $file -Value $csvLines -Encoding ASCII
        Write-Host ("  saved {0} rows -> {1}" -f $rows.Count, $file) -ForegroundColor DarkGray

        $m = Get-Metrics $rows $Target $Side
        if (-not $m.valid) { Write-Warning ("  rep {0} invalid: {1}" -f $rep, $m.reason) }
        $v = Get-VerDict $m $Target
        Show-Metrics $m $v
        $all += [pscustomobject]@{ rep = $rep; m = $m; v = $v }

        # 组间 >= 2.6 s
        if ($rep -lt $Reps) { Write-Host '  [gap] waiting 2.6s' -ForegroundColor DarkGray; Start-Sleep -Milliseconds 2600 }
    }

    $valid = @($all | Where-Object { $_.m.valid })
    if ($valid.Count -eq 0) { Write-Warning 'all reps invalid'; return $null }

    $med = [pscustomobject]@{
        rep_count = $valid.Count
        t90       = Get-Median ($valid | ForEach-Object { $_.m.t90 })
        rise_dev  = Get-Median ($valid | ForEach-Object { $_.m.rise_dev_pct })
        steady    = Get-Median ($valid | ForEach-Object { $_.m.steady_mean })
        err       = Get-Median ($valid | ForEach-Object { $_.m.steady_err_pct })
        dev       = Get-Median ($valid | ForEach-Object { $_.m.steady_dev_pct })
        sd        = Get-Median ($valid | ForEach-Object { $_.m.steady_sd })
        p10       = Get-Median ($valid | ForEach-Object { $_.m.p10 })
        p90       = Get-Median ($valid | ForEach-Object { $_.m.p90 })
        outband   = Get-Median ($valid | ForEach-Object { $_.m.out_band })
        u_mean    = Get-Median ($valid | ForEach-Object { $_.m.u_mean })
        peak      = Get-Median ($valid | ForEach-Object { $_.m.peak_block })
        gaps      = ((($valid | ForEach-Object { $_.m.gaps })) | Measure-Object -Sum).Sum
        pass_rise = (@($valid | Where-Object { $_.v.A_rise }).Count) -ge [math]::Ceiling($valid.Count / 2)
        pass_band = (@($valid | Where-Object { $_.v.B_band }).Count) -ge [math]::Ceiling($valid.Count / 2)
        pass_steady = (@($valid | Where-Object { $_.v.C_steady }).Count) -ge [math]::Ceiling($valid.Count / 2)
    }

    Write-Host ''
    Write-Info '== group median (n reps valid) =='
    Write-Host ('  t90={0}ms  rise_dev={1}%  steady={2}  err={3}%  dev={4}%  sd={5}' -f `
            $med.t90, $med.rise_dev, $med.steady, $med.err, $med.dev, $med.sd) -ForegroundColor Green
    Write-Host ('  p10={0} p90={1} outband={2}  u_mean={3}  peak_block={4}' -f `
            $med.p10, $med.p90, $med.outband, $med.u_mean, $med.peak) -ForegroundColor Green
    $cc = 'Red'; if ($med.pass_rise -and $med.pass_band -and $med.pass_steady) { $cc = 'Green' }
    Write-Host ('  PASS: [1]={0} [2]={1} [3]={2}' -f $med.pass_rise, $med.pass_band, $med.pass_steady) -ForegroundColor $cc

    $sum = Join-Path $OutDir 'summary.csv'
    $line = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16}' -f `
        (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $Side, $Target, `
        $p."${side2}_kp", $p."${side2}_ki", $p."${side2}_kd", $med.rep_count, `
        $med.t90, $med.peak, $med.rise_dev, $med.steady, $med.err, $med.dev, $med.sd, $med.outband, $med.u_mean
    if (-not (Test-Path $sum)) {
        Set-Content -Path $sum -Value 'time,tag,side,target,kp,ki,kd,reps,t90,peak_block,rise_dev,steady,err_pct,dev_pct,sd,outband,u_mean' -Encoding ASCII
    }
    Add-Content -Path $sum -Value $line -Encoding ASCII
    Write-Host ("  appended -> {0}" -f $sum) -ForegroundColor DarkGray
    return [pscustomobject]@{ med = $med; params = $p; reps = $all }
}

function Invoke-SelfTest {
    Write-Info '== selftest: synthetic step response =='
    $rows = New-Object System.Collections.Generic.List[object]
    $rnd = New-Object Random 42
    for ($t = 0; $t -le 1500; $t += 20) {
        $f = [int][math]::Round(500.0 * (1.0 - [math]::Exp(-$t / 80.0)))
        $f += [int][math]::Round(($rnd.NextDouble() - 0.5) * 30)
        $u = 650
        $rows.Add([pscustomobject]@{ ms = 100000 + $t; t = 500; f = $f; u = $u })
    }
    $m = Get-Metrics $rows.ToArray() 500 'l'
    $v = Get-VerDict $m 500
    Show-Metrics $m $v

    Write-Info '== selftest: flat 500 with noise (should pass all) =='
    $rows2 = New-Object System.Collections.Generic.List[object]
    for ($t = 0; $t -le 1500; $t += 20) {
        $rows2.Add([pscustomobject]@{ ms = 200000 + $t; t = 500; f = 500 + [int][math]::Round(($rnd.NextDouble() - 0.5) * 20); u = 650 })
    }
    $m2 = Get-Metrics $rows2.ToArray() 500 'l'
    Show-Metrics $m2 (Get-VerDict $m2 500)

    Write-Info '== selftest: bad gain (only 300 mm/s steady, should fail [3]) =='
    $rows3 = New-Object System.Collections.Generic.List[object]
    for ($t = 0; $t -le 1500; $t += 20) {
        $f = [int][math]::Round(300.0 * (1.0 - [math]::Exp(-$t / 100.0)))
        $rows3.Add([pscustomobject]@{ ms = 300000 + $t; t = 500; f = $f; u = 900 })
    }
    $m3 = Get-Metrics $rows3.ToArray() 500 'l'
    Show-Metrics $m3 (Get-VerDict $m3 500)

    Write-Info '== selftest: kq parsing =='
    $fake = @(
        '[tune] L kp=1.230 ki=0.560 kd=0.000',
        '[tune] R kp=0.864 ki=0.270 kd=0.045',
        '[tune] tgt L=500 R=0 mm/s run=0 limit=4000'
    )
    $q = Parse-TuneParams $fake
    Write-Host ('  L: {0}/{1}/{2}   R: {3}/{4}/{5}   tgt={6}/{7} run={8} limit={9}' -f `
            $q.L_kp, $q.L_ki, $q.L_kd, $q.R_kp, $q.R_ki, $q.R_kd, $q.tgtL, $q.tgtR, $q.run, $q.limit) -ForegroundColor Green
    if ($q.L_kp -ne 1.23 -or $q.R_kd -ne 0.045 -or $q.limit -ne 4000) { Write-Warning '  kq parse MISMATCH'; exit 1 }
    Write-Host '  kq parse OK' -ForegroundColor Green

    Write-Info '== selftest: csv row parsing (both sides) =='
    $csv = @('37100,500,486,4310,0,0,0', '37120,500,492,4300,0,0,0', ' 37140,500,498,4320,0,0,0 ')
    $rL = ConvertTo-Rows $csv 'l'
    $rR = ConvertTo-Rows $csv 'r'
    Write-Host ('  l-side rows={0} first(t={1},f={2},u={3})   r-side first(t={4},f={5},u={6})' -f `
            $rL.Count, $rL[0].t, $rL[0].f, $rL[0].u, $rR[0].t, $rR[0].f, $rR[0].u) -ForegroundColor Green
    if ($rL.Count -ne 3 -or $rL[0].f -ne 486 -or $rR[0].t -ne 0) { Write-Warning '  csv parse MISMATCH'; exit 1 }
    Write-Host '  csv parse OK' -ForegroundColor Green
}

function Show-Help {
    Write-Host 'XS2026 tune runner' -ForegroundColor Cyan
    Write-Host '  .\tools\tune_auto.ps1 check'
    Write-Host '  .\tools\tune_auto.ps1 gains -Kp 1.23 -Ki 0.56 -Kd 0.00 -Side l'
    Write-Host '  .\tools\tune_auto.ps1 group -Tag base -Target 500 -RunMs 1500 -Reps 3 -Side l [-Kp .. -Ki .. -Kd ..]'
    Write-Host '  .\tools\tune_auto.ps1 raw -Cmd "?"'
    Write-Host '  .\tools\tune_auto.ps1 halt'
    Write-Host '  .\tools\tune_auto.ps1 selftest'
}

switch ($Mode) {
    'help' { Show-Help; exit 0 }
    'selftest' { Invoke-SelfTest; exit 0 }
    'check' {
        $sp = Open-Port $Port
        try { [void](Do-Check $sp) } finally { Close-Port $sp }
        exit 0
    }
    'gains' {
        $sp = Open-Port $Port
        try {
            Set-Gains $sp $Side $Kp $Ki $Kd
            [void](Get-TuneParams $sp)
        }
        finally { Close-Port $sp }
        exit 0
    }
    'group' {
        $sp = Open-Port $Port
        try { [void](Do-Group $sp) } finally { Close-Port $sp }
        exit 0
    }
    'raw' {
        if ($Cmd -eq '') { Write-Warning 'need -Cmd "..."'; exit 2 }
        $sp = Open-Port $Port
        try { [void](Send-Cmd $sp $Cmd 1500) } finally { Close-Port $sp }
        exit 0
    }
    'halt' {
        $sp = Open-Port $Port
        try {
            [void](Send-Cmd $sp 'hlt' 300)
            [void](Send-Cmd $sp '0' 400)
            [void](Send-Cmd $sp '?' 500)
        }
        finally { Close-Port $sp }
        exit 0
    }
}

#requires -Version 5.1
<#
  XS2026 速度环自动调参 runner（配合 速度环调参说明.md §9 使用）

  用法（都在工程根目录下执行）：
    .\tools\tune_auto.ps1 check                       # 自检：? / tune / kq，读回当前增益与模式
    .\tools\tune_auto.ps1 gains -Kp 1.23 -Ki 0.56 -Kd 0.00 -Side l
    .\tools\tune_auto.ps1 group -Tag base -Target 500 -RunMs 1500 -Reps 3 -Side l
    .\tools\tune_auto.ps1 group -Tag both -Target 1000 -RunMs 3000 -Reps 3 -Side all -AllowBoth -Fan 50
                                                      # 两侧同跑＝直行，必须 -AllowBoth，且 Target×RunMs ≤ 3000 mm
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
    [ValidateSet('help', 'check', 'gains', 'group', 'step', 'profile', 'raw', 'halt', 'selftest', 'metrics')]
    [string]$Mode = 'help',

    [string]$Port = 'COM12',
    [ValidateSet('l', 'r', 'all')][string]$Side = 'l',

    [int]$Target = 500,
    [int]$RunMs = 1500,
    [int]$Reps = 3,

    [double]$Kp = -1,
    [double]$Ki = -1,
    [double]$Kd = -1,

    # 负压：0 = 不吹；1~100 = 每组实验前发 tf<Fan>，固件先解锁 1.5s 再闭环
    [ValidateRange(0, 100)][int]$Fan = 0,

    # step 模式：先稳住 -From 这个速度，-HoldMs 之后把目标阶跃到 -Target，量整定时间
    [int]$From = 500,
    [int]$HoldMs = 1500,

    # profile 模式：一趟内依次跑多个速度段，CSV 的 t 列记录每次阶跃
    #   格式 "500:1500,1000:1500,1500:1500,1000:1500,500:1500"（速度:停留ms）
    [string]$Steps = '500:1500,1000:1500,1500:1500,1000:1500,500:1500',

    # 两侧同时闭环（-Side all）是直行工况，必须显式解锁，并且目标×时间 ≤ MaxTravelMm
    [switch]$AllowBoth,
    [int]$MaxTravelMm = 3000,

    [string]$Tag = 'run',
    [string]$Cmd = '',
    [string]$OutDir = '',

    # 每次实验的指标追加到这个 txt（留空 = 不写文件）；建议传工程的 双电机直行测试记录.txt
    [string]$LogPath = '',

    # metrics 模式：单个 csv 或一个目录，按当前三条指标离线重算（不碰串口）
    [string]$Csv = ''
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

#---- 安全门：调参期间只允许单侧闭环（见 速度环调参说明.md §9.0）----
# 单轮驱动时车走固定半径圆环、行程有界；两轮驱动是直行，会冲出去撞车。
# 这里用白名单兜住：不在清单里的指令一律拒发，防止手滑把 `1`/`2` 发出去。
$Script:SafeCmdPattern = '^(\?|kq|k[pdi][lr]?\d+(\.\d+)?|0|hlt|tune|tf\d+|vl\d+|vr\d+|bmq|g|z|!.*)$'

function Assert-SafeCmd([string]$Text) {
    $t = $Text.Trim()
    if ($t -match $Script:SafeCmdPattern) { return }
    throw ("拒绝发送 '{0}'：调参期间只允许单侧闭环（vl + '! <ms> l'），两轮指令会让车直行冲出去。见 速度环调参说明.md §9.0" -f $t)
}

function Send-Line($sp, [string]$Text, [switch]$Quiet) {
    Assert-SafeCmd $Text
    if (-not $Quiet) { Write-Host ("  >> " + $Text) -ForegroundColor Yellow }
    $sp.Write($Text + "`r`n")
    Start-Sleep -Milliseconds 60
}

function Send-Cmd($sp, [string]$Text, [int]$WaitMs = 600) {
    Send-Line $sp $Text
    $lines = Collect $sp $WaitMs -Quiet
    # 回显除 CSV 数据行（group 时几百行，太吵）
    foreach ($l in $lines) {
        if ($l -notmatch $CsvRegex) { Write-Host ("  << " + $l) -ForegroundColor DarkGray }
    }
    return $lines
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

# 两侧同时跑时，一行 CSV 里 L/R 各有一套 t/f/u，先整行保留，再按侧别投影
function ConvertTo-FullRows([string[]]$Lines) {
    $rows = New-Object System.Collections.Generic.List[object]
    foreach ($l in $Lines) {
        if ($l -match $CsvRegex) {
            $rows.Add([pscustomobject]@{
                    ms = [int]$Matches[1]
                    tL = [int]$Matches[2]; fL = [int]$Matches[3]; uL = [int]$Matches[4]
                    tR = [int]$Matches[5]; fR = [int]$Matches[6]; uR = [int]$Matches[7]
                })
        }
    }
    return , $rows.ToArray()
}

function Project-Side([object[]]$Rows, [string]$Side1) {
    $out = foreach ($r in $Rows) {
        if ($Side1 -eq 'r') { [pscustomobject]@{ ms = $r.ms; t = $r.tR; f = $r.fR; u = $r.uR } }
        else { [pscustomobject]@{ ms = $r.ms; t = $r.tL; f = $r.fL; u = $r.uL } }
    }
    return , @($out)
}

# 丢掉“陈旧前缀”：蓝牙断开重连时串口缓冲里可能残留上一组的一行，
# 时间戳差好几分钟，会把 dur 拉到几百秒、把 100ms 块均退化成原始采样
# （表现为 worstdev 98%、sd 150 这种假异常）。只在表头连续断层时丢行。
function Drop-StalePrefix([object[]]$Rows) {
    # 先按 ms 排序：陈旧行可能是迟到的最末一行（文件顺序在尾、排序后在头）
    $r = @($Rows | Sort-Object ms)
    while ($r.Count -ge 6) {
        $dt = $r[1].ms - $r[0].ms
        if ($dt -lt 0 -or $dt -gt 60) { $r = @($r[1..($r.Count - 1)]) } else { break }
    }
    return , $r
}

# 丢掉“停车尾巴”：`! <ms>` 到点后固件立刻停机，最后 1~2 帧速度掉到 0，
# 混进 100ms 块均会把 post-t90 偏离虚增到 40% 以上（双侧同时挂的假异常）。
# 只剥「从最后一帧往前、跨度不超过 250ms」的尾帧，避免把真堵转（>0.4s）也剥掉。
function Trim-StopTail([object[]]$Rows, [int]$Target, [string]$Side1) {
    $r = @($Rows)
    if ($r.Count -lt 3) { return , $r }
    $thr = [double]$Target * 0.5
    $tEnd = $r[$r.Count - 1].ms
    while ($r.Count -gt 2) {
        $last = $r[$r.Count - 1]
        if (($tEnd - $last.ms) -gt 250) { break }
        $mine = if ($Side1 -eq 'r') { [double]$last.fR } else { [double]$last.fL }
        if ($mine -ge $thr) { break }
        $r = @($r[0..($r.Count - 2)])
    }
    return , $r
}

function Get-Metrics([object[]]$Rows, [int]$Target, [string]$Side1) {
    $res = [ordered]@{
        valid = $false; reason = ''; n = 0; dur_ms = 0; gaps = 0; gap_txt = ''
        t90 = $null; t50 = $null; tset = $null; peak_raw = $null; peak_block = $null
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
    $aStart = @(); $aMean = @(); $aCnt = @()
    foreach ($k in @($ALL.Keys | Sort-Object)) {
        $arr = $ALL[$k]
        $sum = 0.0; foreach ($v in $arr) { $sum += $v }
        $aStart += ($first + 100 * $k)
        $aMean += ($sum / $arr.Count)
        $aCnt += $arr.Count
    }
    # 只保留样本数 >= 3 的块：最后一块常常只剩 1~2 个点，噪声会把它的均值顶到带外，
    # 让“已进入稳态”的判定永远成立不了（对齐块版本的经典坑）。
    $kStart = @(); $kMean = @()
    for ($i = 0; $i -lt $aMean.Count; $i++) {
        if ($aCnt[$i] -ge 3) { $kStart += $aStart[$i]; $kMean += $aMean[$i] }
    }
    if ($kMean.Count -ge 2) { $aStart = $kStart; $aMean = $kMean }

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

    # ---- ① 调节时间 tset（相对第一帧）
    #     口径来自用户 2026-10-31 规范：“在 100-300ms 内达到目标速度（进入稳态无阶跃）”
    #     判法：从某块起，其后所有块里至少 95% 落在 ±2% 带内（100 ms 块均，样本 >=3）。
    #       - 用块均：块内 5 个采样平均，1 count = 9.18 mm/s 的量化噪声（500 时 1.8%）不会
    #         把“已进入稳态”误判成“还没进带”；
    #       - 允许 5% 的块越界：sd 12 mm/s 的正常稳态也偶尔有块擦出 ±20，要求 100% 会让
    #         判定随噪声随机跳变（实测同一组好数据 tset 在 400~3500 ms 之间乱跳）。
    $tol2 = $Target * 0.02
    $tsetAbs = $aStart[$aStart.Count - 1] + 100
    for ($i = 0; $i -lt $aMean.Count; $i++) {
        $tot = 0; $inb = 0
        for ($j = $i; $j -lt $aMean.Count; $j++) {
            $tot++
            if ([math]::Abs($aMean[$j] - $Target) -le $tol2) { $inb++ }
        }
        if ($tot -gt 0 -and (($inb * 100.0 / $tot) -ge 95.0)) { $tsetAbs = $aStart[$i] + 100; break }
    }
    $res.tset = $tsetAbs - $first

    # ---- 峰值与超调（全段）
    $res.peak_raw = ($rows | Measure-Object -Property f -Maximum).Maximum
    $res.overshoot_raw_pct = [math]::Round((([double]$res.peak_raw / $Target) - 1.0) * 100.0, 1)
    $res.peak_block = ($aMean | Measure-Object -Maximum).Maximum
    if ($null -ne $res.peak_block) {
        $res.overshoot_pct = [math]::Round((($res.peak_block / $Target) - 1.0) * 100.0, 1)
    }

    # ---- 稳态：从 max(去头点, 调节时间 tset) 起的块；不足 2 块则取后 40%
    $stFrom = $s0
    if ($null -ne $tsetAbs -and $tsetAbs -gt $stFrom) { $stFrom = $tsetAbs }
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
    # ① 快速响应（调节时间）：进入 ±2% 稳态带并保持到结束的时刻 ≤ 300 ms（块均判，100 ms 分辨率）
    $rise = ($null -ne $m.tset) -and ($m.tset -le 300)
    # ② 低超调（波动幅度）：上升过程中 100 ms 块均的峰值不超过目标的 110%
    #    2026-10-31 用户定：与 ①③ 统一用 100 ms 块均；原始单帧只作参考（单帧仅 20~26 ms，
    #    一个毛刺就能判挂，和「震荡幅度」的本意不符）。
    $ov = $m.overshoot_pct
    if ($null -eq $ov) { $ov = $m.sm_overshoot_pct }
    $band = ($null -ne $ov) -and ($ov -le 10.0)
    # ③ 稳态精度：调节时间之后的块均均值误差 ≤ ±2%
    $steady = ($null -ne $m.steady_err_pct) -and ([math]::Abs($m.steady_err_pct) -le 2.0)
    return [pscustomobject]@{ A_rise = $rise; B_band = $band; C_steady = $steady; all = ($rise -and $band -and $steady) }
}

function Fmt($v) { if ($null -eq $v) { return 'n/a' } return $v }

function Show-Metrics($m, $v) {
    Write-Host ''
    Write-Host ('  n={0}  dur={1}ms  gaps={2} {3}' -f $m.n, $m.dur_ms, $m.gaps, $m.gap_txt) -ForegroundColor Gray
    Write-Host ('  tset={0} ms (t90={1})   peak_raw={2} ({3}%)   peak_block={4} ({5}%)' -f (Fmt $m.tset), (Fmt $m.t90), $m.peak_raw, (Fmt $m.overshoot_raw_pct), $m.peak_block, (Fmt $m.overshoot_pct))
    Write-Host ('  steady mean={0}  err={1}%  worstdev={2}%  sd={3}   p10={4} p90={5}  outband={6}' -f `
            (Fmt $m.steady_mean), (Fmt $m.steady_err_pct), (Fmt $m.steady_dev_pct), (Fmt $m.steady_sd), $m.p10, $m.p90, $m.out_band)
    Write-Host ('  u_mean={0}  u_max={1}   post-tset dev={2}%' -f $m.u_mean, $m.u_max, (Fmt $m.rise_dev_pct))
    $c = 'Red'; if ($v.all) { $c = 'Green' }
    Write-Host ('  [1]tset<=300ms={0}  [2]peak_block<=10%={1}  [3]|steady err|<=2%={2}' -f $v.A_rise, $v.B_band, $v.C_steady) -ForegroundColor $c
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

function Assert-GroupArgs {
    if ($Side -eq 'all' -and -not $AllowBoth) {
        throw "两侧同时闭环（-Side all）＝直行，必须显式加 -AllowBoth，且已获得用户逐次确认才发车。"
    }
    $travel = [int][math]::Round($Target * $RunMs / 1000.0)
    if ($Side -eq 'all' -and $travel -gt $MaxTravelMm) {
        throw ("目标 {0} mm/s × {1} ms = {2} mm，超过 {3} mm 上限，先降 Target 或 RunMs。" -f $Target, $RunMs, $travel, $MaxTravelMm)
    }
}

function Send-Targets($sp, [string[]]$Sides, [int]$Tgt) {
    foreach ($sd in $Sides) {
        if ($sd -eq 'r') { [void](Send-Cmd $sp ('vr' + $Tgt) 250) }
        else { [void](Send-Cmd $sp ('vl' + $Tgt) 250) }
    }
}

function Do-Group($sp) {
    # 安全规则（速度环调参说明.md §9.0）：
    #   默认一次只许一侧闭环 —— 车绕另一侧轮子转圈，行程有界；
    #   两侧同时闭环＝直行，必须显式 -AllowBoth（用户逐次确认），且目标×时间 ≤ MaxTravelMm。
    Assert-GroupArgs
    $travel = [int][math]::Round($Target * $RunMs / 1000.0)
    $sides = @($Side)
    if ($Side -eq 'all') { $sides = @('l', 'r') }
    if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }
    Write-Info ('== group tag={0} side={1} target={2} run={3}ms reps={4} fan={5} travel={6}mm ==' -f $Tag, $Side, $Target, $RunMs, $Reps, $Fan, $travel)

    # 必须先进 tune：上电后默认 IDLE，此时 `!` 会被固件静默忽略（只回显、不闭环、不出 CSV）
    [void](Send-Cmd $sp 'tune' 400)

    # 负压设定（粘性）：每组都显式下发，避免沿用上一组的意外值
    $fanReply = Send-Cmd $sp ('tf' + $Fan) 400
    $fanReply | Where-Object { $_ -match '\[tune\] fan' } | ForEach-Object { Write-Host ("  << {0}" -f $_) -ForegroundColor DarkGray }

    if ($Kp -ge 0 -or $Ki -ge 0 -or $Kd -ge 0) {
        Write-Info '-- set gains'
        foreach ($sd in $sides) { Set-Gains $sp $sd $Kp $Ki $Kd }
    }
    $p = Get-TuneParams $sp
    foreach ($sd in $sides) {
        $k = $sd.ToUpper()
        Write-Host ('  now {0}: kp={1} ki={2} kd={3}' -f $k, $p."${k}_kp", $p."${k}_ki", $p."${k}_kd") -ForegroundColor Green
    }

    $recs = @()
    for ($rep = 1; $rep -le $Reps; $rep++) {
        Write-Info ("-- rep {0}/{1}" -f $rep, $Reps)
        # 每次停车目标清零，必须重发目标（两侧同时跑时两边都要发）
        Send-Targets $sp $sides $Target

        $pre = Send-Cmd $sp ('! ' + $RunMs + ' ' + $Side) 300
        $busy = @($pre | Where-Object { $_ -match 'busy' })
        if ($busy.Count -gt 0) {
            Write-Warning '  busy: still running, sending hlt and waiting 1s'
            [void](Send-Cmd $sp 'hlt' 300)
            Start-Sleep -Milliseconds 800
            Send-Targets $sp $sides $Target
            $pre = Send-Cmd $sp ('! ' + $RunMs + ' ' + $Side) 300
        }

        $waitMs = $RunMs + 3000
        if ($Fan -gt 0) { $waitMs += 2000 }   # 固件先解锁 1.5s 才开始闭环
        $lines = Collect $sp $waitMs -Until '\[tune\] stop' -Quiet
        # 把发指令后 300 ms 里已经收到的那几行也并进来，避免丢掉启动段
        $lines = @($pre) + @($lines)
        $csvLines = @($lines | Where-Object { $_ -match $CsvRegex })
        $rowsFull = ConvertTo-FullRows $csvLines

        $file = Join-Path $OutDir ('{0:yyyyMMdd_HHmmss}_{1}_{2}_r{3}.csv' -f (Get-Date), $Tag, $Side, $rep)
        Set-Content -Path $file -Value $csvLines -Encoding ASCII
        Write-Host ("  saved {0} rows -> {1}" -f $rowsFull.Count, $file) -ForegroundColor DarkGray

        $rowsClean = Drop-StalePrefix $rowsFull
        if ($rowsClean.Count -ne $rowsFull.Count) {
            Write-Warning ("  dropped {0} stale leading row(s) (bluetooth residue)" -f ($rowsFull.Count - $rowsClean.Count))
        }
        $bySide = @{}
        foreach ($sd in $sides) {
            $m = Get-Metrics (Project-Side (Trim-StopTail $rowsClean $Target $sd) $sd) $Target $sd
            if (-not $m.valid) { Write-Warning ("  rep {0} {1} invalid: {2}" -f $rep, $sd.ToUpper(), $m.reason) }
            $v = Get-VerDict $m $Target
            if ($sides.Count -gt 1) { Write-Host ("  ---- {0} side ----" -f $sd.ToUpper()) -ForegroundColor Cyan }
            Show-Metrics $m $v
            $bySide[$sd] = [pscustomobject]@{ m = $m; v = $v }
        }
        $recs += [pscustomobject]@{ rep = $rep; bySide = $bySide }

        # 逐趟落盘（时间戳齐全，便于事后对账）
        if ($LogPath -ne '') {
            $log = New-Object System.Collections.Generic.List[string]
            $log.Add('------------------------------------------------------------')
            $log.Add(('[{0}] tag={1} rep={2} target={3} run={4}ms fan={5} travel={6}mm' -f `
                        (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $rep, $Target, $RunMs, $Fan, $travel))
            foreach ($sd in $sides) {
                $k = $sd.ToUpper()
                $log.Add(('  gains {0}: kp={1} ki={2} kd={3}' -f $k, $p."${k}_kp", $p."${k}_ki", $p."${k}_kd"))
            }
            $log.Add(('  CSV: {0}' -f $file))
            foreach ($sd in $sides) {
                $e = $bySide[$sd]; $k = $sd.ToUpper()
                $log.Add(('  {0}: t90={1}ms  post-t90 dev={2}%  steady={3}  err={4}%  sd={5}  u_mean={6}  => [1]{7} [2]{8} [3]{9}' -f `
                            $k, (Fmt $e.m.t90), (Fmt $e.m.rise_dev_pct), (Fmt $e.m.steady_mean), (Fmt $e.m.steady_err_pct), `
                            (Fmt $e.m.steady_sd), $e.m.u_mean, $e.v.A_rise, $e.v.B_band, $e.v.C_steady))
            }
            Add-Content -Path $LogPath -Value $log -Encoding UTF8
        }

        # 组间 >= 2.6 s
        if ($rep -lt $Reps) { Write-Host '  [gap] waiting 2.6s' -ForegroundColor DarkGray; Start-Sleep -Milliseconds 2600 }
    }

    $results = @{}
    foreach ($sd in $sides) {
        $valid = @($recs | ForEach-Object { $_.bySide[$sd] } | Where-Object { $_.m.valid })
        if ($valid.Count -eq 0) { Write-Warning ("  {0}: all reps invalid" -f $sd.ToUpper()); continue }

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
        Write-Info ('== group median {0} side (n reps valid) ==' -f $sd.ToUpper())
        Write-Host ('  t90={0}ms  rise_dev={1}%  steady={2}  err={3}%  dev={4}%  sd={5}' -f `
                $med.t90, $med.rise_dev, $med.steady, $med.err, $med.dev, $med.sd) -ForegroundColor Green
        Write-Host ('  p10={0} p90={1} outband={2}  u_mean={3}  peak_block={4}' -f `
                $med.p10, $med.p90, $med.outband, $med.u_mean, $med.peak) -ForegroundColor Green
        $cc = 'Red'; if ($med.pass_rise -and $med.pass_band -and $med.pass_steady) { $cc = 'Green' }
        Write-Host ('  PASS: [1]={0} [2]={1} [3]={2}' -f $med.pass_rise, $med.pass_band, $med.pass_steady) -ForegroundColor $cc

        $sum = Join-Path $OutDir 'summary.csv'
        $k = $sd.ToUpper()
        $line = '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16}' -f `
            (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $sd, $Target, `
            $p."${k}_kp", $p."${k}_ki", $p."${k}_kd", $med.rep_count, `
            $med.t90, $med.peak, $med.rise_dev, $med.steady, $med.err, $med.dev, $med.sd, $med.outband, $med.u_mean
        if (-not (Test-Path $sum)) {
            Set-Content -Path $sum -Value 'time,tag,side,target,kp,ki,kd,reps,t90,peak_block,rise_dev,steady,err_pct,dev_pct,sd,outband,u_mean' -Encoding ASCII
        }
        Add-Content -Path $sum -Value $line -Encoding ASCII
        Write-Host ("  appended -> {0}" -f $sum) -ForegroundColor DarkGray
        $results[$sd] = $med
    }
    return [pscustomobject]@{ results = $results; params = $p; reps = $recs }
}

function Get-StepMetrics([object[]]$Rows, [int]$From, [int]$To) {
    # 速度阶跃的整定指标（用户 2026-10-31 口径）：
    #   ① 整定时间：阶跃后 100~300 ms 内进入目标 ±2%；
    #   ② 超调：上升段原始采样峰值不超过目标的 110%；
    #   ③ 稳态精度：整定之后块均与目标偏差 ≤ ±2%。
    $res = [ordered]@{
        valid = $false; reason = ''; n = 0; step_ms = $null
        pre_mean = $null; pre_sd = $null
        settle = $null; overshoot_pct = $null; peak_raw = $null
        peak_block = $null; peak_block_pct = $null
        steady_mean = $null; steady_err_pct = $null; steady_sd = $null
        u_mean = $null; u_max = $null; blocks = 0
    }
    if ($Rows.Count -lt 20) { $res.reason = 'too few rows'; return [pscustomobject]$res }
    $rows = @($Rows | Sort-Object ms)
    $res.n = $rows.Count

    # 阶跃帧 = 第一次出现 t == To 且上一帧的 t != To
    $idx = -1
    for ($i = 0; $i -lt $rows.Count; $i++) {
        if ($rows[$i].t -eq $To -and ($i -eq 0 -or $rows[$i - 1].t -ne $To)) { $idx = $i; break }
    }
    if ($idx -lt 0) { $res.reason = ('step {0}->{1} not found in t column' -f $From, $To); return [pscustomobject]$res }
    $stepMs = $rows[$idx].ms
    $res.step_ms = $stepMs

    # 阶跃前 200 ms 的实际速度：确认确实稳在 From 上（否则这次阶跃没有意义）
    $preEnd = [math]::Max(0, $idx - 1)
    $pre = @($rows[0..$preEnd] | Where-Object { $_.ms -ge ($stepMs - 200) })
    if ($pre.Count -ge 3) {
        $pv = @($pre | ForEach-Object { [double]$_.f })
        $pm = ($pv | Measure-Object -Average).Average
        $var = 0.0; foreach ($v in $pv) { $var += ($v - $pm) * ($v - $pm) }
        $res.pre_mean = [math]::Round($pm, 1)
        $res.pre_sd = [math]::Round([math]::Sqrt($var / $pv.Count), 1)
    }

    $post = @($rows[$idx..($rows.Count - 1)])
    $res.u_mean = [math]::Round((($post | Measure-Object -Property u -Average).Average), 0)
    $res.u_max = ($post | Measure-Object -Property u -Maximum).Maximum
    # ② 偏离：升速看最大超调，降速看最深下冲（跳过阶跃后头 50 ms —— 那几帧还是旧速度）
    $anaRows = @($post | Where-Object { $_.ms -ge ($stepMs + 50) })
    if ($anaRows.Count -lt 5) { $anaRows = $post }
    if ($To -ge $From) {
        $res.peak_raw = ($anaRows | Measure-Object -Property f -Maximum).Maximum
        $res.overshoot_pct = [math]::Round((($res.peak_raw / $To) - 1.0) * 100.0, 1)
    }
    else {
        $res.peak_raw = ($anaRows | Measure-Object -Property f -Minimum).Minimum
        $res.overshoot_pct = [math]::Round((1.0 - ($res.peak_raw / $To)) * 100.0, 1)
    }

    # 100 ms 对齐块（自阶跃帧起算），每块样本 >= 3 才计入
    $blk = @{}
    foreach ($rw in $post) {
        $bk = [int][math]::Floor(($rw.ms - $stepMs) / 100)
        if (-not $blk.ContainsKey($bk)) { $blk[$bk] = New-Object System.Collections.Generic.List[double] }
        $blk[$bk].Add([double]$rw.f)
    }
    $bStart = @(); $bMean = @()
    foreach ($k in @($blk.Keys | Sort-Object)) {
        $arr = $blk[$k]
        if ($arr.Count -ge 3) {
            $bStart += ($stepMs + 100 * $k)
            $bMean += (($arr | Measure-Object -Average).Average)
        }
    }
    $res.blocks = $bMean.Count
    if ($bMean.Count -lt 3) { $res.reason = ('post step blocks={0} (<3)' -f $bMean.Count); return [pscustomobject]$res }

    # ① 整定：从某块起，其后所有块里 >= 95% 落在 ±2% 带内
    $tol = 0.02 * $To
    $setIdx = -1
    for ($i = 0; $i -lt $bMean.Count; $i++) {
        $ok = 0
        for ($j = $i; $j -lt $bMean.Count; $j++) {
            if ([math]::Abs($bMean[$j] - $To) -le $tol) { $ok++ }
        }
        if ($ok -ge [math]::Ceiling(0.95 * ($bMean.Count - $i))) { $setIdx = $i; break }
    }
    if ($setIdx -lt 0) { $res.reason = 'never settles into +-2% band'; return [pscustomobject]$res }
    $res.settle = $bStart[$setIdx] + 100 - $stepMs
    if ($res.settle -lt 100) { $res.settle = 100 }   # 低于一个测速块宽度没意义

    # 块均的极值也从第二块算起（第一块是阶跃过渡段）
    $mx = $bMean[[math]::Min(1, $bMean.Count - 1)]
    for ($i = 1; $i -lt $bMean.Count; $i++) {
        if ($To -ge $From) { if ($bMean[$i] -gt $mx) { $mx = $bMean[$i] } }
        else { if ($bMean[$i] -lt $mx) { $mx = $bMean[$i] } }
    }
    $res.peak_block = [math]::Round($mx, 1)
    if ($To -ge $From) { $res.peak_block_pct = [math]::Round((($mx / $To) - 1.0) * 100.0, 1) }
    else { $res.peak_block_pct = [math]::Round((1.0 - ($mx / $To)) * 100.0, 1) }

    $st = @()
    for ($i = $setIdx; $i -lt $bMean.Count; $i++) { $st += $bMean[$i] }
    $sm = ($st | Measure-Object -Average).Average
    $sv = 0.0; foreach ($v in $st) { $sv += ($v - $sm) * ($v - $sm) }
    $res.steady_mean = [math]::Round($sm, 1)
    $res.steady_err_pct = [math]::Round((($sm - $To) / $To) * 100.0, 2)
    $res.steady_sd = [math]::Round([math]::Sqrt($sv / $st.Count), 1)
    $res.valid = $true
    return [pscustomobject]$res
}

function Get-StepVerDict($m) {
    $v = [ordered]@{ A_settle = $false; B_over = $false; C_steady = $false }
    if ($m.valid) {
        $v.A_settle = ($m.settle -ge 100 -and $m.settle -le 300)
        $v.B_over = ($m.overshoot_pct -le 10.0)
        $v.C_steady = ((($m.steady_err_pct -as [double]) | ForEach-Object { [math]::Abs($_) }) -le 2.0)
    }
    return [pscustomobject]$v
}

function Show-StepMetrics($m, $v) {
    if (-not $m.valid) { Write-Warning ('  INVALID: {0}' -f $m.reason); return }
    Write-Host ('  pre={0} (sd {1})  settle={2}ms  peak_raw={3} (+{4}%)  peak_block=+{5}%' -f `
            (Fmt $m.pre_mean), (Fmt $m.pre_sd), (Fmt $m.settle), (Fmt $m.peak_raw), (Fmt $m.overshoot_pct), (Fmt $m.peak_block_pct)) -ForegroundColor Gray
    Write-Host ('  steady={0}  err={1}%  sd={2}  u_mean={3}  blocks={4}' -f `
            (Fmt $m.steady_mean), (Fmt $m.steady_err_pct), (Fmt $m.steady_sd), (Fmt $m.u_mean), $m.blocks) -ForegroundColor Gray
    $cc = 'Red'; if ($v.A_settle -and $v.B_over -and $v.C_steady) { $cc = 'Green' }
    Write-Host ('  PASS: [1]settle={0} [2]overshoot={1} [3]steady={2}' -f $v.A_settle, $v.B_over, $v.C_steady) -ForegroundColor $cc
}

function Do-Step($sp) {
    # 速度阶跃：先稳住 From，HoldMs 之后把目标改到 Target —— 只量“速度变化时的整定”，不管 0 起跑。
    if ($Side -eq 'all' -and -not $AllowBoth) { throw 'step -Side all（两侧同时）需要 -AllowBoth' }
    if ($From -le 0 -or $Target -le 0) { throw 'step 需要 -From 和 -Target 都 > 0' }
    if ($HoldMs -ge $RunMs) { throw ('-HoldMs({0}) 必须小于 -RunMs({1})' -f $HoldMs, $RunMs) }
    $sides = @($Side); if ($Side -eq 'all') { $sides = @('l', 'r') }
    $postMs = $RunMs - $HoldMs
    $travel = [int][math]::Round(($From * $HoldMs + $Target * $postMs) / 1000.0)
    if ($travel -gt $MaxTravelMm) { throw ('step 行程 {0} mm 超过上限 {1} mm' -f $travel, $MaxTravelMm) }
    if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }
    Write-Info ('== step tag={0} side={1} {2}->{3} mm/s run={4}ms hold={5}ms reps={6} fan={7} travel={8}mm ==' -f `
            $Tag, $Side, $From, $Target, $RunMs, $HoldMs, $Reps, $Fan, $travel)

    [void](Send-Cmd $sp 'tune' 400)
    $fanReply = Send-Cmd $sp ('tf' + $Fan) 400
    $fanReply | Where-Object { $_ -match '\[tune\] fan' } | ForEach-Object { Write-Host ("  << {0}" -f $_) -ForegroundColor DarkGray }
    if ($Kp -ge 0 -or $Ki -ge 0 -or $Kd -ge 0) {
        Write-Info '-- set gains'
        foreach ($sd in $sides) { Set-Gains $sp $sd $Kp $Ki $Kd }
    }
    $p = Get-TuneParams $sp
    foreach ($sd in $sides) {
        $k = $sd.ToUpper()
        Write-Host ('  now {0}: kp={1} ki={2} kd={3}' -f $k, $p."${k}_kp", $p."${k}_ki", $p."${k}_kd") -ForegroundColor Green
    }

    $armMs = 0; if ($Fan -gt 0) { $armMs = 1500 }
    $recs = @()
    for ($rep = 1; $rep -le $Reps; $rep++) {
        Write-Info ("-- rep {0}/{1}" -f $rep, $Reps)
        Send-Targets $sp $sides $From

        $pre = Send-Cmd $sp ('! ' + $RunMs + ' ' + $Side) 300
        $busy = @($pre | Where-Object { $_ -match 'busy' })
        if ($busy.Count -gt 0) {
            Write-Warning '  busy: still running, sending hlt and waiting 1s'
            [void](Send-Cmd $sp 'hlt' 300)
            Start-Sleep -Milliseconds 800
            Send-Targets $sp $sides $From
            $pre = Send-Cmd $sp ('! ' + $RunMs + ' ' + $Side) 300
        }

        # 先让它稳在 From：锁臂（若带负压）+ HoldMs
        $lines = @($pre)
        $lines += Collect $sp ($armMs + $HoldMs + 200) -Quiet

        # 阶跃：目标直接改到 Target（固件里已经不是软起动，一拍到位）
        Send-Targets $sp $sides $Target

        $lines += Collect $sp ($postMs + 3000) -Until '\[tune\] stop' -Quiet
        $csvLines = @($lines | Where-Object { $_ -match $CsvRegex })
        $rowsFull = ConvertTo-FullRows $csvLines

        $file = Join-Path $OutDir ('{0:yyyyMMdd_HHmmss}_{1}_step{2}_{3}_{4}_r{5}.csv' -f (Get-Date), $Tag, $From, $Target, $Side, $rep)
        Set-Content -Path $file -Value $csvLines -Encoding ASCII
        Write-Host ("  saved {0} rows -> {1}" -f $rowsFull.Count, $file) -ForegroundColor DarkGray

        $rowsClean = Drop-StalePrefix $rowsFull
        $bySide = @{}
        foreach ($sd in $sides) {
            $proj = Project-Side (Trim-StopTail $rowsClean $Target $sd) $sd
            $m = Get-StepMetrics $proj $From $Target
            if (-not $m.valid) { Write-Warning ("  rep {0} {1} invalid: {2}" -f $rep, $sd.ToUpper(), $m.reason) }
            $v = Get-StepVerDict $m
            if ($sides.Count -gt 1) { Write-Host ("  ---- {0} side ----" -f $sd.ToUpper()) -ForegroundColor Cyan }
            Write-Host ('  {0}: {1}->{2}' -f $sd.ToUpper(), $From, $Target) -ForegroundColor Gray
            Show-StepMetrics $m $v
            $bySide[$sd] = [pscustomobject]@{ m = $m; v = $v }
        }
        $recs += [pscustomobject]@{ rep = $rep; bySide = $bySide }

        if ($LogPath -ne '') {
            $log = New-Object System.Collections.Generic.List[string]
            $log.Add('------------------------------------------------------------')
            $log.Add(('[{0}] step tag={1} rep={2} {3}->{4} mm/s run={5}ms hold={6}ms fan={7} travel={8}mm' -f `
                        (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $rep, $From, $Target, $RunMs, $HoldMs, $Fan, $travel))
            foreach ($sd in $sides) {
                $k = $sd.ToUpper()
                $log.Add(('  gains {0}: kp={1} ki={2} kd={3}' -f $k, $p."${k}_kp", $p."${k}_ki", $p."${k}_kd"))
            }
            $log.Add(('  CSV: {0}' -f $file))
            foreach ($sd in $sides) {
                $e = $bySide[$sd]; $k = $sd.ToUpper()
                $log.Add(('  {0}: pre={1} (sd {2})  settle={3}ms  peak_raw={4} (+{5}%)  peak_block=+{6}%  steady={7}  err={8}%  sd={9}  u_mean={10}  => [1]{11} [2]{12} [3]{13}' -f `
                            $k, (Fmt $e.m.pre_mean), (Fmt $e.m.pre_sd), (Fmt $e.m.settle), (Fmt $e.m.peak_raw), (Fmt $e.m.overshoot_pct), `
                            (Fmt $e.m.peak_block_pct), (Fmt $e.m.steady_mean), (Fmt $e.m.steady_err_pct), (Fmt $e.m.steady_sd), `
                            $e.m.u_mean, $e.v.A_settle, $e.v.B_over, $e.v.C_steady))
            }
            Add-Content -Path $LogPath -Value $log -Encoding UTF8
        }

        if ($rep -lt $Reps) { Write-Host '  [gap] waiting 2.6s' -ForegroundColor DarkGray; Start-Sleep -Milliseconds 2600 }
    }

    foreach ($sd in $sides) {
        $valid = @($recs | ForEach-Object { $_.bySide[$sd] } | Where-Object { $_.m.valid })
        if ($valid.Count -eq 0) { Write-Warning ("  {0}: all reps invalid" -f $sd.ToUpper()); continue }
        $settle = Get-Median ($valid | ForEach-Object { $_.m.settle })
        $over = Get-Median ($valid | ForEach-Object { $_.m.overshoot_pct })
        $err = Get-Median ($valid | ForEach-Object { $_.m.steady_err_pct })
        $sdv = Get-Median ($valid | ForEach-Object { $_.m.steady_sd })
        $um = Get-Median ($valid | ForEach-Object { $_.m.u_mean })
        $ps = (@($valid | Where-Object { $_.v.A_settle }).Count) -ge [math]::Ceiling($valid.Count / 2)
        $po = (@($valid | Where-Object { $_.v.B_over }).Count) -ge [math]::Ceiling($valid.Count / 2)
        $pc = (@($valid | Where-Object { $_.v.C_steady }).Count) -ge [math]::Ceiling($valid.Count / 2)
        Write-Host ''
        Write-Info ('== step median {0} side {1}->{2} (n={3}) ==' -f $sd.ToUpper(), $From, $Target, $valid.Count)
        Write-Host ('  settle={0}ms  overshoot=+{1}%  err={2}%  sd={3}  u_mean={4}' -f $settle, $over, $err, $sdv, $um) -ForegroundColor Green
        $cc = 'Red'; if ($ps -and $po -and $pc) { $cc = 'Green' }
        Write-Host ('  PASS: [1]={0} [2]={1} [3]={2}' -f $ps, $po, $pc) -ForegroundColor $cc

        $sum = Join-Path $OutDir 'step_summary.csv'
        if (-not (Test-Path $sum)) {
            Set-Content -Path $sum -Value 'time,tag,side,from,to,kp,ki,kd,reps,settle,overshoot_pct,err_pct,sd,u_mean,pass1,pass2,pass3' -Encoding ASCII
        }
        $k = $sd.ToUpper()
        Add-Content -Path $sum -Value ('{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16}' -f `
                (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $sd, $From, $Target, `
                $p."${k}_kp", $p."${k}_ki", $p."${k}_kd", $valid.Count, $settle, $over, $err, $sdv, $um, $ps, $po, $pc) -Encoding ASCII
    }
}

# 把一趟 profile 的 CSV 按目标列（tL/tR）的变化切成若干段，返回每次“目标真的变了”的边界。
# 固件本来就在 CSV 里打印设定值，所以多段实验完全可以事后按 t 列切分，不需要额外握手。
function Split-ProfileTransitions([object[]]$Rows) {
    $rows = @($Rows | Sort-Object ms)
    $out = New-Object System.Collections.Generic.List[object]
    for ($i = 1; $i -lt $rows.Count; $i++) {
        $a = [int]$rows[$i - 1].t
        $b = [int]$rows[$i].t
        if ($a -ne $b) {
            $out.Add([pscustomobject]@{ idx = $i; from = $a; to = $b; ms = $rows[$i].ms })
        }
    }
    return , $out.ToArray()
}

function Parse-StepPlan([string]$Spec) {
    $plan = New-Object System.Collections.Generic.List[object]
    foreach ($item in ($Spec -split ',')) {
        $it = $item.Trim()
        if ($it -eq '') { continue }
        $p = $it -split ':'
        if ($p.Count -ne 2) { throw ("-Steps 格式应为 speed:holdMs，例如 500:1500,1000:1500，收到 '{0}'" -f $it) }
        $spd = [int]$p[0]
        $hold = [int]$p[1]
        if ($spd -lt 250 -or $spd -gt 2000) { throw ("-Steps 里的速度 {0} 超出 250~2000 mm/s" -f $spd) }
        if ($hold -lt 600 -or $hold -gt 8000) { throw ("-Steps 里的停留 {0} ms 超出 600~8000 ms（至少要够整定 + 稳态统计）" -f $hold) }
        $plan.Add([pscustomobject]@{ speed = $spd; hold = $hold })
    }
    if ($plan.Count -lt 2) { throw '-Steps 至少要两段，否则没有阶跃可测' }
    return , $plan
}

function Do-Profile($sp) {
    # 一趟多段：一次发车按时间表依次改目标，CSV 的 t 列就是每次阶跃的标记。
    # 只允许单侧（另一侧一直 0）：两侧一起改目标＝直行，必须单独走 -AllowBoth 流程。
    if ($Side -eq 'all') { throw 'profile 只支持单侧（-Side l 或 -Side r）' }
    $plan = Parse-StepPlan $Steps
    $totalHold = 0; foreach ($s in $plan) { $totalHold += $s.hold }
    $runMs = $totalHold + 1200
    $spec = ($plan | ForEach-Object { '{0}:{1}' -f $_.speed, $_.hold }) -join ','
    if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }
    Write-Info ('== profile tag={0} side={1} fan={2} steps[{3}] hold={4}ms run={5}ms ==' -f $Tag, $Side, $Fan, $spec, $totalHold, $runMs)

    [void](Send-Cmd $sp 'tune' 400)
    $fanReply = Send-Cmd $sp ('tf' + $Fan) 400
    $fanReply | Where-Object { $_ -match '\[tune\] fan' } | ForEach-Object { Write-Host ("  << {0}" -f $_) -ForegroundColor DarkGray }
    if ($Kp -ge 0 -or $Ki -ge 0 -or $Kd -ge 0) {
        Write-Info '-- set gains'
        Set-Gains $sp $Side $Kp $Ki $Kd
    }
    $p = Get-TuneParams $sp
    $k = $Side.ToUpper()
    Write-Host ('  now {0}: kp={1} ki={2} kd={3}' -f $k, $p."${k}_kp", $p."${k}_ki", $p."${k}_kd") -ForegroundColor Green

    $armMs = 0; if ($Fan -gt 0) { $armMs = 1500 }
    for ($rep = 1; $rep -le $Reps; $rep++) {
        Write-Info ("-- rep {0}/{1}" -f $rep, $Reps)
        Send-Targets $sp @($Side) $plan[0].speed
        $pre = Send-Cmd $sp ('! ' + $runMs + ' ' + $Side) 300
        if (@($pre | Where-Object { $_ -match 'busy' }).Count -gt 0) {
            Write-Warning '  busy: still running, sending hlt and waiting 1s'
            [void](Send-Cmd $sp 'hlt' 300)
            Start-Sleep -Milliseconds 900
            Send-Targets $sp @($Side) $plan[0].speed
            $pre = Send-Cmd $sp ('! ' + $runMs + ' ' + $Side) 300
        }
        $repLines = @($pre)
        if ($armMs -gt 0) {
            Write-Host ('  [arm] vacuum up, waiting {0} ms' -f $armMs) -ForegroundColor DarkGray
            $repLines += Collect $sp ($armMs + 100) -Quiet
        }

        for ($si = 0; $si -lt $plan.Count; $si++) {
            $seg = $plan[$si]
            Write-Host ('  [{0}/{1}] {2} mm/s x {3} ms' -f ($si + 1), $plan.Count, $seg.speed, $seg.hold) -ForegroundColor DarkGray
            $repLines += Collect $sp $seg.hold -Quiet
            if ($si -lt $plan.Count - 1) {
                Send-Targets $sp @($Side) $plan[$si + 1].speed
            }
        }

        [void](Send-Cmd $sp 'hlt' 400)
        $repLines += Collect $sp 1500 -Until '\[tune\] stop' -Quiet

        $csvLines = @($repLines | Where-Object { $_ -match $CsvRegex })
        $file = Join-Path $OutDir ('{0:yyyyMMdd_HHmmss}_{1}_profile_{2}_r{3}.csv' -f (Get-Date), $Tag, $Side, $rep)
        Set-Content -Path $file -Value $csvLines -Encoding ASCII
        $rowsFull = ConvertTo-FullRows $csvLines
        Write-Host ("  saved {0} rows -> {1}" -f $rowsFull.Count, $file) -ForegroundColor DarkGray

        $rowsClean = Drop-StalePrefix $rowsFull
        if ($rowsClean.Count -eq 0) { Write-Warning '  no rows, skip'; continue }
        $lastTgt = [int]$rowsClean[$rowsClean.Count - 1].tL
        if ($Side -eq 'r') { $lastTgt = [int]$rowsClean[$rowsClean.Count - 1].tR }
        $proj = Project-Side (Trim-StopTail $rowsClean $lastTgt $Side) $Side
        $trans = Split-ProfileTransitions $proj
        if ($trans.Count -eq 0) { Write-Warning '  no target change found in t column'; continue }
        Write-Host ("  transitions: {0}" -f $trans.Count) -ForegroundColor Gray

        $log = New-Object System.Collections.Generic.List[string]
        if ($LogPath -ne '') {
            $log.Add('------------------------------------------------------------')
            $log.Add(('[{0}] profile tag={1} rep={2} side={3} steps={4} fan={5}' -f `
                        (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $rep, $Side, $spec, $Fan))
            $log.Add(('  gains {0}: kp={1} ki={2} kd={3}' -f $k, $p."${k}_kp", $p."${k}_ki", $p."${k}_kd"))
            $log.Add(('  CSV: {0}' -f $file))
        }

        for ($j = 0; $j -lt $trans.Count; $j++) {
            $i0 = [math]::Max(0, $trans[$j].idx - 8)          # 阶跃前 ~200 ms 作为 pre 参照
            if ($j -lt $trans.Count - 1) { $i1 = $trans[$j + 1].idx - 1 } else { $i1 = $proj.Count - 1 }
            if ($i1 -le $trans[$j].idx + 4) { continue }
            $slice = $proj[$i0..$i1]
            $m = Get-StepMetrics $slice $trans[$j].from $trans[$j].to
            $v = Get-StepVerDict $m
            Write-Host ('  {0}->{1}:' -f $trans[$j].from, $trans[$j].to) -ForegroundColor Cyan
            Show-StepMetrics $m $v
            if ($LogPath -ne '' -and $m.valid) {
                $log.Add(('  {0}->{1}: pre={2} (sd {3})  settle={4}ms  peak_raw={5} (+{6}%)  peak_block=+{7}%  steady={8}  err={9}%  sd={10}  u_mean={11}  => [1]{12} [2]{13} [3]{14}' -f `
                            $trans[$j].from, $trans[$j].to, (Fmt $m.pre_mean), (Fmt $m.pre_sd), (Fmt $m.settle), `
                            (Fmt $m.peak_raw), (Fmt $m.overshoot_pct), (Fmt $m.peak_block_pct), (Fmt $m.steady_mean), `
                            (Fmt $m.steady_err_pct), (Fmt $m.steady_sd), (Fmt $m.u_mean), $v.A_settle, $v.B_over, $v.C_steady))
            }
            if ($m.valid) {
                $sum = Join-Path $OutDir 'step_summary.csv'
                if (-not (Test-Path $sum)) {
                    Set-Content -Path $sum -Value 'time,tag,side,from,to,kp,ki,kd,reps,settle,overshoot_pct,err_pct,sd,u_mean,pass1,pass2,pass3' -Encoding ASCII
                }
                Add-Content -Path $sum -Value ('{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16}' -f `
                        (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $Tag, $Side, $trans[$j].from, $trans[$j].to, `
                        $p."${k}_kp", $p."${k}_ki", $p."${k}_kd", 1, $m.settle, $m.overshoot_pct, $m.steady_err_pct, `
                        $m.steady_sd, $m.u_mean, $v.A_settle, $v.B_over, $v.C_steady) -Encoding ASCII
            }
        }
        if ($LogPath -ne '') { Add-Content -Path $LogPath -Value $log -Encoding UTF8 }
        if ($rep -lt $Reps) { Write-Host '  [gap] waiting 2.6s' -ForegroundColor DarkGray; Start-Sleep -Milliseconds 2600 }
    }
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

    Write-Info '== selftest: both-side projection (l/f 与 r/f 不能串列) =='
    $csv2 = @(
        '40000,1000,900,1200,1000,1100,1300',
        '40020,1000,950,1210,1000,1050,1310',
        '40040,1000,1000,1220,1000,1000,1320'
    )
    $full = ConvertTo-FullRows $csv2
    $pL = Project-Side $full 'l'
    $pR = Project-Side $full 'r'
    Write-Host ('  full={0} rows  L(first f={1},u={2})  R(first f={3},u={4})' -f `
            $full.Count, $pL[0].f, $pL[0].u, $pR[0].f, $pR[0].u) -ForegroundColor Green
    if ($full.Count -ne 3 -or $pL[0].f -ne 900 -or $pL[0].u -ne 1200 -or $pR[0].f -ne 1100 -or $pR[0].u -ne 1300) {
        Write-Warning '  both-side projection MISMATCH'; exit 1
    }
    Write-Host '  both-side projection OK' -ForegroundColor Green

    Write-Info '== selftest: speed step 500->1000 (settle should land in 100~300 ms) =='
    $srows = New-Object System.Collections.Generic.List[object]
    $rnd2 = New-Object Random 7
    $fv = 500.0
    for ($t = 0; $t -le 3000; $t += 25) {
        $tgt = 500
        if ($t -ge 1200) { $tgt = 1000 }
        # 一阶惯性 tau=80ms 跟踪目标 + 少量噪声
        $fv += ($tgt - $fv) * (1.0 - [math]::Exp(-25.0 / 40.0))
        $noise = ($rnd2.NextDouble() - 0.5) * 12.0
        $srows.Add([pscustomobject]@{ ms = 40000 + $t; t = $tgt; f = [int][math]::Round($fv + $noise); u = 900 })
    }
    $sm = Get-StepMetrics $srows 500 1000
    $sv = Get-StepVerDict $sm
    Show-StepMetrics $sm $sv
    if (-not $sm.valid) { Write-Warning ('  step metrics INVALID: {0}' -f $sm.reason) }
    elseif ($sv.A_settle -and $sv.B_over -and $sv.C_steady) { Write-Host '  step 500->1000 OK' -ForegroundColor Green }
    else { Write-Warning '  step 500->1000 metrics not as expected' }

    Write-Info '== selftest: speed step 1500->1000 (降速也要能测出来) =='
    $drows = New-Object System.Collections.Generic.List[object]
    $dv0 = 1500.0
    for ($t = 0; $t -le 3000; $t += 25) {
        $tgt = 1500
        if ($t -ge 1200) { $tgt = 1000 }
        $dv0 += ($tgt - $dv0) * (1.0 - [math]::Exp(-25.0 / 80.0))
        $drows.Add([pscustomobject]@{ ms = 40000 + $t; t = $tgt; f = [int][math]::Round($dv0); u = 700 })
    }
    $dm = Get-StepMetrics $drows 1500 1000
    $dvd = Get-StepVerDict $dm
    Show-StepMetrics $dm $dvd
    if ($dm.valid -and $dvd.A_settle) { Write-Host '  step 1500->1000 OK' -ForegroundColor Green }
    else { Write-Warning '  step 1500->1000 metrics not as expected' }

    Write-Info '== selftest: profile 三段（500/1000/1500 各 1500ms，应切出 2 个阶跃） =='
    $planT = @(500, 1000, 1500)
    $holdT = 1500
    $prows = New-Object System.Collections.Generic.List[object]
    $fv2 = 500.0
    $tgtIdx = 0
    for ($t = 0; $t -le ($holdT * $planT.Count); $t += 25) {
        $tgtIdx = [math]::Min($planT.Count - 1, [int][math]::Floor($t / $holdT))
        $tgt = $planT[$tgtIdx]
        $fv2 += ($tgt - $fv2) * (1.0 - [math]::Exp(-25.0 / 40.0))
        $prows.Add([pscustomobject]@{ ms = 60000 + $t; t = $tgt; f = [int][math]::Round($fv2); u = 900 })
    }
    $tr = Split-ProfileTransitions $prows
    Write-Host ('  transitions found: {0} -> [{1}]' -f $tr.Count, (($tr | ForEach-Object { '{0}->{1}' -f $_.from, $_.to }) -join ', ')) -ForegroundColor Green
    if ($tr.Count -ne 2) { Write-Warning '  profile split MISMATCH (expected 2 transitions)'; exit 1 }
    $okP = $true
    for ($j = 0; $j -lt $tr.Count; $j++) {
        $i0 = [math]::Max(0, $tr[$j].idx - 8)
        if ($j -lt $tr.Count - 1) { $i1 = $tr[$j + 1].idx - 1 } else { $i1 = $prows.Count - 1 }
        $slice = $prows[$i0..$i1]
        $pm = Get-StepMetrics $slice $tr[$j].from $tr[$j].to
        $pv = Get-StepVerDict $pm
        Write-Host ('  {0}->{1}:' -f $tr[$j].from, $tr[$j].to) -ForegroundColor Cyan
        Show-StepMetrics $pm $pv
        if (-not ($pm.valid -and $pv.A_settle -and $pv.B_over -and $pv.C_steady)) { $okP = $false }
    }
    if ($okP) { Write-Host '  profile split OK' -ForegroundColor Green }
    else { Write-Warning '  profile split metrics not as expected' }
}

function Do-Metrics([string]$Path) {
    $files = @()
    if (Test-Path -LiteralPath $Path -PathType Container) {
        $files = @(Get-ChildItem -LiteralPath $Path -Filter '*.csv' | Sort-Object Name)
    }
    elseif (Test-Path -LiteralPath $Path) { $files = @(Get-Item -LiteralPath $Path) }
    else { throw "找不到 $Path" }
    if ($files.Count -eq 0) { throw "$Path 下没有 csv" }

    $fmt = '{0,-40} {1,-2} {2,6} {3,6} {4,7} {5,8} {6,7} {7,6} {8,9} {9}'
    Write-Host ($fmt -f 'file', 'sd', 'tset', 't90', 'peakB+%', 'steady', 'err%', 'sd', 'u_mean', 'verdict') -ForegroundColor Cyan
    foreach ($f in $files) {
        $lines = @(Get-Content -LiteralPath $f.FullName -Encoding UTF8)
        $rows = Drop-StalePrefix (ConvertTo-FullRows $lines)
        if ($rows.Count -lt 5) { Write-Host ('{0,-40} (too few rows)' -f $f.Name) -ForegroundColor DarkGray; continue }
        foreach ($sd1 in @('l', 'r')) {
            $tcol = @($rows | ForEach-Object { if ($sd1 -eq 'r') { $_.tR } else { $_.tL } } | Where-Object { $_ -gt 0 } | Sort-Object)
            if ($tcol.Count -eq 0) { continue }
            $tgt = [int]$tcol[[int][math]::Floor($tcol.Count / 2)]
            $m = Get-Metrics (Project-Side (Trim-StopTail $rows $tgt $sd1) $sd1) $tgt $sd1
            if (-not $m.valid) { Write-Host ($fmt -f $f.Name, $sd1, $m.reason, '', '', '', '', '', '', 'INVALID') -ForegroundColor DarkGray; continue }
            $v = Get-VerDict $m $tgt
            $c = 'Red'; if ($v.all) { $c = 'Green' }
            $verdict = '{0}{1}{2}' -f $(if ($v.A_rise) { '1' } else { '-' }), $(if ($v.B_band) { '2' } else { '-' }), $(if ($v.C_steady) { '3' } else { '-' })
            Write-Host ($fmt -f $f.Name, $sd1, (Fmt $m.tset), (Fmt $m.t90), (Fmt $m.overshoot_pct), (Fmt $m.steady_mean), (Fmt $m.steady_err_pct), (Fmt $m.steady_sd), $m.u_mean, $verdict) -ForegroundColor $c
        }
    }
}

function Show-Help {
    Write-Host 'XS2026 tune runner' -ForegroundColor Cyan
    Write-Host '  .\tools\tune_auto.ps1 check'
    Write-Host '  .\tools\tune_auto.ps1 gains -Kp 1.23 -Ki 0.56 -Kd 0.00 -Side l'
    Write-Host '  .\tools\tune_auto.ps1 group -Tag base -Target 500 -RunMs 1500 -Reps 3 -Side l [-Kp .. -Ki .. -Kd ..] [-Fan 0~100]'
    Write-Host '  .\tools\tune_auto.ps1 group -Tag both -Target 1000 -RunMs 3000 -Reps 3 -Side all -AllowBoth -Fan 50'
    Write-Host '      -Side all：两侧同时闭环（直行），必须 -AllowBoth，且 Target×RunMs ≤ 3000 mm；每侧单独出指标'
    Write-Host '  .\tools\tune_auto.ps1 step -Tag s500_1000 -From 500 -To 1000 -HoldMs 1500 -RunMs 4000 -Reps 3 -Side l [-Kp .. -Fan 50]'
    Write-Host '      step：先稳在 -From，-HoldMs 后阶跃到 -To，量整定时间/超调/稳态（0 起跑不算）'
    Write-Host '  .\tools\tune_auto.ps1 profile -Tag p1 -Steps "500:1500,1000:1500,1500:1500,1000:1500,500:1500" -Side l -Fan 50'
    Write-Host '      profile：一趟发车跑完多段速度，事后按 CSV 的 t 列切出每次阶跃逐段判①settle ②超调 ③稳态（单侧专用）'
    Write-Host '  .\tools\tune_auto.ps1 raw -Cmd "?"'
    Write-Host '  .\tools\tune_auto.ps1 metrics -Csv tune_data\xxx.csv   # 离线重算三条指标（也可给目录）'
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
        # 参数级安全门放在开串口之前：不合规的请求根本不碰硬件
        Assert-GroupArgs
        $sp = Open-Port $Port
        try { [void](Do-Group $sp) } finally { Close-Port $sp }
        exit 0
    }
    'step' {
        $sp = Open-Port $Port
        try { [void](Do-Step $sp) } finally { Close-Port $sp }
        exit 0
    }
    'profile' {
        $sp = Open-Port $Port
        try { [void](Do-Profile $sp) } finally { Close-Port $sp }
        exit 0
    }
    'raw' {
        if ($Cmd -eq '') { Write-Warning 'need -Cmd "..."'; exit 2 }
        $sp = Open-Port $Port
        try { [void](Send-Cmd $sp $Cmd 1500) } finally { Close-Port $sp }
        exit 0
    }
    'metrics' {
        if ($Csv -eq '') { Write-Warning 'need -Csv <file|dir>'; exit 2 }
        Do-Metrics $Csv
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

param(
    [string]$JetsonAddress = "192.168.2.162",
    [int]$RefreshMilliseconds = 250
)

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

[System.Windows.Forms.Application]::EnableVisualStyles()

$script:angleDeg = $null
$script:limitDeg = 30.0
$script:lastReadingAt = $null
$script:requestInProgress = $false

$form = New-Object System.Windows.Forms.Form
$form.Text = "Ethon Wheel Angle"
$form.ClientSize = New-Object System.Drawing.Size(520, 280)
$form.MinimumSize = New-Object System.Drawing.Size(440, 280)
$form.StartPosition = "CenterScreen"
$form.BackColor = [System.Drawing.Color]::FromArgb(17, 22, 28)
$form.ForeColor = [System.Drawing.Color]::White
$form.Font = New-Object System.Drawing.Font("Segoe UI", 10)

$title = New-Object System.Windows.Forms.Label
$title.Text = "ROAD-WHEEL ANGLE"
$title.AutoSize = $false
$title.Location = New-Object System.Drawing.Point(20, 17)
$title.Size = New-Object System.Drawing.Size(300, 24)
$title.Font = New-Object System.Drawing.Font("Segoe UI Semibold", 11)
$title.ForeColor = [System.Drawing.Color]::FromArgb(153, 163, 175)

$status = New-Object System.Windows.Forms.Label
$status.Text = "CONNECTING"
$status.TextAlign = "MiddleCenter"
$status.Location = New-Object System.Drawing.Point(385, 15)
$status.Size = New-Object System.Drawing.Size(112, 27)
$status.Font = New-Object System.Drawing.Font("Segoe UI Semibold", 9)
$status.BackColor = [System.Drawing.Color]::FromArgb(138, 99, 20)

$angle = New-Object System.Windows.Forms.Label
$angle.Text = "--.-`u{00B0}"
$angle.TextAlign = "MiddleCenter"
$angle.Location = New-Object System.Drawing.Point(20, 48)
$angle.Size = New-Object System.Drawing.Size(477, 75)
$angle.Font = New-Object System.Drawing.Font("Segoe UI", 42, [System.Drawing.FontStyle]::Bold)

$direction = New-Object System.Windows.Forms.Label
$direction.Text = "Waiting for the Jetson"
$direction.TextAlign = "MiddleCenter"
$direction.Location = New-Object System.Drawing.Point(20, 119)
$direction.Size = New-Object System.Drawing.Size(477, 30)
$direction.Font = New-Object System.Drawing.Font("Segoe UI Semibold", 15)
$direction.ForeColor = [System.Drawing.Color]::FromArgb(88, 166, 255)

$gauge = New-Object System.Windows.Forms.Panel
$gauge.Location = New-Object System.Drawing.Point(20, 158)
$gauge.Size = New-Object System.Drawing.Size(477, 65)
$gauge.Anchor = "Left,Right,Bottom"
$gauge.BackColor = $form.BackColor

$details = New-Object System.Windows.Forms.Label
$details.Text = "Read-only feed  |  $JetsonAddress"
$details.Location = New-Object System.Drawing.Point(20, 241)
$details.Size = New-Object System.Drawing.Size(477, 22)
$details.Anchor = "Left,Right,Bottom"
$details.TextAlign = "MiddleCenter"
$details.ForeColor = [System.Drawing.Color]::FromArgb(139, 148, 158)

$gauge.Add_Paint({
    param($sender, $eventArgs)
    $graphics = $eventArgs.Graphics
    $graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $width = $sender.ClientSize.Width
    $centerX = [int]($width / 2)
    $railY = 31
    $margin = 34

    $railPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(55, 65, 75), 8)
    $centerPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(139, 148, 158), 2)
    $graphics.DrawLine($railPen, $margin, $railY, $width - $margin, $railY)
    $graphics.DrawLine($centerPen, $centerX, 15, $centerX, 47)

    $smallFont = New-Object System.Drawing.Font("Segoe UI Semibold", 9)
    $mutedBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(139, 148, 158))
    $graphics.DrawString("LEFT", $smallFont, $mutedBrush, 0, 48)
    $rightSize = $graphics.MeasureString("RIGHT", $smallFont)
    $graphics.DrawString("RIGHT", $smallFont, $mutedBrush, $width - $rightSize.Width, 48)

    if ($null -ne $script:angleDeg) {
        $safeLimit = [Math]::Max(1.0, $script:limitDeg)
        $fraction = [Math]::Max(-1.0, [Math]::Min(1.0, $script:angleDeg / $safeLimit))
        $needleX = [int]($centerX + $fraction * (($width / 2) - $margin))
        $needleColor = if ([Math]::Abs($fraction) -ge 0.9) {
            [System.Drawing.Color]::FromArgb(210, 153, 34)
        } else {
            [System.Drawing.Color]::FromArgb(63, 185, 80)
        }
        $needlePen = New-Object System.Drawing.Pen($needleColor, 5)
        $needleBrush = New-Object System.Drawing.SolidBrush($needleColor)
        $graphics.DrawLine($needlePen, $needleX, 12, $needleX, 50)
        $graphics.FillEllipse($needleBrush, $needleX - 7, $railY - 7, 14, 14)
        $needlePen.Dispose()
        $needleBrush.Dispose()
    }

    $railPen.Dispose()
    $centerPen.Dispose()
    $smallFont.Dispose()
    $mutedBrush.Dispose()
})

$form.Controls.AddRange(@($title, $status, $angle, $direction, $gauge, $details))

$client = New-Object System.Net.Http.HttpClient
$client.Timeout = [TimeSpan]::FromMilliseconds(900)
$endpoint = "http://$JetsonAddress/api/state"

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = [Math]::Max(100, $RefreshMilliseconds)
$timer.Add_Tick({
    if ($script:requestInProgress) { return }
    $script:requestInProgress = $true

    try {
        $json = $client.GetStringAsync($endpoint).GetAwaiter().GetResult()
        $state = $json | ConvertFrom-Json
        $drive = $state.topics.'/ethon/drive_status'.value
        if ($null -eq $drive -or $null -eq $drive.road_wheel_deg) {
            throw "No steering reading in the live feed"
        }

        $rawAngle = [double]$drive.road_wheel_deg
        $renderSign = if ($drive.steer_inverted -eq $true) { 1.0 } else { -1.0 }
        $script:angleDeg = $rawAngle * $renderSign
        if ($null -ne $drive.steer_limit_deg -and [double]$drive.steer_limit_deg -gt 0) {
            $script:limitDeg = [Math]::Abs([double]$drive.steer_limit_deg)
        }
        $script:lastReadingAt = Get-Date

        $angle.Text = "{0:0.0}`u{00B0}" -f [Math]::Abs($script:angleDeg)
        if ([Math]::Abs($script:angleDeg) -lt 0.05) {
            $direction.Text = "CENTERED"
        } elseif ($script:angleDeg -gt 0) {
            $direction.Text = "LEFT"
        } else {
            $direction.Text = "RIGHT"
        }
        $status.Text = "LIVE"
        $status.BackColor = [System.Drawing.Color]::FromArgb(35, 134, 54)
        $details.Text = "Read-only feed  |  Updated $((Get-Date).ToString('HH:mm:ss.fff'))"
        $gauge.Invalidate()
    } catch {
        $status.Text = "OFFLINE"
        $status.BackColor = [System.Drawing.Color]::FromArgb(187, 45, 59)
        if ($null -eq $script:lastReadingAt) {
            $angle.Text = "--.-`u{00B0}"
            $direction.Text = "Waiting for the Jetson"
        } else {
            $age = [Math]::Round(((Get-Date) - $script:lastReadingAt).TotalSeconds, 1)
            $direction.Text = "Last reading ${age}s ago"
        }
    } finally {
        $script:requestInProgress = $false
    }
})

$form.Add_Shown({ $timer.Start() })
$form.Add_FormClosed({
    $timer.Stop()
    $timer.Dispose()
    $client.Dispose()
})

[void][System.Windows.Forms.Application]::Run($form)

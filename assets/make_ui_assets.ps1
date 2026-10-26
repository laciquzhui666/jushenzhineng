# Generates the UI textures used by bedroom_guardian.py:
#   - Chinese speech bubbles (transparent PNG)
#   - digital clock faces for 11:59:40 .. 12:00:00
#   - a bright phone screen
#   - "sleep time" label
# Windows PowerShell 5+ / .NET System.Drawing only.
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$speechDir = Join-Path $root 'speech'
$clockDir = Join-Path $root 'clock'
$phoneDir = Join-Path $root 'phone'
New-Item -ItemType Directory -Force -Path $speechDir, $clockDir, $phoneDir | Out-Null

function New-RoundedPath([single]$x, [single]$y, [single]$w, [single]$h, [single]$r) {
    $p = New-Object System.Drawing.Drawing2D.GraphicsPath
    $d = [single]($r * 2)
    $p.AddArc($x, $y, $d, $d, 180, 90)
    $p.AddArc($x + $w - $d, $y, $d, $d, 270, 90)
    $p.AddArc($x + $w - $d, $y + $h - $d, $d, $d, 0, 90)
    $p.AddArc($x, $y + $h - $d, $d, $d, 90, 90)
    $p.CloseFigure()
    return $p
}

function New-SpeechBubble([string]$text, [string]$file, [single]$fontSize) {
    $W = 1024; $H = 320
    $bmp = New-Object System.Drawing.Bitmap($W, $H, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
    $g.Clear([System.Drawing.Color]::Transparent)

    # --- soft shadow -------------------------------------------------------
    $shadowPath = New-RoundedPath 26 24 972 238 52
    $shadowBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(52, 35, 18, 8))
    $g.FillPath($shadowBrush, $shadowPath)
    $shadowBrush.Dispose(); $shadowPath.Dispose()

    # --- tail with border --------------------------------------------------
    $tail = New-Object System.Drawing.Drawing2D.GraphicsPath
    $tail.AddPolygon([System.Drawing.PointF[]]@(
        (New-Object System.Drawing.PointF(170, 238)),
        (New-Object System.Drawing.PointF(286, 238)),
        (New-Object System.Drawing.PointF(178, 312))))
    $tailPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(255, 221, 199, 170), 5)
    $g.DrawPath($tailPen, $tail)

    # --- bubble body (warm cream gradient) ---------------------------------
    $bubblePath = New-RoundedPath 22 14 980 238 52
    $rect = New-Object System.Drawing.RectangleF(22, 14, 980, 238)
    $brush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
        $rect,
        [System.Drawing.Color]::FromArgb(255, 255, 249, 236),
        [System.Drawing.Color]::FromArgb(255, 255, 238, 212),
        [System.Drawing.Drawing2D.LinearGradientMode]::Vertical)
    $g.FillPath($brush, $bubblePath)
    $brush.Dispose()

    $tailBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 240, 218))
    $g.FillPath($tailBrush, $tail)
    $tailBrush.Dispose(); $tail.Dispose()

    $borderPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(255, 224, 198, 163), 5)
    $g.DrawPath($borderPen, $bubblePath)
    $borderPen.Dispose(); $bubblePath.Dispose()

    # small red accent bar
    $accent = New-RoundedPath 53 86 9 112 4
    $accentBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 205, 54, 46))
    $g.FillPath($accentBrush, $accent)
    $accentBrush.Dispose(); $accent.Dispose()

    # --- text --------------------------------------------------------------
    $font = New-Object System.Drawing.Font('Microsoft YaHei', $fontSize, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
    $format = New-Object System.Drawing.StringFormat
    $format.Alignment = [System.Drawing.StringAlignment]::Center
    $format.LineAlignment = [System.Drawing.StringAlignment]::Center
    $format.FormatFlags = [System.Drawing.StringFormatFlags]::NoWrap
    $textRect = New-Object System.Drawing.RectangleF(78, 24, 906, 214)
    $textBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 66, 43, 29))
    $g.DrawString($text, $font, $textBrush, $textRect, $format)

    # text shadow on the upper-left edge for a slightly embossed look
    $font.Dispose(); $format.Dispose(); $textBrush.Dispose()

    # little robot badge at the bottom-right
    $bx = 912; $by = 54
    $badge = New-RoundedPath $bx $by 62 62 20
    $badgeBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 205, 54, 46))
    $g.FillPath($badgeBrush, $badge)
    $badgeBrush.Dispose(); $badge.Dispose()
    $eyeBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 248, 235))
    $g.FillEllipse($eyeBrush, $bx + 15, $by + 21, 9, 9)
    $g.FillEllipse($eyeBrush, $bx + 38, $by + 21, 9, 9)
    $g.FillRectangle($eyeBrush, $bx + 19, $by + 43, 24, 5)
    $eyeBrush.Dispose()

    $g.Dispose()
    $bmp.Save($file, [System.Drawing.Imaging.ImageFormat]::Png)
    $bmp.Dispose()
}

New-SpeechBubble '不早了' (Join-Path $speechDir 'speech_1.png') 96
New-SpeechBubble '该睡觉了' (Join-Path $speechDir 'speech_2.png') 90
New-SpeechBubble '手机将于次日早上6点整归还' (Join-Path $speechDir 'speech_3.png') 58
New-SpeechBubble '将为您播放助眠曲' (Join-Path $speechDir 'speech_4.png') 74

# --------------------------------------------------------------------------
# Digital clock: seven-segment LED faces
# --------------------------------------------------------------------------
$segments = @{
    '0' = 'ABCDEF'; '1' = 'BC'; '2' = 'ABGED'; '3' = 'ABGCD'; '4' = 'FGBC'
    '5' = 'AFGCD'; '6' = 'AFGECD'; '7' = 'ABC'; '8' = 'ABCDEFG'; '9' = 'ABCDFG'
}

function Draw-Digit($g, [string]$ch, [single]$x, [single]$y, [single]$w, [single]$h, [single]$t) {
    $on = $segments[$ch]
    $cols = @{
        'A' = @(($x + $t), $y, ($w - 2*$t), $t)
        'B' = @(($x + $w - $t), ($y + $t), $t, (($h/2) - 1.5*$t))
        'C' = @(($x + $w - $t), ($y + $h/2 + 0.5*$t), $t, (($h/2) - 1.5*$t))
        'D' = @(($x + $t), ($y + $h - $t), ($w - 2*$t), $t)
        'E' = @($x, ($y + $h/2 + 0.5*$t), $t, (($h/2) - 1.5*$t))
        'F' = @($x, ($y + $t), $t, (($h/2) - 1.5*$t))
        'G' = @(($x + $t), ($y + $h/2 - $t/2), ($w - 2*$t), $t)
    }
    foreach ($k in @('A','B','C','D','E','F','G')) {
        $c = $cols[$k]
        # dim ghost segment
        $ghost = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 28, 4, 3))
        $g.FillRectangle($ghost, [single]$c[0], [single]$c[1], [single]$c[2], [single]$c[3])
        $ghost.Dispose()
        if ($on.Contains($k)) {
            # red glow halo
            $glow = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(60, 255, 24, 10))
            $g.FillRectangle($glow, [single]($c[0]-4), [single]($c[1]-4), [single]($c[2]+8), [single]($c[3]+8))
            $glow.Dispose()
            $bright = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 63, 38))
            $g.FillRectangle($bright, [single]$c[0], [single]$c[1], [single]$c[2], [single]$c[3])
            $bright.Dispose()
        }
    }
}

function Draw-Colon($g, [single]$x, [single]$y, [single]$h, [single]$r) {
    $b = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 63, 38))
    $glow = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(55, 255, 32, 12))
    foreach ($cy in @(($y + $h*0.32), ($y + $h*0.68))) {
        $g.FillEllipse($glow, $x-5, $cy-$r-5, 2*($r+5), 2*($r+5))
        $g.FillEllipse($b, $x-$r, $cy-$r, 2*$r, 2*$r)
    }
    $b.Dispose(); $glow.Dispose()
}

function New-ClockFace([string]$time, [string]$file) {
    $W = 512; $H = 256
    $bmp = New-Object System.Drawing.Bitmap($W, $H, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
    $g.Clear([System.Drawing.Color]::FromArgb(255, 7, 6, 9))

    # faint screen vignette / scanlines
    $vignette = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
        (New-Object System.Drawing.RectangleF(0,0,$W,$H)),
        [System.Drawing.Color]::FromArgb(255, 12, 8, 10),
        [System.Drawing.Color]::FromArgb(255, 4, 3, 6),
        [System.Drawing.Drawing2D.LinearGradientMode]::Vertical)
    $g.FillRectangle($vignette, 0, 0, $W, $H)
    $vignette.Dispose()

    $labelFont = New-Object System.Drawing.Font('Microsoft YaHei', 26, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
    $labelBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 224, 52, 37))
    $labelFmt = New-Object System.Drawing.StringFormat
    $labelFmt.Alignment = [System.Drawing.StringAlignment]::Near
    $g.DrawString('睡觉时间', $labelFont, $labelBrush, 24, 14)
    $g.DrawString('SLEEP  MODE', (New-Object System.Drawing.Font('Bahnschrift', 13, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)), (New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 118, 28, 22))), 360, 26)
    $labelFont.Dispose(); $labelBrush.Dispose(); $labelFmt.Dispose()

    $digits = $time.ToCharArray()
    $dw = 48; $dh = 118; $gap = 8; $t = 9
    $total = 6*$dw + 2*22 + 7*$gap
    $startX = ($W - $total) / 2
    $x = $startX
    $y = 104
    $di = 0
    foreach ($ch in $digits) {
        if ($ch -eq ':') {
            Draw-Colon $g ($x + 11) $y $dh 7
            $x += 22 + $gap
        } else {
            Draw-Digit $g ([string]$ch) $x $y $dw $dh $t
            $x += $dw + $gap
        }
    }

    # power LED
    $ledBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 48, 32))
    $g.FillEllipse($ledBrush, 480, 18, 8, 8)
    $ledBrush.Dispose()

    $g.Dispose()
    $bmp.Save($file, [System.Drawing.Imaging.ImageFormat]::Png)
    $bmp.Dispose()
}

# 11:59:40 ... 11:59:59, then 12:00:00
foreach ($s in 40..59) {
    $time = '11:59:{0:00}' -f $s
    $name = 'clock_{0}.png' -f $time.Replace(':','_')
    New-ClockFace $time (Join-Path $clockDir $name)
}
New-ClockFace '12:00:00' (Join-Path $clockDir 'clock_12_00_00.png')

# --------------------------------------------------------------------------
# Phone screen: a bright, doom-scrolling social feed
# --------------------------------------------------------------------------
$W = 512; $H = 1024
$bmp = New-Object System.Drawing.Bitmap($W, $H)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit

# screen backdrop
$bg = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
    (New-Object System.Drawing.RectangleF(0,0,$W,$H)),
    [System.Drawing.Color]::FromArgb(255, 20, 23, 38),
    [System.Drawing.Color]::FromArgb(255, 8, 9, 16),
    [System.Drawing.Drawing2D.LinearGradientMode]::Vertical)
$g.FillRectangle($bg, 0, 0, $W, $H)
$bg.Dispose()

# status bar
$small = New-Object System.Drawing.Font('Microsoft YaHei', 18, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
$white = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::White)
$g.DrawString('23:59', $small, $white, 22, 14)
$g.FillRectangle($white, 410, 23, 22, 10)
$g.FillRectangle($white, 438, 20, 4, 13)
$g.FillRectangle($white, 446, 17, 4, 16)
$g.FillRectangle($white, 454, 14, 4, 19)
$g.FillRectangle($white, 466, 17, 28, 13)
$small.Dispose()

# app header
$titleFont = New-Object System.Drawing.Font('Microsoft YaHei', 30, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
$g.DrawString('夜猫子', $titleFont, $white, 24, 74)
$titleFont.Dispose()
$subFont = New-Object System.Drawing.Font('Microsoft YaHei', 16, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
$g.DrawString('再刷五分钟就睡', $subFont, (New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 126, 133, 160))), 24, 118)
$subFont.Dispose()

# hero post card
$cardRect = New-Object System.Drawing.RectangleF(20, 166, 472, 388)
$photo = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
    $cardRect,
    [System.Drawing.Color]::FromArgb(255, 255, 138, 88),
    [System.Drawing.Color]::FromArgb(255, 68, 44, 103),
    [System.Drawing.Drawing2D.LinearGradientMode]::Vertical)
$g.FillRectangle($photo, $cardRect)
$photo.Dispose()
# sun / window light
$sun = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(180, 255, 224, 157))
$g.FillEllipse($sun, 84, 248, 150, 150)
$sun.Dispose()
# silhouettes of a skyline / room
$sil = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(130, 35, 25, 55))
$g.FillRectangle($sil, 20, 440, 472, 114)
$sil.Dispose()
$sil2 = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(210, 24, 18, 38))
$g.FillRectangle($sil2, 20, 500, 472, 54)
$sil2.Dispose()
# white overlay text on photo
$photoText = New-Object System.Drawing.Font('Microsoft YaHei', 22, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
$g.DrawString('凌晨两点的城市，也有人醒着', $photoText, $white, 42, 198)
$photoText.Dispose()

# avatar + author
$avatarBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 238, 145, 94))
$g.FillEllipse($avatarBrush, 24, 576, 54, 54)
$avatarBrush.Dispose()
$nameFont = New-Object System.Drawing.Font('Microsoft YaHei', 20, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
$g.DrawString('不睡觉的小鱼', $nameFont, $white, 92, 580)
$nameFont.Dispose()
$bodyFont = New-Object System.Drawing.Font('Microsoft YaHei', 18, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
$g.DrawString('今天的烦恼就留到明天吧。', $bodyFont, (New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 218, 221, 235))), 24, 652)
$g.DrawString('晚安，但还不困。', $bodyFont, (New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 218, 221, 235))), 24, 684)

# engagement row
$iconFont = New-Object System.Drawing.Font('Segoe UI Symbol', 24, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
$red = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 76, 94))
$dim = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 137, 142, 164))
$g.DrawString('♥', $iconFont, $red, 26, 730)
$g.DrawString('1.2万', $bodyFont, $dim, 62, 736)
$g.DrawString('💬 386', $iconFont, $dim, 158, 732)
$g.DrawString('↗', $iconFont, $dim, 310, 730)
$iconFont.Dispose()

# next peeking card
$nextRect = New-Object System.Drawing.RectangleF(20, 806, 472, 176)
$next = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
    $nextRect,
    [System.Drawing.Color]::FromArgb(255, 32, 55, 76),
    [System.Drawing.Color]::FromArgb(255, 19, 28, 44),
    [System.Drawing.Drawing2D.LinearGradientMode]::Vertical)
$g.FillRectangle($next, $nextRect)
$next.Dispose()
$g.DrawString('睡前小测试：你属于哪种夜猫子？', $bodyFont, $white, 40, 840)
$g.DrawString('参与讨论  →', $bodyFont, (New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 190, 112))), 40, 890)
$bodyFont.Dispose()

# notification badge
$badge = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 255, 54, 75))
$g.FillEllipse($badge, 432, 74, 46, 46)
$badge.Dispose()
$badgeFont = New-Object System.Drawing.Font('Microsoft YaHei', 16, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
$g.DrawString('99+', $badgeFont, $white, 437, 85)
$badgeFont.Dispose()

# screen glare
$glare = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
    (New-Object System.Drawing.RectangleF(-100, 0, 420, 1024)),
    [System.Drawing.Color]::FromArgb(38, 255, 255, 255),
    [System.Drawing.Color]::FromArgb(0, 255, 255, 255),
    [System.Drawing.Drawing2D.LinearGradientMode]::Horizontal)
$g.FillRectangle($glare, -100, 0, 420, 1024)
$glare.Dispose()

$g.Dispose()
$bmp.Save((Join-Path $phoneDir 'phone_screen.png'), [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()

# small label used on the clock body
$bmp = New-Object System.Drawing.Bitmap(512, 96, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.Clear([System.Drawing.Color]::Transparent)
$f = New-Object System.Drawing.Font('Microsoft YaHei', 40, [System.Drawing.FontStyle]::Bold, [System.Drawing.GraphicsUnit]::Pixel)
$b = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 210, 48, 34))
$g.DrawString('睡觉时间', $f, $b, 8, 14)
$g.Dispose()
$bmp.Save((Join-Path $root 'clock_label.png'), [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()

Write-Host "UI assets written to $root"

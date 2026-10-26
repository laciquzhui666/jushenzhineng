$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$speech = Join-Path $root 'speech'
$cream = [System.Drawing.Color]::FromArgb(255, 255, 243, 222)

for ($i = 1; $i -le 4; $i++) {
    $srcPath = Join-Path $speech ("speech_{0}.png" -f $i)
    $dstPath = Join-Path $speech ("speech_plate_{0}.png" -f $i)
    $src = [System.Drawing.Bitmap]::FromFile($srcPath)
    $dst = New-Object System.Drawing.Bitmap(1024, 256, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $g = [System.Drawing.Graphics]::FromImage($dst)
    $g.Clear($cream)
    $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
    $g.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::HighQuality
    $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality
    $srcRect = New-Object System.Drawing.Rectangle(24, 16, 976, 234)
    $dstRect = New-Object System.Drawing.Rectangle(0, 0, 1024, 256)
    $g.DrawImage($src, $dstRect, $srcRect, [System.Drawing.GraphicsUnit]::Pixel)
    $g.Dispose()
    $dst.Save($dstPath, [System.Drawing.Imaging.ImageFormat]::Png)
    $dst.Dispose()
    $src.Dispose()
}
Write-Host "speech plates written"

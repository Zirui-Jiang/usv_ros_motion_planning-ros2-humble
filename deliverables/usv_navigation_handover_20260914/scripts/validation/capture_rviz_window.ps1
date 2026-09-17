param(
    [Parameter(Mandatory=$true)][string]$OutputPath,
    [string]$TitlePattern = '*sim_env*RViz*'
)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $OutputPath) { throw 'Refusing to overwrite an existing screenshot.' }
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
public class RvizCapture {
  public delegate bool Callback(IntPtr handle, IntPtr unused);
  [DllImport("user32.dll")] public static extern bool EnumWindows(Callback callback, IntPtr unused);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr handle, StringBuilder title, int count);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr handle);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr handle, out Rect rectangle);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr handle);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr handle, int command);
  [StructLayout(LayoutKind.Sequential)] public struct Rect { public int Left, Top, Right, Bottom; }
  public static Dictionary<IntPtr,string> VisibleWindows() {
    var windows = new Dictionary<IntPtr,string>();
    EnumWindows((handle, unused) => {
      var title = new StringBuilder(1024);
      GetWindowText(handle, title, title.Capacity);
      if (IsWindowVisible(handle) && title.Length > 0) windows[handle] = title.ToString();
      return true;
    }, IntPtr.Zero);
    return windows;
  }
}
'@
$rvizWindows = @([RvizCapture]::VisibleWindows().GetEnumerator() | Where-Object { $_.Value -like $TitlePattern })
if ($rvizWindows.Count -ne 1) { throw "Expected one matching RViz window; found $($rvizWindows.Count)." }
$rvizWindow = $rvizWindows[0]
[void][RvizCapture]::ShowWindow($rvizWindow.Key, 9)
[void][RvizCapture]::SetForegroundWindow($rvizWindow.Key)
Start-Sleep -Milliseconds 800
$rvizRect = New-Object RvizCapture+Rect
if (-not [RvizCapture]::GetWindowRect($rvizWindow.Key, [ref]$rvizRect)) { throw 'Cannot read RViz window bounds.' }
$rvizWidth = $rvizRect.Right - $rvizRect.Left
$rvizHeight = $rvizRect.Bottom - $rvizRect.Top
if ($rvizWidth -lt 100 -or $rvizHeight -lt 100) { throw 'RViz window is too small to capture.' }
$rvizBitmap = [System.Drawing.Bitmap]::new($rvizWidth, $rvizHeight)
$rvizGraphics = [System.Drawing.Graphics]::FromImage($rvizBitmap)
try {
    $rvizGraphics.CopyFromScreen($rvizRect.Left, $rvizRect.Top, 0, 0, $rvizBitmap.Size)
    $rvizBitmap.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
} finally {
    $rvizGraphics.Dispose()
    $rvizBitmap.Dispose()
}
@{title=$rvizWindow.Value; width=$rvizWidth; height=$rvizHeight; screenshot=$OutputPath} | ConvertTo-Json

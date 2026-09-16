# 按标题定位唯一 RViz 窗口并保存截图；只在用户明确运行脚本时捕获，不覆盖已有文件。
param(
    [Parameter(Mandatory=$true)][string]$OutputPath,
    [string]$TitlePattern = '*sim_env*RViz*'
)
$ErrorActionPreference = 'Stop'
if (Test-Path -LiteralPath $OutputPath) { throw 'Refusing to overwrite an existing screenshot.' }
Add-Type -AssemblyName System.Drawing
# 下面的 C# here-string 封装 Win32 窗口查询接口，其正文保持原样。
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
# 要求唯一匹配，避免把其他应用或另一仿真窗口保存为本次证据。
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
# 按实际窗口矩形分配图像；后续 finally 无论成功失败都释放图形资源。
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

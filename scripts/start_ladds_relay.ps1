param(
    [string]$ListenAddress = '',
    [int]$Port = 17880,
    [int]$DomainId = 1,
    [string]$Topic = '0211',
    [string]$ReceiveTopic = '',
    [string]$StopReceiveTopic = '',
    [string]$BerthReceiveTopic = '',
    [string]$SendTopic = '',
    [string]$OwnshipSendTopic = '0411',
    [string]$ObstacleSendTopic = '0141',
    [string]$OwnshipReceiveTopic = '0411',
    [string]$ObstacleReceiveTopic = '0141',
    [ValidateSet('best_effort', 'reliable')][string]$Qos = 'best_effort',
    [string]$BindIp = '',
    [string]$LibraryPath = '',
    [string]$PythonExe = ''
)
# 在 Windows 使用原有 LaDDS 授权；本机 TCP 仅绑定 WSL 虚拟网卡。
$ErrorActionPreference = 'Stop'
if (-not $ReceiveTopic) { $ReceiveTopic = $Topic }
if (-not $SendTopic) { $SendTopic = $Topic }
$laddsRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if (-not $LibraryPath) {
    if ($env:LADDS_LIBRARY_PATH) {
        $LibraryPath = $env:LADDS_LIBRARY_PATH
    } else {
        $laddsLocalLibrary = Join-Path $laddsRoot 'local\ladds\LaDDS.dll'
        $laddsLegacyLibrary = 'D:\自主航行设计文档\0. LaDDS说明\dds联调工具\LaDDS.dll'
        if (Test-Path -LiteralPath $laddsLocalLibrary) {
            $LibraryPath = $laddsLocalLibrary
        } elseif (Test-Path -LiteralPath $laddsLegacyLibrary) {
            $LibraryPath = $laddsLegacyLibrary
        }
    }
}
if (-not $LibraryPath -or -not (Test-Path -LiteralPath $LibraryPath -PathType Leaf)) {
    throw '未找到 LaDDS.dll。请把厂商 SDK/授权完整目录放到 local\ladds，设置 LADDS_LIBRARY_PATH，或传入 -LibraryPath。'
}
$laddsPackage = Join-Path $laddsRoot 'src\core\communication_module\usv_ladds_bridge'
$laddsLibrary = (Resolve-Path -LiteralPath $LibraryPath).Path
if (-not $ListenAddress) {
    $laddsAddresses = @(Get-NetIPAddress -AddressFamily IPv4 |
        Where-Object { $_.InterfaceAlias -like 'vEthernet (WSL*' })
    if ($laddsAddresses.Count -ne 1) {
        throw '无法唯一确定 WSL 虚拟网卡，请通过 -ListenAddress 指定 Windows 的本机 WSL 网卡 IPv4。'
    }
    $ListenAddress = $laddsAddresses[0].IPAddress
}
if (-not $PythonExe) {
    $laddsBundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $laddsBundledPython) { $PythonExe = $laddsBundledPython }
    else { $PythonExe = (Get-Command python -ErrorAction Stop).Source }
}
$laddsPreviousPath = $env:PYTHONPATH
$laddsPreviousEncoding = $env:PYTHONIOENCODING
try {
    $env:PYTHONPATH = $laddsPackage
    if ($laddsPreviousPath) { $env:PYTHONPATH += ';' + $laddsPreviousPath }
    $env:PYTHONIOENCODING = 'utf-8'
    # SDK 在运行目录查找原有授权；不复制或更改授权文件。
    Push-Location -LiteralPath (Split-Path -Parent $laddsLibrary)
    try {
        Write-Host "LaDDS: domain=$DomainId receive=$ReceiveTopic stop=$StopReceiveTopic berth=$BerthReceiveTopic ownship-rx=$OwnshipReceiveTopic obstacle-rx=$ObstacleReceiveTopic send=$SendTopic ownship=$OwnshipSendTopic obstacle=$ObstacleSendTopic qos=$Qos; TCP=$ListenAddress`:$Port"
        $laddsArgs = @('-u', '-m', 'usv_ladds_bridge.relay', '--library', $laddsLibrary,
            '--listen', $ListenAddress, '--port', "$Port", '--domain-id', "$DomainId",
            '--send-topic', $SendTopic, '--ownship-send-topic', $OwnshipSendTopic,
            '--obstacle-send-topic', $ObstacleSendTopic,
            '--receive-topic', $ReceiveTopic,
            '--ownship-receive-topic', $OwnshipReceiveTopic,
            '--obstacle-receive-topic', $ObstacleReceiveTopic,
            '--qos', $Qos)
        if ($BindIp) { $laddsArgs += @('--bind-ip', $BindIp) }
        if ($StopReceiveTopic) { $laddsArgs += @('--stop-receive-topic', $StopReceiveTopic) }
        if ($BerthReceiveTopic) { $laddsArgs += @('--berth-receive-topic', $BerthReceiveTopic) }
        & $PythonExe @laddsArgs
        if ($LASTEXITCODE -ne 0) { throw "LaDDS relay exited: $LASTEXITCODE" }
    } finally { Pop-Location }
} finally {
    $env:PYTHONPATH = $laddsPreviousPath
    $env:PYTHONIOENCODING = $laddsPreviousEncoding
}

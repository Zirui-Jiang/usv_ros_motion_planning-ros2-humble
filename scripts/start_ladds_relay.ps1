param(
    [string]$ListenAddress = '',
    [int]$Port = 17880,
    [int]$DomainId = 1,
    [string]$Topic = '0211',
    [string]$ReceiveTopic = '',
    [string]$StopReceiveTopic = '',
    [string]$SendTopic = '',
    [string]$OwnshipSendTopic = '0411',
    [ValidateSet('best_effort', 'reliable')][string]$Qos = 'best_effort',
    [string]$BindIp = '',
    [string]$LibraryPath = 'D:\自主航行设计文档\0. LaDDS说明\dds联调工具\LaDDS.dll',
    [string]$PythonExe = ''
)
# 在 Windows 使用原有 LaDDS 授权；本机 TCP 仅绑定 WSL 虚拟网卡。
$ErrorActionPreference = 'Stop'
if (-not $ReceiveTopic) { $ReceiveTopic = $Topic }
if (-not $SendTopic) { $SendTopic = $Topic }
$laddsRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
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
        Write-Host "LaDDS: domain=$DomainId receive=$ReceiveTopic stop=$StopReceiveTopic send=$SendTopic ownship=$OwnshipSendTopic qos=$Qos; TCP=$ListenAddress`:$Port"
        $laddsArgs = @('-u', '-m', 'usv_ladds_bridge.relay', '--library', $laddsLibrary,
            '--listen', $ListenAddress, '--port', "$Port", '--domain-id', "$DomainId",
            '--send-topic', $SendTopic, '--ownship-send-topic', $OwnshipSendTopic,
            '--receive-topic', $ReceiveTopic, '--qos', $Qos)
        if ($BindIp) { $laddsArgs += @('--bind-ip', $BindIp) }
        if ($StopReceiveTopic) { $laddsArgs += @('--stop-receive-topic', $StopReceiveTopic) }
        & $PythonExe @laddsArgs
        if ($LASTEXITCODE -ne 0) { throw "LaDDS relay exited: $LASTEXITCODE" }
    } finally { Pop-Location }
} finally {
    $env:PYTHONPATH = $laddsPreviousPath
    $env:PYTHONIOENCODING = $laddsPreviousEncoding
}

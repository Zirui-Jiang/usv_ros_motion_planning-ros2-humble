# 0211 航路联调中继入口：话题固定写在这里，再调用 start_ladds_relay.ps1；-ListenAddress、-Port、-Qos、-LibraryPath 等其余参数原样透传。
# Topics are fixed for route integration. To change a topic, call start_ladds_relay.ps1 directly;
# passing one of these topic parameters here fails as "specified more than once".
$ErrorActionPreference = 'Stop'
$relay = Join-Path $PSScriptRoot 'start_ladds_relay.ps1'
$routeTopics = @{
    DomainId             = 1
    ReceiveTopic         = '0211'
    StopReceiveTopic     = '0331'
    BerthReceiveTopic    = '0212'
    OwnshipReceiveTopic  = '0411'
    ObstacleReceiveTopic = '0141'
    SendTopic            = '0052'
    StatusSendTopic      = '0051'
    ProgressSendTopic    = '0053'
    ThrustSendTopic      = '0054'
    OwnshipSendTopic     = '0411'
    ObstacleSendTopic    = '0141'
}
& $relay @routeTopics @args

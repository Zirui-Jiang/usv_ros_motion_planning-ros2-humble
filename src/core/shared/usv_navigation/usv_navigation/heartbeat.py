# 轻量进程存活与会话跟踪，不导入导航算法；心跳周期和超时均按墙钟秒计。
"""ROS-independent liveness and retired process identity bookkeeping."""
from dataclasses import dataclass, field
import math

HEARTBEAT_PERIOD = 0.2
PEER_TIMEOUT = 1.5


# 保存当前会话、递增序号、最后接收时刻及已退休会话，防止重放恢复旧进程身份。
@dataclass
class PeerSession:
    session: str = ''
    sequence: int = -1
    last_seen: float = float('-inf')
    ready: bool = False
    retired: set = field(default_factory=set)

    # 仅接受新会话或当前会话递增序号，重复乱序消息不刷新在线时间。
    def observe(self, session, sequence, now, ready=True):
        if (not session or session in self.retired or not math.isfinite(now)
                or sequence < 0 or (session == self.session and sequence <= self.sequence)):
            return 'ignored'
        change = 'update'
        if session != self.session:
            change = 'restart' if self.session else 'new'
            if self.session:
                self.retired.add(self.session)
            self.session = session
        self.sequence, self.last_seen, self.ready = sequence, now, ready
        return change

    # 必须已发现会话且墙钟年龄非负、不超过超时，未来或非有限时间均判不新鲜。
    def fresh(self, now, timeout=PEER_TIMEOUT):
        return bool(self.session) and math.isfinite(now) and 0 <= now - self.last_seen <= timeout

#!/usr/bin/env python3
# 仿真 TF 补充节点：周期发布带船舶命名空间的 map 到 odom 单位变换。
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# 本仿真没有独立地图校正，map 与 odom 重合；定时发送使地图坐标系尽早可查询。
class UsvTfExpander(Node):
    """Broadcast a fixed identity transform from <ns>/map to <ns>/odom.

    In this simulation the EKF fuses everything in the odom frame, so there is
    no separate map-level correction and map->odom is a constant identity.
    It is published on a timer (rather than only when an odom message arrives)
    so the <ns>/map frame exists as soon as this node starts -- otherwise the
    costmap times out waiting for the map frame while the EKF / odom relay
    chain is still spinning up ("Invalid frame ID otter/map ... frame does not
    exist").
    """

    def __init__(self):
        super().__init__('usv_tf_expander')
        self.robot_namespace = self.declare_parameter(
            'robot_namespace', 'otter').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)
        # 50 Hz keeps the transform comfortably fresh relative to the costmap's
        # transform_tolerance regardless of how the sim clock advances.
        self.timer = self.create_timer(0.02, self.publish_map_to_odom)

    # 使用当前 ROS 时刻发布零平移、单位旋转，父子坐标系均带 robot_namespace。
    def publish_map_to_odom(self):
        map_2_odom = TransformStamped()
        map_2_odom.header.stamp = self.get_clock().now().to_msg()
        map_2_odom.header.frame_id = f'{self.robot_namespace}/map'
        map_2_odom.child_frame_id = f'{self.robot_namespace}/odom'
        map_2_odom.transform.rotation.w = 1.0
        self.br.sendTransform(map_2_odom)


# 运行 TF 广播节点，退出时释放节点与 ROS 上下文。
def main():
    rclpy.init()
    node = UsvTfExpander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'target_pose', 10)
        
    def publish_pose(self, x, y, z, w):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = w  # 保持默认方向
        self.publisher_.publish(pose)
        self.get_logger().info(f'发布坐标: x={x}, y={y}, z={z}, w={w}')

def main(args=None):
    rclpy.init(args=args)
    publisher = PosePublisher()
    
    try:
        while rclpy.ok():
            # 示例坐标输入（替换为实际输入方式）
            x = float(input("输入X坐标: "))
            y = float(input("输入Y坐标: "))
            z = float(input("输入Z坐标: "))
            w = float(input("输入方向W: "))
            
            publisher.publish_pose(x, y, z, w)
            rclpy.spin_once(publisher, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
        
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

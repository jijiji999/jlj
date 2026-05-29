#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyTestNode(Node):
    def __init__(self):
        # 初始化节点名字为 test_node
        super().__init__('test_node')
        # 打印一行日志
        self.get_logger().info('恭喜你，test 包里的节点已经成功运行了！')

def main(args=None):
    rclpy.init(args=args)
    node = MyTestNode()
    # 让节点保持运行状态
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

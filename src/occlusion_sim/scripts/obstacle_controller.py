#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')
        # /obstacle/cmd_vel にPublishすることに注意
        self.publisher_ = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        msg = Twist()
        # 例えば8秒周期でY軸を行ったり来たりさせる
        elapsed = time.time() - self.start_time
        if (elapsed % 8.0) < 4.0:
            msg.linear.y = 1.0  # 左へ
        else:
            msg.linear.y = -1.0 # 右へ
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

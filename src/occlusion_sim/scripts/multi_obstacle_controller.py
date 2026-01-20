#!/usr/bin/env python3
"""Multi Obstacle Controller - Random walk behavior matching safe_control/dynamic_env"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import random
import math


class MultiObstacleController(Node):
    def __init__(self):
        super().__init__('multi_obstacle_controller')
        
        random.seed(42)
        self.obstacles = {
            'obs_0': {'mode': 1, 'v_max': 0.4, 'theta': random.uniform(-math.pi, math.pi),
                      'y_min': -3.0, 'y_max': 3.0, 'state': None},
            'obs_1': {'mode': 1, 'v_max': 0.5, 'theta': random.uniform(-math.pi, math.pi),
                      'y_min': -2.0, 'y_max': 4.0, 'state': None},
            'obs_2': {'mode': 1, 'v_max': 0.3, 'theta': random.uniform(-math.pi, math.pi),
                      'y_min': -4.0, 'y_max': 2.0, 'state': None},
        }
        self.radius = 0.3
        
        for name in self.obstacles:
            self.create_subscription(Odometry, f'/{name}/odom', 
                                     lambda msg, n=name: self.odom_cb(msg, n), 10)
        
        self.cmd_pubs = {n: self.create_publisher(Twist, f'/{n}/cmd_vel', 10) for n in self.obstacles}
        self.state_pub = self.create_publisher(Odometry, '/obstacle/state', 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Multi Obstacle Controller started')

    def odom_cb(self, msg, name):
        self.obstacles[name]['state'] = msg

    def control_loop(self):
        for name, obs in self.obstacles.items():
            if obs['state'] is None:
                continue
            
            y = obs['state'].pose.pose.position.y
            theta = obs['theta']
            v_max = obs['v_max']
            
            # Random walk: 5% chance of large turn (matches safe_control/dynamic_env)
            if obs['mode'] == 1 and random.random() < 0.05:
                theta += random.gauss(0.0, 0.2)
                obs['theta'] = theta
            
            vx = v_max * math.cos(theta)
            vy = v_max * math.sin(theta)
            
            # Reflect at y bounds
            if y >= obs['y_max']:
                obs['theta'] = -obs['theta']
                vy = -abs(vy)
            elif y <= obs['y_min']:
                obs['theta'] = -obs['theta']
                vy = abs(vy)
            
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            cmd.angular.z = float(obs['theta'])  # Rotate to heading
            self.cmd_pubs[name].publish(cmd)
            
            state = Odometry()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'world'
            state.child_frame_id = name
            state.pose.pose = obs['state'].pose.pose
            state.twist.twist.linear.x = vx
            state.twist.twist.linear.y = vy
            self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = MultiObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

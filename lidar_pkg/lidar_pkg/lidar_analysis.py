#!/usr/bin/env python3
# Lidar data preprocessing

''' To-Do list
1. Edit Codes
 ( Remove calculating-dist and selecting-act part )
 Must moving them to Remote PC's main.py because,
 We can't read custom topic that includes
 String-action and LaserScan-ingredients.

2. 
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import numpy as np
import time

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5

class LidarAnalyzer(Node):
    def __init__(self):
        super().__init__('lidar_analyzer')
        self.subscription_s = self.create_subscription(
            LaserScan, "/scan", self.listner_callback, 10)
        self.subscription_c = self.create_subscription(
            Twist, "/cmd_mock", self.listner_callback_tw, 10)

        self.safe_dist = 0.3
        self.latest_ranges = []
        self.latest_intensities = []
        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0

        self.publisher_r = self.create_publisher(
            LaserScan, "/scanmock", 10)
        self.publisher_tb = self.create_publisher(
            Twist, "/cmd_vel", 10)

        self.time_period = 2.0
        self.s_timer = self.create_timer(self.time_period, self.listner_callback)
        self.p_timer = self.create_timer(self.time_period, self.talker_callback)

    def cal_dist(self, ranges):
        min_dist = np.min(ranges)
        avg_dist = np.mean(ranges)
        return min_dist, avg_dist

    def select_action(self, directions):
        # initialize possiblity of directions
        now_dir_history = {}

        # check 4 directions(in dict.) and make action
        for now_dir, now_ran in directions.items():
            min_dist, avg_dist = self.cal_dist(now_ran)
            if min_dist < self.safe_dist:
                now_dir_history[now_dir] = False
                self.get_logger().warn(f"{now_dir} : Too close! {min_dist} m")
            else:
                now_dir_history[now_dir] = True
                self.get_logger().info(f"{now_dir} : Safe distance: {avg_dist} m left")

        # Default : Go Forward (if there aren't walls at front)
        # Turn Left (walls at front, right)
        # Turn Right (walls at front, left)
        # Turn Around (walls at front, left, right)
        front = now_dir_history['front']
        left = now_dir_history['left']
        back = now_dir_history['back']
        right = now_dir_history['right']

        if front:
            action = "Go Forward"
            self.linear_x = 0.2
            self.angular_z = 0.0

        elif right:
            action = "Turn Right"
            self.linear_x = 0.0
            self.angular_z = np.deg2rad(-35)

        elif left:
            action = "Turn Left"
            self.linear_x = 0.0
            self.angular_z = np.deg2rad(35)

        elif back:
            action = "Turn Around"
            self.linear_x = 0.0
            self.angular_z = np.deg2rad(90)
        else:
            action = "Stop"
            self.linear_x = 0.0
            self.angular_z = 0.0
        return action

    def listner_callback(self, msg: LaserScan):
        self.latest_ranges = msg.ranges
        self.latest_intensities = msg.intensities

        # 1. ROS data to Numpy
        ranges = np.array(self.latest_ranges)
        intensities = np.array(self.latest_intensities)

        # 2. preprocessing : inf data
        ranges = np.where(ranges == float('inf'), 3.5, ranges)

        # 3. check front side
        front_ranges = np.concatenate((ranges[:16], ranges[-15:]), axis=0)
        left_ranges = ranges[75:106]
        back_ranges = ranges[165:196]
        right_ranges = ranges[255:286]

        directions = {
            'front' : front_ranges,
            'left' : left_ranges,
            'back' : back_ranges,
            'right' : right_ranges
        }

        # 4. calculate min and mean, select action
        final_action = self.select_action(directions)
        print(f"\nFinal Action(publish soon) : {final_action} !\n\n")

    def listner_callback_tw(self, msg: Twist):
        self.latest_linear_x = msg.linear_x
        self.latest_angular_z = msg.angular_z

    def talker_callback(self):
        ranges = self.latest_ranges
        intensities = self.latest_intensities

        # 1. Send LaserScan() to websocket-server
        lmsg = LaserScan()
        lmsg.header = Header()
        lmsg.header.stamp = self.get_clock().now().to_msg()
        lmsg.header.frame_id = "lidar"

        lmsg.angle_min = np.deg2rad(ANGLE_MIN_DEG)
        lmsg.angle_max = np.deg2rad(ANGLE_MAX_DEG)
        lmsg.angle_increment = np.deg2rad(ANGLE_INCREMENT_DEG)
        lmsg.range_min = RANGE_MIN
        lmsg.range_max = RANGE_MAX
        lmsg.ranges = ranges
        lmsg.intensities = intensities

        self.publisher_r.publish(lmsg)
        self.get_logger().info("LaserScan topic has sent")

        # 2. Send Twist() to move gazebo-turtlebot3
        tmsg = Twist()
        tmsg.linear.x = self.linear_x
        tmsg.angular.z = self.angular_z
        self.publisher_tb.publish(tmsg)

def main():
    rclpy.init()
    node = LidarAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
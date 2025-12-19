#!/usr/bin/env python3
# Lidar data preprocessing + Timer-based publish

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

cmd_vel_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
stop_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE
)

DECIDE = 1
TURN = 0

class LidarAnalyzer(Node):
    def __init__(self):
        super().__init__('waffle_rvc')

        # yaml Parameters
        self.declare_parameter('max_velocity', 0.2)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('current_linear_x', 0.12)
        self.declare_parameter('turning_speed', 0.5)

        self.max_linear = self.get_parameter(
            'max_velocity').value
        self.max_range_val = self.get_parameter(
            'max_range').value
        self.cur_linear = self.get_parameter(
            'current_linear_x').value
        self.cur_angular = self.get_parameter(
            'turning_speed').value

        self.add_on_set_parameters_callback(self.param_callback)

        # Subscribers
        self.subscriber_ = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.stop_sub = self.create_subscription(
            Bool,
            "stop_signal",
            self.stop_callback,
            stop_qos
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            "cmd_vel",
            cmd_vel_qos
        )

        # Timers (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Parameters
        self.safe_dist = 0.55
        self.STATE = DECIDE
        self.stop_by_yolo = False

        # Data storage
        self.latest_ranges = None
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.current_action = "STOP"
        self.turn_start_time = None
        self.min_turn_time = 0.6  # seconds

    def select_action(self, ranges):
        front_range = np.min(np.concatenate((ranges[:21], ranges[-20:])))
        left_range = np.min(ranges[40:81])
        right_range = np.min(ranges[-80:-39])

        if self.STATE == DECIDE:
            if front_range < self.safe_dist:
                if right_range > left_range:
                    self.angular_z = -self.cur_angular
                    self.current_action = "Turn Right"
                else:
                    self.angular_z = self.cur_angular
                    self.current_action = "Turn Left"

                self.linear_x = 0.0
                self.STATE = TURN
                self.turn_start_time = self.get_clock().now()
            else:
                self.linear_x = min(self.cur_linear,self.max_linear)
                self.angular_z = 0.0
                self.current_action = "Go Forward"

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, self.max_range_val)
        self.latest_ranges = ranges

        if self.STATE == TURN:
            self.check_turn_done(ranges)
            return
        
        self.select_action(ranges)

        if self.current_action != getattr(self, "prev_action", None):
            self.get_logger().info(f"Action: {self.current_action}")
            self.prev_action = self.current_action

    def stop_callback(self, msg: Bool):
        self.stop_by_yolo = msg.data

    def publish_cmd(self):
        """Timer callback: Twist()"""
        if self.stop_by_yolo:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return

        if self.latest_ranges is None:
            return

        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)

    def check_turn_done(self, ranges):
        elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9

        if elapsed < self.min_turn_time:
            return

        front_range = np.min(np.concatenate((ranges[:21], ranges[-20:])))

        if front_range > self.safe_dist + 0.15:
            self.STATE = DECIDE
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.current_action = "Turn Finished"
            self.get_logger().info(f"Action: {self.current_action}")
            self.prev_action = "Turn Finished"
            return

    def param_callback(self, params):
        for param in params:
            if param.name == "current_linear_x":
                if param.value < 0.0 or param.value > self.max_linear:
                    return SetParametersResult(
                        successful=False,
                        reason=f"current_linear_x must be >= 0.0 and <= {self.max_linear}"
                    )
                else:
                    self.cur_linear = param.value

                self.get_logger().info(
                    f"Updated current_linear_x: {self.cur_linear}"
                )

            elif param.name == "turning_speed":
                if abs(param.value) > 1.5:
                    return SetParametersResult(
                        successful=False,
                        reason="angular velocity too large"
                    )

                self.cur_angular = param.value
                self.get_logger().info(
                    f"Updated turning_speed: {self.cur_angular}"
                )

            elif param.name == "max_velocity":
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="max_velocity must be positive"
                    )
                self.max_linear = param.value

        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = LidarAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
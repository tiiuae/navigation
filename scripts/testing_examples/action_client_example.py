#!/usr/bin/python3

## Debug script for easier fog_msgs.action.NavigationAction publishing

import rclpy
import math
import os
from rclpy.action import ActionClient
from rclpy.node import Node

from fog_msgs.action import NavigationAction

from nav_msgs.msg import Path as NavPath
from fog_msgs.srv import Path as SetPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from action_msgs.msg import GoalStatus

WAYPOINTS_LOCAL=[
    [0,0,3,1],
    [-5,4,2,2],
    [7,-5,2,3],
    [0,0,2,4]
]

WAYPOINTS_GPS=[
    [47.397708, 8.5456038, 4, 0],
    [47.3977, 8.5456038, 2, 1.5708],
    [47.39775, 8.5456, 2, -1.5708],
    [47.39758, 8.545706, 1.5, 3.14],
    [47.397708, 8.5456038, 2, -3.14]
]

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Code below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

class NavigationActionClient(Node):

    def __init__(self):
        super().__init__('control_interface_action_client')

        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

        self._action_client = ActionClient(self, NavigationAction, "/" + DRONE_DEVICE_ID + "/navigation")

    def send_goal(self):
        print('Waiting for action server')
        self._action_client.wait_for_server()

        path = NavPath()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "world"

        for wp in WAYPOINTS_GPS:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = "world"
            point = Point()
            point.x = float(wp[0])
            point.y = float(wp[1])
            point.z = float(wp[2])
            pose.pose.position = point
            q = quaternion_from_euler(0,0,wp[3])
            pose.pose.orientation = q
            path.poses.append(pose)

        goal_msg = NavigationAction.Goal()
        goal_msg.path = path
        goal_msg.is_local = False
        
        print('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        # Start a 2 second timer
        # self._goal_handle = goal_handle
        # self._timer = self.create_timer(2.0, self.timer_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal aborted! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error('Goal canceled! Result: {0}'.format(result.message))
        # Shutdown after receiving a result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().error('Goal failed to cancel')

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()
    action_client.get_logger().info('********************************')

    future = action_client.send_goal()
    rclpy.spin(action_client)

    action_client.get_logger().info('=================================')

if __name__ == '__main__':
    main()

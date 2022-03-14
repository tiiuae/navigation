#!/usr/bin/python3

## Debug script for easier fog_msgs.action.NavigationAction publishing

import rclpy
import math
import os

from threading import Event

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fog_msgs.action import NavigationAction

from fog_msgs.srv import Vec4 

from std_srvs.srv import Trigger
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from action_msgs.msg import GoalStatus

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
        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

        super().__init__("navigation_action_client")
        self._action_done_event = Event()
        self._action_cancel_event = Event()
        self._action_response_result = False
        self._action_cancel_result = False
        self._goal_handle = None

        self._action_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_client = ActionClient(self, NavigationAction, "/" + DRONE_DEVICE_ID + "/navigation", callback_group = self._action_callback_group)

        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        self._local_waypoint_service = self.create_service(Vec4, '~/local_waypoint', self.local_waypoint_callback, callback_group = self._service_callback_group) 
        self._gps_waypoint_service = self.create_service(Vec4, '~/gps_waypoint', self.gps_waypoint_callback, callback_group = self._service_callback_group) 
        self._cancel_goal = self.create_service(Trigger, '~/cancel_goal', self.cancel_goal_callback, callback_group = self._service_callback_group) 

    def local_waypoint_callback(self, request, response):
        self.get_logger().info('Incoming local waypoint request: \t{0}'.format(request.goal))
        self.send_goal(request.goal, True)
        
        # Wait for action to be done
        self._action_done_event.wait()
        response.success = self._action_response_result
        if response.success:
            response.message = "Goal accepted"
        else:
            response.message = "Goal rejected"
        return response

    def gps_waypoint_callback(self, request, response):
        self.get_logger().info('Incoming gps waypoint request: \t{0}'.format(request.goal))
        self.send_goal(request.goal, False)
        
        # Wait for action to be done
        self._action_done_event.wait()
        response.success = self._action_response_result
        if response.success:
            response.message = "Goal accepted"
        else:
            response.message = "Goal rejected"
        return response

    def cancel_goal_callback(self, request, response):
        self.get_logger().info('Incoming request to cancel goal')
        if self._goal_handle is None: 
            response.success = False
            response.message = "No active goal"
            self.get_logger().error('{0}'.format(response.message))
            return response

        self.cancel_goal()
        # Wait for action to be done
        self._action_cancel_event.wait()
        response.success = self._action_cancel_result
        if response.success:
            response.message = "Goal canceled"
        else:
            response.message = "Goal failed to cancel"
        return response


    def send_goal(self, goal, is_local):
        self.get_logger().info('Waiting for action server')
        self._action_client.wait_for_server()

        path = NavPath()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "world"

        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = "world"
        point = Point()
        point.x = float(goal[0])
        point.y = float(goal[1])
        point.z = float(goal[2])
        pose.pose.position = point
        q = quaternion_from_euler(0,0,goal[3])
        pose.pose.orientation = q
        path.poses.append(pose)

        goal_msg = NavigationAction.Goal()
        goal_msg.path = path
        goal_msg.is_local = is_local
        
        self.get_logger().info('Sending goal request...')
        self._action_done_event.clear()
        self._action_response_result = False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            # Signal that action is done
            self._action_done_event.set()
            return

        self.get_logger().info('Goal accepted :)')
        self._action_response_result = True
        # Signal that action is done
        self._action_done_event.set()

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        self._goal_handle = goal_handle

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal aborted! Result: {0}'.format(result.message))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error('Goal canceled! Result: {0}'.format(result.message))

        if self._goal_handle.goal_id == future.result().goal_id:
            self._goal_handle = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.mission_progress))

    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        self._action_cancel_event.clear()
        self._action_cancel_result = False
        
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self._action_cancel_result = True
            self._goal_handle = None
        else:
            self.get_logger().error('Goal failed to cancel')

        self._action_cancel_event.set()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()
    action_client.get_logger().info('********************************')

    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor)

    action_client.get_logger().info('=================================')

if __name__ == '__main__':
    main()

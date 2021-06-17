#!/usr/bin/python3

## Debug script for easier nav_msgs.msg.Path publishing

import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import time as pythontime

from nav_msgs.msg import Path as NavPath
from fog_msgs.srv import Path as SetPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

WAYPOINTS_LOCAL=[
    [0,0,3],
    [-5,4,2],
    [7,-5,2],
    [0,0,2]
]

WAYPOINTS_GPS=[
    [47.397708, 8.5456038, 4],
    [47.3977, 8.5456038, 2],
    [47.39775, 8.5456, 2],
    [47.39758, 8.545706, 1.5],
    [47.397708, 8.5456038, 2]
]

class PathPublisherNode(Node):

    def __init__(self):

        DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

        super().__init__("waypoint_publisher")
        self.client_local = self.create_client(SetPath, "/" + DRONE_DEVICE_ID + "/navigation/local_path")
        self.client_gps = self.create_client(SetPath, "/" + DRONE_DEVICE_ID + "/navigation/gps_path")

    def publish(self):
        path = Path()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "world"

        for wp in WAYPOINTS:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = "world"
            point = Point()
            point.x = float(wp[0])
            point.y = float(wp[1])
            point.z = float(wp[2])
            pose.pose.position = point
            path.poses.append(pose)
        print('Publishing: "%s"' % path.poses)
        self.publisher.publish(path)

    # #{ call_service_local
    def call_service_local(self):
        path = NavPath()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "world"

        for wp in WAYPOINTS_LOCAL:
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = "world"
            point = Point()
            point.x = float(wp[0])
            point.y = float(wp[1])
            point.z = float(wp[2])
            pose.pose.position = point
            path.poses.append(pose)
        print('Calling service : "%s"' % self.client_local.srv_name)
        path_req = SetPath.Request()
        path_req.path = path
        self.future = self.client_local.call_async(path_req)
    # #}
    
    # #{ call_service_gps
    def call_service_gps(self):
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
            path.poses.append(pose)
        print('Calling service : "%s"' % self.client_gps.srv_name)
        path_req = SetPath.Request()
        path_req.path = path
        self.future = self.client_gps.call_async(path_req)
    # #}

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()

    rate = node.create_rate(0.1)
    node.call_service_gps()
    # node.call_service_local()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response.success:
                    node.get_logger().info(
                        'Service SUCCEEDED with response "%s"' %
                        (response.message))
                else:
                    node.get_logger().info(
                        'Service FAILED with response "%s"' %
                        (response.message))
            break

        node.call_service_gps()
        # node.call_service_local()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

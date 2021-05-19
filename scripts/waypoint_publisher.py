#!/usr/bin/python3

## Debug script for easy waypoint publishing

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import time as pythontime

from nav_msgs.msg import Path as NavPath
from fog_msgs.srv import Path as SetPath
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

WAYPOINTS=[
    [0,0,3],
    [-5,4,2],
    [7,-5,2],
    [0,0,2]
]

class PathPublisherNode(Node):

    def __init__(self):
        super().__init__("waypoint_publisher")
        self.publisher = self.create_publisher(NavPath, "/uav1/navigation/goto_waypoints", 10)
        self.client = self.create_client(SetPath, "/uav1/navigation/set_path")

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

    def call_service(self):
        path = NavPath()
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
        print('Calling service with payload: "%s"' % path.poses)
        path_req = SetPath.Request()
        path_req.path = path
        self.future = self.client.call_async(path_req)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()

    # rate = node.create_rate(30)
    # call the publisher a few times just to make sure
    # for i in range(30):
    #     node.publish()
    #     pythontime.sleep(0.05)
    #     rclpy.spin_once(node)

    node.call_service()
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
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

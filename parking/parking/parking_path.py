#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from sensor_msgs.msg import Imu
import math
import sys

from math import sqrt, atan2, degrees, pi, asin

sys.path.append("/home/ps/parking/src/parking/parking/")

import reeds_shepp_planner as rs
from utils import *


class FINDEMPTY(Node):
    def __init__(self):
        super().__init__("parking_path")
        qos_profile = QoSProfile(depth=10)

        # 현재 위치 map 기준
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )
        # 골 위치 받기 map 기준
        self.sub_goal_point = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_point_callback, qos_profile
        )
        # 방해물 위치 받기 map 기준
        self.sub_marker_array = self.create_subscription(
            MarkerArray, "/cone_points", self.obs_callback, qos_profile
        )

        self.sub_marker = self.create_subscription(
            Marker, "/max_length_marker", self.maxpoint_callback, qos_profile
        )

        # 차량 방향 필요할진 모르겠음
        self.sub_odometry = self.create_subscription(
            Imu, "/imu/rotated", self.odometry_callback, qos_profile
        )

        # 길 생성 map 기준?
        self.pub_path = self.create_publisher(Path, "/parking_path", qos_profile)

        self.pub_marker_array = self.create_publisher(
            MarkerArray, "/target_speed", qos_profile  # MarkerArray로 발행
        )

        self.start = None
        self.goal = None
        self.obs = None
        self.rotate = None
        self.point1 = None
        self.point2 = None
        self.obs_margin = 7.0
        self.i = 0

    def maxpoint_callback(self, data):
        self.point1 = np.array((data.points[0].x, data.points[0].y))
        self.point2 = np.array((data.points[1].x, data.points[1].y))

    def callback_local(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.start = np.array((x, y))

    def goal_point_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal = np.array((x, y))
        self.parallel_parking_path()

    def odometry_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        rotate = yaw * 180 / pi
        while True:
            if rotate < -180:
                rotate += 360

            if rotate > 180:
                rotate -= 360

            else:
                break

        self.rotate = rotate

    def obs_callback(self, data):
        points = []
        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            points.append((x, y))
        self.obs = np.array(points)

    def parallel_parking_path(self):
        if (
            self.start is None
            or self.goal is None
            or self.point1 is None
            or self.point2 is None
            or self.i == 1
        ):
            # self.get_logger().warn("Missing start, goal, or control points")
            return

        # self.i = 1 #반복 안할거면 필요

        rad = deg2rad(self.rotate - 90)
        rad2 = deg2rad(self.rotate + 135)

        offset_1 = self.obs_margin * np.sin(rad)
        offset_2 = self.obs_margin / 7 * np.cos(rad2)

        PATH = [
            (self.start[0], self.start[1], self.rotate),
            (self.point2[0] + offset_1, self.point2[1] + offset_1, self.rotate),
            (self.goal[0] + offset_2, self.goal[1] + offset_2, self.rotate),
        ]
        self.path_x = []
        self.path_y = []

        for i in range(len(PATH) - 1):
            start_x = float(PATH[i][0])
            start_y = float(PATH[i][1])
            start_yaw = deg2rad(PATH[i][2])

            goal_x = float(PATH[i + 1][0])
            goal_y = float(PATH[i + 1][1])
            goal_yaw = deg2rad(PATH[i + 1][2])

            min_radius = 2  # 자동차 회전 반경
            step_size = 0.8  # 선 만들때 점의 갯수

            x, y, yaw, mode, clen = rs.reeds_shepp_planner(
                start_x,
                start_y,
                start_yaw,
                goal_x,
                goal_y,
                goal_yaw,
                min_radius,
                step_size,
            )
            for x, y in zip(x, y):
                self.path_x.append(x)
                self.path_y.append(y)

        x = self.path_x
        y = self.path_y
        dx = np.diff(x)
        dy = np.diff(y)
        velocity = self.calculate_speed(dx, dy)

        self.publish_path(x, y, dx, dy, velocity)

    def calculate_speed(self, dx, dy):
        yaw = abs(1 - abs(np.arctan2(dy, dx)))
        speeds_kmh = np.clip(yaw * 5, 0, 10)
        return speeds_kmh

    def publish_path(self, x, y, dx, dy, velocity):
        path = Path()
        path.header.frame_id = "map"

        for i in range(len(x) - 1):
            pose = PoseStamped()
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            pose.pose.position.z = -1.0
            yaw = np.arctan2(dy[i], dx[i])
            q = self.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        self.pub_path.publish(path)

        marker_array = MarkerArray()
        for i in range(len(x) - 1):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "velocity"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.pose.position.x = x[i]
            marker.pose.position.y = y[i]
            marker.pose.position.z = -0.5
            marker.pose.orientation.w = 1.0

            if i < len(velocity):
                marker.text = str(round(velocity[i], 2))

            marker_array.markers.append(marker)
        self.pub_marker_array.publish(marker_array)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = FINDEMPTY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


import numpy as np
import math as m
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tf_transformations import *

import sqlite3


class DBWRITE(Node):
    def __init__(self):
        super().__init__("dbwrite")
        qos_profile = QoSProfile(depth=10)
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )

        self.path_x = []
        self.path_y = []
        self.path_cov = []
        self.path = []
        self.euclidean_list = []
        self.distance = 0
        self.ds = 0.1

    def callback_local(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        a = msg.pose.covariance

        cov = (
            a[0] + a[7]
        )  # 유의미한 분산 합(x,y,z_rot)  # 과연 yaw가 유의미한 분산일까?

        p1 = (x, y, cov)
        self.get_logger().info(f"detected_cone{p1}")

        if len(self.path_x) != 0:
            lx = self.path_x[-1]
            ly = self.path_y[-1]

            dist = m.sqrt((x - lx) ** 2 + (y - ly) ** 2)
            self.distance += dist

        if not self.euclidean_duplicate(p1):
            if len(self.path_x) != 0:
                dist = m.sqrt(
                    (p1[0] - self.path_x[-1]) ** 2 + (p1[1] - self.path_y[-1]) ** 2
                )  # 값 비교를 굳이 모든 리스트와 할 필요가 있을까?

            self.path_x.append(x)
            self.path_y.append(y)
            self.path_cov.append(cov)
        else:
            self.get_logger().info("p1 is duplicate")
            euc_dup_list = sorted(self.euclidean_list, key=lambda point: point[2])
            low_cov_point = euc_dup_list[0]
            if low_cov_point == p1:
                self.update_path_with_low_cov_point(euc_dup_list)
            self.euclidean_list.clear()  # Clear the list after handling duplicates

        if len(self.path_x) >= 3:  # 최소 3개의 포인트 필요
            self.interpolate_path()

    def euclidean_duplicate(self, p1):
        threshold = 1
        for x, y, cov in zip(self.path_x, self.path_y, self.path_cov):
            distance = m.sqrt((p1[0] - x) ** 2 + (p1[1] - y) ** 2)
            if distance <= threshold:
                self.euclidean_list.append((x, y, cov))
                return True
        return False

    def update_path_with_low_cov_point(self, euc_dup_list):
        low_cov_point = euc_dup_list[0]
        for x, y, cov in euc_dup_list[1:]:
            if x in self.path_x:
                self.path_x.remove(x)
            if y in self.path_y:
                self.path_y.remove(y)
        self.path_x.append(low_cov_point[0])
        self.path_y.append(low_cov_point[1])

    def interpolate_path(self):

        x = np.array(self.path_x)
        y = np.array(self.path_y)
        t = np.arange(len(x))  # Use indices as parameter t

        try:
            cs_x = CubicSpline(t, x, bc_type="natural")
            cs_y = CubicSpline(t, y, bc_type="natural")

            narrow = int(self.distance / self.ds)

            print(narrow)

            t_new = np.linspace(t[0], t[-1], narrow)
            x_new = cs_x(t_new)
            y_new = cs_y(t_new)

            dx_new = cs_x(t_new, 1)
            dy_new = cs_y(t_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]
            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
            self.write_db()  # Save the path to the database

        except Exception as e:
            self.get_logger().error(
                f"An error occurred during spline interpolation: {e}"
            )

    def write_db(self):
        db_file = "example.db"
        try:
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            cursor.execute(
                """
            CREATE TABLE IF NOT EXISTS data (
                id INTEGER PRIMARY KEY,
                value_x REAL NOT NULL,
                value_y REAL NOT NULL,
                yaw REAL NOT NULL
            )
            """
            )
            data = [(x, y, steer) for x, y, steer in self.path]
            for i, (x, y, yaw) in enumerate(data):
                cursor.execute(
                    "INSERT INTO data (id, value_x, value_y, yaw) VALUES (?, ?, ?, ?) ON CONFLICT(id) DO UPDATE SET value_x=excluded.value_x, value_y=excluded.value_y, yaw=excluded.yaw",
                    (i, x, y, yaw),
                )

            conn.commit()
        except sqlite3.Error as e:
            self.get_logger().error(
                f"An error occurred during database operations: {e}"
            )
        finally:
            self.get_logger().info(
                f"self.path_x:{self.path_x}, self.path_y:{self.path_y}"
            )
            conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = DBWRITE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

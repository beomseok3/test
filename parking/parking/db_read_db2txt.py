#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import matplotlib.pyplot as plt

import sqlite3

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler
import math as m


class DB_READER(Node):
    def __init__(self):
        super().__init__("db_reader")
        qos_profile = QoSProfile(depth=10)
        self.pub_path = self.create_publisher(Path, "path", qos_profile)
        self.db = []
        self.x = []
        self.y = []
        self.steer = []
        self.db_read()

    def db_read(self):
        db_file = "/home/ps/parking/example.db"
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM data")
        rows = cursor.fetchall()
        print(rows)
        for row in rows:
            self.db.append(row)
        conn.close()
        self.publish_txt()

    def publish_txt(self):
        with open("example.txt", "w") as file:
            speed = 3.0
            for i, x, y, steer in self.db:
                steer_deg = m.degrees(steer)
                file.write(f"{x}, {y}, {steer_deg}, {speed}\n")
            self.get_logger().info("file_write_done")

    def plot_path(self):
        plt.figure()
        plt.plot(self.x, self.y, "o-", label="Path")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Path Visualization")
        plt.legend()
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = DB_READER()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

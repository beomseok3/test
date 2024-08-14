# msg
from std_msgs.msg import Int32, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import *


# external_lib
import sqlite3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m
import sys
from ament_index_python.packages import get_package_share_directory

try:
    
    package_path = get_package_share_directory("parking")
    sys.path.append(package_path + "/src")
    from parking import line_ditection_area_addition
    from parking import avoidance
    
except Exception as e:
    print(e)



class STATE_MACHINE(Node):
    def __init__(self):
        super().__init__("state_machine")
        qos_profile = QoSProfile(depth = 10)
        
        #sub_stanley_target_idx //
        self.sub_target_idx = self.create_subscription(Int32,"target_idx",qos_profile)
        self.next = 0
        self.id = ["a1a2","a2b1","b1b2","b2c1","c1c2"]
        self.path = []
        self.pub_path = self.create_publisher(Path,"driving_path",qos_profile)
        self.path_timer = self.create_timer(1,self.callback_path)
        self.state=""
        self.path_db_read()
        self.state_timer = self.create_timer(1,self.state_machine,)
        
    def path_db_read(self):
        db_file = "state_machine_path.db"
        __conn = sqlite3.connect(db_file)
        cursor = __conn.cursor()
        # state recieve
        cursor.execute("SELECT x, y, yaw FROM Path WHERE id == ?",(self.id[self.next]))
        self.rows = cursor.fetchall()
        
        self.self.path = self.Path()
        self.path.header = Header()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = "map"
        for x, y, yaw in self.rows:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                self.path.poses.append(pose)
    
        
        if self.flag == "end":
            self.next += 1
            self.flag = "start"
    
    def state_machine(self):
        if self.flag == "end":
            if self.state == "driving":
                self.path_db_read()
            elif self.state == "parking":
                self.parking()
                pass
            elif self.state == "avoidance":
                self.avoidance()
                pass
            
    def callback_path(self):
        self.pub_path.publish(self.path)
        
    def parking(self):
        # import parking logic
        pass
    
    def avoidance(self):
        # import avoidance
        pass
        
    def publish_parameter(self):
        # publish (crop_box,clustering) parameter
        pass
    
    def switchPath(self):
        # When current path is almost end
        if len(self.path.cx) - 15 < self.target_idx:
                if self.target_idx == len(self.path.cx) -3:
                    self.flag = "end"
                    self.targe_idx = 0

def main(args=None):
    rclpy.init(args=args)
    node = STATE_MACHINE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

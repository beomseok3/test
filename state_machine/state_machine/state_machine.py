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
import time

try:
    
   
    sys.path.append("/home/ps/planning/src/parking/parking")
    # from 
    from line_detection_area_addition import PARKING
    from avoidance import AVOIDANCE
    
    
except Exception as e:
    print(e)



class STATE_MACHINE(Node):
    def __init__(self):
        super().__init__("state_machine")
        qos_profile = QoSProfile(depth = 10)
        
        #sub_stanley_target_idx //
        self.sub_target_idx = self.create_subscription(Int32,"target_idx",self.callback_stanley,qos_profile)
        self.next = 0
        self.id = ["a1a2","a2b1","b1b2","b2c1","c1c2"]
        self.path = []
        self.pub_path = self.create_publisher(Path,"driving_path",qos_profile)
        self.path_timer = self.create_timer(1,self.callback_path)
        self.state=""
        self.flag = ""
        self.target_idx =0
        self.path_db_read()
        self.state_timer = self.create_timer(0.1,self.state_machine,)
        
    def path_db_read(self):
        db_file = "state_machine_path.db"
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        # state recieve
        a= self.id[self.next]
        cursor.execute("SELECT value_x, value_y, yaw FROM Path WHERE id == ?",(a,))
        self.rows = cursor.fetchall()
        
        self.path = Path()
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
        
        conn.close()
        
    
    def state_machine(self):
        db_file = "state_machine_path.db"
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        _ = self.id[self.next]
        cursor.execute("SELECT mission FROM Node WHERE id == ?",(_,))
        self.state = cursor.fetchone()
        self.get_logger().info(f"{self.state}\n")
        if self.flag == "end":
            
            _ = self.id[self.next]
            cursor.execute("SELECT mission FROM Node WHERE id == ?",(_,))
            self.state = cursor.fetchone()
            self.get_logger().info(f"{self.state}\n")
            
            # self.flag = "start"
            if self.state == "driving":
                self.path_db_read()
            elif self.state == "parking":
                PARKING()
                while not self.parking == "done":
                    time.sleep(5)
                self.flag = "start"
                self. next += 1
            elif self.state == "avoidance":
                AVOIDANCE()
                while not self.avoidance == "done":
                    time.sleep(10)
                self.flag = "start"
                self.next += 1
                
                
        
        conn.close()
        
        
        
            
    def callback_path(self):
        self.switchPath()
        if self.state == "driving":
            self.pub_path.publish(self.path)
        
        
    def publish_parameter(self):
        # publish (crop_box,clustering) parameter
        pass
    
    def switchPath(self):
        # When current path is almost end
        self.get_logger().warn(f"{self.target_idx}")
        if len(self.path.poses) - 15 < self.target_idx and self.flag != 'end':
            self.get_logger().fatal("searching_path")
            if self.target_idx >= len(self.path.poses) -5:
                    self.flag = "end"
                    self.target_idx = 0
                    self.next += 1
                    self.get_logger().info("****************=======end=======*****************")
                    
    def callback_stanley(self,msg):
        self.target_idx = msg.data
        
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

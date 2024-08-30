#ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from erp42_msgs.msg import ControlMessage
from geometry_msgs.msg import PoseArray
from tf_transformations import *
#db
import sqlite3
from DB import DB
#utill
import numpy as np
import math as m
import time
from enum import Enum
#stanley
from stanley import Stanley

class PARKING_STATE(Enum):
    SEARCH = 0
    PARKING = 1
    FINISH =  2
    


class CONTROL_PARKING(Node):
    def __init__(self):
        super().__init__("control_parking")
        qos_profile = QoSProfile(depth=10)
        
        #parking_state_machine
        self.state = PARKING_STATE.SEARCH
        self.goal = 0
        
        #cone_pose_map
        self.sub_cone = self.create_subscription(
            PoseArray, "cone_pose_map", self.detection, qos_profile
        )
        #state_machine
        self.state_machine = self.create_timer(
            0.5,self.state_machine
        )
        
        #stanley
        self.st = Stanley()

        #search_path_initialize
        self.search_path = DB("state_machine_path.db")
        rows = self.search_path.read_db_n("data","value_x","value_y","yaw")
        self.path_cx = [row[0] for row in rows]
        self.path_cy = [row[1] for row in rows]
        self.path_cyaw = [row[2] for row in rows]
        print("\nsearch_path_is_loaded")
        
        #cmd_msgs
        self.gear = 2
        self.brake = 0
        self.estop = 0
        self.reverse_path = False

        #parking_path
        self.parking_path =[]
        self.goal_pose=[]
        self.start_pose=[]
        
        #idx_find
        self.end_idx = 0
        
        #detection_triger
        self.enough_low_y = 2.0
        
        
        # if self.goal_finish ==True:
        #     time.sleep(5)
        #     self.estop = 0
        #     self.path_cx =self.parking_path[:][0]
        #     self.path_cy =self.parking_path[:][1]
        #     self.path_cyaw =self.parking_path[:][2]
        # if len(self.path_cx) == target_idx :
        #     self.estop = 1
        #     self.gear = 2
        #     self.goal_finish =True
        #     #도착상태에서 기어만 바꾸고 같은 패스를 주면 어떻게 될까?
        # if len(self.path_cx) == target_idx and self.goal_finish:
        #     self.path_cx = self.row[0]
        #     self.path_cy = self.row[1]
        #     self.path_cyaw = self.row[2]
        #     self.state = "finish"
        
        # if self.parking_path:
        #     if target_idx == self.end_idx:
        #         self.path_cx = self.parking_path[::-1][0]
        #         self.path_cy = self.parking_path[::-1][1]
        #         self.path_cyaw = self.parking_path[::-1][2]
        #         self.gear = 0
        #         self.parking_path.clear()
        

    def state_machine(self):
        # 예시로, 목표 인덱스가 경로의 끝에 도달했는지 확인
        if self.target_idx >= len(self.path_cx) - self.goal:
            self.state = PARKING_STATE(self.state.value + 1)
            self.get_logger().info(f"State changed to {self.state.name}.")
        else:
            self.get_logger().info(f"Continuing in state: {self.state.name}.")

    def calculate_cmd(self,localization):
        
        steer,self.target_idx,_,_ = self.st.stanley_control(localization,self.path_cx,self.path_cy,self.path_cyaw,reverse=self.reverse_path)
        msg = ControlMessage()
        msg.speed = 5*10
        msg.steer = steer
        msg.gear = self.gear
        msg.brake = self.brake
        msg.estop = self.estop
        print(msg)
        return(self.state,msg)
    
        
    def detection(self,msg):
        if not self.detection_finish:
            for p1 in [(pose.position.x, pose.position.y) for pose in msg.poses]:
                if not self.low_y_cone:
                    
                    p1_rotated = self.rotate_points(np.array([p1]),self.angle,np.array([self.local_x,self.local_y])) # path pose로 고정
                    p1_rotated = tuple(p1_rotated[0])
                    
                    y_dis = abs(p1_rotated[1] - self.local_y)
                    
                    if y_dis <= self.enough_low_y:
                        
                        self.low_y_cone.append(p1_rotated)
                        
                        # reserve local
                        self.dangle = self.angle
                        self.dlocal_x = self.local_x # path_pose로 고정
                        self.dlocal_y = self.local_y
                        
                        #detection_area
                        self.min_x = p1_rotated[0] 
                        self.max_x = p1_rotated[0] + 16
                        self.min_y = p1_rotated[1] - 1.0 # path pose 를 기준으로 고정
                        self.max_y = p1_rotated[1] + 0.8
                            
                    else:
                        self.get_logger(f"{p1} is too long:{y_dis}")
                else:  
                    rotated_p1 = self.rotate_points(np.array([p1]), self.dangle, np.array([self.dlocal_x, self.dlocal_y])) #path_pose로 고정
                    rotated_p1 = tuple(rotated_p1[0])
                    if not self.euclidean_duplicate(rotated_p1):
                        if self.min_x <= rotated_p1[0] <= self.max_x and self.min_y <= rotated_p1[1] <= self.max_y:
                            self.low_y_cone.append(rotated_p1)
                            for i in range(len(self.low_y_cone) - 1):
                                dist = self.low_y_cone[i+1][0] - self. low_y_cone[i][0]
                                if dist > 4.0:
                                    self.idx = i # cone_idx
                                    self.calc_path()
                                    break
                        else:
                            self.get_logger().info(f"out of detection_area: {p1}")
                    else:
                        self.get_logger().info(f"{p1} is duplicate")
        else:
            self.get_logger().info("detection_finish!!!")
            
    def calc_path(self):
        db_path = "/home/ps/planning/src/state_machine/db_file/school_gjs.db"
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute('''
                       SELECT value_x, value_y, yaw FROM data
                       ''')
        rows=cursor.fetchall()
        self.parking_path = rows
        
        self.goal_pose = self.parking_path[0][:]
        self.start_pose=self.parking_path[-1][:]
        
        goal_pose_o = self.rotate_points(np.array([self.low_y_cone[self.idx][0] + 1,self.low_y_cone[self.idx][1] - 1.5]),-self.dangle,np.array([self.dlocal_x,self.dlocal_y]))
        dx = goal_pose_o[0]   - self.goal_pose[0]
        dy = goal_pose_o[1] - self.goal_pose[1]
        
        self.parking_path = np.array(self.parking_path)
        
        #rotation
        self.parking_path[:,0] += dx
        self.parking_path[:,1] += dy  
        #translation
        angle = self.dangle - self.start_pose[2]
        self.parking_path[:,:2] = self.rotate_points(self.parking_path[:,:2],angle,self.goal_pose)
        
        #idx찾기
        a = self.search_path.find_idx(self.start_pose[0],self.start_pose[1],"data")
        self.goal = len(self.path_cx) - a
    
    
    
    
    
    
    
    
    def rotate_points(self, points, angle, origin):
        angle_radians = -angle # 반 시계방향
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians)],
                [np.sin(angle_radians), np.cos(angle_radians)],
            ]
        )
        
        # 원점으로 평행이동
        translated_points = points - origin
        # 각 만큼 회전
        rotated_points = np.dot(translated_points, rotation_matrix.T)
        # 회전 중심 위치로 평행이동
        rotated_points += origin
        
        return rotated_points
    
    def euclidean_duplicate(self, p1):
        threshold = 0.8
        for p2 in self.low_y_cone:
            # print(p1)
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False
    


    

def main(args=None):
    rclpy.init(args=args)
    node = CONTROL_PARKING()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

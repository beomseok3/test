# msg
from std_msgs.msg import Int32

# external_lib
import sqlite3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m

class STATE_MACHINE(Node):
    def __init__(self):
        super().__init__("state_machine")
        qos_profile = QoSProfile(depth = 10)
        
        #sub_stanley_target_idx //
        self.sub_target_idx = self.create_subscription(Int32,"target_idx",qos_profile)
        
        
    def path_db_read(self):
        # read_path and apply the state and logic
        pass
        db_file = "exaple.db"
        __conn = sqlite3.connect(db_file)
        cursor = __conn.cursor()
        cursor.execute("SELECT idx,id FROM Path WHERE")
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

            rospy.logwarn("Try to change path...!")

            current_time = rospy.Time.now()
            dt = (current_time - self.request_time).to_sec()

            # prevent duplicated request
            if dt > 1:
                # not end
                if self.selector.path.next is not None:
                    self.request_time = rospy.Time.now()
                    self.selector.goNext()
                    self.selector.makeRequest()

                    # next path is end
                    if self.selector.path.next is None:
                        self.mission_state = MissionState.END
                        rospy.loginfo("Missin State : End")

                    # currnet path is not end
                    else:
                        self.mission_state = self.selector.path.mission_type
                        rospy.loginfo("Mission State : %s" %
                                      str(self.mission_state))

                self.target_idx = 0
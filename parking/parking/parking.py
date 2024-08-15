import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped 
from tf_transformations import *

from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Quaternion

class PARKING(Node):
    def __init__(self):
        super().__init__("parking")
        qos_profile = QoSProfile(depth=10)
        self.sub_cone = self.create_subscription(PoseArray,"cone_pose_map",self.callback_cone,qos_profile)
        self.sub_local = self.create_subscription(Odometry,"localization/kinematic_state",self.callback_local,qos_profile)
        self.pub_path = self.create_publisher(Path,"path",qos_profile)

        self.cone = []
        self.i=1
        self.dis = []
        self.local_x =0 
        self.local_y = 0
        self.domain = []
        self.rotated_domain = []
        self.angle = 0
        self.origin = np.array([])
        self.path_x =[]
        self.path_y =[]
        self.path =[]

    def callback_cone(self,msg):
        for (pose_x,pose_y) in [(pose.position.x, pose.position.y) for pose in msg.poses]: 
                if len(self.cone) <17:
                    p1=(pose_x,pose_y)
                    if not self.euclidean_duplicate(p1):
                        self.cone.append((pose_x,pose_y))
                        print(len(self.cone),self.cone,"\n"*8)        
        
    def callback_local(self,msg):
        if self.i<2:
            if len(self.cone) == 17 :
                print(msg.pose)
                self.i=self.i + 1
                data = self.cone
                points = np.array(data)
                quarternion=[0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
                _,_,self.angle = euler_from_quaternion(quarternion)  # degrees (negative for clockwise rotation)
                self.origin = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])  # rotation center
                rotated_points = self.rotate_points(points, self.angle, self.origin)
                self.local_x = msg.pose.pose.position.x
                self.local_y = msg.pose.pose.position.y
                self.indexing(rotated_points)


    def indexing(self,data):
        # Cluster data based on y values only
        y_values = data[:, 1].reshape(-1, 1)

        # Fit KMeans with 3 clusters
        kmeans_y = KMeans(n_clusters=3, random_state=42)
        kmeans_y.fit(y_values)

        # Get cluster labels
        labels_y = kmeans_y.labels_
        cluster_means = np.array([y_values[labels_y == i].mean() for i in range(3)])
        sorted_cluster_indices = np.argsort(cluster_means)
        label_mapping = {old_label: new_label for new_label, old_label in enumerate(sorted_cluster_indices)}
        sorted_labels_y = np.array([label_mapping[label] for label in labels_y])

        # Add sorted cluster labels to the original data
        clustered_data_y_sorted = np.hstack((data, sorted_labels_y.reshape(-1, 1)))

        # Convert to DataFrame for better display
        clustered_data_y_sorted_df = pd.DataFrame(clustered_data_y_sorted, columns=['X', 'Y', 'Cluster'])

        # Convert each cluster to a sorted list of tuples
        cluster_0 = clustered_data_y_sorted_df[clustered_data_y_sorted_df['Cluster'] == 0][['X', 'Y']].sort_values(by='X').values.tolist()
        cluster_1 = clustered_data_y_sorted_df[clustered_data_y_sorted_df['Cluster'] == 1][['X', 'Y']].sort_values(by='X').values.tolist()
        cluster_2 = clustered_data_y_sorted_df[clustered_data_y_sorted_df['Cluster'] == 2][['X', 'Y']].sort_values(by='X').values.tolist()
                # Plot the clustered data based on y values
        plt.figure(figsize=(10, 6))
        colors = ['red', 'blue', 'green']
        for i in range(3):
            plt.scatter(clustered_data_y_sorted[clustered_data_y_sorted[:, 2] == i][:, 0], 
                        clustered_data_y_sorted[clustered_data_y_sorted[:, 2] == i][:, 1], 
                        color=colors[i], label=f'Cluster {i}')
        plt.xlabel('X values')
        plt.ylabel('Y values')
        plt.title('KMeans Clustering of Data Points Based on Y Values')
        plt.legend()
        plt.grid(True)
        plt.show()

        for i in range(5):
            self.dis.append(cluster_2[i+1][0]-cluster_2[i][0])
        max_dis_idx=self.dis.index(max(self.dis))
        if max_dis_idx == 0:
            #domain_point
            print("case1")
        elif max_dis_idx == 2:
            #domain_point
            print("case2")
            if abs(self.local_x - cluster_2[5][0]) <= abs(self.local_x - cluster_2[0][0]):
                alpha = abs(self.local_y - cluster_2[3][1])
                self.domain.extend([((cluster_0[3][0]+cluster_0[2][0])/2,cluster_1[2][1]), ( ( cluster_0[3][0] + cluster_0[4][0] ) / 2 , ( cluster_2[3][1]+cluster_1[3][1] ) / 2), ( cluster_2[3][0] , cluster_2[3][1]+alpha-0.5 ),  ( self.local_x,self.local_y ) ])
                points = np.array(self.domain)
                self.rotated_domain = self.rotate_points(points, -self.angle, self.origin)
                self.path_x = self.rotated_domain[:,0]
                self.path_y = self.rotated_domain[:,1]
                self.interpolate_path()
            else:
                print('hello')

        else:
            print("case3")

    
    def interpolate_path(self):
        x = np.array(self.path_x)
        y = np.array(self.path_y)
        t = np.arange(len(x))  # Use indices as parameter t

        try:
            cs_x = CubicSpline(t, x, bc_type='natural')
            cs_y = CubicSpline(t, y, bc_type='natural')

            t_new = np.linspace(t[0], t[-1], 150)
            x_new = cs_x(t_new)
            y_new = cs_y(t_new)

            dx_new = cs_x(t_new, 1)
            dy_new = cs_y(t_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]
            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
            self.publish_path()

        except Exception as e:
            self.get_logger().error(f"An error occurred during spline interpolation: {e}")
        
    def publish_path(self):
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id='map'
        for x,y,steer in self.path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            quaternion = quaternion_from_euler(0,0,steer)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path.poses.append(pose)
        self.pub_path.publish(path)




    def rotate_points(self,points, angle, origin):
        angle_radians = -angle
        rotation_matrix = np.array(
            [[np.cos(angle_radians), -np.sin(angle_radians)],
            [np.sin(angle_radians), np.cos(angle_radians)]])
        # Translate points to origin
        translated_points = points - origin
        # Rotate points
        rotated_points = np.dot(translated_points, rotation_matrix.T)
        # Translate points back
        rotated_points += origin
        return rotated_points




    def euclidean_duplicate(self, p1):
        threshold = 0.3
        for p2 in self.cone:
            distance = m.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            if distance <= threshold:
                return True
        return False
    
    # def path_pub(self):



def main(args=None):
    rclpy.init(args=args)
    node = PARKING()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

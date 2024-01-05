import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from tf2_geometry_msgs import PoseStamped as TfPoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_ros
import tf2_py as tf2
#import tf2_geometry_msgs
import math
#from transforms3d.euler import euler2quat
#from tf_transformations import euler_from_quaternion
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray


class PotentialField(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        #* Parameter 불러오기

        # self.getParam()

        self.odom_subscription  = self.create_subscription(PoseStamped, '/orin/current_pose', self.odom_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/orin/scan', self.scan_callback,10)
        self.odom_subscription
        self.lidar_subscription # 사용하지 않는 변수 경고 방지?

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.att_pub = self.create_publisher(PoseStamped, 'attraction_vector', 1)
        self.rep_pub = self.create_publisher(PoseStamped, 'repulsion_vector', 1)
        self.fin_pub = self.create_publisher(PoseStamped, 'Final_Vector', 1)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # x_goal = self.goals[0][0]
        # y_goal = self.goals[0][1]
        x_goal = 5
        y_goal = 0

        self.theta = None
        
        self.get_logger().info(f'x = {x_goal} and y = {y_goal}')
        self.goal_x = float(x_goal)
        self.goal_y = float(y_goal)

    # def getParam(self):

        # 노드의 파라미터 서버에서 파라미터 가져오기
        # self.goals                  = self.get_parameter('/nav_info/goals').value
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert euler angles (roll, pitch, yaw) into a quaternion
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return w, x, y, z

    def quaternion_to_euler(self, w, x, y, z):
        """
        Convert a quaternion into euler angles (pitch, roll, yaw)
        """
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.pi / 2 * np.sign(sinp) # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw  # in radians

    def controller(self):
        x_final = self.V_attraction[0] + self.V_repulsion[0]
        y_final = self.V_attraction[1] + self.V_repulsion[1]

        final_vector = self.publish_vector(x_final, y_final)
        self.fin_pub.publish(final_vector)

        direction = Twist()

        tolerance = 0.2
        angle = math.atan2(y_final, x_final)
        delta = math.pi - abs(math.fmod(abs(angle - self.theta), 2 * math.pi) - math.pi)

        self.get_logger().info(f'angle to goal: {delta}')

        if delta < 0 - tolerance:
            direction.angular.z = -0.2
            direction.linear.x = 0.0
        elif delta > 0 + tolerance:
            direction.angular.z = 0.2
            direction.linear.x = 0.0
        else:
            direction.linear.x = 0.1
            direction.angular.z = 0.0

        self.cmd_pub.publish(direction)

    def publish_vector(self, x, y):
        vector = PoseStamped()
        vector.header.frame_id = '/os_lidar'
        vector.header.stamp = self.get_clock().now().to_msg()
        vector.pose.position.x = self.x_odom
        vector.pose.position.y = self.y_odom
        vector.pose.position.z = 0.0

        angle = math.atan2(y, x)
        quaternion = self.euler_to_quaternion(0, 0, angle)
        vector.pose.orientation = Quaternion(w=quaternion[0], x=quaternion[1], y=quaternion[2], z=quaternion[3])
        return vector

    def compute_attraction(self, x_a, y_a):
        self.get_logger().info(f'GOAL | x : {x_a} | y : {y_a}')
        distance = math.sqrt((x_a - self.x_odom)**2 + (y_a - self.y_odom)**2)
        x_a -= self.x_odom
        y_a -= self.y_odom

        Q_attraction = 100
        F_attraction = Q_attraction / (4 * math.pi * distance**2)
        self.V_attraction = [F_attraction * x_a, F_attraction * y_a]

        attraction_vector = self.publish_vector(self.V_attraction[0], self.V_attraction[1])
        self.att_pub.publish(attraction_vector)

    def odom_callback(self, msg):
        self.x_odom = msg.pose.position.x
        self.y_odom = msg.pose.position.y
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w)
        print(quaternion[0],quaternion[1])
        _, _, self.theta = self.quaternion_to_euler(quaternion[3],quaternion[0],quaternion[1],quaternion[2])

        self.compute_attraction(self.goal_x, self.goal_y)

    def scan_callback(self, msg):

        if (self.theta is None):
            print("none")
            return


        angle_min = msg.angle_min
        step = msg.angle_increment
        
        scan = msg.ranges
        counter = 0
        x_r = 0
        y_r = 0
        
        
        for i in range(len(scan)):
            if 0.1 < scan[i] < 3:
                Q_repulsion = -0.3
                current_q = Q_repulsion / (4 * math.pi * scan[i]**2)  ## math.pi * scan[i]**2 --> 거리를 반지름으로 하는 원의 면적 계산 * 4 = 전체 공간 
                ## 척력 계수 Q_repulsion를 전체 공간에 나누어서, 척력의 크기를 해당 거리에 대한 공간 면적으로 정규화

                print(x_r, y_r, "rrrrrrrrrrrrrrrrr")

                print(current_q," current_q")
                print(math.cos(angle_min + self.theta + step * i))

                x_r -= current_q * math.cos(angle_min + self.theta + step * i)
                y_r -= current_q * math.sin(angle_min + self.theta + step * i)

                print(x_r,y_r ," after")
            else:
                counter += 1

        if counter == 360:
            self.V_repulsion = [0.0001, 0.000000000001]
        else:
            self.V_repulsion = [x_r, y_r]

        print(self.V_repulsion,"reeeeeeeeeeeeee")

        repulsion_vector = self.publish_vector(self.V_repulsion[0], self.V_repulsion[1])
        self.rep_pub.publish(repulsion_vector)
        self.controller()


def main(args=None):
    rclpy.init(args=args)
    #node = PotentialField(args[1], args[2])
    node = PotentialField()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
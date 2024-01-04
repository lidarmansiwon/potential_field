import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf2_geometry_msgs import PoseStamped as TfPoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_ros
import tf2_geometry_msgs
import math

class PotentialField(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        #* Parameter 불러오기

        # self.getParam()

        self.odom_subscription  = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback,10)
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
        x_goal = 3
        y_goal = 4
        
        self.get_logger().info(f'x = {x_goal} and y = {y_goal}')
        self.goal_x = float(x_goal)
        self.goal_y = float(y_goal)

    # def getParam(self):

        # 노드의 파라미터 서버에서 파라미터 가져오기
        # self.goals                  = self.get_parameter('/nav_info/goals').value



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
            direction.linear.x = 0
        elif delta > 0 + tolerance:
            direction.angular.z = 0.2
            direction.linear.x = 0
        else:
            direction.linear.x = 0.1
            direction.angular.z = 0

        self.cmd_pub.publish(direction)

    def publish_vector(self, x, y):
        vector = PoseStamped()
        vector.header.frame_id = '/odom'
        vector.header.stamp = self.get_clock().now().to_msg()
        vector.pose.position.x = self.x_odom
        vector.pose.position.y = self.y_odom
        vector.pose.position.z = 0

        angle = math.atan2(y, x)
        quaternion = tf2_ros.transformations.quaternion_from_euler(0, 0, angle)
        vector.pose.orientation = tf2_geometry_msgs.Quaternion(*quaternion)

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
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.theta = tf2_ros.transformations.euler_from_quaternion(quaternion)

        self.compute_attraction(self.goal_x, self.goal_y)

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        step = msg.angle_increment
        scan = msg.ranges
        counter = 0
        x_r = 0
        y_r = 0

        for i in range(len(scan)):
            if 0.1 < scan[i] < 100:
                Q_repulsion = 1
                current_q = Q_repulsion / (4 * math.pi * scan[i]**2)
                x_r -= current_q * math.cos(angle_min + self.theta + step * i)
                y_r -= current_q * math.sin(angle_min + self.theta + step * i)
            else:
                counter += 1

        if counter == 360:
            self.V_repulsion = [0.0001, 0.000000000001]
        else:
            self.V_repulsion = [x_r, y_r]

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
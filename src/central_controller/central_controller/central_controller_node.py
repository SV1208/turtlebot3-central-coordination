import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt

THRESHOLD_DISTANCE = 2.6

class CentralController(Node):
    def __init__(self):
        super().__init__('centrol_controller')
        self.navigator_tb1 = BasicNavigator(namespace="tb1")
        self.navigator_tb2 = BasicNavigator(namespace='tb2')

        self.tb1_pose = None
        self.tb2_pose = None

        self.tb1_goal_pose = None
        self.tb2_goal_pose = None

        self.create_subscription(PoseStamped, "/tb1/goal_pose", self.tb1_goal_pose_callback, 10)
        self.create_subscription(PoseStamped, "/tb2/goal_pose", self.tb2_goal_pose_callback, 10)

        self.create_subscription(Odometry, '/tb1/odom', self.tb1_odom_callback, 10)
        self.create_subscription(Odometry, '/tb2/odom', self.tb2_odom_callback, 10)

        self.timer = self.create_timer(1.0, self.control_loop)

        self.tb1_moving = False
        self.tb2_moving = False

        self.tb1_goal_sent = False
        self.tb2_goal_sent = False

        self.stopped_tb2 = False
        self.count = 0
        self.jao = True
        self.prev_distance = None

        self.get_logger().info("Central Controller Started")
    
    def create_goal_pose(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 0.0
        self.prev_dx = None
        self.prev_dy = None

        return goal


    def control_loop(self):
        if self.tb1_pose ==None and self.tb2_pose ==None:
            self.get_logger().info("waiting for robot pose")
            return
        
        tb1_goal = self.create_goal_pose(-1.1, -2.8)
        tb2_goal = self.create_goal_pose(3.3, 1.0)

        self.navigator_tb1.goToPose(tb1_goal)

        # if (self.count>20):
        #     self.navigator_tb2.goToPose(tb2_goal)
        # self.count+=1
        # self.get_logger().info("count: "+str(self.count))
        if self.jao:
            self.navigator_tb2.goToPose(tb2_goal)



        # this control loop runs when both the robot has a goal pose
        # if self.tb1_goal_pose ==None and self.tb2_goal_pose ==None:
        #     self.get_logger().info("waiting for robot goal pose")
        #     return
        dx = self.tb1_pose.pose.position.x - self.tb2_pose.pose.position.x
        dy = self.tb1_pose.pose.position.y - self.tb2_pose.pose.position.y
        distance = abs(dx) + abs(dy)

        if self.prev_distance == None:
            self.prev_distance =distance
        

        change = distance - self.prev_distance

        self.prev_distance = distance
        
        # if self.prev_dx == None or self.prev_dy == None:
        #     self.prev_dx = dx
        #     self.prev_dy = dy
                   


        self.get_logger().info("Distance between tb1 and tb2: "+str(round(distance,2)))
        self.get_logger().info("Change: "+str(round(change,3)))
        
        if (distance < THRESHOLD_DISTANCE):
            # if (abs(dx) < THRESHOLD_DISTANCE or abs(dy)<THRESHOLD_DISTANCE):
            # when too close, stop tb2 and let tb1 move
            # self.get_logger().info("Distance below threshold: "+str(THRESHOLD_DISTANCE))
            # if self.tb2_moving:
                # self.stopped_tb2 = True
            # self.get_logger().info("Cancelling tb2 task")
            self.jao = False
        else:
            self.jao = True

        if (change > 0.0):
            self.jao = True
        
        if (not self.jao):
            self.navigator_tb2.cancelTask()
            self.get_logger().info("Cancelled")
        # else:
        #     if not self.tb2_moving and self.stopped_tb2:
        #         self.get_logger().info("Again giving the goal to tb2")
        #         tb2_goal_pose = self.tb2_goal_pose
        #         self.navigator_tb2.goToPose(tb2_goal_pose)
        #         self.stopped_tb2 = False
    
    def tb1_goal_pose_callback(self, msg):
        self.tb1_goal_pose = msg
    
    def tb2_goal_pose_callback(self, msg):
        self.tb2_goal_pose = msg
    
    def tb1_odom_callback(self, msg: Odometry):
        self.tb1_pose = msg.pose
    
    
    def tb2_odom_callback(self, msg: Odometry):
        self.tb2_pose = msg.pose
        any_linear_speed = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.linear.y)
        angular_speed = abs(msg.twist.twist.angular.z)

        if (any_linear_speed<0.01 and angular_speed<0.01):
            self.tb2_moving = False
        else:
            self.tb2_moving = True

        

def main(args=None):
    rclpy.init(args=args)
    node = CentralController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
# Need to start the map with this command: ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=/path/to/your/map.yaml

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

class myCommander(Node):
    def __init__(self):
        super().__init__('nav node')
        self.nav = BasicNavigator()

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.w = 1.0
        self.nav.setInitialPose(init_pose)

        self.nav._setInitialPose(init_pose)
        self.nav.waitUntilNav2Active() # waits for the nav stuff to start
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.orientation.w = 1.0
        
        path_goals = nav.getPathThroughPoses(init_pose, goal_pose)
        self.get_logger().info(f"Got path with {len(path_msg.poses)} poses")

def main(args=None):
    rclpy.init(args=args)
    node = MyCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

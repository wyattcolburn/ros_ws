from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy

class MyCommander:
    def __init__(self):
        self.nav = BasicNavigator()
        
        # Wait for Nav2 to come up
        self.nav.waitUntilNav2Active()
        
        # Set initial pose
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.5
        init_pose.pose.orientation.w = 1.0
        self.nav.setInitialPose(init_pose)

        self.nav.lifecycleStartup()  # Optional but useful for ensuring lifecycle activation

        self.run(init_pose)

    def run(self, init_pose):
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.6
        goal_pose.pose.orientation.w = 1.0

        # Get path
        path = self.nav.getPath(init_pose, goal_pose)
        local_goals = []
        if path is not None and len(path.poses) > 0:
            for pose in path.poses:
                local_goals.append(pose.pose)
                print(f"poses from getPath {pose.pose}")
            print(f"Found {len(local_goals)} waypoints in the path.")
        else:
            print("No path was found.")

def main(args=None):
    rclpy.init(args=args)

    # Create MyCommander and let it handle everything
    commander = MyCommander()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


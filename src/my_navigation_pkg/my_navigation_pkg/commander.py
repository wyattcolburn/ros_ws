from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from transforms3d.euler import quat2euler

from std_msgs.msg import String

class MyCommander(Node):
    def __init__(self):
        
        super().__init__("command_node")
        self.publisher_ = self.create_publisher(String, "local_goals", 10)
        self.timer_ = self.create_timer(10, self.timer_callback)
        self.nav = BasicNavigator()
        
        # Wait for Nav2 to come up
        self.nav.waitUntilNav2Active()
        
        # Set initial pose, bullshit values but still
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.5
        init_pose.pose.orientation.z = 1.0
        self.nav.setInitialPose(init_pose)

        self.nav.lifecycleStartup()  

        self.message = self.run(init_pose)
    def timer_callback(self):
        self.publisher_.publish(self.message)
    
    def run(self, init_pose):
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.6 # no idea how we would do this dynamically
        goal_pose.pose.position.y = 1.6
        init_pose.pose.orientation.z = 1.0

        # Get path
        path = self.nav.getPath(init_pose, goal_pose)
        local_goals = []
        local_goals_msg = ""
        if path is not None and len(path.poses) > 0:
            for pose in path.poses:
                local_goals.append(pose.pose)
                print(f"poses from getPath {pose.pose}")
                q = pose.pose.orientation
                yaw, pitch, roll = quat2euler([q.w, q.x, q.y, q.z])


                #if local_goals_msg:
                #    local_goals_msg += "|" 
                local_goals_msg += f"{pose.pose.position.x}|{pose.pose.position.y}|{yaw}|"

                print(local_goals_msg)
            print(f"Found {len(local_goals)} waypoints in the path.")
        else:
            print("No path was found.")

        msg = String()
        msg.data = local_goals_msg
        return msg 
        # sample mesasge which needs to be filtered
        #geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=2.2351741790771484e-07, y=0.5500002361834042, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))


def main(args=None):
    rclpy.init(args=args)

    # Create MyCommander and let it handle everything
    commander = MyCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


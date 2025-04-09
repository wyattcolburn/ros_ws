import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from matplotlib import pyplot
from matplotlib import patches
import math

class local_goal:
    def __init__(self, center_x, center_y, yaw):
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.yaw = float(yaw)

class Obstacle:
    def __init__(self, center_x, center_y):
        self.center_x = center_x
        self.center_y = center_y

class Visual(Node):
    def __init__(self):
        super().__init__("Visual")
        self.lg_subscription_ = self.create_subscription(Path, "/plan", self.lg_callback, 10)
        self.middle_man_subscription = self.create_subscription(Float64MultiArray, "neuralNetInput", self.middle_man_callback, 10)
        self.ODOM_COUNT = 4
        self.LIDAR_COUNT = 640

        self.lg_list = []
        self.ob_list=[]

        self.lidar_data=[]
        self.latest_middleman_msg = None

        self.odom_x = 0
        self.odom_y = 0

    def lg_callback(self, msg):

        print("I HEAR PLAN MESSAGES ***********************")
        print(f"len of msg.poses is : {len(msg.poses)}")
        for pose in msg.poses:


            goal = local_goal(pose.pose.position.x, pose.pose.position.y, 1)
            self.lg_list.append(goal) 
        self.plot_lg()
    def middle_man_callback(self, msg):

        self.get_logger().info("I hear middle man data")
        len_msg_data = len(msg.data)
        self.get_logger().info('Len of msg_data is "%d"' % len_msg_data)

        self.odom_x = msg.data[0]
        self.odom_y = msg.data[1]
        self.latest_middleman_msg = msg
        for i in range(self.ODOM_COUNT, self.LIDAR_COUNT + self.ODOM_COUNT):
            self.lidar_data.append(msg.data[i])

        for i in range(self.ODOM_COUNT + self.LIDAR_COUNT, len_msg_data,2):
            obs = Obstacle(msg.data[i], msg.data[i+1])
            self.ob_list.append(obs)

            print(f'obs {i/2} is at coord_x {obs.center_x} and coord_y {obs.center_y}')
        self.plot_lg()


        return

    def plot_lg(self):

        print(f"len of lg_list {len(self.lg_list)} and len of ob_list {len(self.ob_list)}")
        if not self.lg_list:
            return
        if not self.ob_list:
            return
        figure = pyplot.figure(figsize=(14, 10))
        ax = figure.add_subplot(1, 1, 1)

        for goal in self.lg_list:
            ax.plot(goal.center_x, goal.center_y, 'bo')  # Draw goal center as blue dot

        ax.plot(self.odom_x, self.odom_y, 'yo')
            # kDraw yaw as arrow
            #dx = 0.5 * math.cos(goal.yaw)
            #dy = 0.5 * math.sin(goal.yaw)
            #ax.arrow(goal.center_x, goal.center_y, dx, dy,
            #         head_width=0.1, head_length=0.2, fc='red', ec='red')

        ax.set_title("Local Goals")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True)
        ax.axis('equal')
       

        #obs1 = self.ob_list[0]
        #obs2 = self.ob_list[1]
        #print(f"ob1 is {obs1.center_x} and {obs1.center_y} ob2 is {obs2.center_x, obs2.center_y}")
        #circle = patches.Circle((obs1.center_x, obs1.center_y), radius=2,
        #                            edgecolor='red', facecolor='none', linewidth=2)
        #circle1 = patches.Circle((obs2.center_x, obs2.center_y), radius=2,
        #                            edgecolor='red', facecolor='none', linewidth=2)
        #ax.add_patch(circle)
        #ax.add_patch(circle1)
        for obs in self.ob_list[0:6]:
            circle = patches.Circle((obs.center_x, obs.center_y), radius=1,
                                        edgecolor='red', facecolor='none', linewidth=2)
            ax.add_patch(circle)


        angle_incrementor = (2* math.pi)/ self.LIDAR_COUNT
        origin_point = (self.latest_middleman_msg.data[0], self.latest_middleman_msg.data[1])
        print(f"origin point is {origin_point}") 
        for i in range(len(self.lidar_data)):
            angle = angle_incrementor* i
            dx = math.cos(angle) * self.lidar_data[i]
            dy = math.sin(angle) * self.lidar_data[i]

            projected_point = (origin_point[0] + dx, origin_point[1] + dy)

            pyplot.plot([origin_point[0], projected_point[0]], [origin_point[1], projected_point[1]])



        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        pyplot.show()




def main(args=None):
    rclpy.init(args=args)
    sub = Visual()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


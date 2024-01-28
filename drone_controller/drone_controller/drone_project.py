import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.a = True
        self.x = 1.0
        self.y = 1.0
        # Current pose subscriber
        self.gt_pose_sub = self.create_subscription(
            Pose,
            '/drone/gt_pose',
            self.pose_callback,
            1)

        self.gt_pose = None

        # Control command publisher
        self.command_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Callback for executing a control commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Feel fre to fill with your code! Add some objects to represent a goal points to achieve

    def pose_callback(self, data):
        self.gt_pose = data
        print(f"{data}")

    def timer_callback(self):
        # HINT: Check a current pose. Use it to check if a drone achieved a desired pose.
        print(f"Current pose: {self.gt_pose}")
        # HINT: Use a self.command_pub to publish a command
        one = [1.0, 1.0]
        two = [1.0, 5.0]
        three = [5.0, 5.0]
        four = [5.0, 1.0]
        msg = Twist()
        msg.linear.z = 7.0
        xpos = 0.0
        ypos = 0.0
        if self.gt_pose is not None:
            xpos = self.gt_pose.position.x
            ypos = self.gt_pose.position.y
        if xpos < 1.1 and ypos < 1.1:
            self.x = two[0]
            self.y = two[1]
            b = 1
        if xpos < 1.1 and ypos > 4.9:
            self.x = three[0]
            self.y = three[1]
        if xpos > 4.9 and ypos > 4.9:
            self.x = four[0]
            self.y = four[1]
        if xpos > 4.9 and ypos < 1.1:
            self.x = one[0]
            self.y = one[1]

        msg.linear.x = self.x
        msg.linear.y = self.y
        self.command_pub.publish(msg)
        # Fill with your code!
        print("Published!")


def main(args=None):
    rclpy.init(args=args)

    node = DroneController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
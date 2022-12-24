import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from create_msgs.msg import Cliff


class AndanteCmdVelSmoother(Node):
    def __init__(self):
        super().__init__("andante_cmd_vel_smoother")

        self.declare_parameter("rate", 30)
        self.declare_parameter("cutoff_freq", 10)
        self.declare_parameter("max_linear_acc", 0.5)
        self.declare_parameter("max_angler_acc", 1.0)

        self.cutoff_freq = self.get_parameter("cutoff_freq").value
        self.max_linear_acc = self.get_parameter("max_linear_acc").value
        self.max_angler_acc = self.get_parameter("max_angler_acc").value

        self.ref_cmd_vel = None
        self.cmd_vel = Twist()
        self.pre_time = self.get_clock().now()

        self.create_subscription(
            Twist, "cmd_vel_unfiltered", self.cmd_vel_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.freq = self.get_parameter("rate").value
        self.create_timer(
            1.0 / self.freq, self.cmd_vel_smoother
        )

    def cmd_vel_callback(self, msg):
        self.ref_cmd_vel = msg

    def cmd_vel_smoother(self):
        if self.ref_cmd_vel is None:
            return

        filtered_linear_x = self.calc_lpf(
            self.cmd_vel.linear.x, self.ref_cmd_vel.linear.x)
        filtered_linear_y = self.calc_lpf(
            self.cmd_vel.linear.y, self.ref_cmd_vel.linear.y)
        filtered_angular_z = self.calc_lpf(
            self.cmd_vel.angular.z, self.ref_cmd_vel.angular.z)

        time_diff = (self.get_clock().now() -
                     self.pre_time).nanoseconds / 1000000000

        # clamp
        if filtered_linear_x < self.cmd_vel.linear.x - self.max_linear_acc * time_diff:
            filtered_linear_x = self.cmd_vel.linear.x - self.max_linear_acc * time_diff
        if filtered_linear_x > self.cmd_vel.linear.x + self.max_linear_acc * time_diff:
            filtered_linear_x = self.cmd_vel.linear.x + self.max_linear_acc * time_diff
        if filtered_linear_y < self.cmd_vel.linear.y - self.max_linear_acc * time_diff:
            filtered_linear_y = self.cmd_vel.linear.y - self.max_linear_acc * time_diff
        if filtered_linear_y > self.cmd_vel.linear.y + self.max_linear_acc * time_diff:
            filtered_linear_y = self.cmd_vel.linear.y + self.max_linear_acc * time_diff
        if filtered_angular_z < self.cmd_vel.angular.z - self.max_angler_acc * time_diff:
            filtered_angular_z = self.cmd_vel.angular.z - self.max_angler_acc * time_diff
        if filtered_angular_z > self.cmd_vel.angular.z + self.max_angler_acc * time_diff:
            filtered_angular_z = self.cmd_vel.angular.z + self.max_angler_acc * time_diff

        self.cmd_vel.linear.x = filtered_linear_x
        self.cmd_vel.linear.y = filtered_linear_y
        self.cmd_vel.angular.z = filtered_angular_z

        self.cmd_vel_pub.publish(self.cmd_vel)
        self.pre_time = self.get_clock().now()

    def calc_lpf(self, cur, ref):
        retval = (cur * self.cutoff_freq + ref * self.freq) / \
            (self.freq + self.cutoff_freq)
        return retval


def main(args=None):
    rclpy.init(args=args)
    andante_cmd_vel_smoother = AndanteCmdVelSmoother()
    rclpy.spin(andante_cmd_vel_smoother)
    andante_cmd_vel_smoother.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

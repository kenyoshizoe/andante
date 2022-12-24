import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from create_msgs.msg import Bumper, Cliff


class AndanteCmdVelSmoother(Node):
    def __init__(self):
        super().__init__("andante_cmd_vel_smoother")

        self.declare_parameter("rate", 30)
        self.declare_parameter("cutoff_freq", 10)
        self.declare_parameter("max_linear_acc", 0.5)
        self.declare_parameter("max_angler_acc", 1.0)
        self.declare_parameter("cmd_vel_reset_time", 1.0)
        self.declare_parameter("emergency_reset_time", 5.0)

        self.cutoff_freq = self.get_parameter("cutoff_freq").value
        self.max_linear_acc = self.get_parameter("max_linear_acc").value
        self.max_angler_acc = self.get_parameter("max_angler_acc").value
        self.cmd_vel_reset_time = self.get_parameter(
            "cmd_vel_reset_time").value
        self.emergency_reset_time = self.get_parameter(
            "emergency_reset_time").value

        self.ref_cmd_vel = Twist()
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

        self.emergency = False
        self.pre_cmded_time = self.get_clock().now()
        self.pre_emergency_time = self.get_clock().now()
        self.create_subscription(
            Bumper, "bumper", self.bumper_callback, 10
        )
        self.create_subscription(
            Cliff, "cliff", self.cliff_callback, 10
        )
        self.create_subscription(
            Empty, "wheeldrop", self.wheeldrop_callback, 10
        )

    def cmd_vel_callback(self, msg):
        if self.emergency:
            return
        self.ref_cmd_vel = msg
        self.pre_cmded_time = self.get_clock().now()

    def cmd_vel_smoother(self):
        if (self.get_clock().now() - self.pre_cmded_time).nanoseconds / 1e9 > self.cmd_vel_reset_time:
            self.ref_cmd_vel = Twist()
        if (self.get_clock().now() - self.pre_emergency_time).nanoseconds / 1e9 > self.emergency_reset_time:
            self.emergency = False
        if self.emergency:
            self.cmd_vel = Twist()
            self.ref_cmd_vel = Twist()

        filtered_linear_x = self.calc_lpf(
            self.cmd_vel.linear.x, self.ref_cmd_vel.linear.x)
        filtered_linear_y = self.calc_lpf(
            self.cmd_vel.linear.y, self.ref_cmd_vel.linear.y)
        filtered_angular_z = self.calc_lpf(
            self.cmd_vel.angular.z, self.ref_cmd_vel.angular.z)

        time_diff = (self.get_clock().now() -
                     self.pre_time).nanoseconds / 1e9

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

    def bumper_callback(self, msg):
        if msg.is_left_pressed or msg.is_right_pressed:
            self.emergency = True
            self.pre_emergency_time = self.get_clock().now()

    def cliff_callback(self, msg):
        if msg.is_cliff_left or msg.is_cliff_front_left or msg.is_cliff_right or msg.is_cliff_front_right:
            self.emergency = True
            self.pre_emergency_time = self.get_clock().now()

    def wheeldrop_callback(self, msg):
        self.emergency = True
        self.pre_emergency_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    andante_cmd_vel_smoother = AndanteCmdVelSmoother()
    rclpy.spin(andante_cmd_vel_smoother)
    andante_cmd_vel_smoother.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

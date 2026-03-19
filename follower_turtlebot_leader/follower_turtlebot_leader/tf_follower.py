import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math


class TFFollower(Node):

    def __init__(self):

        super().__init__('tf_follower')

        self.declare_parameter('leader', 'robot1')
        self.declare_parameter('follower', 'robot2')

        leader = self.get_parameter('leader').value
        follower = self.get_parameter('follower').value

        self.leader_frame = leader + '/base_link'
        self.follower_frame = follower + '/base_link'

        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{follower}/cmd_vel',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):

        try:

            trans = self.tf_buffer.lookup_transform(
                self.follower_frame,
                self.leader_frame,
                rclpy.time.Time()
            )

            dx = trans.transform.translation.x
            dy = trans.transform.translation.y

            distance = math.sqrt(dx*dx + dy*dy)

            cmd = Twist()

            follow_distance = 0.6

            if distance > follow_distance:

                angle = math.atan2(dy, dx)

                cmd.linear.x = 0.22
                cmd.angular.z = 2.0 * angle

            else:

                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

        except:
            pass


def main(args=None):

    rclpy.init(args=args)

    node = TFFollower()

    rclpy.spin(node)

    rclpy.shutdown()

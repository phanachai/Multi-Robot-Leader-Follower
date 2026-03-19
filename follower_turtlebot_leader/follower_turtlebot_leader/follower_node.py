import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('target_robot', 'robot1')
        self.declare_parameter('my_robot', 'robot2')

        self.target_robot = self.get_parameter('target_robot').value
        self.my_robot = self.get_parameter('my_robot').value

        self.path_history = [] 
        self.record_distance = 0.05 # บันทึกจุดทุกๆ 5 ซม.

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # --- ตัวแปรหลักของ Pure Pursuit ---
        self.lookahead_distance = 0.25 # ระยะ Ld: มองข้ามช็อตไปข้างหน้า 25 ซม. (ปรับให้ยาวขึ้นถ้าอยากให้เลี้ยวโค้งกว้างๆ)
        self.desired_velocity = 0.075   # ความเร็วเดินหน้าคงที่ v (m/s)

        self.create_subscription(Odometry, f'/{self.target_robot}/odom', self.leader_odom_callback, 10)
        self.create_subscription(Odometry, f'/{self.my_robot}/odom', self.follower_odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.my_robot}/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"True Pure Pursuit [{self.my_robot}] is tracking [{self.target_robot}]...")

    def leader_odom_callback(self, msg):
        target_x = msg.pose.pose.position.x
        target_y = msg.pose.pose.position.y

        if not self.path_history:
            self.path_history.append((target_x, target_y))
            return

        last_x, last_y = self.path_history[-1]
        dist_from_last = math.sqrt((target_x - last_x)**2 + (target_y - last_y)**2)

        if dist_from_last >= self.record_distance:
            self.path_history.append((target_x, target_y))

    def follower_odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def get_lookahead_point(self):
        # ค้นหาจุดในประวัติเส้นทาง ที่อยู่ห่างออกไปเท่ากับหรือมากกว่า Lookahead Distance
        for pt in self.path_history:
            dist = math.sqrt((pt[0] - self.robot_x)**2 + (pt[1] - self.robot_y)**2)
            if dist >= self.lookahead_distance:
                return pt
        
        # ถ้าไม่มีจุดไหนไกลพอเลย (เช่นตอนใกล้ถึงปลายทาง) ให้คืนค่าจุดสุดท้าย
        if self.path_history:
            return self.path_history[-1]
        return None

    def control_loop(self):
        cmd = Twist()

        # 1. ลบจุดเก่าที่เราวิ่งผ่านมาแล้วทิ้งไป (เก็บกินเศษขนมปังที่อยู่ใกล้กว่า 10 ซม.)
        while self.path_history:
            pt = self.path_history[0]
            dist = math.sqrt((pt[0] - self.robot_x)**2 + (pt[1] - self.robot_y)**2)
            if dist < 0.1:
                self.path_history.pop(0)
            else:
                break

        if not self.path_history:
            self.cmd_pub.publish(cmd)
            return

        # 2. หาจุดเป้าหมายข้ามช็อต (Lookahead Point)
        target_pt = self.get_lookahead_point()
        if not target_pt:
            self.cmd_pub.publish(cmd)
            return

        tx, ty = target_pt

        # 3. แปลงพิกัดโลก (Global) ให้เป็นพิกัดสัมพัทธ์กับตัวหุ่น (Local Frame)
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        
        # ใช้ 2D Rotation Matrix
        local_x = math.cos(self.robot_yaw) * dx + math.sin(self.robot_yaw) * dy
        local_y = -math.sin(self.robot_yaw) * dx + math.cos(self.robot_yaw) * dy

        # 4. คำนวณสมการ Pure Pursuit
        L_sq = local_x**2 + local_y**2
        if L_sq < 0.0001:
            self.cmd_pub.publish(cmd)
            return

        curvature = (2.0 * local_y) / L_sq

        # 5. สั่งการความเร็ว (Kinematics)
        # คำนวณระยะทางถึงจุดสุดท้าย เพื่อชะลอความเร็วตอนจบ
        final_pt = self.path_history[-1]
        final_dist = math.sqrt((final_pt[0] - self.robot_x)**2 + (final_pt[1] - self.robot_y)**2)

        if final_dist < self.lookahead_distance:
            # ชะลอความเร็วเมื่อเข้าใกล้จุดสุดท้าย (เบรกนิ่มๆ)
            v = max(self.desired_velocity * (final_dist / self.lookahead_distance), 0.0)
        else:
            v = self.desired_velocity

        # ω = v * γ
        angular_vel = v * curvature

        cmd.linear.x = v
        # จำกัดความเร็วการหมุนไม่ให้หุ่นสะบัดแรงเกินไป
        cmd.angular.z = max(min(angular_vel, 1.5), -1.5)

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty

msg = """
仿真控制已就绪！
---------------------------
移动控制:   云台控制:
   w          t (抬头)
 a s d        g (低头)
              f (左转)
              h (右转)
              r (归位)

空格键 : 刹车停止
CTRL-C : 退出
"""

class SimControlNode(Node):
    def __init__(self):
        super().__init__('sim_control_node')
        # 底盘速度发布者
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 云台角度发布者 (假设你的URDF里云台关节叫 pan_joint 和 tilt_joint)
        self.joint_pub = self.create_publisher(JointTrajectory, '/set_joint_angles', 10)
        
        self.pan = 0.0
        self.tilt = 0.0

    def send_drive(self, x, z):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self.drive_pub.publish(twist)

    def send_joints(self):
        traj = JointTrajectory()
        traj.joint_names = ['pan_joint', 'tilt_joint'] # 这里的名字必须和URDF里的joint name一致
        point = JointTrajectoryPoint()
        point.positions = [self.pan, self.tilt]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000 # 0.1s 移动到位
        traj.points.append(point)
        self.joint_pub.publish(traj)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = SimControlNode()
    print(msg)

    try:
        while True:
            tty.setraw(sys.stdin.fileno())
            r, w, e = select.select([sys.stdin], [], [], 0.1)
            if r:
                key = sys.stdin.read(1)
                if key == 'w': node.send_drive(0.5, 0.0)
                elif key == 's': node.send_drive(-0.5, 0.0)
                elif key == 'a': node.send_drive(0.0, 1.0)
                elif key == 'd': node.send_drive(0.0, -1.0)
                elif key == ' ': node.send_drive(0.0, 0.0)
                elif key == 'f': node.pan += 0.1; node.send_joints()
                elif key == 'h': node.pan -= 0.1; node.send_joints()
                elif key == 't': node.tilt += 0.1; node.send_joints()
                elif key == 'g': node.tilt -= 0.1; node.send_joints()
                elif key == 'r': node.pan = 0.0; node.tilt = 0.0; node.send_joints()
                elif key == '\x03': break # CTRL-C
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

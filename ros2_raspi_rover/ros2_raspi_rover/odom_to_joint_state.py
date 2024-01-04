import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from math import pi

WIDTH = 0.09 # 車体の幅(m)
RADIUS = 0.03 # 車輪の半径(m)

class WheelStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_state_publisher')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 車輪の半径と車輪間の距離
        self.wheel_radius = RADIUS  # 例: 0.1m
        self.wheel_separation = WIDTH  # 例: 0.5m
        
        # 左右の車輪の回転角（位置）の初期値
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        # 現在の時間を取得
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # 前回のコールバックからの経過時間（秒）
        
        # オドメトリデータから車輪の速度を計算
        vx = msg.twist.twist.linear.x  # 前進速度
        vth = msg.twist.twist.angular.z  # 回転速度
        
        # 左右の車輪の速度を計算
        right_wheel_vel = (vx + vth * self.wheel_separation / 2) / self.wheel_radius
        left_wheel_vel = (vx - vth * self.wheel_separation / 2) / self.wheel_radius
        
        # 左右の車輪の回転角（位置）を更新
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt
        
        # JointStateメッセージを作成
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [left_wheel_vel, right_wheel_vel]
        
        # パブリッシュ
        self.joint_pub.publish(joint_state)
        
        # 時間を更新
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WheelStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
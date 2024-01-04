import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

WIDTH = 0.09 # 車体の幅(m)
RADIUS = 0.03 # 車輪の半径(m)
TIMEOUT = 0.5 # タイムアウト時間 (秒)

class TwistSubRpmPub(Node):

    def __init__(self):
        super().__init__('twist_subscriber')
        self.cmd_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.left_wheel_publisher = self.create_publisher(
            Float64, 
            'target_left_radsec',
            10
        )
        self.right_wheel_publisher = self.create_publisher(
            Float64,
            'target_right_radsec',
            10
        )
        self.timer = self.create_timer(TIMEOUT, self.timeout_callback)
        self.cmd_subscription  # prevent unused variable warning
        self.last_recieved = self.get_clock().now()
        self.left_wheel_publisher
        self.right_wheel_publisher

    def InverseKinematics(self, x, theta):
        """ 逆運動学方程式

        Args:
            x (float64): x方向速度(m/s)
            theta (float64): z軸周りの回転角(rad/s)

        Returns:
            rw, lw (float64): 各ホイールの回転速度(rad/s) 
        """
        # 逆運動学方程式を使用して車輪の角速度を計算
        lw_radsec= (x - (theta * WIDTH / 2)) / RADIUS
        rw_radsec = (x + (theta * WIDTH / 2)) / RADIUS
        
        return lw_radsec, rw_radsec

    def listener_callback(self, msg):
         # 逆運動学方程式にx(m/s), y(m/s), theta(rad/s)を代入
        wheels = self.InverseKinematics(msg.linear.x, msg.angular.z)
        left_wheel_radsec =  wheels[0]
        right_wheel_radsec = wheels[1]
        
        # 計算したrpmを各ホイールに発信
        # TO Do: arudinoで受け取ってPID制御するように書き換える
        # 計算された角速度をパブリッシュ
        left_wheel_msg = Float64()
        left_wheel_msg.data = left_wheel_radsec
        self.left_wheel_publisher.publish(left_wheel_msg)

        right_wheel_msg = Float64()
        right_wheel_msg.data = right_wheel_radsec
        self.right_wheel_publisher.publish(right_wheel_msg)

        self.last_recieved = self.get_clock().now()
    
    def timeout_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_recieved).nanoseconds * 1e-9 >= TIMEOUT:
            left_wheel_msg = Float64()
            left_wheel_msg.data = 0.0
            self.left_wheel_publisher.publish(left_wheel_msg)

            right_wheel_msg = Float64()
            right_wheel_msg.data = 0.0
            self.right_wheel_publisher.publish(right_wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    pubsub = TwistSubRpmPub()
    while rclpy.ok():
        rclpy.spin_once(pubsub) # spin_onceにしないと/cmd_velトピックの値で回り続ける。
    else:
        pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
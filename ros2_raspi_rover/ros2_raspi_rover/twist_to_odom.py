import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
# from tf.transformations import quaternion_from_euler
import math

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')
        
        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Transform broadcaster for TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Variables to store pose and velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last time callback was called
        self.last_time = self.get_clock().now().to_msg()

    def cmd_vel_callback(self, msg):
        # Get current time
        current_time = self.get_clock().now().to_msg()
        
        # Compute time difference
        dt = (current_time.sec + current_time.nanosec / 1e9) - (self.last_time.sec + self.last_time.nanosec / 1e9)
        
        # Compute odometry
        dx = msg.linear.x * dt
        dy = msg.linear.y * dt
        dtheta = msg.angular.z * dt
        
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dtheta
        
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = euler_to_quaternion(0, 0, self.theta)
        # q = Quaternion(*quaternion_from_euler(0, 0, self.theta))
        odom.pose.pose.orientation = q
        
        odom.twist.twist = msg
        
        # Publish Odometry message
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        # Update last time
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

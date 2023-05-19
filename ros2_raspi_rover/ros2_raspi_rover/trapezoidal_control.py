import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrapezoidalController(Node):
    def __init__(self):
        super().__init__('trapezoidal_controller')
        self.pub = self.create_publisher(Twist, '/diff_drive_robot/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.current_linear_speed = 0.0
        self.target_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.target_angular_speed = 0.0
        self.linear_acceleration = 0.1  # This should be set according to your robot's capabilities
        self.linear_deceleration = 0.1  # This should be set according to your robot's capabilities
        self.angular_acceleration = 0.1  # This should be set according to your robot's capabilities
        self.angular_deceleration = 0.1  # This should be set according to your robot's capabilities
        self.timer = self.create_timer(0.1, self.update)  # creates a timer with 10Hz rate

    def cmd_vel_callback(self, msg):
        if self.current_linear_speed == 0.0 and self.current_angular_speed == 0.0:
            self.target_linear_speed = msg.linear.x
            self.target_angular_speed = msg.angular.z

    def update(self):
        dt = 0.1  # as we set the timer frequency to 10Hz

        # linear speed
        if self.current_linear_speed < self.target_linear_speed:
            self.current_linear_speed += self.linear_acceleration * dt
            if self.current_linear_speed > self.target_linear_speed:
                self.current_linear_speed = self.target_linear_speed
        elif self.current_linear_speed > self.target_linear_speed:
            self.current_linear_speed -= self.linear_deceleration * dt
            if self.current_linear_speed < self.target_linear_speed:
                self.current_linear_speed = self.target_linear_speed

        # angular speed
        if self.current_angular_speed < self.target_angular_speed:
            self.current_angular_speed += self.angular_acceleration * dt
            if self.current_angular_speed > self.target_angular_speed:
                self.current_angular_speed = self.target_angular_speed
        elif self.current_angular_speed > self.target_angular_speed:
            self.current_angular_speed -= self.angular_deceleration * dt
            if self.current_angular_speed < self.target_angular_speed:
                self.current_angular_speed = self.target_angular_speed

        # Here we control both the linear speed in x direction and the angular speed in z direction.
        twist = Twist()
        twist.linear.x = self.current_linear_speed
        twist.angular.z = self.current_angular_speed
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = TrapezoidalController()
    rclpy.spin(controller)  # handles callbacks and timers

    # Destroy the node explicitly (optional)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()

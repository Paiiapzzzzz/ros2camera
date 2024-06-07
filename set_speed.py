import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
class SetSpeed(Node):
    def __init__(self):
        super().__init__("set_speed")
        self.speed_pub_ = self.create_publisher(Float32,'desired_speed',10)
        self.timer_ = self.create_timer(1.0, self.send_speed_command)
        self.get_logger().info("Set speed started")

    def send_speed_command(self):
        msg = Float32()
        msg.data = 5.0
        self.speed_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    set_speed_publisher = SetSpeed()

    try:
        while rclpy.ok():
            rclpy.spin_once(set_speed_publisher)
    except KeyboardInterrupt:
        pass

    set_speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
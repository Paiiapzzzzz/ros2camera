import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String, Float32, Float64

class SteeringAngleSubscriber(Node):
    def __init__(self):
        super().__init__('steering_angle_subscriber')
        self.subscription = self.create_subscription(Float32, 'steering_angle_desired', self.steering_angle_callback, 10)
        self.subscription 
        self.last_received_time = self.get_clock().now()

        # Timer to check if data is received
        self.check_timer = self.create_timer(2.0, self.check_received_data)

    def steering_angle_callback(self, msg):
        steering_angle = round(msg.data, 0)
        self.get_logger().info(f'Received Steering Angle: {steering_angle} deg')
        self.last_received_time = self.get_clock().now()
    
    def check_received_data(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_received_time).nanoseconds
        time_diff_sec = time_diff / 1e9
        
        if time_diff_sec > 2.0:
            self.get_logger().warn("No data received from 'steering_angle_desired' topic")
    
class ActualSpeedSubscriber(Node):
    def __init__(self):
        super().__init__('actual_speed_subscriber')
        self.subscription = self.create_subscription(Float32, 'actual_speed', self.actual_speed_callback, 10)
        self.last_received_time = self.get_clock().now()

        # Timer to check if data is received
        self.check_timer = self.create_timer(2.0, self.check_received_data)

    def actual_speed_callback(self, msg):
        actual_speed = msg.data
        self.get_logger().info(f'Received Actual Speed: {actual_speed} km/hr')
        self.last_received_time = self.get_clock().now()

    def check_received_data(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_received_time).nanoseconds
        time_diff_sec = time_diff / 1e9
        
        if time_diff_sec > 2.0:
            self.get_logger().warn("No data received from 'actual_speed' topic")

def main(args=None):
    rclpy.init(args=args)
    steering_angle_subscriber = SteeringAngleSubscriber()
    actual_speed_subscriber = ActualSpeedSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(steering_angle_subscriber)
            rclpy.spin_once(actual_speed_subscriber)
    except KeyboardInterrupt:
        pass

    steering_angle_subscriber.destroy_node()
    actual_speed_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


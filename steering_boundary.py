import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SteeringBoundarySubscriber(Node):
    def __init__(self, steering_boundary_publisher):
        super().__init__('steering_boundary_subscriber')
        self.subscription = self.create_subscription(Float32, 'control_action', self.control_action_callback, 10)
        self.steering_boundary_publisher = steering_boundary_publisher

    def control_action_callback(self, msg):
        control_action = msg.data
        y = control_action
        print("control_action:", y)
        if y < -55:     
            self.steering_boundary_publisher.publish_steering_angle(-55.0)
        elif y > 55:  
            self.steering_boundary_publisher.publish_steering_angle(55.0)
        else:
            self.steering_boundary_publisher.publish_steering_angle(y)  
        

class SteeringBoundaryPublisher(Node):
    def __init__(self):
        super().__init__('steering_boundary_publisher')
        self.publisher_ = self.create_publisher(Float32, 'steering_angle_desired', 10)

    def publish_steering_angle(self, y):
        msg = Float32()
        msg.data = y
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Steering Angle: {y}')

def main(args=None):
    rclpy.init(args=args)
    steering_boundary_publisher = SteeringBoundaryPublisher()
    steering_boundary_subscriber = SteeringBoundarySubscriber(steering_boundary_publisher)
    rclpy.spin(steering_boundary_subscriber)
    steering_boundary_publisher.destroy_node()
    steering_boundary_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
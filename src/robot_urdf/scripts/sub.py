import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers

class MarkerSubscriber(Node):
    def __init__(self):
        super().__init__('marker_subscriber')

        # Subscribe to the ArucoMarkers topic to receive the marker IDs
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',  # This is the topic we want to listen to
            self.marker_callback,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning

    def marker_callback(self, msg):
        # Check if there are markers detected
        if msg.marker_ids:
            for marker_id in msg.marker_ids:
                self.get_logger().info(f'Marker ID detected: {marker_id}')
        else:
            self.get_logger().info('No markers detected')


def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
    marker_subscriber = MarkerSubscriber()

    # Spin to keep the node running
    rclpy.spin(marker_subscriber)

    # Clean up
    marker_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


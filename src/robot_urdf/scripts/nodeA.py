import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers

class ArucoIDSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_id_subscriber')

        # Subscribe to the /aruco_markers topic
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.marker_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def marker_callback(self, msg):
        # Check if there are any marker IDs
        if msg.marker_ids:
            marker_ids_list = list(msg.marker_ids)  # Convert the marker_ids array to a list
            self.get_logger().info(f'Marker IDs: {marker_ids_list}')
        else:
            self.get_logger().info('No markers detected.')

def main():
    rclpy.init()
    node = ArucoIDSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


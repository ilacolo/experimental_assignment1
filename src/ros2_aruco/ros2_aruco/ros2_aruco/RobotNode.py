import rclpy
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time

global z
global allFound

# Global list to store unique marker IDs
unique_marker_ids = []
marker_ids_found = []
allFound = False


class MarkerSubscriber(Node):

    """
    Class to subscribe to the ArucoMarkers and the /camera/image_raw topic,
    publish the angular velocity to the /cmd_vel topic 
    and the custom image to the processed_image topic
    """

    def __init__(self):
        super().__init__('RobotNode')

        # Publisher for the angular velocity
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for the custom image topic
        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)

        # Status variable to control the flow of the program
        self.status = 1

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the ArucoMarkers topic to receive the marker IDs
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',  
            self.marker_callback,
            10  
        )
        self.subscription

        # Subscriber for the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        self.camera_subscription


    def marker_callback(self, msg):

        """
        Callback function for the ArucoMarkers topic,
        during self.status == 1, the function will store the unique marker IDs
        during self.status == 2, the function will find the markers and call the cv2Process function
        to draw a circle around them.

        Input: msg (ArucoMarkers) - Message containing the marker IDs
        Output: None
        """


        global unique_marker_ids, marker_ids_found, allFound

        if msg.marker_ids:
            for marker_id in msg.marker_ids:
            
                if self.status == 1:

                    if marker_id not in unique_marker_ids:
                        unique_marker_ids.append(marker_id)  
                        self.get_logger().info(f'Marker ID detected: {marker_id}')

                    if len(unique_marker_ids) >= 5 and not allFound:
                        self.get_logger().info("Detected 5 unique markers: {}".format(unique_marker_ids))
                        self.status = 2

                elif self.status == 2:

                    idToFind = self.getIdToFind(marker_ids_found, unique_marker_ids)                    

                    if marker_id == idToFind:

                        self.get_logger().info(f'Found marker: {marker_id}')

                        if self.cv2Process(marker_id):
                            marker_ids_found.append(marker_id)

                    if len(marker_ids_found) == len(unique_marker_ids):
                        self.get_logger().info("All found!!!")
                        allFound = True
                        self.publish_movement(0.0)
                        self.status = 3

        else:
            self.get_logger().info('No markers detected')


    def publish_movement(self, angular_z):
        
        """
        Function to publish the angular velocity to the /cmd_vel topic,
        Input: angular_z (float) - Angular velocity to publish
        """

        msg = Twist()
        msg.angular.z = angular_z
        self.publisher_.publish(msg)


    def getIdToFind(self, v1, v2):

        """
        Function to find the ID of the marker to find,
        Input: v1 (list) - List of found marker IDs
               v2 (list) - List of all unique marker IDs
        Output: id (int) - ID of the marker to find
        """
        id = 0

        if len(v1) == 0:
            id = min(v2)
        else:
            v2.sort()
            id = v2[len(v1)]

        return id

    def camera_callback(self, msg):

        """
        Callback function for the /camera/image_raw topic,
        the function will convert the image message to a cv
        """
        
        try:
            bridge = CvBridge()
            self.current_image = self.bridge.imgmsg_to_cv2(msg,
                                             desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    



    def cv2Process(self, marker_id):

        """
        Function to draw a circle around the detected marker,
        the processed image will be published to the processed_image topic

        Input: marker_id (int) - ID of the marker to draw a circle around
        Output: state (bool) - True if the marker was found and the circle was drawn, False otherwise
        """

        global marker_ids_found
        img = self.current_image
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        self.get_logger().info(f'Image dtype: {img.dtype}, shape: {img.shape}')
        
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        
        marker_corners, marker_ids, rejected_candidates = aruco.detectMarkers(img, aruco_dict, parameters=parameters)

        self.get_logger().info(f'marker_ids: {marker_ids}')
        self.get_logger().info(f'marker_corners: {marker_corners}')
        self.get_logger().info(f'rejected_candidates: {rejected_candidates}')

        state = False
        if marker_ids is not None and len(marker_ids) > 0:

            for i, corners in enumerate(marker_corners):

                center_x = int(np.mean(corners[0][:, 0]))
                center_y = int(np.mean(corners[0][:, 1]))

                if marker_ids[i][0] == marker_id:
                    cv2.circle(img, (center_x, center_y), 50, (0, 0, 255), 5)
                    self.get_logger().info(f'Center of marker {marker_ids[i][0]}: ({center_x}, {center_y})')
                    cv2.imwrite(f'img{len(marker_ids_found)}.png', img)
                    cv2.imshow('Detected Markers', img)
                    cv2.waitKey(1)
                    try:
                        processed_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
                        self.image_publisher.publish(processed_image_msg)
                        self.get_logger().info("Published processed image")
                    except Exception as e:
                        self.get_logger().error(f'Failed to publish processed image: {e}')

                    state = True
        else:
            self.get_logger().info("No markers detected.")
        
        
        return state


def main(args=None):
    rclpy.init(args=args)

    """
    Main function to initialize the MarkerSubscriber and publish the angular velocity
    """
    time.sleep(5)
    marker_subscriber = MarkerSubscriber()

    global allFound
    time.sleep(5)
    marker_subscriber.publish_movement(0.5)

    while not allFound:
        rclpy.spin_once(marker_subscriber)

    marker_subscriber.publish_movement(0.0)
    time.sleep(5)

    marker_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

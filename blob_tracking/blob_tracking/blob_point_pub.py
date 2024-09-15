import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import List, Tuple

class BlobPointPublisher(Node):
    def __init__(self) -> None:
        super().__init__('blob_point_publisher')

        self.subscription = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Point, '/point_blob', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg:Image) -> None:
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        except Exception as e:
            self.get_logger().error("error converting ROS Image to OpenCV format : {0}".format(e))
            return 

        # convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define lower and upper bounds for orange color in HSV
        lower_orange = np.array([3, 0, 233])
        upper_orange = np.array([9, 255, 255])

        # create a binary mask for orange color
        mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # draw coordinate frame
        self.draw_frame(cv_image)

        # find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # draw a red circle around the blob contour
        for contour in contours:
            self.draw_blob_contour_circle(cv_image, contour)

        # if no contours are found return
        if not contours:
            return

        # find largest contour
        blob_contour = max(contours, key=cv2.contourArea)

        # calculate centroid of contour
        moment = cv2.moments(blob_contour)
        if moment["m00"] != 0:
            blob_x = float(moment["m10"] / moment["m00"]) # x coordinate of centroid
            blob_y = float(moment["m01"] / moment["m00"]) # y coordinate of centroid

            x, y = self.get_blob_relative_position(cv_image, blob_x, blob_y)

            # publish blob
            self.publish_blob(x, y)

            # draw detected blobs
            self.draw_keypoints(cv_image, [(blob_x, blob_y)])

        # display image
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

    def draw_frame(self, 
                image: np.ndarray,
                dimension = 0.1, 
                line = 2) -> None:
        
        """Draw X Y coordinate frame at center of image"""

        rows = image.shape[0]
        cols = image.shape[1]
        size = min([rows, cols])
        center_x = int(cols/2.0)
        center_y = int(rows/2.0)

        line_length = int(size * dimension)

        # -- X
        image = cv2.line(image, (center_x, center_y), (center_x+line_length, center_y), (0,0,255), line)

        # -- Y
        image = cv2.line(image, (center_x, center_y), (center_x, center_y + line_length), (0,255,0), line)

    def draw_blob_contour_circle(self, img:np.ndarray, contour:np.ndarray) -> None:
        """Draw a red circle around the detected blob"""

        # Get bounding rectangle of contour
        x, y, w, h = cv2.boundingRect(contour)

        # calculate center of rectangle
        center_x = x + w //2
        center_y = y + h //2

        # calculate radius of circle based on contour size
        radius = max(w,h) //2

        cv2.circle(img, (center_x, center_y), radius, (0,0,255), 2)

    def get_blob_relative_position(self, image:np.ndarray, x:float, y:float) -> Tuple[float, float]:
        """ Get blob position relative to coordinate frame placed at center of image"""
        # the shape attribute of a numpy array return a tuple representing dimensions of array
        # for an image the shape tuple consists of 3 elements: (height, width, channels)

        rows = float(image.shape[0]) # height
        cols = float(image.shape[1]) # width

        # coordinate of center of image
        center_x = 0.5 * cols
        center_y = 0.5 * rows

        # the x and y coordinates of keypoint are computed relative to center of image 
        x = (x - center_x) / (center_x)
        y = (y - center_y) / (center_y)

        return x, y

    def publish_blob(self, x: float, y: float) -> None:
        """ Publish the blob position to /point_blob """
        # Create a Point message
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y

        # Publish the Point message
        self.publisher.publish(point_msg)


    def draw_keypoints(self, img: np.ndarray, keypoints: List[Tuple[float, float]]) -> None:
        """ Draw the detected blob keypoints in red """
        # Draw red circles at the detected blob keypoints
        for kp in keypoints:
            # Convert keypoints to integers
            kp = (int(kp[0]), int(kp[1]))
            cv2.circle(img, kp, 5, (0, 0, 255), -1)


def main(args = None) -> None:
    rclpy.init(args=args)
    blob_point_publisher = BlobPointPublisher()
    rclpy.spin(blob_point_publisher)
    blob_point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    

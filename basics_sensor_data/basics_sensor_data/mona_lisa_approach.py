import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MonaLisaApproach(Node):
    def __init__(self) -> Node:
        super().__init__('mona_lisa_approach')
        # subscribing to laser topic scan /deepmind_robot1/laser_scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/deepmind_robot1/laser_scan',
            self.laser_callback,
            10
        )

        # publisher to /cmd_vel topic to move robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.mona_lisa_distance = None

    def laser_callback(self, msg: LaserScan) -> None:
        """Callback function for laser scan data"""

        # find index corresponding to direction directly in front of robot
        # we want to access the index corresponding to direction directly ahead of robot
        middle_index = len(msg.ranges) //2
        self.mona_lisa_distance = msg.ranges[middle_index]

    def timer_callback(self) -> None:
        # check if mona lisa distance is greater than 0.5
        if self.mona_lisa_distance > 0.5:
            self.move_forward() # move the robot forward
        else:
            self.stop_robot() # stop the robot
            rclpy.shutdown()
    
    def move_forward(self) -> None:
        """Move the robot forward"""

        print(f'MOVE FORWARD... Mona Lisa is {self.mona_lisa_distance:.2f} meters away')
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        self.publisher.publish(twist_msg)

    def stop_robot(self) -> None:
        """Stop the robot"""

        print(f'STOP...')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)

def main(args = None) -> None:
    rclpy.init(args=args)
    mona_lisa_approach = MonaLisaApproach()
    rclpy.spin(mona_lisa_approach)
    mona_lisa_approach.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



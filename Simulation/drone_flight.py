import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import csv
import time
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
initial_time=time.time()
class WaypointFollower(Node):

    def __init__(self, csv_file):

        super().__init__('waypoint_follower')

        # Publisher and Subscriber setup
        self.geo_pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.image_sub = self.create_subscription(Image, '/downward_camera/image_raw', self.camera_callback, 10)
        
        # Load waypoints from CSV file
        self.waypoints = []
        self.load_waypoints(csv_file)
        self.current_waypoint_index = 0
        self.current_position = None  # Initialize current position
        self.photo_taken_at_waypoint = False  # Flag to ensure only one photo per waypoint

        # QoS profile for pose subscription
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile)

        # Timer to regularly check and publish waypoints (10 Hz)
        self.create_timer(0.1, self.timer_callback)

        # Image handling setup
        self.bridge = CvBridge()
        self.image_counter = 0
        self.image_folder = "/home/thomas/drone_images"  # Folder to save images
        os.makedirs(self.image_folder, exist_ok=True)  # Create folder if not exists
        self.csv_file = os.path.join(self.image_folder, "image_data.csv")
        self.create_csv()

    def create_csv(self):
        """Create the CSV file for logging image data"""
        with open(self.csv_file, 'w', newline='') as csvfile:
            fieldnames = ['image_name', 'timestamp', 'latitude', 'longitude', 'altitude']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def camera_callback(self, msg):
        """Captures images when the drone reaches a waypoint and logs metadata"""
        global cv_image, image_filename, image_path
        self.image_counter += 1
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Create image filename with current timestamp
        image_filename = f"image_{self.image_counter}_{int(time.time())}.png"
        image_path = os.path.join(self.image_folder, image_filename)
        
        
        
    def load_waypoints(self, csv_file):
        """Load waypoints from CSV file"""
        try:
            with open(csv_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header if there is one
                for i, row in enumerate(reader):
                    if len(row) == 2:  # Expecting 2D waypoints (latitude, longitude)
                        try:
                            latitude, longitude = map(float, row)
                            altitude = 10.0  # Fixed altitude for all waypoints
                            self.waypoints.append((latitude, longitude, altitude))
                        except ValueError:
                            self.get_logger().error(f"Invalid data on row {i}: {row}")
                    else:
                        self.get_logger().error(f"Invalid data on row {i}: {row}")
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")

    def position_callback(self, msg):
        """Update current position from the received pose message"""
        self.current_position = msg.pose.position

    def timer_callback(self):
        """Regularly check and publish waypoints"""
        if self.current_position is None:
            self.get_logger().info("Waiting for the current position...")
            return
        
        if self.current_waypoint_index < len(self.waypoints):
            latitude, longitude, altitude = self.waypoints[self.current_waypoint_index]
            geo_pose = PoseStamped()
            geo_pose.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
            geo_pose.header.frame_id = 'map'  # Set the frame ID
            geo_pose.pose.position.x =longitude -50
            geo_pose.pose.position.y = 50 - latitude
            geo_pose.pose.position.z = altitude
            self.get_logger().info(f"Publishing waypoint: {longitude-50}, {50-latitude}, {altitude}")
            self.geo_pose_pub.publish(geo_pose)
            i=0
            # Check if the drone has reached the waypoint
            if self.has_reached_waypoint(longitude-50, 50-latitude, altitude):
                self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached.")
# Save image
                cv2.imwrite(image_path, cv_image)

                # Log image metadata into CSV
                with open(self.csv_file, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([image_filename, time.time()-initial_time, self.current_position.x, self.current_position.y, self.current_position.z])
                
                self.get_logger().info(f"Image saved: {image_filename} at position: {self.current_position.x}, {self.current_position.y}, {self.current_position.z}")

                # Set flag to prevent additional photo captures at the current waypoint
                self.current_waypoint_index += 1  # Move to next waypoint
            else:
                self.get_logger().info(f"Drone is still moving to waypoint {self.current_waypoint_index + 1}.")

        else:
            self.get_logger().info("All waypoints reached.")

    def has_reached_waypoint(self, target_x, target_y, target_z, threshold=0.5):
        """Check if the drone has reached the target waypoint"""
        if self.current_position:
            current_x = self.current_position.x
            current_y = self.current_position.y
            current_z = self.current_position.z

            # Calculate the distance to the target waypoint
            distance = ((current_x - target_x) ** 2 + (current_y - target_y) ** 2 + (current_z - target_z) ** 2) ** 0.5
            return distance < threshold  # Check if within threshold distance
        return False

def main():
    rclpy.init()

    # Path to the CSV file (adjust if needed)
    csv_file = '/home/thomas/inter_iit/src/my_gazebo_world/src/waypoints_final.csv'

    waypoint_follower = WaypointFollower(csv_file)

    try:
        rclpy.spin(waypoint_follower)
    except KeyboardInterrupt:
        pass

    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

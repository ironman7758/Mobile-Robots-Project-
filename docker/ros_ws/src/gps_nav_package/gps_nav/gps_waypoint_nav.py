import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from pyproj import Proj

class GPSNav(Node):
    def __init__(self):
        super().__init__('gps_nav_node')

        self.origin_lat = None
        self.origin_lon = None
        self.proj_utm = Proj(proj='utm', zone=50, south=True, ellps='WGS84', preserve_units=False)

        self.sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.navigator = BasicNavigator()

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn("Waiting for valid GPS fix...")
            return

        self.origin_lat = msg.latitude
        self.origin_lon = msg.longitude
        self.get_logger().info(f"GPS origin fix received: {self.origin_lat}, {self.origin_lon}")

        # Now start navigation
        self.sub.destroy()
        self.start_navigation()

    def gps_to_xy(self, lat, lon):
        ox, oy = self.proj_utm(self.origin_lon, self.origin_lat)
        x, y = self.proj_utm(lon, lat)
        return x - ox, y - oy

    def start_navigation(self):
        self.navigator.waitUntilNav2Active()
        goals = []

        with open(os.path.join(os.path.dirname(__file__), '../waypoints/waypoints.csv'), newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                lat, lon = float(row[0]), float(row[1])
                x, y = self.gps_to_xy(lat, lon)

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0  # face forward

                goals.append(pose)

        self.navigator.followWaypoints(goals)
        while not self.navigator.isTaskComplete():
            self.get_logger().info("Driving to waypoint...")
            rclpy.spin_once(self)

        self.get_logger().info("Navigation complete!")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
